#!/usr/bin/env python3
"""Offline USD/source checks. Run using the CAD .venv, which supplies usd-core."""
import hashlib
import json
from pathlib import Path
from collections import Counter
import numpy as np
from pxr import Usd, UsdGeom, UsdPhysics

ROOT = Path(__file__).resolve().parents[1]
source = ROOT / "Delta+Robot"
inventory = json.loads((ROOT / "artifacts/inspection/cad_inventory.json").read_text())
for entry in inventory["original_files"]:
    actual = hashlib.sha256((source / entry["name"]).read_bytes()).hexdigest()
    assert actual == entry["sha256"], f"Original modified: {entry['name']}"
# Check ownership against actual CAD ball-joint locations. A merged visual can
# look correct at the reference pose even when assigned to the wrong body.
geometry = json.loads((ROOT / "assets/generated/robot_geometry.json").read_text())
rotation = np.array(geometry["cad_to_base_rotation"])
origin = np.array(geometry["cad_origin_mm"])
mounts = {f"upper_{i}": np.array(geometry["elbows_reference"])[i] for i in range(3)}
mounts["platform"] = (np.array(geometry["platform_reference"])
                      + np.array(geometry["platform_offsets"]).reshape(-1, 3))
fasteners_checked = 0
for owner in mounts:
    for index in geometry["groups"][owner]:
        part = inventory["parts"][index]
        top_component = part["path"].split("/")[1]
        if index < 214 or not top_component.startswith(("M3x50", "Space 3mm", "M3 SL Nut")):
            continue
        center = (np.mean(part["bounds_mm"], axis=0)-origin) @ rotation.T * 0.001
        distances = {body: np.linalg.norm(points-center, axis=1).min() for body, points in mounts.items()}
        nearest = min(distances, key=distances.get)
        assert distances[nearest] < 0.025, f"Fastener {index} has no nearby mounting joint"
        assert owner == nearest, f"Fastener {index} belongs to {nearest}, assigned to {owner}"
        fasteners_checked += 1
stage = Usd.Stage.Open(str(ROOT / "assets/generated/sorting_cell.usdc"))
assert stage
counts = Counter(p.GetTypeName() for p in stage.Traverse())
assert UsdGeom.GetStageMetersPerUnit(stage) == 1.0
assert UsdGeom.GetStageUpAxis(stage) == "Z"
assert counts["PhysicsRevoluteJoint"] == 3
assert counts["PhysicsSphericalJoint"] == 12
assert not any(p.HasAPI(UsdPhysics.ArticulationRootAPI) for p in stage.Traverse())
for p in stage.Traverse():
    if p.IsA(UsdPhysics.Joint):
        j = UsdPhysics.Joint(p)
        for target in j.GetBody0Rel().GetTargets()+j.GetBody1Rel().GetTargets():
            assert stage.GetPrimAtPath(target).HasAPI(UsdPhysics.RigidBodyAPI)
    if p.IsA(UsdGeom.Mesh):
        assert not p.HasAPI(UsdPhysics.CollisionAPI), "CAD visual should not be a dynamic triangle collider"
report = {"original_files_unchanged": len(inventory["original_files"]),
          "meters_per_unit": 1.0, "up_axis": "Z", "joint_counts": {k: counts[k] for k in counts if "Joint" in k},
          "articulation_roots": 0, "meshes": counts["Mesh"],
          "moving_fasteners_checked": fasteners_checked, "passed": True}
(ROOT / "artifacts/inspection/scene_audit.json").write_text(json.dumps(report, indent=2))
print(json.dumps(report, indent=2))
