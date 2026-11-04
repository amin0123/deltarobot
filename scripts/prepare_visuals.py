#!/usr/bin/env python3
"""Make lighter derived CAD meshes; retain full tessellations and source checksums."""
import json
from pathlib import Path
import numpy as np
import trimesh

ROOT = Path(__file__).resolve().parents[1]
out = ROOT / "assets/generated"
geometry = json.loads((out / "robot_geometry.json").read_text())
parts = json.loads((ROOT / "artifacts/inspection/cad_inventory.json").read_text())["parts"]
rotation = np.array(geometry["cad_to_base_rotation"])
origin = np.array(geometry["cad_origin_mm"]) * 0.001
result, counts = {}, {}
for group, ids in geometry["groups"].items():
    meshes = []
    for i in ids:
        data = np.load(out / parts[i]["mesh"])
        m = trimesh.Trimesh(vertices=data["vertices"], faces=data["triangles"], process=True)
        target = 600 if parts[i]["name"] == "Rod" else 3500
        if len(m.faces) > target:
            m = m.simplify_quadric_decimation(face_count=target)
        m.vertices = (m.vertices-origin) @ rotation.T
        meshes.append(m)
    mesh = trimesh.util.concatenate(meshes)
    result[group + "_vertices"] = np.asarray(mesh.vertices, np.float32)
    result[group + "_triangles"] = np.asarray(mesh.faces, np.int32)
    counts[group] = len(mesh.faces)
np.savez_compressed(out / "visuals.npz", **result)
(out / "visual_simplification.json").write_text(json.dumps(counts, indent=2))
print(counts, "total", sum(counts.values()))
