#!/usr/bin/env python3
"""Sample the commissioned TCP workspace and validate every configured bin route."""
import json
from pathlib import Path
import sys
import numpy as np
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "ros2_ws/src/delta_sorting"))
from delta_sorting.config import load_geometry, load_config
from delta_sorting.kinematics import DeltaKinematics, KinematicsError

cfg = load_config(ROOT / "ros2_ws/src/delta_sorting/config/cell.yaml")
k = DeltaKinematics(load_geometry(ROOT / "assets/generated/robot_geometry.json"), cfg)
rows = []
rejected = 0
for x in np.linspace(-.18, .18, 37):
    for y in np.linspace(-.18, .18, 37):
        for z in np.linspace(-.385, -.280, 22):
            try:
                q = k.inverse([x, y, z])
                rows.append([x, y, z, *q])
            except KinematicsError:
                rejected += 1
np.savetxt(ROOT / "assets/generated/workspace.csv", rows, delimiter=",",
           header="tcp_x,tcp_y,tcp_z,shoulder_0,shoulder_1,shoulder_2", comments="")
routes = {}
for color, xy in cfg["bins"].items():
    duration, q = k.path(cfg["robot"]["home"], [*xy, cfg["bin_geometry"]["release_z"]], .12)
    routes[color] = {"duration": duration, "max_joint_radians": float(q.max())}
report = {"accepted_grid_points": len(rows), "rejected_grid_points": rejected,
          "note": "IK and bounds only; grid points are not all collision-free tool paths.", "bin_routes": routes}
(ROOT / "artifacts/inspection/workspace.json").write_text(json.dumps(report, indent=2))
print(json.dumps(report, indent=2))
