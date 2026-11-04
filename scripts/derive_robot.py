#!/usr/bin/env python3
"""Derive mechanism coordinates from tessellated STEP audit, with explicit provenance."""
import json
from pathlib import Path
import numpy as np

ROOT = Path(__file__).resolve().parents[1]


def main():
    inventory = json.loads((ROOT / "artifacts/inspection/cad_inventory.json").read_text())
    parts = inventory["parts"]
    rotation = np.array([[1, 0, 0], [0, 0, 1], [0, -1, 0]])

    def sphere(i):
        return np.array(next(s["origin_mm"] for s in parts[i]["surfaces"] if s["type"] == "sphere"))

    # Original STEP instance IDs are recorded so these measurements are auditable.
    rods = [[(245, 247), (252, 254)], [(263, 265), (270, 272)], [(216, 218), (221, 223)]]
    gears = [27, 96, 159]
    lower_cad = np.array([[sphere(lo) for lo, hi in pair] for pair in rods])
    upper_cad = np.array([[sphere(hi) for lo, hi in pair] for pair in rods])
    pivots = []
    for gear, endpoints in zip(gears, upper_cad):
        bearing = next(s for s in parts[gear]["surfaces"] if abs(s["radius_mm"] - 6.25) < 1e-4)
        origin, axis = np.array(bearing["origin_mm"]), np.array(bearing["axis"])
        pivots.append(origin + axis * np.dot(endpoints.mean(0) - origin, axis))
    pivots = np.array(pivots)
    cad_origin = pivots.mean(0)
    transform = lambda x: (np.array(x) - cad_origin) @ rotation.T * 0.001
    shoulders = transform(pivots)
    elbows = transform(upper_cad)
    wrists = transform(lower_cad)
    platform = wrists.mean((0, 1))
    offsets = wrists - platform
    directions = elbows.mean(1) - shoulders
    upper_lengths = np.linalg.norm(directions, axis=1)
    radial = directions.copy()
    radial[:, 2] = 0
    radial /= np.linalg.norm(radial, axis=1)[:, None]
    axes = np.cross([0, 0, 1], radial)
    q0 = np.arctan2(-directions[:, 2], np.sum(directions * radial, axis=1))
    lower_lengths = np.linalg.norm(wrists - elbows, axis=2)
    # Keep the supplied moving plate; replace the finger/servo subassembly with a vacuum cup.
    groups = {"base": list(range(27)), "upper_0": list(range(27, 55)),
              "upper_1": list(range(96, 124)), "upper_2": list(range(159, 187)),
              "platform": [228] + list(range(229, 233)) + list(range(237, 241)) + list(range(255, 259))}
    moving = set(sum(groups.values(), []))
    for i, pair in enumerate(rods):
        for j, (lo, hi) in enumerate(pair):
            ids = list(range(lo - 2, hi + 1))
            groups[f"rod_{i}_{j}"] = ids
            moving.update(ids)
    # Fixed drive hardware and support frame in original mechanical arrangement.
    groups["base"] += [i for i in range(214) if i not in moving]
    # Fasteners at the three elbows move with their upper arm.
    # The STEP interleaves wrist fasteners 255..258 with elbow fasteners
    # 259, 260, 266, 267. Keep them on their actual mounting bodies; a wrong
    # group looks correct at the CAD reference pose but floats during motion.
    for i, ids in enumerate([[241, 242, 248, 249], [259, 260, 266, 267], [224, 225, 226, 227]]):
        groups[f"upper_{i}"].extend(ids)
    included = set(sum(groups.values(), []))
    output = {"cad_origin_mm": cad_origin.tolist(), "cad_to_base_rotation": rotation.tolist(),
              "shoulders": shoulders.tolist(), "axes": axes.tolist(), "radial": radial.tolist(),
              "upper_lengths": upper_lengths.tolist(), "lower_lengths": lower_lengths.tolist(),
              "elbows_reference": elbows.tolist(), "platform_reference": platform.tolist(),
              "platform_offsets": offsets.tolist(), "q_reference": q0.tolist(),
              "joint_limits": [float(np.deg2rad(5)), float(np.deg2rad(85))],
              "tool_offset": [0, 0, -0.038], "groups": groups,
              "measurement_sources": {"gear_ids": gears, "ball_ids_lower_upper": rods},
              "omitted_ids": [p["id"] for p in parts if p["id"] not in included],
              "notes": ["q=0 is the outward horizontal upper arm; q increases downwards.",
                        "Joint limits are conservative commissioning assumptions, not manufacturer limits.",
                        "Belts, bearing internals and fasteners are merged into fixed or upper rigid bodies.",
                        "Original finger/servo gripper, cable routing and unidentified final solid are omitted."]}
    (ROOT / "assets/generated/robot_geometry.json").write_text(json.dumps(output, indent=2))
    for key in ["cad_origin_mm", "shoulders", "upper_lengths", "lower_lengths", "platform_reference", "q_reference"]:
        print(key, output[key])
    print("Total triangles:", sum(p["triangles"] for p in parts))


if __name__ == "__main__":
    main()
