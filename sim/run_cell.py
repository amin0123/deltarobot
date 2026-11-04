#!/usr/bin/env python3
"""Build and run the physical workcell; sorting decisions run in external ROS nodes."""
import argparse
import json
from pathlib import Path
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "ros2_ws/src/delta_sorting"))
parser = argparse.ArgumentParser()
parser.add_argument("--headless", action="store_true")
parser.add_argument("--build-only", action="store_true")
parser.add_argument("--exit-on-complete", action="store_true")
parser.add_argument("--capture-overview", action="store_true", help="Save rendered overview snapshots alongside camera frames")
parser.add_argument("--max-time", type=float, default=1000)
parser.add_argument("--scenario", choices=["normal", "empty", "failed_grasp", "uncertain"], default="normal")
parser.add_argument("--config", default=str(ROOT / "ros2_ws/src/delta_sorting/config/cell.yaml"))
parser.add_argument("--output", default=str(ROOT / "artifacts/latest"))
args, kit_args = parser.parse_known_args()
from isaacsim import SimulationApp
app = SimulationApp({"headless": args.headless, "width": 1280, "height": 720,
                     "renderer": "RaytracedLighting", "anti_aliasing": 0,
                     "multi_gpu": False})

import carb
import numpy as np
import omni.usd
import omni.replicator.core as rep
import isaacsim.core.experimental.utils.app as app_utils
from isaacsim.core.simulation_manager import SimulationManager
from pxr import UsdGeom, Gf
from delta_sorting.config import load_config, load_geometry
from scene import CellScene

app_utils.enable_extension("isaacsim.ros2.bridge")
app.update()
import rclpy
from ros_bridge import CellBridge

cfg = load_config(args.config)
geometry = load_geometry(ROOT / "assets/generated/robot_geometry.json")
output = Path(args.output)
output.mkdir(parents=True, exist_ok=True)
stage = omni.usd.get_context().get_stage()
SimulationManager.switch_physics_engine("physx")
scene = CellScene(stage, cfg, geometry).build(args.scenario)
# Keep a separately usable USD artifact. Runtime adapters are started by this script.
stage.GetRootLayer().Export(str(output / "cell.usdc"))
if args.scenario == "normal":
    stage.GetRootLayer().Export(str(ROOT / "assets/generated/sorting_cell.usdc"))
print("SCENE_SAVED", flush=True)
if args.build_only:
    app.close()
    raise SystemExit(0)
try:
    from omni.kit.viewport.utility import get_active_viewport
    viewport = get_active_viewport()
    if viewport:
        viewport.set_active_camera("/World/Overview")
except ImportError:
    pass
SimulationManager.setup_simulation(dt=cfg["simulation"]["physics_dt"], device="cpu")
# RGB only. No semantic/instance/depth annotator is connected to the perception pipeline.
render_product = rep.create.render_product("/World/Camera", (cfg["camera"]["width"], cfg["camera"]["height"]))
rgb = rep.AnnotatorRegistry.get_annotator("rgb")
rgb.attach(render_product)
overview_rgb = None
if args.capture_overview:
    overview_product = rep.create.render_product("/World/Overview", (1280, 720))
    overview_rgb = rep.AnnotatorRegistry.get_annotator("rgb")
    overview_rgb.attach(overview_product)
for _ in range(10):
    app.update()
SimulationManager.initialize_physics()
rclpy.init()
bridge = CellBridge(scene, args.scenario)
from std_msgs.msg import Bool
completed = [False]
bridge.create_subscription(Bool, "/evaluation/done", lambda message: completed.__setitem__(0, message.data), 5)
dt = cfg["simulation"]["physics_dt"]
image_every = max(1, round(1.0 / (dt*cfg["camera"]["frame_rate"])))
counter, images = 0, 0
started = time.monotonic()
try:
    while app.is_running():
        t = SimulationManager.get_simulation_time()
        if t >= args.max_time or (args.exit_on_complete and completed[0]):
            break
        # Process a bounded callback batch so DDS never starves the physics loop.
        for _ in range(3):
            rclpy.spin_once(bridge, timeout_sec=0)
        bridge.before_step(dt)
        SimulationManager.step()
        t = SimulationManager.get_simulation_time()
        if counter % 4 == 0:
            bridge.publish_state(t, truth=counter % 24 == 0)
        if counter % image_every == 0:
            rep.orchestrator.step(rt_subframes=1, delta_time=0.0, pause_timeline=False)
            rgba = rgb.get_data()
            if bridge.publish_image(rgba, t):
                images += 1
                if images in (10, 50) or images % 200 == 0:
                    from PIL import Image
                    Image.fromarray(rgba).save(output / f"camera_{images:05d}.png")
                    if overview_rgb is not None:
                        Image.fromarray(overview_rgb.get_data()).save(output / f"overview_{images:05d}.png")
            if counter % (image_every*100) == 0:
                print("SIM", round(t, 2), "q", np.round(bridge.q, 3).tolist(), "images", images,
                      "tool", np.round(bridge.tool_world-scene.offset, 4).tolist(), flush=True)
        counter += 1
        # Yield CPU to external ROS executors during fast headless stepping.
        if counter % 4 == 0:
            time.sleep(0.0005)
except KeyboardInterrupt:
    pass
finally:
    summary = {"sim_time": SimulationManager.get_simulation_time(), "physics_steps": counter,
               "rgb_frames": images, "wall_seconds": time.monotonic()-started,
               "grasp_calls": bridge.grasp_calls, "scenario": args.scenario}
    (output / "simulation.json").write_text(json.dumps(summary, indent=2))
    print("SIMULATION_RESULT", json.dumps(summary), flush=True)
    bridge.destroy_node()
    rclpy.try_shutdown()
    app.close()
