"""Small installation probe. Run using the installed Isaac Sim python.sh."""
from isaacsim import SimulationApp

app = SimulationApp({"headless": True, "width": 640, "height": 480})
import json
import sys
from pathlib import Path
import omni.usd
import isaacsim.core.experimental.utils.app as app_utils
from isaacsim.core.simulation_manager import SimulationManager
from pxr import UsdGeom, UsdPhysics

app_utils.enable_extension("isaacsim.ros2.bridge")
app.update()
import rclpy
from sensor_msgs.msg import Image

stage = omni.usd.get_context().get_stage()
SimulationManager.switch_physics_engine("physx")
SimulationManager.setup_simulation(dt=1.0 / 240.0, device="cpu")
cube = UsdGeom.Cube.Define(stage, "/Probe")
cube.CreateSizeAttr(0.02)
UsdGeom.XformCommonAPI(cube).SetTranslate((0, 0, 1))
UsdPhysics.RigidBodyAPI.Apply(cube.GetPrim())
UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
app_utils.play()
for _ in range(20):
    app.update()
rclpy.init()
node = rclpy.create_node("delta_isaac_probe")
result = {"python": sys.version, "engine": SimulationManager.get_active_physics_engine(),
          "time": SimulationManager.get_simulation_time(), "rclpy": rclpy.__file__,
          "cube_z": UsdGeom.XformCache().GetLocalToWorldTransform(cube.GetPrim()).ExtractTranslation()[2]}
print("PROBE_RESULT", json.dumps(result), flush=True)
Path("artifacts/inspection/isaac_probe.json").write_text(json.dumps(result, indent=2))
node.destroy_node()
rclpy.shutdown()
app_utils.stop()
app.close()
