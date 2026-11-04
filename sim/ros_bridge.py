"""Isaac's supported in-process rclpy adapter: sensors and physical actuators only.

No color classification, target selection, IK, or sorting sequence lives here.
The evaluator topic is deliberately separate from all production node inputs.
"""
import json
import time
import numpy as np
from scipy.spatial.transform import Rotation
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from builtin_interfaces.msg import Time
from geometry_msgs.msg import PoseStamped, TransformStamped
from sensor_msgs.msg import Image, CameraInfo, JointState
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool, Trigger
from rosgraph_msgs.msg import Clock
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster
from pxr import Gf, UsdPhysics
from isaacsim.core.experimental.prims import RigidPrim


def stamp(seconds):
    ns = int(round(seconds*1e9))
    return Time(sec=ns//1_000_000_000, nanosec=ns % 1_000_000_000)


def transform(parent, child, position, xyzw, timestamp):
    t = TransformStamped()
    t.header.frame_id, t.child_frame_id, t.header.stamp = parent, child, timestamp
    t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = map(float, position)
    t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w = map(float, xyzw)
    return t


class CellBridge(Node):
    def __init__(self, scene, scenario="normal"):
        super().__init__("isaac_cell_bridge", parameter_overrides=[Parameter("use_sim_time", value=True)])
        self.scene, self.cfg, self.g, self.scenario = scene, scene.cfg, scene.g, scenario
        self.time = 0.0
        self.q = np.array(self.g["q_reference"])
        self.target, self.command = self.q.copy(), self.q.copy()
        self.last_command = -100.0
        self.estopped, self.belt_running = False, False
        self.held = None
        self.grasp_calls = 0
        self.wall_heartbeat = time.monotonic()
        self.joint_names = [f"shoulder_{i}" for i in range(3)]
        self.robots = RigidPrim([f"/World/Robot/Upper_{i}" for i in range(3)] + ["/World/Robot/Platform"])
        self.caps = RigidPrim(scene.cap_paths) if scene.cap_paths else None
        self.clock_pub = self.create_publisher(Clock, "/clock", 10)
        self.joints_pub = self.create_publisher(JointState, "/joint_states", 10)
        self.pose_pub = self.create_publisher(PoseStamped, "/delta/tool_pose", 10)
        self.image_pub = self.create_publisher(Image, "/camera/image_raw", qos_profile_sensor_data)
        self.info_pub = self.create_publisher(CameraInfo, "/camera/camera_info", qos_profile_sensor_data)
        self.grip_pub = self.create_publisher(Bool, "/gripper/attached", 10)
        self.belt_pub = self.create_publisher(Bool, "/conveyor/running", 10)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.stop_pub = self.create_publisher(Bool, "/cell/estopped", latched)
        self.truth_pub = self.create_publisher(String, "/evaluation/ground_truth", 5)
        self.create_subscription(JointState, "/delta/joint_command", self.joint_command, 5)
        self.create_service(SetBool, "/conveyor/set_running", self.conveyor)
        self.create_service(SetBool, "/gripper/set_enabled", self.gripper)
        self.create_service(SetBool, "/cell/estop", self.estop)
        self.create_service(Trigger, "/cell/reset", self.reset)
        self.tf = TransformBroadcaster(self)
        self.static_tf = StaticTransformBroadcaster(self)
        self.static_tf.sendTransform([
            transform("world", "base_link", scene.offset, [0, 0, 0, 1], stamp(0)),
            transform("base_link", "camera_optical", np.array(self.cfg["camera"]["position"])-scene.offset,
                      [1, 0, 0, 0], stamp(0)),
            transform("world", "conveyor", [0, 0, self.cfg["conveyor"]["top"]], [0, 0, 0, 1], stamp(0))])
        self.read_state()
        self.stop_pub.publish(Bool(data=False))

    def read_state(self):
        self.robot_pos, self.robot_quat = [x.numpy().copy() for x in self.robots.get_world_poses()]
        for i in range(3):
            q = self.robot_quat[i]
            rv = Rotation.from_quat(q[[1, 2, 3, 0]]).as_rotvec()
            self.q[i] = self.g["q_reference"][i] + np.dot(rv, self.g["axes"][i])
        self.platform_rotation = Rotation.from_quat(self.robot_quat[3][[1, 2, 3, 0]])
        self.tool_world = self.robot_pos[3] + self.platform_rotation.apply(self.g["tool_offset"])
        if self.caps:
            self.cap_pos, self.cap_quat = [x.numpy().copy() for x in self.caps.get_world_poses()]
        else:
            self.cap_pos, self.cap_quat = np.zeros((0, 3)), np.zeros((0, 4))

    def joint_command(self, message):
        if self.estopped:
            return
        if message.name != self.joint_names or len(message.position) != 3:
            return
        q = np.asarray(message.position)
        age = self.time - (message.header.stamp.sec+message.header.stamp.nanosec*1e-9)
        if not np.isfinite(q).all() or age > 0.5 or age < -0.1:
            return
        if np.any(q < self.g["joint_limits"][0]) or np.any(q > self.g["joint_limits"][1]):
            return
        self.command = q
        self.last_command = self.time

    def conveyor(self, request, response):
        if request.data and (self.estopped or self.held is not None):
            response.success, response.message = False, "cell stopped or gripper occupied"
            return response
        self.belt_running = request.data
        self.set_belt()
        response.success, response.message = True, "running" if request.data else "stopped"
        return response

    def set_belt(self):
        velocity = self.cfg["conveyor"]["speed"] if self.belt_running and not self.estopped else 0.0
        self.scene.belt_surface.GetSurfaceVelocityAttr().Set(Gf.Vec3f(velocity, 0, 0))

    def release(self):
        path = "/World/Robot/Joints/VacuumAttachment"
        if self.scene.stage.GetPrimAtPath(path):
            self.scene.stage.RemovePrim(path)
        self.held = None

    def gripper(self, request, response):
        if not request.data:
            self.release()
            response.success, response.message = True, "vacuum released"
            return response
        self.grasp_calls += 1
        if self.estopped or self.belt_running or self.scenario == "failed_grasp":
            response.success, response.message = False, "vacuum interlock or injected vacuum failure"
            return response
        if self.held is not None:
            response.success, response.message = True, "already attached"
            return response
        self.read_state()
        candidates = []
        for i, (pos, wxyz) in enumerate(zip(self.cap_pos, self.cap_quat)):
            rot = Rotation.from_quat(wxyz[[1, 2, 3, 0]])
            top = pos + rot.apply([0, 0, self.cfg["caps"]["height"]/2])
            delta = self.tool_world-top
            normal = rot.apply([0, 0, 1])
            if np.linalg.norm(delta[:2]) < 0.007 and abs(delta[2]) < 0.004 and normal[2] > 0.97:
                candidates.append((np.linalg.norm(delta), i, rot))
        if not candidates:
            response.success, response.message = False, "no cap sealing surface within cup tolerance"
            return response
        _, i, cap_rot = min(candidates, key=lambda entry: entry[0])
        joint = UsdPhysics.FixedJoint.Define(self.scene.stage, "/World/Robot/Joints/VacuumAttachment")
        joint.CreateBody0Rel().SetTargets(["/World/Robot/Platform"])
        joint.CreateBody1Rel().SetTargets([self.scene.cap_paths[i]])
        joint.CreateLocalPos0Attr(Gf.Vec3f(*self.g["tool_offset"]))
        joint.CreateLocalPos1Attr(Gf.Vec3f(*cap_rot.inv().apply(self.tool_world-self.cap_pos[i]).tolist()))
        joint.CreateLocalRot0Attr(Gf.Quatf(1))
        xyzw = (cap_rot.inv()*self.platform_rotation).as_quat()
        joint.CreateLocalRot1Attr(Gf.Quatf(float(xyzw[3]), Gf.Vec3f(*xyzw[:3].tolist())))
        joint.CreateBreakForceAttr(20.0)
        joint.CreateBreakTorqueAttr(0.5)
        joint.CreateCollisionEnabledAttr(False)
        joint.CreateExcludeFromArticulationAttr(True)
        self.held = i
        response.success, response.message = True, "vacuum attached"
        return response

    def estop(self, request, response):
        if request.data:
            self.estopped = True
            self.belt_running = False
            self.read_state()
            self.command = self.target = self.q.copy()
            self.set_belt()
            self.stop_pub.publish(Bool(data=True))
            response.success, response.message = True, "stop latched; vacuum held; use /cell/reset to clear"
        else:
            response.success, response.message = False, "use /cell/reset to clear stop"
        return response

    def reset(self, request, response):
        if self.held is not None:
            response.success, response.message = False, "release held cap in a safe location before reset"
            return response
        self.read_state()
        self.estopped = False
        self.belt_running = False
        self.command = self.target = self.q.copy()
        self.set_belt()
        self.stop_pub.publish(Bool(data=False))
        response.success, response.message = True, "reset complete; /sorting/reset can restart the task"
        return response

    def before_step(self, dt):
        # A missed ROS stream holds the last reached drive target, never extrapolates.
        if self.time-self.last_command > 0.5:
            self.command = self.target.copy()
        step = self.cfg["robot"]["joint_speed"]*dt
        self.target += np.clip(self.command-self.target, -step, step)
        for i, drive in enumerate(self.scene.drives):
            drive.GetTargetPositionAttr().Set(float(np.rad2deg(self.target[i]-self.g["q_reference"][i])))

    def publish_state(self, sim_time, truth=False):
        self.time = sim_time
        self.clock_pub.publish(Clock(clock=stamp(sim_time)))
        self.read_state()
        now = stamp(sim_time)
        joints = JointState()
        joints.header.stamp = now
        joints.name, joints.position = self.joint_names, self.q.tolist()
        linear, angular = self.robots.get_velocities()
        joints.velocity = [float(np.dot(a, axis)) for a, axis in zip(angular.numpy()[:3], self.g["axes"])]
        self.joints_pub.publish(joints)
        pose = PoseStamped()
        pose.header.frame_id, pose.header.stamp = "base_link", now
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = map(float, self.tool_world-self.scene.offset)
        q = self.platform_rotation.as_quat()
        pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w = map(float, q)
        self.pose_pub.publish(pose)
        self.tf.sendTransform([transform("base_link", "tool0", self.tool_world-self.scene.offset, q, now)] + [
            transform("base_link", f"upper_{i}", self.robot_pos[i]-self.scene.offset,
                      self.robot_quat[i][[1, 2, 3, 0]], now) for i in range(3)])
        if self.held is not None and np.linalg.norm(self.cap_pos[self.held]-self.tool_world) > 0.035:
            self.release()
        self.grip_pub.publish(Bool(data=self.held is not None))
        self.belt_pub.publish(Bool(data=self.belt_running))
        if truth:
            message = {"time": sim_time, "held_id": self.held, "grasp_calls": self.grasp_calls,
                       "q": self.q.tolist(), "tool": (self.tool_world-self.scene.offset).tolist(),
                       "caps": [{"id": i, "color": self.scene.cap_colors[i], "position": p.tolist()}
                                for i, p in enumerate(self.cap_pos)]}
            self.truth_pub.publish(String(data=json.dumps(message)))

    def publish_image(self, rgba, capture_time):
        if rgba is None or not hasattr(rgba, "shape") or rgba.ndim != 3 or rgba.shape[2] < 3:
            return False
        rgb = np.ascontiguousarray(rgba[:, :, :3], dtype=np.uint8)
        message = Image()
        message.header.stamp = stamp(capture_time)
        message.header.frame_id = "camera_optical"
        message.height, message.width = rgb.shape[:2]
        message.encoding, message.step, message.data = "rgb8", rgb.shape[1]*3, rgb.tobytes()
        c = self.cfg["camera"]
        f = c["width"]*c["focal_length_mm"]/c["horizontal_aperture_mm"]
        info = CameraInfo()
        info.header = message.header
        info.width, info.height = message.width, message.height
        info.distortion_model, info.d = "plumb_bob", [0.0]*5
        info.k = [f, 0.0, c["width"]/2, 0.0, f, c["height"]/2, 0.0, 0.0, 1.0]
        info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        info.p = [f, 0.0, c["width"]/2, 0.0, 0.0, f, c["height"]/2, 0.0, 0.0, 0.0, 1.0, 0.0]
        self.info_pub.publish(info)
        self.image_pub.publish(message)
        return True
