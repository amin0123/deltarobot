"""ROS action server for validated, time-scaled Delta joint motion."""
import time
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from rclpy.task import Future
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from delta_interfaces.action import MoveDelta
from .config import node_config, load_geometry
from .kinematics import DeltaKinematics, KinematicsError


class Motion(Node):
    def __init__(self):
        super().__init__("delta_motion")
        self.cfg = node_config(self)
        self.kin = DeltaKinematics(load_geometry(), self.cfg)
        self.q, self.pose, self.active, self.hold = None, None, None, None
        self.busy, self.stopped = False, False
        self.last_feedback = -100.0
        self.wall_feedback = time.monotonic()
        self.pub = self.create_publisher(JointState, "/delta/joint_command", 5)
        self.create_subscription(JointState, "/joint_states", self.joints, 10)
        self.create_subscription(PoseStamped, "/delta/tool_pose", self.tool, 10)
        self.create_subscription(Bool, "/cell/estopped", self.stop,
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.group = ReentrantCallbackGroup()
        self.server = ActionServer(self, MoveDelta, "/delta/move_to", self.execute,
                                   goal_callback=self.goal, cancel_callback=lambda _: CancelResponse.ACCEPT,
                                   callback_group=self.group)
        self.create_timer(0.02, self.tick, callback_group=self.group)
        self.create_timer(1.0, self.watchdog, clock=Clock(clock_type=ClockType.STEADY_TIME), callback_group=self.group)

    def now(self):
        return self.get_clock().now().nanoseconds*1e-9

    def joints(self, message):
        if message.name != [f"shoulder_{i}" for i in range(3)] or len(message.position) != 3:
            return
        self.q = np.array(message.position)
        self.last_feedback = self.now()
        self.wall_feedback = time.monotonic()

    def tool(self, message):
        if message.header.frame_id == "base_link":
            p = message.pose.position
            self.pose = np.array([p.x, p.y, p.z])

    def stop(self, message):
        self.stopped = message.data
        if self.stopped and self.active:
            self.finish(False, "emergency stop")

    def goal(self, request):
        if self.busy or self.stopped or self.q is None or self.pose is None or self.now()-self.last_feedback > 0.5:
            return GoalResponse.REJECT
        try:
            self.kin.path(self.pose, [request.target.x, request.target.y, request.target.z], request.speed)
        except (KinematicsError, ValueError):
            return GoalResponse.REJECT
        self.busy = True
        return GoalResponse.ACCEPT

    async def execute(self, handle):
        target = np.array([handle.request.target.x, handle.request.target.y, handle.request.target.z])
        future = Future()
        try:
            duration, qpath = self.kin.path(self.pose, target, handle.request.speed)
        except (ValueError, KinematicsError) as error:
            handle.abort()
            self.busy = False
            return MoveDelta.Result(success=False, message=str(error))
        self.active = {"handle": handle, "future": future, "target": target, "duration": duration,
                       "path": qpath, "start": self.now(), "settled": None}
        self.get_logger().info(f"Move to {target.round(4).tolist()} in {duration:.2f}s")
        return await future

    def finish(self, success, message, canceled=False):
        action = self.active
        if not action:
            return
        self.hold = action["path"][-1].copy() if success else self.q.copy()
        if canceled:
            action["handle"].canceled()
        elif success:
            action["handle"].succeed()
        else:
            action["handle"].abort()
        action["future"].set_result(MoveDelta.Result(success=success, message=message))
        self.active = None
        self.busy = False

    def watchdog(self):
        if self.active and time.monotonic()-self.wall_feedback > 10:
            self.finish(False, "wall-time feedback watchdog: simulator or DDS stalled")

    def publish(self, q):
        message = JointState()
        message.header.stamp = self.get_clock().now().to_msg()
        message.name = [f"shoulder_{i}" for i in range(3)]
        message.position = q.tolist()
        self.pub.publish(message)

    def tick(self):
        if self.stopped:
            return
        action = self.active
        if action:
            if action["handle"].is_cancel_requested:
                self.finish(False, "goal canceled", canceled=True)
                return
            t = self.now()-action["start"]
            if self.now()-self.last_feedback > 0.5:
                self.finish(False, "joint feedback stale")
                return
            if t > action["duration"]+5:
                self.finish(False, "motion tracking timeout")
                return
            index = np.clip(t/action["duration"], 0, 1)*(len(action["path"])-1)
            lo = min(int(index), len(action["path"])-2)
            desired = action["path"][lo] + (index-lo)*(action["path"][lo+1]-action["path"][lo])
            self.publish(desired)
            error = float(np.linalg.norm(self.pose-action["target"]))
            action["handle"].publish_feedback(MoveDelta.Feedback(position_error=error))
            if (t >= action["duration"] and error < self.cfg["robot"]["position_tolerance"]
                    and np.max(abs(self.q-action["path"][-1])) < 0.012):
                if action["settled"] is None:
                    action["settled"] = self.now()
                elif self.now()-action["settled"] >= 0.15:
                    self.finish(True, f"tool error {error:.6f} m")
            else:
                action["settled"] = None
        elif self.hold is not None:
            self.publish(self.hold)


def main():
    rclpy.init()
    node = Motion()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
