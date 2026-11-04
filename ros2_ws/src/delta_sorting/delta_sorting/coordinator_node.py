"""Explicit stop-and-pick state machine. Uses only ROS visual targets and feedback."""
import json
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool, Trigger
from geometry_msgs.msg import Point
from delta_interfaces.msg import CapTargets
from delta_interfaces.action import MoveDelta
from .config import node_config, load_geometry
from .kinematics import DeltaKinematics, KinematicsError


class Coordinator(Node):
    def __init__(self):
        super().__init__("sorting_coordinator")
        self.cfg = node_config(self)
        self.declare_parameter("max_attempts", self.cfg["task"]["max_attempts"])
        self.max_attempts = self.get_parameter("max_attempts").value
        self.kin = DeltaKinematics(load_geometry(), self.cfg)
        self.state = "INIT" if self.cfg["task"]["autostart"] else "IDLE"
        self.entered, self.last_attempt, self.targets_time = 0.0, 0.0, -100.0
        self.targets, self.target, self.operation, self.goal_handle = [], None, None, None
        self.attached, self.estopped = False, False
        self.attempts, self.retries, self.suppress_until = 0, 0, 0.0
        self.wait_until, self.capture_after = 0.0, 0.0
        self.motion = ActionClient(self, MoveDelta, "/delta/move_to")
        self.belt = self.create_client(SetBool, "/conveyor/set_running")
        self.vacuum = self.create_client(SetBool, "/gripper/set_enabled")
        self.estop_client = self.create_client(SetBool, "/cell/estop")
        self.events = self.create_publisher(String, "/sorting/events", 20)
        self.state_pub = self.create_publisher(String, "/sorting/state", 10)
        self.create_subscription(CapTargets, "/caps/targets", self.detections, 5)
        self.create_subscription(Bool, "/gripper/attached", lambda m: setattr(self, "attached", m.data), 10)
        self.create_subscription(Bool, "/cell/estopped", self.stop_message,
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_service(Trigger, "/sorting/stop", self.stop_service)
        self.create_service(Trigger, "/sorting/reset", self.reset_service)
        self.create_timer(0.05, self.tick)

    def now(self):
        return self.get_clock().now().nanoseconds*1e-9

    def event(self, kind, **kwargs):
        self.events.publish(String(data=json.dumps({"event": kind, "time": self.now(),
                                                   "attempt": self.attempts, **kwargs})))

    def transition(self, state):
        self.state, self.entered = state, self.now()
        self.state_pub.publish(String(data=state))
        self.get_logger().info(f"{state} (attempt {self.attempts})")

    def detections(self, message):
        if message.header.frame_id != "base_link":
            return
        self.targets_time = message.header.stamp.sec + message.header.stamp.nanosec*1e-9
        self.targets = message.caps

    def select(self):
        if self.now()-self.targets_time > 0.4 or self.targets_time < self.capture_after:
            return None
        for cap in sorted(self.targets, key=lambda c: -c.position.x):
            p = [cap.position.x, cap.position.y, cap.position.z]
            if cap.color not in self.cfg["bins"] or cap.confidence < self.cfg["perception"]["min_confidence"]:
                continue
            try:
                self.kin.inverse(p)
                self.kin.path([p[0], p[1], self.cfg["robot"]["travel_z"]], p, self.cfg["robot"]["approach_speed"])
            except (ValueError, KinematicsError):
                continue
            return cap
        return None

    def service(self, client, enabled, state):
        self.operation = {"kind": "service", "future": client.call_async(SetBool.Request(data=enabled))}
        self.transition(state)

    def move(self, target, state, slow=False):
        goal = MoveDelta.Goal(target=Point(x=float(target[0]), y=float(target[1]), z=float(target[2])),
                              speed=self.cfg["robot"]["approach_speed" if slow else "cartesian_speed"])
        self.operation = {"kind": "goal", "future": self.motion.send_goal_async(goal)}
        self.goal_handle = None
        self.transition(state)

    def outcome(self):
        if not self.operation:
            return None
        op = self.operation
        if self.now()-self.entered > self.cfg["task"]["operation_timeout"]:
            if self.goal_handle:
                self.goal_handle.cancel_goal_async()
            self.operation = None
            return False
        if not op["future"].done():
            return None
        try:
            result = op["future"].result()
            if op["kind"] == "goal":
                if not result.accepted:
                    self.operation = None
                    return False
                self.goal_handle = result
                self.operation = {"kind": "motion", "future": result.get_result_async()}
                return None
            self.operation = None
            if op["kind"] == "motion":
                result = result.result
            if not result.success:
                self.get_logger().warning(result.message)
            return result.success
        except Exception as error:
            self.get_logger().error(str(error))
            self.operation = None
            return False

    def stop_message(self, message):
        self.estopped = message.data
        if message.data and self.state != "FAULT":
            if self.goal_handle:
                self.goal_handle.cancel_goal_async()
            self.operation = None
            self.event("fault", reason="external emergency stop")
            self.transition("FAULT")

    def stop_service(self, request, response):
        self.fault("operator safe stop")
        response.success, response.message = True, "task stopped and physical stop requested"
        return response

    def reset_service(self, request, response):
        if self.estopped or self.attached:
            response.success, response.message = False, "clear /cell/reset with empty cup first"
        elif self.state not in ("FAULT", "IDLE", "DONE"):
            response.success, response.message = False, "task is already active"
        else:
            self.operation = None
            self.last_attempt = self.now()
            self.transition("INIT")
            response.success, response.message = True, "task restarting; existing cap positions retained"
        return response

    def fault(self, reason):
        if self.goal_handle:
            self.goal_handle.cancel_goal_async()
        self.operation = None
        if self.estop_client.service_is_ready():
            self.estop_client.call_async(SetBool.Request(data=True))
        self.event("fault", reason=reason)
        self.transition("FAULT")

    def failed(self, reason):
        self.event("attempt_failed", reason=reason)
        if self.attached:
            self.fault("failure with cap attached: " + reason)
        else:
            self.service(self.vacuum, False, "RECOVER_RELEASE")

    def tick(self):
        now = self.now()
        state = self.state
        if state in ("FAULT", "DONE", "IDLE"):
            return
        if state == "INIT":
            if (now > 1.0 and self.motion.server_is_ready() and self.belt.service_is_ready()
                    and self.vacuum.service_is_ready()):
                self.last_attempt = now
                self.service(self.belt, False, "BOOT_STOP")
            return
        if state == "FEED":
            if now < self.suppress_until:
                return
            if self.select():
                self.service(self.belt, False, "STOPPING")
            elif now-self.last_attempt > self.cfg["task"]["empty_feed_timeout"]:
                self.event("empty_or_exhausted")
                self.service(self.belt, False, "FINISH_STOP")
            return
        if state == "SETTLE":
            if now >= self.wait_until:
                self.capture_after = now
                self.transition("DETECT")
            return
        if state == "DETECT":
            if self.attempts >= self.max_attempts:
                self.service(self.belt, False, "FINISH_STOP")
                return
            cap = self.select()
            if cap:
                self.target = cap
                self.attempts += 1
                self.last_attempt, self.retries = now, 0
                p = cap.position
                self.event("attempt_start", color=cap.color, confidence=cap.confidence, target=[p.x, p.y, p.z])
                self.move([p.x, p.y, self.cfg["robot"]["travel_z"]], "APPROACH")
            elif now-self.entered > self.cfg["task"]["detection_timeout"]:
                self.service(self.belt, True, "FEED_START")
            return
        if state == "GRIP_RETRY":
            if now >= self.wait_until:
                self.service(self.vacuum, True, "GRIP")
            return
        if state == "VERIFY_RELEASE":
            if now >= self.wait_until:
                if self.attached:
                    self.fault("vacuum did not release")
                else:
                    self.event("released", color=self.target.color)
                    self.move(self.cfg["robot"]["home"], "RETURN")
            return
        outcome = self.outcome()
        if outcome is None:
            return
        if not outcome:
            if state == "GRIP" and self.retries < self.cfg["task"]["max_pick_retries"]:
                self.retries += 1
                self.wait_until = now+0.25
                self.transition("GRIP_RETRY")
            elif state in ("APPROACH", "DESCEND", "GRIP", "LIFT", "TRANSFER", "BIN_DESCEND"):
                self.failed(f"{state} failed or timed out")
            else:
                self.fault(f"{state} failed or timed out")
            return
        if state == "BOOT_STOP":
            self.service(self.vacuum, False, "BOOT_RELEASE")
        elif state == "BOOT_RELEASE":
            self.move(self.cfg["robot"]["home"], "HOMING")
        elif state == "HOMING":
            self.service(self.belt, True, "FEED_START")
        elif state == "FEED_START":
            self.capture_after = now
            self.transition("FEED")
        elif state == "STOPPING":
            self.wait_until = now+self.cfg["task"]["settle_time"]
            self.transition("SETTLE")
        elif state == "APPROACH":
            p = self.target.position
            self.move([p.x, p.y, p.z+0.001], "DESCEND", slow=True)
        elif state == "DESCEND":
            self.service(self.vacuum, True, "GRIP")
        elif state == "GRIP":
            self.event("vacuum_confirmed")
            p = self.target.position
            self.move([p.x, p.y, self.cfg["robot"]["travel_z"]], "LIFT", slow=True)
        elif state == "LIFT":
            if not self.attached:
                self.failed("cap not attached after lift")
            else:
                self.event("lift_confirmed")
                self.move([*self.cfg["bins"][self.target.color], self.cfg["robot"]["travel_z"]], "TRANSFER")
        elif state == "TRANSFER":
            self.move([*self.cfg["bins"][self.target.color], self.cfg["bin_geometry"]["release_z"]], "BIN_DESCEND", slow=True)
        elif state == "BIN_DESCEND":
            self.service(self.vacuum, False, "RELEASE")
        elif state == "RELEASE":
            self.wait_until = now+0.3
            self.transition("VERIFY_RELEASE")
        elif state == "RETURN":
            self.capture_after = now
            self.transition("DETECT")
        elif state == "RECOVER_RELEASE":
            # Lift vertically before returning after an unsuccessful seal.
            p = self.target.position
            self.move([p.x, p.y, self.cfg["robot"]["travel_z"]], "RECOVER_LIFT", slow=True)
        elif state == "RECOVER_LIFT":
            self.move(self.cfg["robot"]["home"], "RECOVER_HOME")
        elif state == "RECOVER_HOME":
            self.suppress_until = now+3.5
            self.service(self.belt, True, "FEED_START")
        elif state == "FINISH_STOP":
            self.event("run_complete")
            self.transition("DONE")


def main():
    rclpy.init()
    node = Coordinator()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
