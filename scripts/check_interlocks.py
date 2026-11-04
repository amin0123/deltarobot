#!/usr/bin/env python3
"""Live ROS/PhysX negative tests; run with the empty scene and task autostart=false."""
import argparse
import json
from pathlib import Path
import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, DurabilityPolicy
from geometry_msgs.msg import Point, PoseStamped
from std_msgs.msg import Bool
from std_srvs.srv import SetBool, Trigger
from delta_interfaces.action import MoveDelta


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", default="artifacts/interlocks/checks.json")
    args = parser.parse_args()
    rclpy.init()
    n = Node("interlock_test")
    pose, estopped = [None], [False]
    n.create_subscription(PoseStamped, "/delta/tool_pose", lambda m: pose.__setitem__(0, m), 10)
    n.create_subscription(Bool, "/cell/estopped", lambda m: estopped.__setitem__(0, m.data),
                          QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    action = ActionClient(n, MoveDelta, "/delta/move_to")
    stop = n.create_client(SetBool, "/cell/estop")
    reset = n.create_client(Trigger, "/cell/reset")
    vacuum = n.create_client(SetBool, "/gripper/set_enabled")
    belt = n.create_client(SetBool, "/conveyor/set_running")
    end = n.create_publisher(Bool, "/evaluation/done", 5)
    deadline = time.monotonic()+120
    while pose[0] is None or not action.server_is_ready() or not stop.service_is_ready():
        rclpy.spin_once(n, timeout_sec=0.1)
        assert time.monotonic() < deadline, "simulator and ROS startup timed out"

    def result(future, timeout=40):
        rclpy.spin_until_future_complete(n, future, timeout_sec=timeout)
        assert future.done(), "ROS operation timed out"
        return future.result()

    def pump(seconds=.3):
        deadline = time.monotonic()+seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(n, timeout_sec=.03)

    checks = {}
    goal = MoveDelta.Goal(target=Point(x=1.0, y=0.0, z=-.35), speed=.1)
    unreachable = result(action.send_goal_async(goal))
    checks["unreachable_action_rejected"] = not unreachable.accepted
    assert checks["unreachable_action_rejected"]
    goal.target.x = float("nan")
    checks["nan_action_rejected"] = not result(action.send_goal_async(goal)).accepted
    assert checks["nan_action_rejected"]
    checks["empty_grasp_rejected"] = not result(vacuum.call_async(SetBool.Request(data=True))).success
    assert checks["empty_grasp_rejected"]
    assert result(belt.call_async(SetBool.Request(data=True))).success
    assert result(stop.call_async(SetBool.Request(data=True))).success
    pump()
    assert estopped[0]
    checks["conveyor_start_rejected_while_stopped"] = not result(belt.call_async(SetBool.Request(data=True))).success
    goal = MoveDelta.Goal(target=Point(x=0., y=-.125, z=-.3), speed=.12)
    checks["motion_rejected_while_stopped"] = not result(action.send_goal_async(goal)).accepted
    assert checks["conveyor_start_rejected_while_stopped"] and checks["motion_rejected_while_stopped"]
    checks["reset_clears_stop"] = result(reset.call_async(Trigger.Request())).success
    pump()
    assert checks["reset_clears_stop"] and not estopped[0]
    handle = result(action.send_goal_async(goal))
    assert handle.accepted
    moved = result(handle.get_result_async())
    assert moved.result.success, moved.result.message
    pump()
    p = pose[0].pose.position
    error = float(np.linalg.norm(np.array([p.x, p.y, p.z])-[0, -.125, -.3]))
    checks["controlled_joint_motion_after_reset"] = moved.result.success
    checks["measured_tool_error_m"] = error
    assert error < .002
    checks["passed"] = True
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(checks, indent=2))
    print(json.dumps(checks, indent=2), flush=True)
    for _ in range(5):
        end.publish(Bool(data=True))
        pump(.1)
    n.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
