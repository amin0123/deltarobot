"""Independent evaluator. Ground truth is used only here, never to command picks."""
import json
from pathlib import Path
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String, Bool
from .config import node_config


class Evaluation(Node):
    def __init__(self):
        super().__init__("sorting_evaluation")
        self.cfg = node_config(self)
        self.declare_parameter("report_dir", "artifacts/latest")
        self.out = Path(self.get_parameter("report_dir").value)
        self.out.mkdir(parents=True, exist_ok=True)
        self.records, self.truth, self.end_time = {}, None, None
        self.events_file = (self.out / "events.jsonl").open("w", buffering=1)
        self.done = self.create_publisher(Bool, "/evaluation/done", 5)
        self.create_subscription(String, "/evaluation/ground_truth", self.ground_truth, 10)
        self.create_subscription(String, "/sorting/events", self.event, 20)
        self.create_timer(0.2, self.tick)

    def event(self, message):
        event = json.loads(message.data)
        self.events_file.write(json.dumps(event)+"\n")
        key, kind = event["attempt"], event["event"]
        if kind == "attempt_start":
            self.records[key] = {"attempt": key, "color": event["color"], "target": event["target"],
                                 "start_time": event["time"], "cap_id": None, "picked": False,
                                 "correct_placement": False, "status": "active"}
        elif kind == "released" and key in self.records:
            self.records[key]["release_time"] = event["time"]
            self.records[key]["status"] = "settling"
        elif kind == "attempt_failed" and key in self.records:
            self.records[key].update(status="failed", reason=event["reason"])
        elif kind in ("run_complete", "empty_or_exhausted"):
            self.end_time = event["time"]
        elif kind == "fault":
            if key in self.records:
                self.records[key].update(status="fault", reason=event["reason"])
            self.end_time = event["time"]
        self.write_report()

    def ground_truth(self, message):
        self.truth = json.loads(message.data)
        active = [r for r in self.records.values() if r["status"] == "active"]
        held = self.truth["held_id"]
        if active and held is not None:
            record = active[-1]
            record["cap_id"] = held
            cap = self.truth["caps"][held]
            if cap["position"][2] > self.cfg["conveyor"]["top"]+0.035:
                record["picked"] = True

    def tick(self):
        if not self.truth:
            return
        now = self.get_clock().now().nanoseconds*1e-9
        for record in self.records.values():
            if record["status"] != "settling" or now-record["release_time"] < 1.2:
                continue
            cap_id = record["cap_id"]
            if cap_id is None:
                record.update(status="failed", reason="no physical attachment observed")
                continue
            cap = self.truth["caps"][cap_id]
            position = np.array(cap["position"])
            half = self.cfg["bin_geometry"]["inner_width"]/2-self.cfg["caps"]["radius"]+0.002
            destination = None
            for color, centre in self.cfg["bins"].items():
                if (np.max(abs(position[:2]-centre)) <= half and
                        self.cfg["bin_geometry"]["floor"] <= position[2] <= self.cfg["bin_geometry"]["rim"]):
                    destination = color
            correct = record["picked"] and destination == cap["color"] == record["color"]
            record.update(status="placed" if correct else "misplaced", actual_color=cap["color"],
                          destination=destination, final_position=position.tolist(), correct_placement=correct)
            self.get_logger().info(f"Attempt {record['attempt']}: picked={record['picked']} correct={correct} bin={destination}")
            self.write_report()
        if self.end_time is not None and now-self.end_time > 2.0:
            self.write_report(finished=True)
            self.done.publish(Bool(data=True))

    def write_report(self, finished=False):
        attempts = list(self.records.values())
        report = {"finished": finished, "attempted": len(attempts),
                  "successful_picks": sum(r["picked"] for r in attempts),
                  "correct_placements": sum(r["correct_placement"] for r in attempts),
                  "failures": sum(r["status"] in ("failed", "misplaced", "fault") for r in attempts),
                  "attempts": attempts}
        if self.truth:
            report["sim_time"] = self.truth["time"]
            report["actual_gripper_requests"] = self.truth["grasp_calls"]
        temp = self.out / "results.json.tmp"
        temp.write_text(json.dumps(report, indent=2))
        temp.replace(self.out / "results.json")


def main():
    rclpy.init()
    node = Evaluation()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.events_file.close()
        node.destroy_node()
        rclpy.try_shutdown()
