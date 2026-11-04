import numpy as np
from scipy.spatial.transform import Rotation
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import qos_profile_sensor_data
from tf2_ros import Buffer, TransformListener, TransformException
from sensor_msgs.msg import CameraInfo
from geometry_msgs.msg import Point
from delta_interfaces.msg import PixelCaps, CapTarget, CapTargets
from .config import node_config, load_geometry
from .kinematics import DeltaKinematics, KinematicsError
from .vision import pixel_to_plane


class Projection(Node):
    def __init__(self):
        super().__init__("cap_projection")
        self.cfg = node_config(self)
        self.kin = DeltaKinematics(load_geometry(), self.cfg)
        self.info = None
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.create_subscription(CameraInfo, "/camera/camera_info", self.calibration, qos_profile_sensor_data)
        self.create_subscription(PixelCaps, "/caps/pixels", self.project, 5)
        self.pub = self.create_publisher(CapTargets, "/caps/targets", 5)

    def calibration(self, message):
        self.info = message

    def project(self, message):
        if self.info is None or self.info.header.frame_id != message.header.frame_id:
            return
        if any(abs(v) > 1e-9 for v in self.info.d):
            self.get_logger().error("Projection expects rectified images (zero distortion)", throttle_duration_sec=5)
            return
        try:
            tf = self.buffer.lookup_transform("base_link", message.header.frame_id, Time.from_msg(message.header.stamp))
        except TransformException:
            return
        t, q = tf.transform.translation, tf.transform.rotation
        rotation = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
        plane_z = self.cfg["conveyor"]["top"] + self.cfg["caps"]["height"] - self.cfg["robot"]["base_height"]
        out = CapTargets(header=message.header, rejected=message.rejected)
        out.header.frame_id = "base_link"
        for cap in message.caps:
            try:
                p = pixel_to_plane(cap.u, cap.v, self.info.k, [t.x, t.y, t.z], rotation, plane_z)
                if not (self.cfg["conveyor"]["pick_x"][0] <= p[0] <= self.cfg["conveyor"]["pick_x"][1]
                        and self.cfg["conveyor"]["pick_y"][0] <= p[1] <= self.cfg["conveyor"]["pick_y"][1]):
                    out.rejected += 1
                    continue
                self.kin.inverse(p)
                # Physical diameter is also an image-only plausibility check.
                edge = pixel_to_plane(cap.u+cap.radius_px, cap.v, self.info.k, [t.x, t.y, t.z], rotation, plane_z)
                if not 0.010 < np.linalg.norm(edge-p) < 0.019:
                    out.rejected += 1
                    continue
                out.caps.append(CapTarget(position=Point(x=p[0], y=p[1], z=p[2]),
                                          color=cap.color, confidence=cap.confidence))
            except (ValueError, KinematicsError):
                out.rejected += 1
        self.pub.publish(out)


def main():
    rclpy.init()
    node = Projection()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
