import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from delta_interfaces.msg import PixelCap, PixelCaps
from .config import node_config
from .vision import detect_caps


class Perception(Node):
    def __init__(self):
        super().__init__("cap_perception")
        self.cfg = node_config(self)["perception"]
        self.bridge = CvBridge()
        self.publisher = self.create_publisher(PixelCaps, "/caps/pixels", 5)
        self.create_subscription(Image, "/camera/image_raw", self.image, qos_profile_sensor_data)

    def image(self, message):
        try:
            detections, rejected = detect_caps(self.bridge.imgmsg_to_cv2(message, "rgb8"), self.cfg)
        except (ValueError, TypeError) as error:
            self.get_logger().error(str(error), throttle_duration_sec=3.0)
            return
        output = PixelCaps(header=message.header, rejected=rejected)
        for d in detections:
            output.caps.append(PixelCap(u=d.u, v=d.v, radius_px=d.radius, color=d.color, confidence=d.confidence))
        self.publisher.publish(output)


def main():
    rclpy.init()
    node = Perception()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
