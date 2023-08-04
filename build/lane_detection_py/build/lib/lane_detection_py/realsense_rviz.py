import cv2
import numpy as np
import pyrealsense2 as rs
from sensor_msgs.msg import Image
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge

class RealSenseNode(Node):
    def __init__(self):
        super().__init__('realsense_node')
        self.publisher_ = self.create_publisher(Image, 'realsense_image', 10)
        self.bridge = CvBridge()

        # Configure depth and color streams
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        self.pipeline.start(config)

        self.timer = self.create_timer(1.0 / 30, self.publish_image)

    def publish_image(self):
        frames = self.pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()

        if color_frame:
            color_image = np.asanyarray(color_frame.get_data())
            ros_image = self.bridge.cv2_to_imgmsg(color_image, encoding="bgr8")
            self.publisher_.publish(ros_image)

def main(args=None):
    rclpy.init(args=args)
    realsense_node = RealSenseNode()
    rclpy.spin(realsense_node)
    realsense_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
