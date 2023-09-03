import rclpy
from rclpy.node import Node
import cv2
import numpy as np
import pyrealsense2 as rs
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class ImageAcquisitionNode(Node):
    def __init__(self):
        super().__init__('image_acquisition_node')
        self.publisher = self.create_publisher(Image, 'camera/image', 10)
        self.bridge = CvBridge()
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        self.pipeline.start(self.config)

        self.timer = self.create_timer(1.0 / 30, self.publish_image)

    def publish_image(self):
        frames = self.pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()

        if color_frame:
            color_image = np.asanyarray(color_frame.get_data())
            ros_image = self.bridge.cv2_to_imgmsg(color_image, encoding="bgr8")
            self.publisher.publish(ros_image)

def main(args=None):
    rclpy.init(args=args)
    image_acquisition_node = ImageAcquisitionNode()
    rclpy.spin(image_acquisition_node)
    image_acquisition_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
