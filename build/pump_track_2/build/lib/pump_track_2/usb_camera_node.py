import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class UsbCameraNode(Node):

    def __init__(self):
        super().__init__('usb_camera_node')
        self.publisher = self.create_publisher(Image, 'usb_camera/image', 10)
        self.bridge = CvBridge()

        # Open the USB camera (you may need to adjust the device index)
        self.cap = cv2.VideoCapture(6)

        self.timer = self.create_timer(1.0 / 30, self.publish_frame)  # Adjust frame rate as needed

    def publish_frame(self):
        if self.cap.isOpened():
            ret, frame = self.cap.read()
            if ret:
                img_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
                self.publisher.publish(img_msg)

def main(args=None):
    rclpy.init(args=args)
    node = UsbCameraNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

