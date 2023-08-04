import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import matplotlib.pyplot as plt

class LaneDetectionNode(Node):
    def __init__(self):
        super().__init__('lane_detection_node')
        self.subscription = self.create_subscription(
            Image,
            'camera/image',
            self.image_callback,
            10
        )
        self.publisher = self.create_publisher(Image, 'lane_detection_result', 10)
        self.bridge = CvBridge()

    def region_of_interest(self, img, vertices):
        mask = np.zeros_like(img[:, :, 0])
        cv2.fillPoly(mask, vertices, 255)
        masked_image = cv2.bitwise_and(img, img, mask=mask)
        return masked_image

    def image_callback(self, msg):
        try:
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Get the dimensions of the image
            height, width, _ = image.shape
            point1 = height - 100
            point2 = (height//2) + 100

            # Define the center region to extract color
            region_of_interest_vertices = [
                (100, point1),
                (100, point2),
                (width-100, point2),
                (width-100, point1),
            ]

            cropped_image = self.region_of_interest(image, np.array([region_of_interest_vertices], np.int32))

            # Convert the annotated image to a ROS image message
            ros_cropped_image = self.bridge.cv2_to_imgmsg(cropped_image, encoding="bgr8")

            # Publish the cropped image
            self.publisher.publish(ros_cropped_image)

        except Exception as e:
            self.get_logger().info(f"Error processing image: {str(e)}")

def main(args=None):
    rclpy.init(args=args)
    lane_detection_node = LaneDetectionNode()
    rclpy.spin(lane_detection_node)
    lane_detection_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
