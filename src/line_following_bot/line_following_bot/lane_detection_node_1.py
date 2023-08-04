import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

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

    def image_callback(self, msg):
        try:
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Extract color from the center part of the image
            center_x = image.shape[1] // 2
            center_y = image.shape[0] // 2
            roi_size = 50  # Adjust this to the size of the center region
            center_roi = image[center_y - roi_size // 2:center_y + roi_size // 2,
                                center_x - roi_size // 2:center_x + roi_size // 2]
            center_color = np.mean(center_roi, axis=(0, 1))  # Calculate average color

            # Define color range for lane detection (adjust these values)
            lower_lane_color = center_color - np.array([40, 40, 40])
            upper_lane_color = center_color + np.array([40, 40, 40])

            # Apply color thresholding to create a mask for lane pixels
            lane_mask = cv2.inRange(image, lower_lane_color, upper_lane_color)

            # Perform additional image processing (e.g., edge detection)
            gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            edges = cv2.Canny(gray_image, threshold1=50, threshold2=150)

            # Combine lane mask with edge detection results
            combined_mask = cv2.bitwise_or(lane_mask, edges)

            # Convert the combined mask to a BGR image
            annotated_image = cv2.cvtColor(combined_mask, cv2.COLOR_GRAY2BGR)

            # Convert the annotated image to a ROS image message
            ros_annotated_image = self.bridge.cv2_to_imgmsg(annotated_image, encoding="bgr8")

            # Publish the annotated image
            self.publisher.publish(ros_annotated_image)

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
