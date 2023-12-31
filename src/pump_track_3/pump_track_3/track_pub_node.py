import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class LineFollower(Node):
    def __init__(self):
        super().__init__('line_follower')
        self.bridge = CvBridge()

        self.publisher = self.create_publisher(Image, '/line_instructions', 10)
        self.timer = self.create_timer(0.1, self.process_image)

        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        self.pipeline.start(self.config)

        self.cen_x = 320
        self.roi_start_row = int(480 * 0.35)
        self.roi_end_row = int(480 * 0.5)
        self.roi_start_col = int(640 * 0.05)
        self.roi_end_col = int(640 * 0.95)

        self.yellow_color = [0, 255, 255]

    def process_image(self):
        fs = self.pipeline.wait_for_frames()
        color_f = fs.get_color_frame()
        color_a = np.asanyarray(color_f.get_data())
        color_i = cv2.GaussianBlur(color_a, (7, 7), 0)

        roi = color_i[self.roi_start_row:self.roi_end_row, self.roi_start_col:self.roi_end_col]

        dominant_color = np.mean(roi, axis=(0, 1))
        lower_bound = dominant_color - np.array([10, 70, 70])
        upper_bound = dominant_color + np.array([130, 255, 255])

        mask = cv2.inRange(roi, lower_bound, upper_bound)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        max_area = 0
        max_contour = None
        for contour in contours:
            area = cv2.contourArea(contour)
            if area > max_area:
                max_area = area
                max_contour = contour

        if max_contour is not None:
            largest_component_mask = np.zeros_like(mask)
            cv2.drawContours(largest_component_mask, [max_contour], -1, (255, 255, 0), thickness=cv2.FILLED)
            roi[largest_component_mask > 0] = self.yellow_color

        color_i[self.roi_start_row:self.roi_end_row, self.roi_start_col:self.roi_end_col] = roi
        cv2.rectangle(color_i, (self.roi_start_col, self.roi_start_row), (self.roi_end_col, self.roi_end_row), (0, 255, 0), 2)
        cv2.line(color_i, (self.cen_x, self.roi_start_row), (self.cen_x, self.roi_end_row), (0, 0, 255), 2)

        left_region = largest_component_mask[:, :self.cen_x - self.roi_start_col].copy()
        right_region = largest_component_mask[:, self.cen_x - self.roi_start_col:].copy()

        l_area = np.sum(left_region == 255)
        r_area = np.sum(right_region == 255)

        if l_area > r_area * 1.2:
            direction = 'Left'
        elif r_area > l_area * 1.2:
            direction = 'Right'
        else:
            direction = 'Straight'

        # Create an Image message to publish
        msg = self.bridge.cv2_to_imgmsg(color_i, encoding="bgr8")

        # Publish the Image message
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = LineFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
