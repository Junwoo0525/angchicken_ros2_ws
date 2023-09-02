import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO
import time

class ObjectDetectionNode(Node):
    def __init__(self):
        super().__init__('object_detection_node')
        self.bridge = CvBridge()
        self.model = YOLO('/home/choi/angchicken_ros2_ws/src/pump_track/army/army_b_3.pt')  # Update with your model path
        self.class_list = open('/home/choi/angchicken_ros2_ws/src/pump_track/army/army.txt', 'r').read().split('\n')  # Update with your class list path
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',  # Update with your camera topic
            self.image_callback,
            10
        )
        self.subscription

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        detection = self.model(frame)[0]
        m_detected = False

        for data in detection.boxes.data.tolist():
            xmin, ymin, xmax, ymax = map(int, data[:4])
            label = int(data[5])
            confidence = float(data[4])

            if self.class_list[label] in ["army", "enemy"]:
                if self.class_list[label] == "army":
                    cv2.rectangle(frame, (xmin, ymin), (xmax, ymax), (0, 255, 0), 2)
                else:
                    cv2.rectangle(frame, (xmin, ymin), (xmax, ymax), (0, 0, 255), 2)
                cv2.putText(frame, self.class_list[label]+' '+str(round(confidence, 2)) + '%', (xmin, ymin), cv2.FONT_ITALIC, 1, (255, 255, 255), 2)
                m_detected = True

        if m_detected:
            # Publish the frame with bounding boxes as a new image message
            detected_image_msg = self.bridge.cv2_to_imgmsg(frame, 'bgr8')
            # Publish the image message to a new topic or use the same camera topic

def main(args=None):
    rclpy.init(args=args)
    object_detection_node = ObjectDetectionNode()
    rclpy.spin(object_detection_node)
    object_detection_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
