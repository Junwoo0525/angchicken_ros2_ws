import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import os
import time
from std_msgs.msg import String

class ArucoMarkerDetectionNode(Node):

    def __init__(self):
        super().__init__('aruco_marker_detection_node')
        self.subscription = self.create_subscription(
            Image,
            'usb_camera/image',  # Update with your actual image topic
            self.image_callback,
            10)
        self.bridge = CvBridge()

        self.aruco_dict = cv2.aruco.custom_dictionary(0, 5, 1)
        self.aruco_dict.bytesList = np.empty(shape=(8, 4, 4), dtype=np.uint8)

        self.alphabet = ['K', 'O', 'R', 'E', 'A', 'M', 'Y', 'v']

        self.font = cv2.FONT_HERSHEY_SIMPLEX
        self.f_scale = 1.0
        self.f_color = (0, 255, 0)
        self.f_thick = 4

        self.save_intv = 1  # Image saving interval (1 second)
        self.last_save_t = time.time()  # Time of the last saved image
        self.save_count = 0
        self.m_detected = False  # Whether a marker is detected

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # ArUco marker detection
        corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(frame, self.aruco_dict)
        m_f = frame.copy()

        # Marker detection state update
        if ids is not None:
            marker_detected = True
        else:
            marker_detected = False

        if marker_detected:
            for i, m_id in enumerate(ids):
                m_corners = corners[i][0]

                # Calculate marker width and height
                m_wid = np.linalg.norm(m_corners[0] - m_corners[1])
                m_hei = np.linalg.norm(m_corners[1] - m_corners[2])

                if m_wid > 40 and m_hei > 40:
                    m_f = cv2.aruco.drawDetectedMarkers(m_f, corners, ids)

                    fir_corner = m_corners[0]
                    sec_corner = m_corners[1]
                    fou_corner = m_corners[3]

                    if fir_corner[1] > sec_corner[1] + 30:
                        rota = "-90"
                    elif fou_corner[0] + 30 < fir_corner[0]:
                        rota = "90"
                    else:
                        rota = "0"

                    m_index = int(m_id[0])
                    if m_index == 0 and rota == "-90":
                        letter = 'Y'
                    elif 0 <= m_index < len(self.alphabet):
                        letter = self.alphabet[m_index]
                    else:
                        letter = ""

                    if letter:
                        txt_size = cv2.getTextSize(letter, self.font, self.f_scale, self.f_thick)[0]

                        if rota == "-90":
                            text_x = int(((fir_corner[0] + sec_corner[0]) / 2 - txt_size[0] / 2) + 40)
                            text_y = int(((fir_corner[1] + sec_corner[1]) / 2 + txt_size[1] / 2) - 20)
                        elif rota == "90":
                            text_x = int((fir_corner[0] + fou_corner[0]) / 2 - txt_size[0] / 2)
                            text_y = int(((fir_corner[1] + fou_corner[1]) / 2 + txt_size[1] / 2) + 20)
                        else:
                            text_x = int((fir_corner[0] + sec_corner[0]) / 2 - txt_size[0] / 2)
                            text_y = int(((fir_corner[1] + sec_corner[1]) / 2 + txt_size[1] / 2) + 20)

                        cv2.putText(m_f, letter, (text_x, text_y), self.font, self.f_scale, self.f_color, self.f_thick)

        f_resize = cv2.resize(m_f, None, fx=1.0, fy=1.0)
        cv2.imshow('frame', f_resize)

        cur_t = time.time()

        if self.m_detected == True and cur_t - self.last_save_t >= self.save_intv:
            img_p = os.path.join("/home/choi/angchicken_ros2_ws/src/pump_track_2/pump_track_2/picture/", f"{self.save_count}.jpg")
            cv2.imwrite(img_p, m_f)
            self.save_count += 1
            self.last_save_t = cur_t

        if cv2.waitKey(1) & 0xFF == ord('q'):
            cv2.destroyAllWindows()

def main(args=None):
    rclpy.init(args=args)
    node = ArucoMarkerDetectionNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
