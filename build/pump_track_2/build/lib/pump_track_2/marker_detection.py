import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import os
import time
import cv2.aruco as aruco

class MarkerDetectionNode(Node):
    def __init__(self):
        super().__init__('marker_detection_node')
        self.image_pub = self.create_publisher(Image, 'marker_topic', 10)
        self.bridge = CvBridge()

        self.aruco_dict = aruco.custom_dictionary(0, 5, 1)
        self.aruco_dict.bytesList = np.empty(shape=(8, 4, 4), dtype=np.uint8)
        self.setup_aruco_dictionary()

        self.alphabet = ['K', 'O', 'R', 'E', 'A', 'M', 'Y', 'v']

        self.font = cv2.FONT_HERSHEY_SIMPLEX
        self.f_scale = 1.0
        self.f_color = (0, 255, 0)
        self.f_thick = 4

        self.cap = cv2.VideoCapture(0)  # Change the camera index as needed
        self.save_interval = 1  # Image saving interval in seconds
        self.last_save_time = time.time()
        self.save_count = 0
        self.marker_detected = False

    def setup_aruco_dictionary(self):
        mybits_list = [
            [[1,0,0,0,1],[1,0,0,1,0],[1,1,1,0,0],[1,0,0,1,0],[1,0,0,0,1]],
            [[0,1,1,1,0],[1,0,0,0,1],[1,0,0,0,1],[1,0,0,0,1],[0,1,1,1,0]],
            [[1,1,1,1,0],[1,0,0,0,1],[1,1,1,1,0],[1,0,0,1,0],[1,0,0,0,1]],
            [[1,1,1,1,1],[1,0,0,0,0],[1,1,1,1,0],[1,0,0,0,0],[1,1,1,1,1]],
            [[0,0,1,0,0],[0,1,0,1,0],[1,1,1,1,1],[1,0,0,0,1],[1,0,0,0,1]],
            [[1,0,0,0,1],[1,1,0,1,1],[1,0,1,0,1],[1,0,0,0,1],[1,0,0,0,1]],
            [[1,0,0,0,1],[0,1,0,1,0],[0,0,1,0,0],[0,0,1,0,0],[0,0,1,0,0]],
            [[0,1,0,1,0],[1,1,1,1,1],[1,1,1,1,1],[0,1,1,1,0],[0,0,1,0,0]]
        ]

        for i, mybits in enumerate(mybits_list):
            self.aruco_dict.bytesList[i] = aruco.Dictionary_getByteListFromBits(np.array(mybits, dtype=np.uint8))

    def run(self):
        while rclpy.ok():
            ret, frame = self.cap.read()
            if ret:
                corners, ids, _ = aruco.detectMarkers(frame, self.aruco_dict)
                marker_frame = frame.copy()

                if ids is not None:
                    self.marker_detected = True
                else:
                    self.marker_detected = False

                if self.marker_detected:
                    for i, m_id in enumerate(ids):
                        m_corners = corners[i][0]

                        m_wid = np.linalg.norm(m_corners[0] - m_corners[1])
                        m_hei = np.linalg.norm(m_corners[1] - m_corners[2])

                        if m_wid > 40 and m_hei > 40:
                            marker_frame = aruco.drawDetectedMarkers(marker_frame, corners, ids)

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

                                cv2.putText(marker_frame, letter, (text_x, text_y), self.font, self.f_scale, self.f_color, self.f_thick)

                cv2.imshow('frame', marker_frame)
                cur_time = time.time()

                if self.marker_detected and cur_time - self.last_save_time >= self.save_interval:
                    img_path = os.path.join("/home/choi/angchicken_ros2_ws/src/line_following_bot/line_following_bot/picture/", f"{self.save_count}.jpg")
                    cv2.imwrite(img_path, marker_frame)
                    self.save_count += 1
                    self.last_save_time = cur_time

                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

        self.cap.release()
        cv2.destroyAllWindows()

def main(args=None):
    rclpy.init(args=args)
    node = MarkerDetectionNode()
    node.run()
    rclpy.shutdown()

if __name__ == '__main__':
    main()