import numpy as np
import cv2
import cv2.aruco as aruco
import pyrealsense2 as rs
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MarkerDetector(Node):


    def __init__(self):
        super().__init__('marker_detector')
        self.marker_pub = self.create_publisher(String, 'marker_topic',10)

        # 마커 생성하기
        self.aruco_dict = aruco.custom_dictionary(0, 7, 1)
        self.aruco_dict.bytesList = np.empty(shape = (8, 7, 4), dtype = np.uint8)

        # 0=K 1=O 2=R 3=E 4=A 5=M 6=Y 7=v
        mybits = np.array([[1,1,1,1,1,1,1],[1,0,1,1,1,0,1],[1,0,1,1,0,1,1],[1,0,0,0,1,1,1],[1,0,1,1,0,1,1],[1,0,1,1,1,0,1],[1,1,1,1,1,1,1]], dtype = np.uint8)
        self.aruco_dict.bytesList[0] = aruco.Dictionary_getByteListFromBits(mybits)
        mybits = np.array([[1,1,1,1,1,1,1],[1,1,0,0,0,1,1],[1,0,1,1,1,0,1],[1,0,1,1,1,0,1],[1,0,1,1,1,0,1],[1,1,0,0,0,1,1],[1,1,1,1,1,1,1]], dtype = np.uint8)
        self.aruco_dict.bytesList[1] = aruco.Dictionary_getByteListFromBits(mybits)
        mybits = np.array([[1,1,1,1,1,1,1],[1,0,0,0,0,1,1],[1,0,1,1,1,0,1],[1,0,0,0,0,1,1],[1,0,1,1,0,1,1],[1,0,1,1,1,0,1],[1,1,1,1,1,1,1]], dtype = np.uint8)
        self.aruco_dict.bytesList[2] = aruco.Dictionary_getByteListFromBits(mybits)
        mybits = np.array([[1,1,1,1,1,1,1],[1,0,0,0,0,0,1],[1,0,1,1,1,1,1],[1,0,0,0,0,1,1],[1,0,1,1,1,1,1],[1,0,0,0,0,0,1],[1,1,1,1,1,1,1]], dtype = np.uint8)
        self.aruco_dict.bytesList[3] = aruco.Dictionary_getByteListFromBits(mybits)
        mybits = np.array([[1,1,1,1,1,1,1],[1,1,1,0,1,1,1],[1,1,0,1,0,1,1],[1,0,0,0,0,0,1],[1,0,1,1,1,0,1],[1,0,1,1,1,0,1],[1,1,1,1,1,1,1]], dtype = np.uint8)
        self.aruco_dict.bytesList[4] = aruco.Dictionary_getByteListFromBits(mybits)
        mybits = np.array([[1,1,1,1,1,1,1],[1,0,1,1,1,0,1],[1,0,0,1,0,0,1],[1,0,1,0,1,0,1],[1,0,1,1,1,0,1],[1,0,1,1,1,0,1],[1,1,1,1,1,1,1]], dtype = np.uint8)
        self.aruco_dict.bytesList[5] = aruco.Dictionary_getByteListFromBits(mybits)
        mybits = np.array([[1,1,1,1,1,1,1],[1,0,1,1,1,0,1],[1,1,0,1,0,1,1],[1,1,1,0,1,1,1],[1,1,1,0,1,1,1],[1,1,1,0,1,1,1],[1,1,1,1,1,1,1]], dtype = np.uint8)
        self.aruco_dict.bytesList[6] = aruco.Dictionary_getByteListFromBits(mybits)
        mybits = np.array([[0,0,0,0,0,0,0],[0,0,1,0,1,0,0],[0,1,1,1,1,1,0],[0,1,1,1,1,1,0],[0,0,1,1,1,0,0],[0,0,0,1,0,0,0],[0,0,0,0,0,0,0]], dtype = np.uint8)
        self.aruco_dict.bytesList[7] = aruco.Dictionary_getByteListFromBits(mybits)

        for i in range(len(self.aruco_dict.bytesList)):
            img_path = "/home/choi/angchicken_ros2_ws/src/pump_track_2/pump_track_2/custom_m/" + str(i) + ".png"
            cv2.imwrite(img_path, aruco.drawMarker(self.aruco_dict, i, 128))

        self.pipeline = rs.pipeline() #add
        self.config = rs.config() #add

        pipeline_wrapper = rs.pipeline_wrapper(self.pipeline) #add
        pipeline_profile = self.config.resolve(pipeline_wrapper) #add
        device = pipeline_profile.get_device()
        device_product_line = str(device.get_info(rs.camera_info.product_line))

        found_rgb = False

        for s in device.sensors:
            if s.get_info(rs.camera_info.name) == 'RGB Camera':
                found_rgb = True
                break
        if not found_rgb:
            print("English.")
            exit(0)

        self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30) #add
        self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        self.m_detector = None #add

    def m_detect(self):
        self.pipeline.start(self.config)

        dep_sensor = self.pipeline.get_active_profile().get_device().first_depth_sensor() #add
        dep_scale = dep_sensor.get_depth_scale()
        print("Depth scale:", dep_scale)

        while True:
            fs = self.pipeline.wait_for_frames() #add
            depth_f = fs.get_depth_frame()
            color_f = fs.get_color_frame()

            if not depth_f or not color_f:
                continue

            depth_i = np.asarray(depth_f.get_data())
            color_i = np.asarray(color_f.get_data())

            hsv_f = cv2.cvtColor(color_i, cv2.COLOR_BGR2HSV)
            lower_blue = np.array([90, 50, 50])
            upper_blue = np.array([130, 255, 255])
            blue_mask = cv2.inRange(hsv_f, lower_blue, upper_blue)

            unsharp_mask = cv2.addWeighted(blue_mask, 3.5, blue_mask, -0.5, 0)
            corners, ids, rejectedImgPoints = aruco.detectMarkers(unsharp_mask, self.aruco_dict)

            if ids is not None:
                valid_markers = []

                for i in range(len(ids)):
                    m_size = np.mean(np.linalg.norm(corners[i][0] - corners[i][0][0], axis=1))

                    if m_size >= 50.0:
                        valid_markers.append(i)

                if valid_markers:
                    f_m = aruco.drawDetectedMarkers(color_i.copy(), [corners[i] for i in valid_markers], None)
                    self.m_pub() #add
                else:
                    f_m = color_i.copy()

            else:
                f_m = color_i.copy()

            cv2.imshow('frame', f_m)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    def m_pub(self):
        msg = String()
        msg.data = 'M.D'
        self.marker_pub.publish(msg)
        self.get_logger().info('Pub: %s' % msg.data)

def main(args=None):
    rclpy.init(args=args)
    m_detector = MarkerDetector()
    try:
        m_detector.m_detect()
    except KeyboardInterrupt:
        if m_detector.m_detector:
            m_detector.pipeline.stop()
            m_detector.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
