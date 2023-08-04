import numpy as np
import cv2
import cv2.aruco as aruco
import os
import time

aruco_dict = aruco.custom_dictionary(0, 5, 1)
aruco_dict.bytesList = np.empty(shape = (8, 4, 4), dtype = np.uint8)

mybits = np.array([[1,0,0,0,1],[1,0,0,1,0],[1,1,1,0,0],[1,0,0,1,0],[1,0,0,0,1]], dtype = np.uint8)
aruco_dict.bytesList[0] = aruco.Dictionary_getByteListFromBits(mybits)
mybits = np.array([[0,1,1,1,0],[1,0,0,0,1],[1,0,0,0,1],[1,0,0,0,1],[0,1,1,1,0]], dtype = np.uint8)
aruco_dict.bytesList[1] = aruco.Dictionary_getByteListFromBits(mybits)
mybits = np.array([[1,1,1,1,0],[1,0,0,0,1],[1,1,1,1,0],[1,0,0,1,0],[1,0,0,0,1]], dtype = np.uint8)
aruco_dict.bytesList[2] = aruco.Dictionary_getByteListFromBits(mybits)
mybits = np.array([[1,1,1,1,1],[1,0,0,0,0],[1,1,1,1,0],[1,0,0,0,0],[1,1,1,1,1]], dtype = np.uint8)
aruco_dict.bytesList[3] = aruco.Dictionary_getByteListFromBits(mybits)
mybits = np.array([[0,0,1,0,0],[0,1,0,1,0],[1,1,1,1,1],[1,0,0,0,1],[1,0,0,0,1]], dtype = np.uint8)
aruco_dict.bytesList[4] = aruco.Dictionary_getByteListFromBits(mybits)
mybits = np.array([[1,0,0,0,1],[1,1,0,1,1],[1,0,1,0,1],[1,0,0,0,1],[1,0,0,0,1]], dtype = np.uint8)
aruco_dict.bytesList[5] = aruco.Dictionary_getByteListFromBits(mybits)
mybits = np.array([[1,0,0,0,1],[0,1,0,1,0],[0,0,1,0,0],[0,0,1,0,0],[0,0,1,0,0]], dtype = np.uint8)
aruco_dict.bytesList[6] = aruco.Dictionary_getByteListFromBits(mybits)
mybits = np.array([[0,1,0,1,0],[1,1,1,1,1],[1,1,1,1,1],[0,1,1,1,0],[0,0,1,0,0]], dtype = np.uint8)
aruco_dict.bytesList[7] = aruco.Dictionary_getByteListFromBits(mybits)

alphabet = ['K', 'O', 'R', 'E', 'A', 'M', 'Y', 'v']

font = cv2.FONT_HERSHEY_SIMPLEX
f_scale = 1.0
f_color = (0, 255, 0)
f_thick = 4

for i in range(len(aruco_dict.bytesList)):
    img_path = "/home/choi/angchicken_ros2_ws/src/line_following_bot/line_following_bot/picture/" + str(i) + ".png"
    cv2.imwrite(img_path, aruco.drawMarker(aruco_dict, i, 128))

cap = cv2.VideoCapture(4)
save_intv = 1  # 사진 저장 간격 (1초)
last_save_t = time.time()  # 마지막으로 사진을 저장한 시간
save_count = 0
m_detected = False  # 마커가 검출되었는지 여부

while True:
    ret, frame = cap.read()
    if ret:
        corners, ids, rejectedImgPoints = aruco.detectMarkers(frame, aruco_dict)
        m_f = frame.copy()

        # 마커 검출 상태 업데이트
        if ids is not None:
            marker_detected = True
        else:
            marker_detected = False

        if marker_detected:
            for i, m_id in enumerate(ids):
                m_corners = corners[i][0]

                m_wid = np.linalg.norm(m_corners[0] - m_corners[1])
                m_hei = np.linalg.norm(m_corners[1] - m_corners[2])

                if m_wid > 40 and m_hei > 40:
                    m_f = aruco.drawDetectedMarkers(m_f, corners, ids)

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
                    elif 0 <= m_index < len(alphabet):
                        letter = alphabet[m_index]
                    else:
                        letter=""

                    if letter:
                        txt_size = cv2.getTextSize(letter, font, f_scale, f_thick)[0]

                        if rota == "-90":
                            text_x = int(((fir_corner[0] + sec_corner[0]) / 2 - txt_size[0] / 2) + 40)
                            text_y = int(((fir_corner[1] + sec_corner[1]) / 2 + txt_size[1] / 2) - 20)
                        elif rota == "90":
                            text_x = int((fir_corner[0] + fou_corner[0]) / 2 - txt_size[0] / 2)
                            text_y = int(((fir_corner[1] + fou_corner[1]) / 2 + txt_size[1] / 2) + 20)
                        else:
                            text_x = int((fir_corner[0] + sec_corner[0]) / 2 - txt_size[0] / 2)
                            text_y = int(((fir_corner[1] + sec_corner[1]) / 2 + txt_size[1] / 2) + 20)

                        cv2.putText(m_f, letter, (text_x, text_y), font, f_scale, f_color, f_thick)

        f_resize = cv2.resize(m_f, None, fx=1.0, fy=1.0)
        cv2.imshow('frame', f_resize)

        cur_t = time.time()

        if m_detected == True and cur_t - last_save_t >= save_intv:
            img_p = os.path.join("/home/choi/angchicken_ros2_ws/src/line_following_bot/line_following_bot/picture/", f"{save_count}.jpg")
            cv2.imwrite(img_p, m_f)
            save_count += 1
            last_save_t = cur_t

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

cap.release()
cv2.destroyAllWindows()
