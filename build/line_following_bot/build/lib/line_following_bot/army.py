import cv2
from ultralytics import YOLO
import time
import os


# Get the directory of the current script
script_dir = os.path.dirname(os.path.abspath(__file__))

# Construct the full path to the 'army_l_3.pt' file
file_path = os.path.join(script_dir, 'army', 'army_l_3.pt')

GREEN = (0, 255, 0)
WHITE = (255, 255, 255)

model = YOLO('/home/choi/angchicken_ros2_ws/src/line_following_bot/line_following_bot/army/army_b_3.pt')
class_list = open('/home/choi/angchicken_ros2_ws/src/line_following_bot/line_following_bot/army/army.txt', 'r').read().split('\n')

cap = cv2.VideoCapture(4)
save_intv = 1
last_save_t = time.time()
save_count = 0
m_detected = False

while True:
    ret, frame = cap.read()
    if not ret:
        break

    detection = model(frame)[0]
    m_detected = False

    for data in detection.boxes.data.tolist():
        xmin, ymin, xmax, ymax = map(int, data[:4])
        label = int(data[5])
        confidence = float(data[4])

        if class_list[label] in ["army", "enemy"]:
            if class_list[label] == "army":
                cv2.rectangle(frame, (xmin, ymin), (xmax, ymax), (0, 255, 0), 2)
            else:
                cv2.rectangle(frame, (xmin, ymin), (xmax, ymax), (0, 0, 255), 2)
            cv2.putText(frame, class_list[label]+' '+str(round(confidence, 2)) + '%', (xmin, ymin), cv2.FONT_ITALIC, 1, WHITE, 2)
            m_detected = True
            cur_t = time.time()

            if m_detected == True and cur_t - last_save_t >= save_intv:
                img_p = os.path.join("/home/choi/angchicken_ros2_ws/src/line_following_bot/line_following_bot/picture", f"{save_count}.jpg")
                cv2.imwrite(img_p, frame)
                save_count += 1
                last_save_t = cur_t

    cv2.imshow('IFF', frame)
    print ({m_detected})

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
