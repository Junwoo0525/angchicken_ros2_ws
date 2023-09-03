import datetime
import cv2
import numpy as np
import pyrealsense2 as rs
from ultralytics import YOLO
from cal_angle_imu import cal_angle

CONFIDENCE_THRESHOLD = 0.6

GREEN = (0, 255, 0)
WHITE = (255, 255, 255)
GRAY = (102, 102, 102)

coco128 = open('/home/choi/angchicken_ros2_ws/src/pump_track_3/pump_track_3/Air_drop_box/air-drop-box.txt', 'r')
data = coco128.read()
class_list = data.split('\n')
# coco128.close()
model = YOLO('/home/choi/angchicken_ros2_ws/src/pump_track_3/pump_track_3/best.pt')

pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

# set imu pipeline
imu_pipeline = rs.pipeline()
imu_config = rs.config()
imu_config.enable_stream(rs.stream.accel, rs.format.motion_xyz32f, 63) # acceleration
imu_config.enable_stream(rs.stream.gyro, rs.format.motion_xyz32f, 200)  # gyroscope
imu_profile = imu_pipeline.start(imu_config)

pipeline.start(config)

# align
align_to = rs.stream.color
align = rs.align(align_to)

profile = pipeline.get_active_profile()
dep_sensor = profile.get_device().first_depth_sensor()
dep_scale = dep_sensor.get_depth_scale()

def accel_data(accel):
    return np.asarray([accel.x, accel.y, accel.z])

while True:
    start = datetime.datetime.now()
    fs = pipeline.wait_for_frames()
    color_f = fs.get_color_frame()
    depth_f = fs.get_depth_frame()
    align_f = align.process(fs)
    imu_frames = imu_pipeline.wait_for_frames()
    accel_frames = imu_frames.first_or_default(rs.stream.accel, rs.format.motion_xyz32f)

    if not color_f or not depth_f:
        print("Cam E")
        break

    color_i = np.asanyarray(color_f.get_data())
    detection = model(color_i)[0]

    for data in detection.boxes.data.tolist(): # data : [xmin, ymin, xmax, ymax, confidence_score, class_id]
        confidence = float(data[4])
        label = int(data[5])

        if confidence >= CONFIDENCE_THRESHOLD and class_list[label] == 'box':
            xmin, ymin, xmax, ymax = map(int, data[:4])

            xcen = (xmin + xmax) // 2
            ycen = (ymin + ymax) // 2

            # depth_val = depth_f.get_distance(xcen, ycen)
            align_dep_f = align_f.get_depth_frame()
            depth_val = align_dep_f.get_distance(xcen, ycen)

            print(f'Depth value : {depth_val}')
            dist = depth_val * dep_scale

            detect_point = (xcen-320,240-ycen)

            cv2.rectangle(color_i, (xmin, ymin), (xmax, ymax), GREEN, 2) #frame -> color_i
            # cv2.putText(color_i, class_list[label]+' '+str(round(confidence, 2)) + '%', (xmin, ymin), cv2.FONT_ITALIC, 1, WHITE, 2)
            # cv2.putText(color_i, f'{dist*100000:.2f}cm',(xmin, ymin-20), cv2.FONT_HERSHEY_SIMPLEX,  0.5, GREEN, 2) #add

            cv2.circle(color_i, (xcen, ycen), 4, GRAY)
            circle_dist = align_dep_f.get_distance(xcen, ycen)
            circle_dist = circle_dist

            accel_array = accel_data(accel_frames.as_motion_frame().get_motion_data())
            accel_array = np.round_(accel_array, 2)
            accel_array = np.float32(accel_array)

            return_value = cal_angle(accel_array[1], accel_array[2], detect_point[0], detect_point[1], circle_dist)
            real_x = return_value[0]
            real_z = return_value[1]
            yawing_angle = return_value[2]
            imu_angle = return_value[3]

            real_x = real_x*100

            print("real_dist: %f [cm]" % real_x) # 차체와 물체 사이의 거리
            print("real_z: %f [meter]" % real_z) # 중앙을 기준으로 물체가 위쪽에 있는지 아닌지 확인(값이 +면 중앙 기준 위 -면 중앙 기준 아래)
            print("yawing_angle: %f [rad]" % yawing_angle) #물체가 좌측에 있는지 아니면 우측에 있는지 확인 (좌측이면 값이 음수로 출력 우측이면 값이 양수로 출력 중앙이면 0가까이 출력)
            print("imu_angle: %f [rad]" % imu_angle) # 카메라의 현재 각도

            cv2.putText(color_i, class_list[label]+' '+str(round(confidence, 2)) + '%', (xmin, ymin), cv2.FONT_ITALIC, 1, WHITE, 2)
            cv2.putText(color_i, f'{real_x:.2f}cm',(xmin, ymin-20), cv2.FONT_HERSHEY_SIMPLEX,  0.5, GREEN, 2) #add
        # print(confidence*100) # 정확도 출력

    end = datetime.datetime.now()

    total = (end - start).total_seconds()
    print(f'Time to process 1 frame: {total * 1000:.0f} ms')

    fps = f'FPS: {1 / total:.2f}'
    cv2.putText(color_i, fps, (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

    cv2.imshow('frame', color_i)

    if cv2.waitKey(1) == ord('q'):
        break

# cap.release()
pipeline.stop()
cv2.destroyAllWindows()
