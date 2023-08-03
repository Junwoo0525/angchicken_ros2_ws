import cv2
import numpy as np
import pyrealsense2 as rs

# RealSense 파이프라인 생성
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 360, rs.format.z16, 30) #여기 부분 정수값으로 바꿔야함
config.enable_stream(rs.stream.color, 640, 360, rs.format.bgr8, 30)
pipeline.start(config)

# cap = cv2.VideoCapture(1)
# cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640) #여기 부분 정수값으로 바꿔야함
# cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 360)

# if cap.isOpened() == False:
#     print("카메라 못 찾음.")
#     exit()

while True:
    frames = pipeline.wait_for_frames()
    depth_frame = frames.get_depth_frame()
    color_frame = frames.get_color_frame()

    if not depth_frame or not color_frame:
        print("NONO depth frame.")
        break
    
    # ret, img_frame = cap.read()
    # if ret == False:
    #     print("캡처 불가.")
    #     break
    
    depth_image = np.asanyarray(depth_frame.get_data())
    color_image = np.asanyarray(color_frame.get_data())
    
    depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
    
    # if len(depth_image.shape) == 2: #gray 스케일이면 컬러로 변환해주라함 # 안 씀
    #     depth_image = cv2.cvtColor(depth_image, cv2.COLOR_GRAY2RGB)
    
    img_frame = color_image
    
    img_hsv = cv2.cvtColor(img_frame, cv2.COLOR_BGR2HSV)

    lower_yellow = (17, 82, 71)
    upper_yellow = (30, 255, 194)
    img_mask = cv2.inRange(img_hsv, lower_yellow, upper_yellow)

    # 모폴로지를 쓰는 이유? 노이즈 제거
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (11, 11))
    img_mask = cv2.morphologyEx(img_mask, cv2.MORPH_CLOSE, kernel)

    # bitwise
    img_result = cv2.bitwise_and(img_frame, img_frame, mask=img_mask)

    # 각 특성 분석. 이건 검색해봐야함
    nlabels, labels, stats, centroids = cv2.connectedComponentsWithStats(img_mask)

    for i in range(nlabels):
        # 배경 제외, 첫 번째 물체부터 분석한다고 함 뭔 소리래 좀 더 검색하자
        if i < 1:
            continue

        # 면적, 중심 좌표, 왼쪽 상단 좌표, 너비, 높이 정보 얻기위해 씀
        area = stats[i, cv2.CC_STAT_AREA]
        center_x = int(centroids[i, 0])
        center_y = int(centroids[i, 1])
        left = stats[i, cv2.CC_STAT_LEFT]
        top = stats[i, cv2.CC_STAT_TOP]
        width = stats[i, cv2.CC_STAT_WIDTH]
        height = stats[i, cv2.CC_STAT_HEIGHT]

        # 면적 10000 이상이면
        if area > 10000:
            cv2.rectangle(img_frame, (left, top), (left + width, top + height), (0, 0, 255), 3)
            cv2.circle(img_frame, (center_x, center_y), 5, (255, 0, 0), 3)

            # 물체 중심 좌표에서의 거리를 측정 왜 측정하지?
            # frames = pipeline.wait_for_frames()
            # depth_frame = frames.get_depth_frame()
            dist = depth_frame.get_distance(center_x, center_y)
            print(dist)

    cv2.imshow("D", depth_colormap)
    cv2.imshow("CD", img_result)
    cv2.imshow("C",img_frame)

    key = cv2.waitKey(1)
    if key == 27:
        break

#cap.release()
cv2.destroyAllWindows()