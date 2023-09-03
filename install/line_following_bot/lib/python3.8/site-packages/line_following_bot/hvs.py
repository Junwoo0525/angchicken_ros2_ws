import cv2
import numpy as np
import pyrealsense2 as rs

frameWidth = 640
frameHeight = 480

# Configure depth and color streams
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, frameWidth, frameHeight, rs.format.z16, 30)
config.enable_stream(rs.stream.color, frameWidth, frameHeight, rs.format.bgr8, 30)

# Start streaming
pipeline.start(config)

try:
    while True:
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        if not color_frame:
            continue

        img = np.asanyarray(color_frame.get_data())
        imgHsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        # Define HSV range values
        lower = np.array([121, 0, 94])  # HUE min, SAT min, VALUE min
        upper = np.array([179, 255, 255])  # HUE max, SAT max, VALUE max

        mask = cv2.inRange(imgHsv, lower, upper)
        result = cv2.bitwise_and(img, img, mask=mask)

        mask = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
        hStack = np.hstack([img, mask, result])
        cv2.imshow('Horizontal Stacking', hStack)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    pipeline.stop()

cv2.destroyAllWindows()
