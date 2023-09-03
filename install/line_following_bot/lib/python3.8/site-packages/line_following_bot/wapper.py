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

def empty(a):
    pass

cv2.namedWindow("Control")
cv2.resizeWindow("Control", 640, 240)
cv2.createTrackbar("Warp Factor", "Control", 0, 100, empty)

try:
    while True:
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        depth_frame = frames.get_depth_frame()
        if not color_frame or not depth_frame:
            continue

        img = np.asanyarray(color_frame.get_data())
        depth_image = np.asanyarray(depth_frame.get_data())

        warp_factor = cv2.getTrackbarPos("Warp Factor", "Control") / 100.0

        # Apply the warp_factor to the depth values
        warped_depth = depth_image * warp_factor

        # Process the warped_depth further as needed (e.g., visualization, etc.)

        cv2.imshow("Color Image", img)
        cv2.imshow("Warped Depth Image", warped_depth)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    pipeline.stop()

cv2.destroyAllWindows()
