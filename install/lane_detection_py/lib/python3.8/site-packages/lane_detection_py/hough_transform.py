import cv2
import numpy as np
import pyrealsense2 as rs

# Initialize RealSense pipeline
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
pipeline.start(config)

while True:
    # Capture frames
    frames = pipeline.wait_for_frames()
    color_frame = frames.get_color_frame()

    if not color_frame:
        continue

    color_image = np.asanyarray(color_frame.get_data())

    # Convert color image to HSV
    hsv_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2HSV)

    # Define color range for lane
    lower_lane_color = np.array([121, 0, 100])  # Modify as needed
    upper_lane_color = np.array([179, 255, 255])  # Modify as needed

    # Create a mask to identify lane color
    lane_mask = cv2.inRange(hsv_image, lower_lane_color, upper_lane_color)

    # Find lane contour
    contours, _ = cv2.findContours(lane_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if len(contours) > 0:
        lane_contour = max(contours, key=cv2.contourArea)
        cv2.drawContours(color_image, [lane_contour], -1, (0, 255, 0), 2)

        # Find middle point of lane contour
        M = cv2.moments(lane_contour)
        if M["m00"] != 0:
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])
            cv2.circle(color_image, (cX, cY), 5, (255, 0, 0), -1)

            # Apply edge detection around the middle point
            gray_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)
            edges = cv2.Canny(gray_image, threshold1=100, threshold2=200)
            cv2.imshow("Edges", edges)

    cv2.imshow("Color Image", color_image)

    key = cv2.waitKey(1)
    if key == 27:
        break

cv2.destroyAllWindows()
pipeline.stop()
