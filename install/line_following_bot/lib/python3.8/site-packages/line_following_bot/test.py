import cv2
import numpy as np

# 1. Load the foggy image captured by the RealSense IR camera

image_path = "/home/choi/angchicken_ros2_ws/src/line_following_bot/line_following_bot/picture/bluetrack4.jpg"
foggy_image = cv2.imread(image_path)

target_height = 480
target_width = 640
resized_image = cv2.resize(foggy_image, (target_width, target_height))
roi_top_left = (0, 200)
roi_bottom_right = (635, 320)
min_slope = 0.05
max_slope = np.inf
x_less_than_320 = -np.inf
x_greater_than_320 = np.inf
roi = resized_image[roi_top_left[1]:roi_bottom_right[1], roi_top_left[0]:roi_bottom_right[0]]


# 2. Preprocess the foggy image to enhance visibility
# You can experiment with different preprocessing techniques. Here's an example:
gray_image = cv2.cvtColor(resized_image, cv2.COLOR_BGR2GRAY)
blurred_image = cv2.GaussianBlur(gray_image, (5, 5), 0)

# 3. Apply Canny edge detection to find edges in the image
edges = cv2.Canny(blurred_image, threshold1=30, threshold2=100)

# 4. Perform Hough Line Transform to detect lines
lines = cv2.HoughLinesP(edges, 1, np.pi / 180, threshold=50, minLineLength=50, maxLineGap=10)

# 5. Draw detected lines on the original image
if lines is not None:
    for line in lines:
        x1, y1, x2, y2 = line[0]
        cv2.line(foggy_image, (x1, y1), (x2, y2), (0, 255, 0), 2)

# 6. Display the result with detected lines
cv2.imshow('Line Detection in Fog', edges)
cv2.waitKey(0)
cv2.destroyAllWindows()
