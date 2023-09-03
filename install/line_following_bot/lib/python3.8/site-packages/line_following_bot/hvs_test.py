import cv2
import numpy as np

def region(image):
    height, width = image.shape[:2]
    vertices = np.array([
        [(0, height-100), (0, 650), (width, 650), (width, height-100)]
    ], dtype=np.int32)
    mask = np.zeros_like(image)
    cv2.fillPoly(mask, vertices, 255)
    masked_image = cv2.bitwise_and(image, mask)
    return masked_image

# Assuming you've already loaded the image using cv2.imread()
image_path = "/home/choi/angchicken_ros2_ws/src/line_following_bot/line_following_bot/picture/bluetrack.jpg"
image = cv2.imread(image_path)


# Convert the image to the HSV color space
hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

# Define the range of blue color in HSV
lower_blue = np.array([90, 50, 50])  # Lower bound of blue in HSV
upper_blue = np.array([130, 255, 255])  # Upper bound of blue in HSV

# Create a mask to extract blue regions
blue_mask = cv2.inRange(hsv_image, lower_blue, upper_blue)

# Apply the mask to the original image
blue_extracted = cv2.bitwise_and(image, image, mask=blue_mask)

masked_image = region(blue_extracted)

# Display the original image and the extracted blue color
cv2.imshow('Original Image', masked_image)
cv2.imshow('Extracted Blue Color', blue_extracted)
cv2.waitKey(0)
cv2.destroyAllWindows()
