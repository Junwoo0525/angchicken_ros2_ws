import matplotlib.pyplot as plt
import numpy as np
import cv2

def region_of_interest(img, vertices):
    mask = np.zeros_like(img)
    cv2.fillPoly(mask, vertices, 255)
    masked_image = cv2.bitwise_and(img, mask)
    return masked_image

def draw_lines(img, lines, color=[255, 0, 0], thickness=3):
    if lines is None:
        return img
    img_copy = np.copy(img)
    for line in lines:
        for x1, y1, x2, y2 in line:
            cv2.line(img_copy, (x1, y1), (x2, y2), color, thickness)
    img_combined = cv2.addWeighted(img, 0.8, img_copy, 1.0, 0.0)
    return img_combined

# Load the image from file
image_path = '/home/choi/angchicken_ros2_ws/src/line_following_bot/line_following_bot/picture/bluetrack.jpg'
image = cv2.imread(image_path)

# Get the dimensions of the image
height, width, _ = image.shape

# Define the area of interest (lower half of the image)
roi_top = height // 2
roi_bottom = height
roi_left = width // 4  # Adjust the left and right based on your desired ROI
roi_right = 3 * width // 4

# Define vertices for the ROI polygon
region_of_interest_vertices = [
    (roi_left, roi_bottom),
    (roi_right, roi_bottom),
    (width, roi_top),
    (0, roi_top)
]

# Extract color from the area of interest
roi = image[roi_top:roi_bottom, roi_left:roi_right, :]
average_color = np.mean(roi, axis=(0, 1))

# Define a color range around the average color for color thresholding
color_threshold = 50
lower_color = average_color - color_threshold
upper_color = average_color + color_threshold

# Create a mask based on the color range
color_mask = cv2.inRange(image, lower_color, upper_color)

# Apply Canny Edge Detection
gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
cannyed_image = cv2.Canny(gray_image, 100, 200)

# Combine color mask and edge-detected mask
combined_mask = cv2.bitwise_or(cannyed_image, color_mask)

# Apply Hough Line Transform
lines = cv2.HoughLinesP(
    combined_mask,
    rho=6,
    theta=np.pi / 60,
    threshold=160,
    lines=np.array([]),
    minLineLength=40,
    maxLineGap=25
)

# Draw the detected lines on the original image
line_image = draw_lines(image, lines)

# Create subplots to display images at each step
plt.figure(figsize=(15, 10))

plt.subplot(2, 3, 1)
plt.imshow(cv2.cvtColor(image, cv2.COLOR_BGR2RGB))
plt.title("Original Image")

plt.subplot(2, 3, 2)
plt.imshow(color_mask, cmap='gray')
plt.title("Color Mask")

plt.subplot(2, 3, 3)
plt.imshow(cannyed_image, cmap='gray')
plt.title("Canny Edge Detection")

plt.subplot(2, 3, 4)
plt.imshow(combined_mask, cmap='gray')
plt.title("Combined Mask")

plt.subplot(2, 3, 5)
plt.imshow(line_image)
plt.title("Detected Lines")

plt.tight_layout()
plt.show()
