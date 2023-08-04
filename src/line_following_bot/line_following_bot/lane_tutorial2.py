import cv2
import matplotlib.pyplot as plt
import numpy as np

# Load the image from the specified path
image_path = "/home/choi/angchicken_ros2_ws/src/line_following_bot/line_following_bot/picture/bluetrack.jpg"
image = cv2.imread(image_path)

# Convert the image to grayscale
def grey(image):
    return cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

# Apply Gaussian Blur to reduce noise and smoothen the image
def gauss(image):
    return cv2.GaussianBlur(image, (11, 11), 0)


def median(image):
    return cv2.medianBlur(image,9)

# Detect edges using Canny edge detection
def canny(image):
    edges = cv2.Canny(image, 50, 100)
    return edges


# Define a region of interest and mask the image
def region(image):
    height, width = image.shape
    vertices = np.array([
        [(0, 800), (0, 650), (width, 650),(width,800)]
    ])
    mask = np.zeros_like(image)
    mask = cv2.fillPoly(mask, vertices, 255)
    mask = cv2.bitwise_and(image, mask)
    return mask

"""
# Display detected lines on a black image
def display_lines(image, lines):
    lines_image = np.zeros_like(image)
    if lines is not None:
        for line in lines:
            x1, y1, x2, y2 = line
            cv2.line(lines_image, (x1, y1), (x2, y2), (255, 0, 0), 10)
    return lines_image

# Calculate average lines from detected Hough lines
def average(image, lines):
    left = []
    right = []

    if lines is not None:
        for line in lines:
            x1, y1, x2, y2 = line.reshape(4)
            parameters = np.polyfit((x1, x2), (y1, y2), 1)
            slope = parameters[0]
            y_int = parameters[1]
            if slope < 0:
                left.append((slope, y_int))
            else:
                right.append((slope, y_int))

    right_avg = np.average(right, axis=0)
    left_avg = np.average(left, axis=0)
    left_line = make_points(image, left_avg)
    right_line = make_points(image, right_avg)
    return np.array([left_line, right_line])

# Calculate endpoints for the lines based on slope and y-intercept
def make_points(image, average):
    slope, y_int = average
    y1 = image.shape[0]
    y2 = int(y1 * (3/5))
    x1 = int((y1 - y_int) // slope)
    x2 = int((y2 - y_int) // slope)
    return np.array([x1, y1, x2, y2])
...
"""

# Apply image processing steps
grey_image = grey(image)
blurred_image = gauss(grey_image)
blur_m = median(blurred_image)
edges = canny(blurred_image)
edges_m = canny(blur_m)
isolated_region = region(edges)
regi = region(grey_image)
"""
# Detect lines using Hough Line Transform
lines = cv2.HoughLinesP(isolated_region, 2, np.pi/180, 100, np.array([]), minLineLength=40, maxLineGap=5)

# Calculate averaged lines and display them on a black image
averaged_lines = average(image, lines)
lane_lines_image = display_lines(image, averaged_lines)

# Combine lane lines image with the original image
lanes = cv2.addWeighted(image, 0.8, lane_lines_image, 1, 1)

"""

plt.figure(figsize=(15, 10))

plt.subplot(2, 3, 1)
plt.imshow(cv2.cvtColor(image, cv2.COLOR_BGR2RGB))
plt.title("Original Image")

plt.subplot(2, 3, 2)
plt.imshow(isolated_region, cmap='gray')
plt.title("isolated_region")

plt.subplot(2, 3, 3)
plt.imshow(regi, cmap='gray')
plt.title("regi")

plt.subplot(2, 3, 4)
plt.imshow(blur_m, cmap='gray')
plt.title("madian")

plt.subplot(2, 3, 5)
plt.imshow(edges, cmap='gray')
plt.title("gaussian")

plt.subplot(2, 3, 6)
plt.imshow(edges_m, cmap='gray')
plt.title("madian")

plt.tight_layout()
plt.show()
