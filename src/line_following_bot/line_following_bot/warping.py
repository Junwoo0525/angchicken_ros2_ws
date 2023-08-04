import cv2
import numpy as np


def getHistogram(img, display=False, minVal=0.1, region=4):
    histValues = np.sum(img, axis=0)
    maxValue = np.max(histValues)
    minValue = minVal * maxValue
    indexArray = np.where(histValues >= minValue)
    basePoint = int(np.average(indexArray))

    imgHist = None

    if display:
        imgHist = np.zeros((img.shape[0], img.shape[1], 3), np.uint8)
        for x, intensity in enumerate(histValues):
            if intensity > minValue:
                color = (255, 0, 255)  # Pink color for columns above threshold
            else:
                color = (0, 0, 255)  # Red color for columns below threshold

            end_point_y = img.shape[0] - (int(intensity) // 255 // region)
            cv2.line(imgHist, (x, img.shape[0]), (x, end_point_y), color, 1)

        cv2.circle(imgHist, (basePoint, img.shape[0]), 20, (0, 255, 255), cv2.FILLED)

    return basePoint, imgHist

def getInstructions(line_position, width, threshold=0.05):
    center = width // 2
    left_limit = int(center - threshold * center)
    right_limit = int(center + threshold * center)

    if line_position < left_limit:
        return "Turn Left"
    elif line_position > right_limit:
        return "Turn Right"
    else:
        return "Go Straight"

# Load the image file
image_path = "/home/choi/angchicken_ros2_ws/src/line_following_bot/line_following_bot/picture/bluetrack15.jpg"
image = cv2.imread(image_path)

# Resize the image
target_height = 480
target_width = 640
resized_image = cv2.resize(image, (target_width, target_height))


roi_top_left = (5, 290)
roi_bottom_right = (635, 320)
min_slope = 0.05
max_slope = np.inf
x_less_than_320 = -np.inf
x_greater_than_320 = np.inf
roi = resized_image[roi_top_left[1]:roi_bottom_right[1], roi_top_left[0]:roi_bottom_right[0]]
blurred = cv2.GaussianBlur(roi, (5, 5), 0)

# Convert the blurred 'roi' to HSV
hsv_image = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

# Define the range of blue color in HSV
lower_blue = np.array([90, 50, 50])
upper_blue = np.array([130, 255, 255])
blue_mask = cv2.inRange(hsv_image, lower_blue, upper_blue)

# Resize the blue mask to match the dimensions of resized_image
blue_mask_resized = cv2.resize(blue_mask, (target_width, target_height))

# Apply the bitwise AND operation with the resized image and blue mask
blue_extracted = cv2.bitwise_and(resized_image, resized_image, mask=blue_mask_resized)

mask = np.zeros_like(blue_mask_resized)

roi_vertices = np.array([[(roi_top_left[0], 0), roi_top_left, roi_bottom_right, (roi_bottom_right[0], 0)]], dtype=np.int32)
cv2.fillPoly(mask, roi_vertices, 255)
masked_edges = cv2.bitwise_and(blue_mask_resized, mask)


base_point, hist_image = getHistogram(masked_edges, display=True)

# Calculate the line position relative to the ROI's left edge
line_position = base_point - roi_top_left[0]

# Determine instructions based on the line position
instructions = getInstructions(line_position, roi_bottom_right[0] - roi_top_left[0])

print("Instructions:", instructions)

# Display images (optional)
cv2.imshow('Histogram', hist_image)
cv2.imshow('Video', resized_image)
cv2.imshow('Video2', masked_edges)
cv2.waitKey(0)
cv2.destroyAllWindows()
