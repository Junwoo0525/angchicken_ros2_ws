import cv2
import numpy as np

def getHistogram(img, display=False, minVal=0.1, region=4):
    histValues = np.sum(img, axis=0)

    maxValue = np.max(histValues)
    minValue = minVal * maxValue

    indexArray = np.where(histValues >= minValue)

    basePoint = int(np.average(indexArray))
    imgHist = None  # Initialize imgHist

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

def getLaneCurve(img, display=False, minVal=0.1, region=4):
    base_point, _ = getHistogram(img, region=region)  # Calculate base point using histogram in bottom region
    middle_point, _ = getHistogram(img)  # Calculate middle point using histogram of the whole image
    curve_raw = middle_point - base_point

    if display:
        img_result = img.copy()
        cv2.putText(img_result, f"Curve: {curve_raw}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        cv2.imshow('Lane Curve', img_result)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

    return curve_raw

def stackImages(scale, imgArray):
    rows = len(imgArray)
    cols = len(imgArray[0])
    rowsAvailable = isinstance(imgArray[0], list)
    width = imgArray[0][0].shape[1]
    height = imgArray[0][0].shape[0]

    if rowsAvailable:
        for x in range(0, rows):
            for y in range(0, cols):
                if imgArray[x][y].shape[:2] == imgArray[0][0].shape[:2]:
                    imgArray[x][y] = cv2.resize(imgArray[x][y], (0, 0), None, scale, scale)
                else:
                    imgArray[x][y] = cv2.resize(imgArray[x][y], (imgArray[0][0].shape[1], imgArray[0][0].shape[0]), None, scale, scale)
                if len(imgArray[x][y].shape) == 2:
                    imgArray[x][y] = cv2.cvtColor(imgArray[x][y], cv2.COLOR_GRAY2BGR)
        imageBlank = np.zeros((height, width, 3), np.uint8)
        hor = [imageBlank] * rows
        hor_con = [imageBlank] * rows

        for x in range(0, rows):
            hor[x] = np.hstack(imgArray[x])
        ver = np.vstack(hor)
    else:
        for x in range(0, rows):
            if imgArray[x].shape[:2] == imgArray[0].shape[:2]:
                imgArray[x] = cv2.resize(imgArray[x], (0, 0), None, scale, scale)
            else:
                imgArray[x] = cv2.resize(imgArray[x], (imgArray[0].shape[1], imgArray[0].shape[0]), None, scale, scale)
            if len(imgArray[x].shape) == 2:
                imgArray[x] = cv2.cvtColor(imgArray[x], cv2.COLOR_GRAY2BGR)
        hor = np.hstack(imgArray)
        ver = hor

    return ver

# Load the image file
image_path = "/home/choi/angchicken_ros2_ws/src/line_following_bot/line_following_bot/picture/bluetrack3.jpg"
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


curve_raw = getLaneCurve(masked_edges, display=True, region=4)

base_point, hist_image = getHistogram(masked_edges, display=True)

display_option = 2  # Change this value to 0, 1, or 2 as needed

if display_option == 2:
    imgStacked = stackImages(0.7, [
        [resized_image, masked_edges, hist_image],  # Replace masked_edges with your actual variable
        # Add more images as needed for the second row
    ])
    cv2.imshow('ImageStack', imgStacked)
elif display_option == 1:
    img_result = resized_image.copy()
    cv2.putText(img_result, f"Curve: {curve_raw}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
    cv2.imshow('Result', img_result)

cv2.waitKey(0)
cv2.destroyAllWindows()
