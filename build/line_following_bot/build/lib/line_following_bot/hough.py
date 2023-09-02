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

def getHistogram(image, display=False, minVal=0.1, region=4):
    histValues = np.sum(image, axis=0)
    maxValue = np.max(histValues)
    minValue = minVal * maxValue
    indexArray = np.where(histValues >= minValue)
    basePoint = int(np.average(indexArray))
    if display:
        imgHist = np.zeros((image.shape[0], image.shape[1], 3), np.uint8)
        for x, intensity in enumerate(histValues):
            if intensity > minValue:
                color = (255, 0, 255)
            else:
                color = (0, 0, 255)
            cv2.line(imgHist, (x, image.shape[0]), (x, image.shape[0] - (intensity // 255 // region)), color, 1)
        cv2.circle(imgHist, (basePoint, image.shape[0]), 20, (0, 255, 255), cv2.FILLED)
        return imgHist
    return None

def main():
    image_path = "/home/choi/angchicken_ros2_ws/src/line_following_bot/line_following_bot/picture/bluetrack.jpg"
    image = cv2.imread(image_path)

    # Display the original image
    cv2.imshow('Original Image', image)
    cv2.waitKey(0)

    # Apply median blur to the image
    blur_image = cv2.medianBlur(image, 9)

    # Display the blurred image
    cv2.imshow('Blurred Image', blur_image)
    cv2.waitKey(0)

    # Convert the image to the HSV color space
    hsv_image = cv2.cvtColor(blur_image, cv2.COLOR_BGR2HSV)

    # Display the HSV image
    cv2.imshow('HSV Image', hsv_image)
    cv2.waitKey(0)

    # Define the range of blue color in HSV
    lower_blue = np.array([90, 50, 50])
    upper_blue = np.array([130, 255, 255])

    # Create a mask to extract blue regions
    blue_mask = cv2.inRange(hsv_image, lower_blue, upper_blue)

    # Display the blue mask
    cv2.imshow('Blue Mask', blue_mask)
    cv2.waitKey(0)

    # Apply the mask to the original image
    blue_extracted = cv2.bitwise_and(image, image, mask=blue_mask)

    # Display the blue-extracted image
    cv2.imshow('Blue Extracted Image', blue_extracted)
    cv2.waitKey(0)

    # Apply region masking to the blue_extracted image
    masked_image = region(blue_extracted)

    # Display the masked image
    cv2.imshow('Masked Image', masked_image)
    cv2.waitKey(0)

    # Apply Canny edge detection
    edges = cv2.Canny(masked_image, threshold1=50, threshold2=150)

    # Display the edges

    histogram = getHistogram(masked_image, display=True, minVal=0.1)

    # Display the histogram


    # Perform Hough Transform for curve detection
    curves = cv2.HoughLinesP(edges, 1, np.pi/180, threshold=50, minLineLength=50, maxLineGap=5)
    if curves is not None:
        for curve in curves:
            x1, y1, x2, y2 = curve[0]
            cv2.line(image, (x1, y1), (x2, y2), (0, 255, 0), 2)
    else:
        print("No curves were detected.")

    # Display the result with detected curves
    cv2.imshow('Detected Curves', histogram)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
