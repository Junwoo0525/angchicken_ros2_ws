import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import numpy as np
import cv2
import math

# Load the image from file
image_path = '/home/choi/angchicken_ros2_ws/src/line_following_bot/line_following_bot/picture/bluetrack.jpg'
color_image = cv2.imread(image_path)

# Get the dimensions of the image
height, width, _ = color_image.shape

def region_of_interest(img, vertices):
    # Define a blank mask
    mask = np.zeros_like(img)

    # Fill inside the polygon defined by vertices with white color
    cv2.fillPoly(mask, vertices, 255)

    # Bitwise AND operation to keep only the region of interest
    masked_image = cv2.bitwise_and(img, mask)
    return masked_image

# Define the center region to extract color
region_of_interest_vertices = [
    (0, height),
    (width, height),
    (width, height // 2),
    (0, height // 2)
]

# Convert to grayscale
gray_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)

# Call Canny Edge Detection
cannyed_image = cv2.Canny(gray_image, 100, 200)

# Apply region of interest cropping
cropped_image = region_of_interest(
    cannyed_image,
    np.array([region_of_interest_vertices], np.int32)
)

plt.subplot(2, 3, 1)
plt.imshow(cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB))
plt.title("Original Image")


# Display the cropped image
plt.subplot(2, 3, 2)
plt.imshow(gray_image, cmap='gray')
plt.title("Color Mask")

plt.subplot(2, 3, 3)
plt.imshow(cropped_image, cmap='gray')
plt.title("Color Mask")
plt.tight_layout()
plt.show()
