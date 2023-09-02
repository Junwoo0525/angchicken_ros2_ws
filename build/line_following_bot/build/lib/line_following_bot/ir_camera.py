import cv2
import numpy as np

# Load the image
image_path = "/home/choi/angchicken_ros2_ws/src/line_following_bot/line_following_bot/picture/bluetrack10.jpg"
image = cv2.imread(image_path)

# Generate random fog
fog_density = 0.5  # Adjust the fog density as needed
for _ in range(int(image.shape[0] * image.shape[1] * fog_density)):
    x = np.random.randint(0, image.shape[1])
    y = np.random.randint(0, image.shape[0])
    color = np.random.randint(200, 256)  # Adjust color intensity as needed
    cv2.circle(image, (x, y), np.random.randint(1, 5), (color, color, color), -1)

# Save the foggy image
output_path = "/home/choi/angchicken_ros2_ws/src/line_following_bot/line_following_bot/picture/foggy_bluetrack10.jpg"
cv2.imwrite(output_path, image)

# Display the foggy image (optional)
cv2.imshow('Foggy Image', image)
cv2.waitKey(0)
cv2.destroyAllWindows()
