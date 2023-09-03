import cv2
import numpy as np
import pyrealsense2 as rs

# Initialize the RealSense pipeline
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

pipeline.start(config)

while True:
    # Wait for a new frame from the camera
    frames = pipeline.wait_for_frames()
    color_frame = frames.get_color_frame()

    # Convert the color frame to an OpenCV format
    if color_frame:
        color_image = np.asanyarray(color_frame.get_data())

        # Convert the color image to grayscale
        gray_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)

        # Apply a threshold to identify low-visibility areas
        # Adjust the threshold value as needed
        threshold_value = 100
        _, binary_image = cv2.threshold(gray_image, threshold_value, 255, cv2.THRESH_BINARY)

        # Calculate the percentage of white pixels in the binary image
        white_pixel_percentage = np.sum(binary_image == 255) / (binary_image.shape[0] * binary_image.shape[1])

        # Define a threshold for identifying foggy situations
        fog_threshold = 0.5  # Adjust as needed

        # Check if the percentage of white pixels exceeds the fog threshold
        if white_pixel_percentage > fog_threshold:
            print("Foggy situation detected!")
        else:
            print("Normal situation")

        # Display the color image and binary image (for visualization)
        cv2.imshow('Color Image', color_image)
        cv2.imshow('Binary Image', binary_image)

    # Exit the loop when the 'q' key is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the RealSense pipeline and close OpenCV windows
pipeline.stop()
cv2.destroyAllWindows()
