import cv2
import pyrealsense2 as rs
import numpy as np

# Initialize the RealSense camera pipeline
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)  # Set the desired camera configuration

# Start the pipeline
pipeline.start(config)

try:
    while True:
        # Wait for a new frame from the camera
        frames = pipeline.wait_for_frames()

        # Get the color frame
        color_frame = frames.get_color_frame()

        if not color_frame:
            continue

        # Convert the color frame to a numpy array
        color_image = np.asanyarray(color_frame.get_data())

        # Resize the image to 160x120
        resized_image = cv2.resize(color_image, (160, 120))

        # Display the resized image
        cv2.imshow('Resized Image', resized_image)

        # Exit the loop when 'q' is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    # Stop the RealSense pipeline and close OpenCV windows
    pipeline.stop()
    cv2.destroyAllWindows()
