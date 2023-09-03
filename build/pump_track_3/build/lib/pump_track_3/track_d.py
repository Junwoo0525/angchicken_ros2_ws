import pyrealsense2 as rs
import numpy as np
import cv2

def main():
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    pipeline.start(config)

    cen_x = 320
    roi_start_row = int(480 * 0.35)
    roi_end_row = int(480 * 0.5)
    roi_start_col = int(640 * 0.05)
    roi_end_col = int(640 * 0.95)

    yellow_color = [0, 255, 255]

    while True:
        fs = pipeline.wait_for_frames()
        color_f = fs.get_color_frame()
        color_a = np.asanyarray(color_f.get_data())
        color_i = cv2.GaussianBlur(color_a, (7, 7), 0)

        roi = color_i[roi_start_row:roi_end_row, roi_start_col:roi_end_col]
        
        dominant_color = np.mean(roi, axis=(0, 1))
        lower_bound = dominant_color - np.array([10, 70, 70])
        upper_bound = dominant_color + np.array([130, 255, 255])

        mask = cv2.inRange(roi, lower_bound, upper_bound)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        max_area = 0
        max_contour = None
        for contour in contours:
            area = cv2.contourArea(contour)
            if area > max_area:
                max_area = area
                max_contour = contour

        if max_contour is not None:
            largest_component_mask = np.zeros_like(mask)
            cv2.drawContours(largest_component_mask, [max_contour], -1, (255, 255, 0), thickness=cv2.FILLED)
            roi[largest_component_mask > 0] = yellow_color

        color_i[roi_start_row:roi_end_row, roi_start_col:roi_end_col] = roi
        cv2.rectangle(color_i, (roi_start_col, roi_start_row), (roi_end_col, roi_end_row), (0, 255, 0), 2)
        cv2.line(color_i, (cen_x, roi_start_row), (cen_x, roi_end_row), (0, 0, 255), 2)

        left_region = largest_component_mask[:, :cen_x - roi_start_col].copy()
        right_region = largest_component_mask[:, cen_x - roi_start_col:].copy()

        l_area = np.sum(left_region == 255)
        r_area = np.sum(right_region == 255)

        cv2.putText(color_i, f"L Area: {l_area}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        cv2.putText(color_i, f"R Area: {r_area}", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

        if l_area > r_area * 1.2:
            direction = 'Left'
        elif r_area > l_area * 1.2:
            direction = 'Right'
        else:
            direction = 'Straight'
        print(direction)

        cv2.imshow('Track', color_i)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    pipeline.stop()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
