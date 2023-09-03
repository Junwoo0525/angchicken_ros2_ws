import numpy as np
import pyrealsense2 as rs

def calculate_tilt_angle(accel_data, reference_data):
    delta_accel = accel_data - reference_data
    delta_accel = delta_accel * 9.81
    
    pitch_rad = np.arctan2(delta_accel[1], np.sqrt(delta_accel[0]**2 + delta_accel[2]**2))
    pitch_deg = np.degrees(pitch_rad)
    return pitch_deg

def moving_average(data_array, num_samples):
    data_samples = []
    for value in data_array:
        data_samples.append(value)
        if len(data_samples) > num_samples:
            data_samples.pop(0)
    return np.mean(data_samples)

state = 0
threshold_angle = 12.0
threshold_accel = 3

num_samples = 20
accel_x_samples = []
tilt_angle_samples = []
state_samples = []

p = rs.pipeline()
conf = rs.config()
conf.enable_stream(rs.stream.accel)
conf.enable_stream(rs.stream.gyro)
prof = p.start(conf)

initial_reference_data = None

try:
    while True:
        f = p.wait_for_frames()
        accel = np.array([f[0].as_motion_frame().get_motion_data().x,
                            f[0].as_motion_frame().get_motion_data().y,
                            f[0].as_motion_frame().get_motion_data().z])
        gyro = np.array([f[1].as_motion_frame().get_motion_data().x,
                            f[1].as_motion_frame().get_motion_data().y,
                            f[1].as_motion_frame().get_motion_data().z])
        
        if initial_reference_data is None:
            initial_reference_data = accel
            print("Initial IMU Reading Captured")
        
        tilt_angle = calculate_tilt_angle(accel, initial_reference_data)
        
        accel_x_change = accel[0] - initial_reference_data[0]
        
        accel_x_filtered = moving_average(accel_x_samples, num_samples)
        tilt_angle_filtered = moving_average(tilt_angle_samples, num_samples)
        
        state_samples.append(state)
        if len(state_samples) > num_samples:
            state_samples.pop(0)
        result_state = int(round(moving_average(state_samples, num_samples)))
        
        accel_x_samples.append(accel_x_change)
        tilt_angle_samples.append(tilt_angle)
        
        if len(accel_x_samples) > num_samples:
            accel_x_samples.pop(0)
        if len(tilt_angle_samples) > num_samples:
            tilt_angle_samples.pop(0)
        
        if tilt_angle_filtered > threshold_angle and accel_x_filtered < -threshold_accel:
            state = 1  # Right
        elif tilt_angle_filtered > threshold_angle and accel_x_filtered > threshold_accel:
            state = -1  # Left
        else:
            state = 0  # N

        print(f'Result_State: {result_state}, State: {state}, Tilt_AngleF: {tilt_angle_filtered:.2f}, Accel_XF: {accel_x_filtered:.2f}')

finally:
    p.stop()