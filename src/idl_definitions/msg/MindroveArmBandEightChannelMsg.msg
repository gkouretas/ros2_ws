# Local information
uint64 frame                   # Local stamp, used to determine if sub skips frames 
float64 world_timestamp        # Timestamp taken when data is received

# Mindrove provided data
float64[] source_timestamp     # Source timestamp, same clock as "world" since connected via WiFi
float64[] battery_percentage   # Battery percentage, 0-100%

# Armband has 8 channels + 2 references
uint64 num_emg_samples         # Number of samples being queried at a given time
float64[] control_timestamp     # Control timestamp for EMG samples, based upon the sampling frequency
float64[] c1
float64[] c2
float64[] c3
float64[] c4
float64[] c5
float64[] c6
float64[] c7
float64[] c8

# TODO: should this not be a vector, seems like it's always 1 sample...
uint64 num_imu_samples          # Number of samples being queried at a given time
geometry_msgs/Vector3[] accel
geometry_msgs/Vector3[] gyro

# TODO: pose estimation from IMU? Or have separate node handler...
# geometry_msgs/Quaternion[] quat
# geometry_msgs/Vector3 euler # temp: for debugging