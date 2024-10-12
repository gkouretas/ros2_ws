# Deprecated...
uint64 frame
float64 world_timestamp
uint64 num_samples
float64[] source_timestamp

float64[] emg
geometry_msgs/Vector3[] accel
geometry_msgs/Vector3[] gyro
# TODO: pose estimation from accelerometer? Or have separate node handler...
# geometry_msgs/Quaternion[] quat
# geometry_msgs/Vector3 euler # temp: for debugging