Available controllers
---------------------

forward_position_controller[position_controllers/JointGroupPositionController]
scaled_joint_trajectory_controller[ur_controllers/ScaledJointTrajectoryController]
joint_state_broadcaster[joint_state_broadcaster/JointStateBroadcaster]
io_and_status_controller[ur_controllers/GPIOController]
speed_scaling_state_broadcaster[ur_controllers/SpeedScalingStateBroadcaster]
force_torque_sensor_broadcaster[force_torque_sensor_broadcaster/ForceTorqueSensorBroadcaster]    

ros2 run ur_client_library start_ursim.sh -a 192.168.57 -m ur10e -v 5.9.4 -d 1
ros2 launch ur_robot_driver ur10e.launch.py robot_ip:=192.168.57.101 headless_mode:=true

- `use_fake_hardware`: true -> uses ROS2 commands
- `use_fake_hardware`: false -> uses UrSim
ros2 control load_controller --set-state {configured} forward_velocity_controller
ros2 control set_controller_state forward_velocity_controller active
ros2 launch ur_robot_driver test_forward_velocity_controller.launch.py
ros2 control set_controller_state forward_velocity_controller inactive

ros2 control set_controller_state scaled_joint_trajectory_controller active
ros2 launch ur_robot_driver test_scaled_joint_trajectory_controller.launch.py
ros2 control set_controller_state scaled_joint_trajectory_controller inactive

docker network rm ursim_net
systemctl status docker
sudo systemctl start docker

ros2 run ur_client_library start_ursim.sh -a 192.168.57 -m ur10e -v 5.9.4 headless:=true && ros2 launch ur_robot_driver ur10e.launch.py robot_ip:=192.168.57.101 headless_mode:=true && ros2 launch ur_robot_driver test_scaled_joint_trajectory_controller.launch.py