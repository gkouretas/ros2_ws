from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5e robot_ip:=192.168.56.101
    rviz_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare("ur_robot_driver"), '/launch', '/ur_control.launch.py']
        ),
        launch_arguments = [("ur_type", "ur10e"), ("robot_ip", "192.168.56.101")]
    )

    return LaunchDescription([rviz_node])