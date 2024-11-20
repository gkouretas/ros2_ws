import subprocess
from launch.conditions import IfCondition
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess, LogInfo, RegisterEventHandler
)
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, NotSubstitution, ThisLaunchFileDir
from launch.event_handlers import OnProcessExit

def generate_launch_description():
    # ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5e robot_ip:=192.168.56.101
    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            'run_sim',  # Name of the argument
            default_value='false',  # Default value if not provided
            description='true to run polyscope simulation, false otherwise'
        )
    )
        
    declared_arguments.append(
        DeclareLaunchArgument(
            'network_id',  # Name of the argument
            default_value='192.168.56',  # Default value if not provided
            description='UR network ID'
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            'robot_host_id',  # Name of the argument
            default_value='.101',  # Default value if not provided
            description='UR robot host ID'
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            'robot_ip',
            default_value = [LaunchConfiguration("network_id"), LaunchConfiguration("robot_host_id")],
            # TODO: only allow valid IP addresses
            description = "Robot IP address, should not set directly"
        )
    )

    # is_sim_running = subprocess.run(
    #     ["docker", "ps", "--filter", "name=ursim", "--format", "{{.Names}}"], capture_output=True, text=True
    # ).stdout == "ursim"

    # sim_terminate = ExecuteProcess(
    #     cmd = ["docker", "stop", "ursim"]
    # )

    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([ThisLaunchFileDir(), "/launch_sim.launch.py"]),
        launch_arguments = {
            "network_id": LaunchConfiguration("network_id"),
            "robot_host_id": LaunchConfiguration("robot_host_id"),
            "detached": "1" # always detached to not block
        }.items(),
        condition = IfCondition(LaunchConfiguration("run_sim")),
    )

    # import pdb; pdb.set_trace()

    # print(sim_launch.get_sub_entities())
    # for entity in sim_launch.get_sub_entities():
    #     print(entity)
    # exit()

    ur_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([FindPackageShare("ur_robot_driver"), "/launch/ur_control.launch.py"]),
        launch_arguments = {
            "ur_type": "ur10e",
            "robot_ip": LaunchConfiguration("robot_ip"),
        }.items(),
    )

    # ur_control_start = RegisterEventHandler(
    #     OnProcessExit(
    #         target_action = sim_launch,
    #         on_exit = [ur_control_launch]
    #     )
    # )

    # ur_control_exit = RegisterEventHandler(
    #     OnProcessExit(
    #         target_action = ur_control_launch,
    #         on_exit = sim_terminate
    #     )
    # )

    return LaunchDescription(
        declared_arguments + [sim_launch, ur_control_launch],
        # on_exit = sim_terminate
    )