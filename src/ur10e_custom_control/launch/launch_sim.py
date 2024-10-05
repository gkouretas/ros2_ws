from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # ros2 run ur_client_library start_ursim.sh -m ur10e
    sim_node = Node(
        package = 'ur_client_library',
        executable = 'start_ursim.sh',
        arguments = '-m ur10e',
        output = 'screen',
        emulate_tty = True
    )

    return LaunchDescription([sim_node])