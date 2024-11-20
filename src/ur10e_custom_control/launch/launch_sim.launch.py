from dataclasses import dataclass
from typing import Optional

from launch import LaunchDescription

from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    RegisterEventHandler,
)

from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackagePrefix, FindPackageShare

@dataclass(frozen = True)
class SimArgument:
    cmd: str
    name: str
    default: Optional[str] = None
    description: Optional[str] = None
    choices: Optional[list[str]] = None

    @property
    def ros2_config(self) -> LaunchConfiguration:
        return LaunchConfiguration(self.name)

_SIM_ARGUMENTS = (
    SimArgument("-m", "model", "ur5e", "UR model", ["ur3", "ur5", "ur10", "ur3e", "ur5e", "ur10e", "ur16e", "ur20", "ur30"]),
    SimArgument("-v", "version", "latest", "Polyscope software version for simulation", None),
    SimArgument("-d", "detached", "0", "if true, detach sim from cmd prompt. if false, blocks", ["0", "1"]),
    SimArgument("-i", "network_ip", "192.168.56.101", "IP address to reach the robot on", None),
)

def generate_launch_description() -> LaunchDescription:
    declared_arguments = []
    for arg in _SIM_ARGUMENTS:
        declared_arguments.append(
            DeclareLaunchArgument(
                name = arg.name,
                default_value=arg.default,
                description=arg.description,
                choices=arg.choices
            )
        )

    _args = []
    for arg in _SIM_ARGUMENTS:
        _args.append(arg.cmd)
        _args.append(arg.ros2_config)
    
    sim_process = ExecuteProcess(
        cmd = [
            PathJoinSubstitution(
                [
                    FindPackagePrefix("ur_client_library"),
                    "lib",
                    "ur_client_library",
                    "start_ursim.sh",
                ]
            )
        ] + _args,
        name = "start_ursim",
        output = "both"
    ) 

    return LaunchDescription(declared_arguments + [sim_process])