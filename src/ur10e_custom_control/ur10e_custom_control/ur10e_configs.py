from enum import Enum

UR10E_FORCE_TORQUE_NODE = "tcp_fts_sensor"
UR10E_FORCE_PUBLISHER = "force"
UR10E_TORQUE_PUBLISHER = "torque"

UR_JOINT_LIST: list[str] = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]

class URControlModes(str, Enum):
    JOINT_TRAJECTORY = "joint_trajectory_controller"
    SCALED_JOINT_TRAJECTORY = "scaled_joint_trajectory_controller"
    FORWARD_VELOCITY = "forward_velocity_controller"
    FORWARD_POSITION = "forward_position"

"""
controller_manager:
  ros__parameters:
    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster

    io_and_status_controller:
      type: ur_controllers/GPIOController

    speed_scaling_state_broadcaster:
      type: ur_controllers/SpeedScalingStateBroadcaster

    force_torque_sensor_broadcaster:
      type: force_torque_sensor_broadcaster/ForceTorqueSensorBroadcaster

    joint_trajectory_controller:
      type: joint_trajectory_controller/JointTrajectoryController

    scaled_joint_trajectory_controller:
      type: ur_controllers/ScaledJointTrajectoryController

    forward_velocity_controller:
      type: velocity_controllers/JointGroupVelocityController

    forward_position_controller:
      type: position_controllers/JointGroupPositionController
"""

UR_CONTROL_MODE_TO_CONTROL_TYPE: dict[URControlModes, ]