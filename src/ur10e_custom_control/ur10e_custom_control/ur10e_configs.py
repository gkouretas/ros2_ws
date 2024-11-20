from enum import Enum
from control_msgs.action import FollowJointTrajectory
from std_msgs.msg._float64_multi_array import Float64MultiArray
from ur10e_typedefs import (
    URService,
)

UR10E_FORCE_TORQUE_NODE = "tcp_fts_sensor"
UR10E_FORCE_PUBLISHER = "force"
UR10E_TORQUE_PUBLISHER = "torque"

UR_QOS_PROFILE = 1

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

    @property
    def action_type(self) -> type:
        if self.value == self.JOINT_TRAJECTORY or self.value == self.SCALED_JOINT_TRAJECTORY:
            return FollowJointTrajectory
        else:
            return None

    @property
    def action_type_topic(self) -> str:
        if self.value == self.JOINT_TRAJECTORY or self.value == self.SCALED_JOINT_TRAJECTORY:
            return self.value + "/follow_joint_trajectory"
        else:
            raise None
        
    @property
    def publish_topic(self) -> type:
        if self.value == self.FORWARD_VELOCITY or self.value == self.FORWARD_POSITION:
            return Float64MultiArray
        else:
            return None
        
    @property
    def publish_topic_name(self) -> str:
        if self.value == self.FORWARD_VELOCITY or self.value == self.FORWARD_POSITION:
            return "/" + self.value + "/commands"
        else:
            return None

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

# UR_CONTROL_MODE_TO_CONTROL_TYPE: dict[URControlModes, ]