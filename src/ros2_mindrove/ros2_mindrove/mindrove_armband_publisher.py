# import ahrs
# import numpy as np

from typing import Tuple
from mindrove_interface import MindroveInterface

from mindrove.utils import LogLevels

from mindrove_typedefs import *
from mindrove_configs import *

from idl_definitions.msg import MindroveArmBandMsg
from geometry_msgs.msg import Vector3
# from geometry_msgs.msg import Quaternion
from python_utils.ros2_utils.comms.node_manager import get_node

# Hardware definition for real arm band
MindroveArmBand = MindroveHardwareDescription(
   board_id = BoardIds.MINDROVE_WIFI_BOARD, 
   params = MindroveInputParamsConstructor( # see https://docs.mindrove.com/SupportedBoards.html#streaming-board
      ip_address = "192.168.4.1",
      ip_port = 4210,
      timeout = 10
   ),
   preset = MindRovePresets.DEFAULT_PRESET, 
   channels = (
      MindroveChannels.EMG,
      MindroveChannels.ACCEL,
      MindroveChannels.GYRO # TODO: are there more available for the armband?
   )
)

# Hardware definition for simulated arm band
SimulatedArmBand = MindroveHardwareDescription(
   board_id = BoardIds.SYNTHETIC_BOARD,
   params = MindroveInputParamsConstructor(),
   preset = MindRovePresets.DEFAULT_PRESET,
   channels = MindroveArmBand.channels
)

# from scipy.spatial.transform import Rotation

class MindroveArmbandRosPublisher(MindroveInterface):
    def __init__(self, simulated: bool = False, additional_channels: Tuple[str, ...] = ..., log_level: LogLevels = LogLevels.LEVEL_OFF) -> None:
        super().__init__(SimulatedArmBand if simulated else MindroveArmBand, additional_channels, log_level)
        self.mindrove_publisher = get_node(MINDROVE_ROS_NODE).create_publisher(MindroveArmBandMsg, MINDROVE_ROS_TOPIC_NAME, 0)
        
      #   self._pose_estimator = ahrs.filters.Madgwick(frequency = 50.0)
      #   self._last_pose = np.array([1.0, 0.0, 0.0, 0.0]) # identity

    def data_callback(self, timestamp: float, data: MindroveSampleBuffer) -> None:
         """
         Publish arm band message packet

         Args:
            timestamp (float): Timestamp taken when data was received
            data (MindroveSampleBuffer): Data buffer from Mindrove
         """
         msg = MindroveArmBandMsg() 
         msg.frame = self._num
         msg.num_samples = self._buf_size
         msg.world_timestamp = timestamp
         msg.source_timestamp = data["timestamp"].flatten().tolist()

         msg.emg = data[MindroveChannels.EMG].flatten().tolist()
         msg.accel = []
         msg.gyro = []
         # msg.quat = []
      
         for accel, gyro in zip(data[MindroveChannels.ACCEL].T, data[MindroveChannels.GYRO].T):
            msg.accel.append(Vector3(x = accel[0], y = accel[1], z = accel[2]))
            msg.gyro.append(Vector3(x = gyro[0], y = gyro[1], z = gyro[2]))

         #    self._last_pose = self._pose_estimator.updateIMU(self._last_pose, gyro, accel)
         #    msg.quat.append(Quaternion(w = self._last_pose[0], x = self._last_pose[1], y = self._last_pose[2], z = self._last_pose[3])) 
                  
         # _euler = Rotation.from_quat(self._last_pose, scalar_first = True).as_euler(seq = 'xyz', degrees = True)
         # msg.euler = Vector3(x = _euler[0], y = _euler[1], z = _euler[2])         

         self.mindrove_publisher.publish(msg)