"""
Utility functions for processing Mindrove signals
"""
from mindrove import BoardShim
from mindrove_typedefs import *

def get_scaling_factors(interface: BoardShim) -> Dict[MindroveChannels, float]:
   """
   Get relevant scaling factors to convert from ADC -> real units.

   Currently included:
   - EMG: uV
   - ACCEL: m/s^2
   - GYRO: dps

   Args:
       interface (BoardShim): BoardShim interface

   Returns:
       Dict[MindroveChannels, float]: Dictionary that can be keyed with
       a Mindrove channel enum / string identifier.
   """
   # Expected format is: x.y.z
   version = tuple((int(x) for x in interface.get_version().split(".")))
   
   if version >= (0, 0, 1):
      # Per Mindrove, the IMU data is already scaled
      # Apply an additional 9.81x for the accelerometer
      # to convert to m/s^2
      return {
         MindroveChannels.EMG: 1.0,
         MindroveChannels.ACCEL: 9.81,
         MindroveChannels.GYRO: 0.0174533 # 180 / pi
      }
   else:
      return {
         MindroveChannels.EMG: 0.045,
         MindroveChannels.ACCEL: 0.061035e-3 * 9.81,
         MindroveChannels.GYRO: 0.01526
      }
