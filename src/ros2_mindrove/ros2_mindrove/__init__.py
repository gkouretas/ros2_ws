import rclpy
import os
import sys

sys.path.append("~/ros2_ws")
sys.path.append(os.path.dirname(__file__))

from python_utils.ros2_utils.comms.node_manager import create_simple_node
from mindrove_configs import *

# Create module node
rclpy.init()
create_simple_node(MINDROVE_ROS_NODE)