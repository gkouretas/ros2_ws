import rclpy
import sys
import os

sys.path.append("~/ros2_ws")

# Add the main project to the file path, since the ROS file structure goes
# |_ project
# |__ scripts
# |____ *.py
sys.path.append(os.path.dirname(__file__))

from ur10e_configs import *
from utils.comms.node_manager import create_simple_node

rclpy.init()
create_simple_node(UR10E_FORCE_TORQUE_NODE)