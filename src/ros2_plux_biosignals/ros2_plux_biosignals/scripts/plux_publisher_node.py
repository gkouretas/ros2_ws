#!/usr/bin/python3
"""
Script for executing a publisher node
"""
from __future__ import annotations

import rclpy
import signal
import os
import sys

# Add the main project to the file path, since the ROS file structure goes
# |_ project
# |__ scripts
# |____ *.py
sys.path.append(os.path.dirname(os.path.dirname(__file__)))

# Project based / ROS imports
from ros2_plux_biosignals.plux_publisher import MyPluxThread
from ros2_plux_biosignals.plux_configs import *
from idl_definitions.msg import PluxMsg
from utils.comms.node_manager import get_node

def main() -> None:
    log_publisher = get_node(PLUX_ROS_NODE).create_publisher(PLUX_ROS_DEBUG_PUBLISHER)
    plux_thread = MyPluxThread(log_publisher)
    signal.signal(signal.SIGINT, plux_thread.signal_handler)
    plux_thread.start()

if __name__ == "__main__":
    main()