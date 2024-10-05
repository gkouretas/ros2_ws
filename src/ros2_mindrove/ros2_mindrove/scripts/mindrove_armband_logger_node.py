#!/usr/bin/python3
"""
Script for executing a logger node for the Mindrove armband
"""
from __future__ import annotations

import rclpy

# Project based / ROS imports
from ros2_mindrove.mindrove_configs import *
from ros2_mindrove.mindrove_armband_logger import ROSMindroveCSVLogger
from python_utils.ros2_utils.comms.node_manager import get_node

def main() -> None:
    ROSMindroveCSVLogger(log_name = MINDROVE_ROS_NODE)
    rclpy.spin(get_node(MINDROVE_ROS_NODE))

if __name__ == "__main__":
    main()