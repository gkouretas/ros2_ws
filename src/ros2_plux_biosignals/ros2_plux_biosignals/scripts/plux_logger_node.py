#!/usr/bin/python3
"""
Script for executing a logger node
"""
from __future__ import annotations

import rclpy

# Project based / ROS imports
from ros2_plux_biosignals.plux_configs import *
from ros2_plux_biosignals.plux_logger import ROSPluxCSVLogger

def main() -> None:
    ROSPluxCSVLogger(log_name = PLUX_ROS_NODE)
    rclpy.spin()

if __name__ == "__main__":
    main()