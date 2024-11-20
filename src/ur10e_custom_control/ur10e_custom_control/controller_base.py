import abc
import time, timeit

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from ur10e_configs import (
    URControlModes,
    UR_JOINT_LIST
)

from ur_dashboard_msgs.msg import RobotMode
from ur_dashboard_msgs.srv import GetRobotMode, IsProgramRunning
from ur_dashboard_msgs.srv._get_robot_mode import GetRobotMode_Response
from ur_dashboard_msgs.srv._is_program_running import IsProgramRunning_Response
from typing import final

class URController(Node, abc.ABC):
    def __init__(self, name: str, control_name: URControlModes) -> None:
        Node.__init__(self, name)

        self._run: bool = True
        self._action_client: ActionClient
        self._joints = UR_JOINT_LIST
        self._control_name = control_name

        self.declare_parameter("controller_name", self._control_name.value)
        self.declare_parameter("joints", UR_JOINT_LIST)

        self.set_control_mode()
        self.wait_for_client_to_be_ready()
        
        self.get_logger().info(f"Initialized {self.get_name()}")

    @final
    def set_control_mode(self):
        # TODO: check that control mode is active and, if not, switch it
        if self._control_name.action_type is None:
            return
        
        self._action_client = ActionClient(self, self._control_name.action_type, self._control_name.action_type_topic)
        while not self._action_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info(f"Waiting for action server for type {self._control_name.action_type} ({self._control_name})")
        self.get_logger().info("Connected to action client")

    @final
    def wait_for_client_to_be_ready(self):
        """
        Hacky method for waiting until the client is ready to execute. Two things that are being checked:
        1) The robot is "running"
        2) There is an active program running

        NOTE: there is no name for the external program that is sent, so there is no way for checking that
        the external program is executing remotely. It is assumed to be since there isn't anything else that
        we would want to run...
        """

        self._state_client = self.create_client(GetRobotMode, "/dashboard_client/get_robot_mode")
        while not self._state_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting to query robot mode...')
        self.get_logger().info("Connected to robot mode client")

        self.get_logger().info("Waiting to enter running state...")
        
        running = False
        request = GetRobotMode.Request()
        while not running:
            future = self._state_client.call_async(request)
            rclpy.spin_until_future_complete(self, future)
            mode: GetRobotMode_Response = future.result()
            running = (mode.robot_mode.mode == RobotMode.RUNNING)
            self.get_logger().info(f"Current mode: {mode}")
            time.sleep(1.0)

        self.get_logger().info("Waiting to start of external program...")

        self._state_client = self.create_client(IsProgramRunning, "/dashboard_client/program_running")
        while not self._state_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting to get the program state...')
        self.get_logger().info("Connected to program client")

        self.get_logger().info("Waiting...")
        
        running = False
        request = IsProgramRunning.Request()
        while not running:
            future = self._state_client.call_async(request)
            rclpy.spin_until_future_complete(self, future)
            is_program_running: IsProgramRunning_Response = future.result()
            running = is_program_running.program_running
            self.get_logger().info(f"{is_program_running}")
            time.sleep(1.0)

        self.get_logger().info("Program is running, ready to execute controller")
    
    @final
    def stop(self) -> None:
        self._run = False

    @property
    def is_running(self):
        return self._run

    def run(self) -> None:
        sampling_rate = 1.0 / 500.0
        while self._run:
            t = timeit.default_timer()
            if self.compute():
                self.publish()

            delay_s = max(0.0, sampling_rate - (timeit.default_timer() - t))
            time.sleep(delay_s)

    @abc.abstractmethod
    def compute(self) -> bool:
        pass

    @abc.abstractmethod
    def publish(self) -> bool:
        pass

    @abc.abstractmethod
    def reset(self) -> None:
        pass