import numpy as np

from controller_base import URController
from ur10e_custom_control.ur10e_configs import (
    URControlModes,
    UR_QOS_PROFILE
)

from rclpy.task import Future

from builtin_interfaces.msg import Duration
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from control_msgs.action import FollowJointTrajectory

from threading import Lock

class HomeURRobot(URController):
    UR_HOME_POSITION = [0.0, np.radians(-90.0), 0.0, np.radians(-90.0), 0.0, 0.0]
    def __init__(self, name: str, duration: int = 10) -> None:
        super().__init__(name, URControlModes.SCALED_JOINT_TRAJECTORY)
        self._is_first: bool = True
        self._initial_position: list[float] = None
        self._trajectory: JointTrajectory = None
        self._mutex = Lock()
        
        self._duration = duration

    def compute(self):
        if self._is_first:
            self.get_logger().info("Computing home trajectory...")
            self._trajectory = JointTrajectory()
            self._trajectory.joint_names = self._joints
            
            home_position = JointTrajectoryPoint(
                positions = HomeURRobot.UR_HOME_POSITION,
                velocities = [0.0] * len(self._joints),
                time_from_start = Duration(sec=self._duration, nanosec=0)
            )

            self._trajectory.points = [
                home_position
            ]

            self.get_logger().info("Setup home trajectory")

            self._is_first = False

            return True
        else:
            self._trajectory = None
            self._mutex.release()

            return False

    def publish(self):
        if self._trajectory is None:
            return False
        
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = self._trajectory

        goal.goal_time_tolerance = Duration(sec=0, nanosec=1000)

        self.get_logger().info(f"Goal: {goal}")
        self.get_logger().info(f"Publishing trajectory: {goal.trajectory}")

        self._send_goal_future = self._action_client.send_goal_async(goal)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

        return True
    
    def reset(self):
        self._is_first = False

    def goal_response_callback(self, future: Future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected :(")
            raise RuntimeError("Goal rejected :(")

        self.get_logger().info("Goal accepted :)")

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future: Future):
        result: FollowJointTrajectory.Result = future.result().result
        self.get_logger().info(f"Done with result: {self.error_code_to_str(result.error_code)}")
        self.stop()

    @staticmethod
    def error_code_to_str(error_code):
        if error_code == FollowJointTrajectory.Result.SUCCESSFUL:
            return "SUCCESSFUL"
        if error_code == FollowJointTrajectory.Result.INVALID_GOAL:
            return "INVALID_GOAL"
        if error_code == FollowJointTrajectory.Result.INVALID_JOINTS:
            return "INVALID_JOINTS"
        if error_code == FollowJointTrajectory.Result.OLD_HEADER_TIMESTAMP:
            return "OLD_HEADER_TIMESTAMP"
        if error_code == FollowJointTrajectory.Result.PATH_TOLERANCE_VIOLATED:
            return "PATH_TOLERANCE_VIOLATED"
        if error_code == FollowJointTrajectory.Result.GOAL_TOLERANCE_VIOLATED:
            return "GOAL_TOLERANCE_VIOLATED"


def main():
    import time
    import rclpy

    from ur_robot_node import URRobot
    from ur10e_typedefs import URService
    from controller_manager_msgs.srv import SwitchController
    from ur_dashboard_msgs.srv import GetRobotMode
    from std_srvs.srv import Trigger

    robot = URRobot("ur_custom_controller")
    
    robot.get_logger().info(f"Power on: {robot.call_service(URService.DashboardClient.SRV_POWER_ON, Trigger.Request())}")
    robot.get_logger().info(f"Brake release: {robot.call_service(URService.DashboardClient.SRV_BRAKE_RELEASE, Trigger.Request())}")
    time.sleep(1)
    robot.get_logger().info(f"Running mode: {robot.call_service(URService.DashboardClient.SRV_GET_ROBOT_MODE, GetRobotMode.Request())}")
    robot.get_logger().info(f"Resend command: {robot.call_service(URService.IOAndStatusController.SRV_RESEND_ROBOT_PROGRAM, Trigger.Request())}")

    switch_request = SwitchController.Request()
    switch_request.activate_controllers = ["scaled_joint_trajectory"]
    robot.call_service(URService.ControllerManager.SRV_SWITCH_CONTROLLER, switch_request)

    robot.send_trajectory([[0.0, np.radians(-90.0), 0.0, np.radians(-90.0), 0.0, 0.0]], [Duration(sec = 10)])
    time.sleep(1)
    robot.call_service(URService.DashboardClient.SRV_STOP, Trigger.Request())
    rclpy.spin(robot)

    # controller = HomeURRobot("ur_custom_controller")
    # threading.Thread(target = controller.run, daemon = True).start()
    # rclpy.spin(controller)
    # controller.stop()
