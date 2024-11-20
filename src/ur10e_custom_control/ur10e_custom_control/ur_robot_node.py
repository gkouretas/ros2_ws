import rclpy
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory

from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.client import Client, SrvTypeRequest, Future

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from ur_msgs.srv import SetIO
from controller_manager_msgs.srv import SwitchController
from std_srvs.srv import Trigger
from ur10e_typedefs import URService

from ur10e_configs import UR_JOINT_LIST

from typing import Iterable, Optional

_DEFAULT_SERVICE_TIMEOUT_SEC = 10
_DEFAULT_ACTION_TIMEOUT_SEC = 10

class URRobot(Node):
    def __init__(self, node_name: str, initial_services: Iterable[URService] = [], **kwargs) -> None:
        super().__init__(node_name, **kwargs)

        self.service_clients: dict[URService, Optional[Client]] = {
            k: None for k in URService.URServices
        }

        for service in initial_services:
            assert service in self.service_clients.keys(), f"Invalid service: {service}"
            self.service_clients[service] = URService.init_service(self, service, _DEFAULT_SERVICE_TIMEOUT_SEC)

        self.jtc_action_client = self.wait_for_action(
            "/scaled_joint_trajectory_controller/follow_joint_trajectory",
            FollowJointTrajectory,
        )
    
    def wait_for_action(self, action_name: str, action_type: type, timeout: int =_DEFAULT_ACTION_TIMEOUT_SEC):
        client = ActionClient(self, action_type, action_name)
        if client.wait_for_server(timeout) is False:
            raise Exception(
                f"Could not reach action server '{action_name}' within timeout of {timeout}"
            )

        self.get_logger().info(f"Successfully connected to action '{action_name}'")
        return client

    def set_io(self, pin, value):
        """Set an IO"""
        if self.service_clients[URService.IOAndStatusController.SRV_SET_IO] is None:
            self.service_clients[URService.IOAndStatusController.SRV_SET_IO] = \
                URService.init_service(self, URService.IOAndStatusController.SRV_SET_IO, _DEFAULT_SERVICE_TIMEOUT_SEC)

        set_io_req = SetIO.Request()
        set_io_req.fun = 1
        set_io_req.pin = pin
        set_io_req.state = value

        self.call_service(URService.IOAndStatusController.SRV_SET_IO, set_io_req)

    def send_trajectory(self, waypts, time_vec):
        """Send robot trajectory."""
        if len(waypts) != len(time_vec):
            raise Exception("waypoints vector and time vec should be same length")

        # Construct test trajectory
        joint_trajectory = JointTrajectory()
        joint_trajectory.joint_names = UR_JOINT_LIST
        for i in range(len(waypts)):
            point = JointTrajectoryPoint()
            point.positions = waypts[i]
            point.time_from_start = time_vec[i]
            joint_trajectory.points.append(point)

        # Sending trajectory goal
        # TODO: make non-blocking
        goal_response = self.call_action(
            self.jtc_action_client, FollowJointTrajectory.Goal(trajectory=joint_trajectory)
        )
        if not goal_response.accepted:
            raise Exception("trajectory was not accepted")

        # Verify execution
        result: FollowJointTrajectory.Result = self.get_result(self.jtc_action_client, goal_response)
        return result.error_code == FollowJointTrajectory.Result.SUCCESSFUL

    def call_service(self, srv: URService, request: SrvTypeRequest):
        service = self.service_clients.get(srv)
        if service is None:
            self.service_clients[srv] = URService.init_service(self, srv, _DEFAULT_SERVICE_TIMEOUT_SEC)
            service = self.service_clients[srv]

        future: Future = service.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            return future.result()
        else:
            raise Exception(f"Exception while calling service: {future.exception()}")

    def call_action(self, ac_client: ActionClient, g):
        future = ac_client.send_goal_async(g)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            return future.result()
        else:
            raise Exception(f"Exception while calling action: {future.exception()}")

    def get_result(self, ac_client: ActionClient, goal_response):
        future_res = ac_client._get_result_async(goal_response)
        rclpy.spin_until_future_complete(self, future_res)
        if future_res.result() is not None:
            return future_res.result().result
        else:
            raise Exception(f"Exception while calling action: {future_res.exception()}")
