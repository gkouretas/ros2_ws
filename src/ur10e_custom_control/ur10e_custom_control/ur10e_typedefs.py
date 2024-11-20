from dataclasses import dataclass
from enum import Enum
from rclpy.node import Node
from rclpy.client import Client

from std_srvs.srv import Trigger
from ur_dashboard_msgs.msg import RobotMode

from controller_manager_msgs.srv import ListControllers, SwitchController
from ur_dashboard_msgs.srv import (
    GetLoadedProgram,
    GetProgramState,
    GetRobotMode,
    IsProgramRunning,
    Load,
)
from ur_msgs.srv import SetIO, GetRobotSoftwareVersion

class URService:
    class DashboardClient(str, Enum):
        __namespace = "/dashboard_client"
        
        SRV_POWER_ON = __namespace + "/power_on"
        SRV_POWER_OFF = __namespace + "/power_off"
        SRV_BRAKE_RELEASE = __namespace + "/brake_release"
        SRV_UNLOCK_PROTECTIVE_STOP = __namespace + "/unlock_protective_stop"
        SRV_RESTART_SAFETY = __namespace + "/restart_safety"
        SRV_GET_ROBOT_MODE = __namespace + "/get_robot_mode"
        SRV_LOAD_INSTALLATION = __namespace + "/load_installation"
        SRV_LOAD_PROGRAM = __namespace + "/load_program"
        SRV_CLOSE_POPUP = __namespace + "/close_popup"
        SRV_GET_LOADED_PROGRAM = __namespace + "/get_loaded_program"
        SRV_PROGRAM_STATE = __namespace + "/program_state"
        SRV_PROGRAM_RUNNING = __namespace + "/program_running"
        SRV_PLAY = __namespace + "/play"
        SRV_STOP = __namespace + "/stop"

    class ControllerManager(str, Enum):
        __namespace = "/controller_manager"

        SRV_SWITCH_CONTROLLER = __namespace + "/switch_controller"
        SRV_LIST_CONTROLLERS = __namespace + "/list_controllers"
    
    class IOAndStatusController(str, Enum):
        __namespace = "/io_and_status_controller"

        SRV_SET_IO = __namespace + "/set_io"
        SRV_RESEND_ROBOT_PROGRAM = __namespace + "/resend_robot_program"
    
    class URConfigurationController(str, Enum):
        __namespace = "/ur_configuration_controller"

        SRV_GET_ROBOT_SOFTWARE_VERSION = __namespace + "/get_robot_software_version"

    URServiceType = DashboardClient | ControllerManager | IOAndStatusController | URConfigurationController

    _UR_SERVICE_MAP: dict[str, type] = {
        DashboardClient.SRV_POWER_ON: Trigger,
        DashboardClient.SRV_POWER_OFF: Trigger,
        DashboardClient.SRV_BRAKE_RELEASE: Trigger,
        DashboardClient.SRV_UNLOCK_PROTECTIVE_STOP: Trigger,
        DashboardClient.SRV_RESTART_SAFETY: Trigger,
        DashboardClient.SRV_GET_ROBOT_MODE: GetRobotMode,
        DashboardClient.SRV_LOAD_INSTALLATION: Load,
        DashboardClient.SRV_CLOSE_POPUP: Trigger,
        DashboardClient.SRV_GET_LOADED_PROGRAM: GetLoadedProgram,
        DashboardClient.SRV_PROGRAM_STATE: GetProgramState,
        DashboardClient.SRV_PROGRAM_RUNNING: IsProgramRunning,
        DashboardClient.SRV_PLAY: Trigger,
        DashboardClient.SRV_STOP: Trigger,
        ControllerManager.SRV_SWITCH_CONTROLLER: SwitchController,
        ControllerManager.SRV_LIST_CONTROLLERS: ListControllers,
        IOAndStatusController.SRV_SET_IO: SetIO,
        IOAndStatusController.SRV_RESEND_ROBOT_PROGRAM: Trigger,
        URConfigurationController.SRV_GET_ROBOT_SOFTWARE_VERSION: GetRobotSoftwareVersion
    }

    URServices: tuple[URServiceType] = tuple(_UR_SERVICE_MAP.keys())

    @classmethod
    def init_service(cls, node: Node, service: URServiceType, timeout: int) -> Client:
        ur_service_type = cls._UR_SERVICE_MAP.get(service)
        if ur_service_type is None:
            raise TypeError(f"{service} is not a valid service")
        
        client = node.create_client(ur_service_type, service.value)
        if not client.wait_for_service(timeout):
            raise TimeoutError(f"Timed out waiting for {service} to connect")
        
        return client