import sys
import threading

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor, MultiThreadedExecutor

from PyQt5.QtWidgets import *

from ur_control_qt import URControlQtWindow
from ur10e_configs import UR_QOS_PROFILE
from sensor_msgs.msg import JointState
from builtin_interfaces.msg import Duration

from typing import Callable
from functools import partial

class URExerciseControlWindow(URControlQtWindow):
    def __init__(self, node: Node):
        super().__init__(node)

        self.service_tab = self._create_tab(name = "Exercise Tab", layout = QVBoxLayout(), tab_create_func = self.__conf_exercise_tab)
        self._state_lock = threading.RLock()

        self._state_reception_counter = 0
        self._exercise_traj_waypoints: list[list[float]] = []
        self._exercise_traj_time: list[Duration] = []

    def __conf_exercise_tab(self, layout: QLayout) -> None:
        def _start_freedrive_exercise_trajectory(_):
            # TODO: set freedrive mode
            # TODO: verify we are in freedrive mode???

            assert self.create_subscriber(msg_type = JointState, topic = "/joint_states", callback = self._update_state, qos_profile = UR_QOS_PROFILE), \
                "Unable to create joint position subscriber"
            
            self._state_lock.acquire()
            self._exercise_traj_waypoints.clear()
            self._exercise_traj_time.clear()
            self._state_reception_counter = 0
            self._state_lock.release()

        def _stop_freedrive_exercise_trajectory(_):
            # TODO: this crashes the program, idk why
            # Remove subscriber since we don't need it
            # self.remove_subscriber("/joint_states")

            # Subtract initial timestamp
            self._state_lock.acquire()
            self._state_reception_counter = -1
            self._state_lock.release()

            print("Trajectory")
            for w, d in zip(self._exercise_traj_waypoints, self._exercise_traj_time):
                print(f"Duration: {d.sec + d.nanosec*1e-9}s, pos: {w}")

        def _move_to_start(_):
            # 10s trajectory to get to home pose
            self._robot.send_trajectory(
                waypts = [self._exercise_traj_waypoints[0]],
                time_vec = [Duration(sec = 10)]
            )

        def _run_exercise_trajectory(_):
            if self._robot is None: return
            if len(self._exercise_traj_waypoints) > 1:
                self._robot.send_trajectory(
                    waypts = self._exercise_traj_waypoints[1:], 
                    time_vec = self._exercise_traj_time[1:],
                    blocking = True
                )
            else:
                self._node.get_logger().warning("No waypoints configured")

        def _save_exercise_trajectory(_):
            pass

            # Launch tab
        _launch_map: dict[QPushButton, Callable] = {
            QPushButton("START FREEDRIVE EXERCISE", self): _start_freedrive_exercise_trajectory,
            QPushButton("STOP FREEDRIVE EXERCISE", self): _stop_freedrive_exercise_trajectory,
            QPushButton("MOVE TO START", self): _move_to_start,
            QPushButton("RUN EXERCISE TRAJECTORY", self): _run_exercise_trajectory,
            QPushButton("SAVE TRAJECTORY", self): _save_exercise_trajectory
        }

        for button, callback_func in _launch_map.items():
            button.clicked.connect(partial(callback_func, button))
            layout.addWidget(button)

    def _update_state(self, msg: JointState):
        self._state_lock.acquire()

        if self._state_reception_counter == -1:
            # "Invalid" counter, somewhat hacky...
            self._state_lock.release()
            return

        # Add 1 waypoint / second
        if self._state_reception_counter % 500 == 0:
            print(f"Adding {msg.position} at {msg.header.stamp}")
            self._exercise_traj_waypoints.append(msg.position)
            if len(self._exercise_traj_time) > 0:
                _time_f64 = (msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9) - \
                (self._exercise_traj_time[0].sec + self._exercise_traj_time[0].nanosec*1e-9)
                self._exercise_traj_time.append(
                    Duration(sec = int(_time_f64), nanosec = int((_time_f64 - int(_time_f64)) * 1e9))
                )
            else:
                self._exercise_traj_time.append(
                    Duration(sec = msg.header.stamp.sec, nanosec = msg.header.stamp.nanosec)
                )
        
        self._state_reception_counter += 1
        self._state_lock.release()

def main():
    # Create the application
    app = QApplication(sys.argv)

    node = Node("ur_custom_node")
    
    # Create the main window
    main_window = URExerciseControlWindow(node)
    main_window.show()

    executor = SingleThreadedExecutor()
    executor.add_node(node)
    executor.add_node(main_window._robot._node)

    threading.Thread(target = executor.spin, daemon = False).start()
    
    # Run the application's event loop
    sys.exit(app.exec_())