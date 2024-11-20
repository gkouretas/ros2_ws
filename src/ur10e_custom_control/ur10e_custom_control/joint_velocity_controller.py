from controller_base import URController
from ur10e_custom_control.ur10e_configs import (
    URControlModes,
    UR_QOS_PROFILE
)

class JointVelocityController(URController):
    def __init__(self, name: str) -> None:
        super().__init__(name, URControlModes.FORWARD_VELOCITY)
        self._velocity_publisher = self.create_publisher(self._control_name.publish_topic, self._control_name.publish_topic_name, UR_QOS_PROFILE)
        self._velocity = [3.1415, 3.1415, 3.1415, 3.1415, 3.1415, 3.1415]
        self._counter = 0

    def compute(self):
        self._counter += 1
        if self._counter > 100:
            self._velocity = [-x for x in self._velocity]
            self._counter = 0

        return True

    def publish(self):
        msg = self._control_name.publish_topic()
        msg.data = self._velocity
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self._velocity_publisher.publish(msg)

    def reset(self):
        self._velocity = [0.0 for _ in self._velocity]

def main():
    import threading
    import rclpy

    controller = JointVelocityController("ur_custom_controller")
    threading.Thread(target = controller.run, daemon = True).start()
    rclpy.spin(controller)
    controller.stop()