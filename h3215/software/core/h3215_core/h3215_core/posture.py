"""Stands the robot up into its home pose, or relaxes it.

Services:
    ~/stand   (std_srvs/Trigger)  torque on if it was off, then a smooth move from the present
                                  pose to the home pose
    ~/relax   (std_srvs/Trigger)  torque off: the servos go limp and the robot sits down on
                                  its body

Parameters:
    joints, home     the leg joints and their home angles, in the leg controller's order
                     (generated from the model: h3215_description/config/joints.yaml)
    duration         seconds the move to the home pose takes (default 3.0)
    rate             commands per second during the move (default 50.0)
    hardware         the ros2_control hardware component to switch (default "h3215")
    stand_on_start   stand up as soon as the joint states arrive (default false)
"""

import rclpy
from controller_manager_msgs.srv import SetHardwareComponentState
from lifecycle_msgs.msg import State
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import Trigger

from .trajectory import interpolate

COMMAND_TOPIC = "/leg_position_controller/commands"
HARDWARE_SERVICE = "/controller_manager/set_hardware_component_state"


class Posture(Node):
    def __init__(self):
        super().__init__("posture")
        self.joints = list(self.declare_parameter("joints", [""]).value)
        self.home = list(self.declare_parameter("home", [0.0]).value)
        self.duration = self.declare_parameter("duration", 3.0).value
        rate = self.declare_parameter("rate", 50.0).value
        self.hardware = self.declare_parameter("hardware", "h3215").value
        self.stand_on_start = self.declare_parameter("stand_on_start", False).value
        if len(self.joints) != len(self.home) or not all(self.joints):
            raise ValueError("parameters joints and home must have the same length")

        self.present: dict[str, float] = {}
        self.move = None  # (start pose, start time) while moving to the home pose

        # Three callback groups, so nothing waits on itself: the services block until the
        # controller manager answers, its answer arrives in the client's group, and the joint
        # states and the move timer keep running in the default one
        services = MutuallyExclusiveCallbackGroup()
        client = MutuallyExclusiveCallbackGroup()
        self.command = self.create_publisher(Float64MultiArray, COMMAND_TOPIC, 10)
        self.create_subscription(JointState, "/joint_states", self.on_joint_states, 10)
        self.switch = self.create_client(
            SetHardwareComponentState, HARDWARE_SERVICE, callback_group=client
        )
        self.create_service(Trigger, "~/stand", self.on_stand, callback_group=services)
        self.create_service(Trigger, "~/relax", self.on_relax, callback_group=services)
        self.create_timer(1.0 / rate, self.step)

    def on_joint_states(self, msg: JointState):
        self.present.update(zip(msg.name, msg.position, strict=False))
        if self.stand_on_start and self.pose() is not None:
            self.stand_on_start = False
            self.start_move()

    def pose(self) -> list[float] | None:
        """The present angle of every joint, or None until all of them have been seen."""
        if not all(j in self.present for j in self.joints):
            return None
        return [self.present[j] for j in self.joints]

    def on_stand(self, _request, response):
        pose = self.pose()
        if pose is None:
            response.success, response.message = False, "no joint states yet"
            return response
        # The leg controller keeps sending its last command: make it the present pose before
        # torque comes on, so nothing jumps
        self.publish(pose)
        if not self.set_hardware(State.PRIMARY_STATE_ACTIVE, "active"):
            response.success, response.message = False, "cannot turn the servos on"
            return response
        self.start_move()
        response.success, response.message = True, f"standing up in {self.duration:g} s"
        return response

    def on_relax(self, _request, response):
        self.move = None
        if not self.set_hardware(State.PRIMARY_STATE_INACTIVE, "inactive"):
            response.success, response.message = False, "cannot turn the servos off"
            return response
        response.success, response.message = True, "servos off"
        return response

    def start_move(self):
        self.move = (self.pose(), self.get_clock().now())

    def step(self):
        if self.move is None:
            return
        start, t0 = self.move
        s = (self.get_clock().now() - t0).nanoseconds / 1e9 / self.duration
        self.publish(interpolate(start, self.home, s))
        if s >= 1.0:
            self.move = None
            self.get_logger().info("standing in the home pose")

    def publish(self, pose: list[float]):
        self.command.publish(Float64MultiArray(data=pose))

    def set_hardware(self, state_id: int, label: str) -> bool:
        if not self.switch.wait_for_service(timeout_sec=2.0):
            self.get_logger().error(f"{HARDWARE_SERVICE} is not available")
            return False
        request = SetHardwareComponentState.Request()
        request.name = self.hardware
        request.target_state = State(id=state_id, label=label)
        result = self.switch.call(request)
        if not result.ok:
            self.get_logger().error(f"cannot set {self.hardware} {label}")
        return result.ok


def main():
    rclpy.init()
    node = Posture()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
