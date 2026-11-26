"""Battery state and servo health, from the servos' own readings.

Every servo measures its supply voltage and its temperature and reports error flags. The
hardware passes them on as state interfaces, and joint_state_broadcaster publishes them on
/dynamic_joint_states.

Publishes, once per `period`:
    /battery       (sensor_msgs/BatteryState)       pack voltage (median over the servos) and
                                                    an estimate of the charge
    /diagnostics   (diagnostic_msgs/DiagnosticArray) one status for the battery, one per servo

Parameters:
    cells                   Li-ion cells in series (default 3)
    cell_warn, cell_error   V per cell below which the battery is low / empty (3.4, 3.2)
    temperature_warn        degrees C above which a servo is hot (default 60)
    temperature_error       degrees C above which it is too hot (default 70)
    period                  seconds between reports (default 1.0)
"""

import statistics

import rclpy
from control_msgs.msg import DynamicJointState
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.node import Node
from sensor_msgs.msg import BatteryState

from .battery import charge

OK, WARN, ERROR = DiagnosticStatus.OK, DiagnosticStatus.WARN, DiagnosticStatus.ERROR


class Monitor(Node):
    def __init__(self):
        super().__init__("monitor")
        self.cells = self.declare_parameter("cells", 3).value
        self.cell_warn = self.declare_parameter("cell_warn", 3.4).value
        self.cell_error = self.declare_parameter("cell_error", 3.2).value
        self.temperature_warn = self.declare_parameter("temperature_warn", 60.0).value
        self.temperature_error = self.declare_parameter("temperature_error", 70.0).value
        period = self.declare_parameter("period", 1.0).value

        self.servos: dict[str, dict[str, float]] = {}  # joint -> interface -> value
        self.create_subscription(DynamicJointState, "/dynamic_joint_states", self.on_state, 10)
        self.battery = self.create_publisher(BatteryState, "/battery", 10)
        self.diagnostics = self.create_publisher(DiagnosticArray, "/diagnostics", 10)
        self.create_timer(period, self.report)

    def on_state(self, msg: DynamicJointState):
        for joint, values in zip(msg.joint_names, msg.interface_values, strict=False):
            self.servos[joint] = dict(zip(values.interface_names, values.values, strict=False))

    def report(self):
        voltages = [s["voltage"] for s in self.servos.values() if s.get("voltage", 0.0) > 0.0]
        if not voltages:
            return
        now = self.get_clock().now().to_msg()
        voltage = statistics.median(voltages)
        battery = BatteryState()
        battery.header.stamp = now
        battery.voltage = voltage
        battery.percentage = charge(voltage, self.cells)
        battery.present = True
        battery.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_LION
        battery.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
        battery.cell_voltage = [voltage / self.cells] * self.cells  # the servos see the pack only
        self.battery.publish(battery)

        array = DiagnosticArray()
        array.header.stamp = now
        array.status.append(self.battery_status(voltage, battery.percentage))
        array.status += [self.servo_status(j, s) for j, s in self.servos.items()]
        self.diagnostics.publish(array)

    def battery_status(self, voltage: float, percentage: float) -> DiagnosticStatus:
        cell = voltage / self.cells
        if cell < self.cell_error:
            level, message = ERROR, "empty: charge it"
        elif cell < self.cell_warn:
            level, message = WARN, "low"
        else:
            level, message = OK, "ok"
        if level != OK:
            self.get_logger().warn(
                f"battery {message}: {voltage:.1f} V", throttle_duration_sec=30.0
            )
        return DiagnosticStatus(
            level=level,
            name="h3215: battery",
            message=message,
            values=[
                KeyValue(key="voltage", value=f"{voltage:.1f}"),
                KeyValue(key="charge", value=f"{100 * percentage:.0f} %"),
            ],
        )

    def servo_status(self, joint: str, state: dict[str, float]) -> DiagnosticStatus:
        temperature = state.get("temperature", 0.0)
        flags = int(state.get("status", 0.0))
        if flags:
            level, message = ERROR, f"error flags 0x{flags:02x}"
        elif temperature > self.temperature_error:
            level, message = ERROR, "too hot"
        elif temperature > self.temperature_warn:
            level, message = WARN, "hot"
        else:
            level, message = OK, "ok"
        if level != OK:
            self.get_logger().warn(
                f"{joint}: {message} ({temperature:.0f} C)", throttle_duration_sec=10.0
            )
        return DiagnosticStatus(
            level=level,
            name=f"h3215: servo {joint}",
            message=message,
            values=[
                KeyValue(key="temperature", value=f"{temperature:.0f}"),
                KeyValue(key="voltage", value=f"{state.get('voltage', 0.0):.1f}"),
                KeyValue(key="error flags", value=f"0x{flags:02x}"),
            ],
        )


def main():
    rclpy.init()
    node = Monitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
