// ros2_control system for a bus of Feetech STS servos, one per joint.
//
// Hardware parameters: port, baudrate, stall_torque (N*m, to turn the load reading into an
// effort estimate), timeout_us (per reply, default 5000), max_missed_cycles (default 5).
// Joint parameters: id, offset (rad, default 0), direction (1 or -1, default 1):
//   angle = direction * (steps - 2048) * 2 pi / 4096 + offset
// Interfaces: position command; position, velocity and effort states. The effort comes from
// the load reading (a PWM duty cycle) and is only an estimate. Optional states, published by
// joint_state_broadcaster on /dynamic_joint_states: voltage (V), temperature (degrees C) and
// status (the servo's error flags, 0 = no error).

#pragma once

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "h3215_hardware/sts_bus.hpp"
#include "hardware_interface/system_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace h3215_hardware
{

class StsSystem : public hardware_interface::SystemInterface
{
public:
  CallbackReturn on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params) override;
  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  struct Joint
  {
    std::string name;
    uint8_t id;
    double offset;
    int direction;
    hardware_interface::StateInterface::SharedPtr position;
    hardware_interface::StateInterface::SharedPtr velocity;
    hardware_interface::StateInterface::SharedPtr effort;
    hardware_interface::StateInterface::SharedPtr voltage;  // the optional ones may be null
    hardware_interface::StateInterface::SharedPtr temperature;
    hardware_interface::StateInterface::SharedPtr status;
    hardware_interface::CommandInterface::SharedPtr command;
  };

  double to_angle(const Joint & joint, double steps) const;
  int to_steps(const Joint & joint, double angle) const;
  bool set_torque(bool on);
  bool read_bus();

  StsBus bus_;
  std::string port_;
  int baudrate_ = 1000000;
  double stall_torque_ = 2.94;
  int max_missed_cycles_ = 5;
  int missed_cycles_ = 0;

  std::vector<Joint> joints_;
  std::vector<uint8_t> ids_;
  // Bus buffers, sized once in on_init
  std::vector<uint8_t> state_bytes_;
  std::vector<uint8_t> errors_;
  std::unique_ptr<bool[]> received_;
  std::vector<uint8_t> goal_bytes_;
};

}  // namespace h3215_hardware
