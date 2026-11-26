#include "h3215_hardware/sts_system.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "h3215_hardware/sts_protocol.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"

namespace h3215_hardware
{

namespace
{

constexpr double kRadPerStep = 2.0 * M_PI / sts::kStepsPerTurn;

std::string param(
  const std::unordered_map<std::string, std::string> & params, const std::string & key,
  const std::string & fallback)
{
  const auto it = params.find(key);
  return it == params.end() ? fallback : it->second;
}

}  // namespace

hardware_interface::CallbackReturn StsSystem::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & params)
{
  if (SystemInterface::on_init(params) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }
  const auto & hw = info_.hardware_parameters;
  try {
    port_ = param(hw, "port", "/dev/ttyACM0");
    baudrate_ = std::stoi(param(hw, "baudrate", "1000000"));
    stall_torque_ = std::stod(param(hw, "stall_torque", "2.94"));
    max_missed_cycles_ = std::stoi(param(hw, "max_missed_cycles", "5"));
    bus_.set_timeout(std::chrono::microseconds(std::stoi(param(hw, "timeout_us", "5000"))));

    for (const auto & info : info_.joints) {
      Joint joint;
      joint.name = info.name;
      joint.id = static_cast<uint8_t>(std::stoi(param(info.parameters, "id", "")));
      joint.offset = std::stod(param(info.parameters, "offset", "0"));
      joint.direction = std::stoi(param(info.parameters, "direction", "1"));
      if (joint.direction != 1 && joint.direction != -1) {
        RCLCPP_ERROR(get_logger(), "joint %s: direction must be 1 or -1", info.name.c_str());
        return CallbackReturn::ERROR;
      }
      if (info.command_interfaces.size() != 1 ||
        info.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
      {
        RCLCPP_ERROR(get_logger(), "joint %s: needs one position command", info.name.c_str());
        return CallbackReturn::ERROR;
      }
      joints_.push_back(joint);
      ids_.push_back(joint.id);
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "bad parameter: %s (every joint needs an id)", e.what());
    return CallbackReturn::ERROR;
  }

  const size_t n = joints_.size();
  state_bytes_.resize(n * sts::kStateSize);
  errors_.resize(n);
  received_ = std::make_unique<bool[]>(n);
  goal_bytes_.resize(n * 2);
  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn StsSystem::on_configure(const rclcpp_lifecycle::State &)
{
  for (size_t k = 0; k < joints_.size(); ++k) {
    auto & joint = joints_[k];
    auto optional = [&](const std::string & name) {
      for (const auto & state : info_.joints[k].state_interfaces) {
        if (state.name == name) {
          return get_state_interface_handle(joint.name + "/" + name);
        }
      }
      return hardware_interface::StateInterface::SharedPtr();
    };
    joint.voltage = optional("voltage");
    joint.temperature = optional("temperature");
    joint.status = optional("status");
    joint.position = get_state_interface_handle(joint.name + "/" + hardware_interface::HW_IF_POSITION);
    joint.velocity = get_state_interface_handle(joint.name + "/" + hardware_interface::HW_IF_VELOCITY);
    joint.effort = get_state_interface_handle(joint.name + "/" + hardware_interface::HW_IF_EFFORT);
    joint.command = get_command_interface_handle(joint.name + "/" + hardware_interface::HW_IF_POSITION);
  }

  const std::string error = bus_.open(port_, baudrate_);
  if (!error.empty()) {
    RCLCPP_ERROR(get_logger(), "%s", error.c_str());
    return CallbackReturn::ERROR;
  }
  std::string missing;
  for (const auto & joint : joints_) {
    if (!bus_.ping(joint.id)) {
      missing += " " + joint.name + " (id " + std::to_string(joint.id) + ")";
    }
  }
  if (!missing.empty()) {
    RCLCPP_ERROR(get_logger(), "no reply from:%s", missing.c_str());
    bus_.close();
    return CallbackReturn::ERROR;
  }
  if (!read_bus()) {
    RCLCPP_ERROR(get_logger(), "cannot read the servos' state");
    bus_.close();
    return CallbackReturn::ERROR;
  }
  RCLCPP_INFO(get_logger(), "%zu servos on %s", joints_.size(), port_.c_str());
  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn StsSystem::on_cleanup(const rclcpp_lifecycle::State &)
{
  bus_.close();
  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn StsSystem::on_activate(const rclcpp_lifecycle::State &)
{
  // Goal = where each servo is now, then power on: nothing moves until a controller commands it
  if (!read_bus()) {
    RCLCPP_ERROR(get_logger(), "cannot read the servos' state");
    return CallbackReturn::ERROR;
  }
  for (auto & joint : joints_) {
    double position = 0.0;
    std::ignore = get_state(joint.position, position, true);
    std::ignore = set_command(joint.command, position, true);
  }
  if (write(rclcpp::Time(), rclcpp::Duration(0, 0)) != hardware_interface::return_type::OK ||
    !set_torque(true))
  {
    RCLCPP_ERROR(get_logger(), "cannot enable the servos");
    return CallbackReturn::ERROR;
  }
  missed_cycles_ = 0;
  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn StsSystem::on_deactivate(const rclcpp_lifecycle::State &)
{
  return set_torque(false) ? CallbackReturn::SUCCESS : CallbackReturn::ERROR;
}

hardware_interface::return_type StsSystem::read(const rclcpp::Time &, const rclcpp::Duration &)
{
  if (read_bus()) {
    missed_cycles_ = 0;
    return hardware_interface::return_type::OK;
  }
  // Keep the last state through a few bad cycles, then stop: the controllers would be blind
  if (++missed_cycles_ > max_missed_cycles_) {
    RCLCPP_ERROR(get_logger(), "servos missing for %d cycles in a row", missed_cycles_);
    return hardware_interface::return_type::ERROR;
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type StsSystem::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  for (size_t k = 0; k < joints_.size(); ++k) {
    double angle = std::numeric_limits<double>::quiet_NaN();
    std::ignore = get_command(joints_[k].command, angle, false);
    if (!std::isfinite(angle)) {
      std::ignore = get_state(joints_[k].position, angle, false);  // hold position
    }
    const int steps = to_steps(joints_[k], angle);
    goal_bytes_[2 * k] = static_cast<uint8_t>(steps & 0xFF);
    goal_bytes_[2 * k + 1] = static_cast<uint8_t>(steps >> 8);
  }
  return bus_.sync_write(ids_, sts::kGoalPosition, 2, goal_bytes_.data())
           ? hardware_interface::return_type::OK
           : hardware_interface::return_type::ERROR;
}

bool StsSystem::read_bus()
{
  const size_t n = joints_.size();
  const size_t count = bus_.sync_read(
    ids_, sts::kPresentPosition, sts::kStateSize, state_bytes_.data(), errors_.data(),
    received_.get());
  for (size_t k = 0; k < n; ++k) {
    if (!received_[k]) {
      continue;
    }
    const auto & joint = joints_[k];
    const sts::State s = sts::decode_state(state_bytes_.data() + k * sts::kStateSize);
    std::ignore = set_state(joint.position, to_angle(joint, s.position), false);
    std::ignore = set_state(joint.velocity, joint.direction * s.speed * kRadPerStep, false);
    std::ignore = set_state(joint.effort, joint.direction * s.load / 1000.0 * stall_torque_, false);
    if (joint.voltage) {
      std::ignore = set_state(joint.voltage, s.voltage, false);
    }
    if (joint.temperature) {
      std::ignore = set_state(joint.temperature, static_cast<double>(s.temperature), false);
    }
    if (joint.status) {
      std::ignore = set_state(joint.status, static_cast<double>(errors_[k]), false);
    }
    if (errors_[k] != 0) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000, "%s: servo error flags 0x%02x", joint.name.c_str(),
        errors_[k]);
    }
  }
  return count == n;
}

bool StsSystem::set_torque(bool on)
{
  std::vector<uint8_t> values(joints_.size(), on ? 1 : 0);
  return bus_.sync_write(ids_, sts::kTorqueEnable, 1, values.data());
}

double StsSystem::to_angle(const Joint & joint, double steps) const
{
  return joint.direction * (steps - sts::kCenterStep) * kRadPerStep + joint.offset;
}

int StsSystem::to_steps(const Joint & joint, double angle) const
{
  const double steps = sts::kCenterStep + joint.direction * (angle - joint.offset) / kRadPerStep;
  return std::clamp(static_cast<int>(std::lround(steps)), 0, sts::kStepsPerTurn - 1);
}

}  // namespace h3215_hardware

PLUGINLIB_EXPORT_CLASS(h3215_hardware::StsSystem, hardware_interface::SystemInterface)
