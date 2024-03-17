// Copyright 2022 ROBOTIS CO., LTD.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Author: Darby Lim

#include <array>
#include <chrono>
#include <cmath>
#include <memory>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

#include "open_manipulator_hardware/open_manipulator_system.hpp"

namespace robotis
{
namespace open_manipulator_hardware
{
auto logger = rclcpp::get_logger("open_manipulator");
hardware_interface::CallbackReturn OpenManipulatorManipulationSystemHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (
    hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  id_ = stoi(info_.hardware_parameters["manipulator_id"]);
  usb_port_ = info_.hardware_parameters["manipulator_usb_port"];
  usb_device_type_ = info_.hardware_parameters["manipulator_usb_device"];
  baud_rate_ = stoi(info_.hardware_parameters["manipulator_baud_rate"]);
  heartbeat_ = 0;

  joints_acceleration_[0] = stoi(info_.hardware_parameters["dxl_joints_profile_acceleration"]);
  joints_acceleration_[1] = stoi(info_.hardware_parameters["dxl_joints_profile_acceleration"]);
  joints_acceleration_[2] = stoi(info_.hardware_parameters["dxl_joints_profile_acceleration"]);
  joints_acceleration_[3] = stoi(info_.hardware_parameters["dxl_joints_profile_acceleration"]);

  joints_velocity_[0] = stoi(info_.hardware_parameters["dxl_joints_profile_velocity"]);
  joints_velocity_[1] = stoi(info_.hardware_parameters["dxl_joints_profile_velocity"]);
  joints_velocity_[2] = stoi(info_.hardware_parameters["dxl_joints_profile_velocity"]);
  joints_velocity_[3] = stoi(info_.hardware_parameters["dxl_joints_profile_velocity"]);

  gripper_acceleration_ = stoi(info_.hardware_parameters["dxl_gripper_profile_acceleration"]);
  gripper_velocity_ = stoi(info_.hardware_parameters["dxl_gripper_profile_velocity"]);

  if (usb_device_type_ == "opencr")
  {
    usbdevice_ = std::make_unique<OpenCR>(id_);
    RCLCPP_INFO(logger, "Created USB device of type '%s'", usb_device_type_.c_str());
  }
  else if (usb_device_type_ == "u2d2")
  {
    usbdevice_ = std::make_unique<U2d2>();
    RCLCPP_INFO(logger, "Created USB device of type '%s'", usb_device_type_.c_str());
  }
  else
  {
    RCLCPP_FATAL(logger, "USB device type %s should be either 'opencr' or 'u2d2'", usb_device_type_.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }
  
  if (usbdevice_->open_port(usb_port_, baud_rate_)) {
    RCLCPP_INFO(logger, "Succeeded to open port %s with baudrate %d", usb_port_.c_str(), baud_rate_);
  } else {
    RCLCPP_FATAL(logger, "Failed to open port %s with baudrate %d", usb_port_.c_str(), baud_rate_);
    return hardware_interface::CallbackReturn::ERROR;
  }

  int32_t model_number = usbdevice_->ping();
  RCLCPP_INFO(logger, "OpenCR Model Number %d", model_number);

  if (usbdevice_->is_connect_manipulator()) {
    RCLCPP_INFO(logger, "Connected manipulator");
  } else {
    RCLCPP_FATAL(logger, "Not connected manipulator");
    return hardware_interface::CallbackReturn::ERROR;
  }

  dxl_joint_commands_.resize(4, 0.0);
  dxl_joint_commands_[0] = 0.0;
  dxl_joint_commands_[1] = -1.57;
  dxl_joint_commands_[2] = 1.37;
  dxl_joint_commands_[3] = 0.26;

  dxl_gripper_commands_.resize(2, 0.0);

  dxl_positions_.resize(info_.joints.size(), 0.0);
  dxl_velocities_.resize(info_.joints.size(), 0.0);

  RCLCPP_INFO(logger, "Manipulator init successful");

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
OpenManipulatorManipulationSystemHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (uint8_t i = 0; i < info_.joints.size(); i++) {
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &dxl_positions_[i]));
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &dxl_velocities_[i]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
OpenManipulatorManipulationSystemHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  command_interfaces.emplace_back(
    hardware_interface::CommandInterface(
      info_.joints[2].name, hardware_interface::HW_IF_POSITION, &dxl_joint_commands_[0]));
  command_interfaces.emplace_back(
    hardware_interface::CommandInterface(
      info_.joints[3].name, hardware_interface::HW_IF_POSITION, &dxl_joint_commands_[1]));
  command_interfaces.emplace_back(
    hardware_interface::CommandInterface(
      info_.joints[4].name, hardware_interface::HW_IF_POSITION, &dxl_joint_commands_[2]));
  command_interfaces.emplace_back(
    hardware_interface::CommandInterface(
      info_.joints[5].name, hardware_interface::HW_IF_POSITION, &dxl_joint_commands_[3]));

  command_interfaces.emplace_back(
    hardware_interface::CommandInterface(
      info_.joints[6].name, hardware_interface::HW_IF_POSITION, &dxl_gripper_commands_[0]));
  command_interfaces.emplace_back(
    hardware_interface::CommandInterface(
      info_.joints[7].name, hardware_interface::HW_IF_POSITION, &dxl_gripper_commands_[1]));

  return command_interfaces;
}

hardware_interface::CallbackReturn OpenManipulatorManipulationSystemHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(logger, "Ready for start");
  usbdevice_->send_heartbeat(heartbeat_++);

  RCLCPP_INFO(logger, "Joints torque ON");
  usbdevice_->joints_torque(opencr::ON);

  usbdevice_->send_heartbeat(heartbeat_++);
  RCLCPP_INFO(logger, "Set profile acceleration and velocity to joints");
  usbdevice_->set_joint_profile_acceleration(joints_acceleration_);
  usbdevice_->set_joint_profile_velocity(joints_velocity_);

  RCLCPP_INFO(logger, "Set profile acceleration and velocity to gripper");
  usbdevice_->set_gripper_profile_acceleration(gripper_acceleration_);
  usbdevice_->set_gripper_profile_velocity(gripper_velocity_);

  RCLCPP_INFO(logger, "Set goal current value to gripper");
  usbdevice_->set_gripper_current();

  RCLCPP_INFO(logger, "System starting");
  usbdevice_->play_sound(opencr::SOUND::ASCENDING);

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn OpenManipulatorManipulationSystemHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(logger, "Ready for stop");
  usbdevice_->play_sound(opencr::SOUND::DESCENDING);

  RCLCPP_INFO(logger, "System stopped");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type OpenManipulatorManipulationSystemHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  RCLCPP_INFO_ONCE(logger, "Start to read manipulator states");

  if (usbdevice_->read_all() == false) {
    RCLCPP_WARN(logger, "Failed to read all control table");
  }
  dxl_positions_[2] = usbdevice_->get_joint_positions()[opencr::joints::JOINT1];
  dxl_velocities_[2] = usbdevice_->get_joint_velocities()[opencr::joints::JOINT1];

  dxl_positions_[3] = usbdevice_->get_joint_positions()[opencr::joints::JOINT2];
  dxl_velocities_[3] = usbdevice_->get_joint_velocities()[opencr::joints::JOINT2];

  dxl_positions_[4] = usbdevice_->get_joint_positions()[opencr::joints::JOINT3];
  dxl_velocities_[4] = usbdevice_->get_joint_velocities()[opencr::joints::JOINT3];

  dxl_positions_[5] = usbdevice_->get_joint_positions()[opencr::joints::JOINT4];
  dxl_velocities_[5] = usbdevice_->get_joint_velocities()[opencr::joints::JOINT4];

  dxl_positions_[6] = usbdevice_->get_gripper_position();
  dxl_velocities_[6] = usbdevice_->get_gripper_velocity();

  dxl_positions_[7] = usbdevice_->get_gripper_position();
  dxl_velocities_[7] = usbdevice_->get_gripper_velocity();

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type OpenManipulatorManipulationSystemHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  RCLCPP_INFO_ONCE(logger, "Start to write manipulator commands");
  usbdevice_->send_heartbeat(heartbeat_++);

  if (usbdevice_->set_joint_positions(dxl_joint_commands_) == false) {
    RCLCPP_ERROR(logger, "Can't control joints");
  }

  if (usbdevice_->set_gripper_position(dxl_gripper_commands_[0]) == false) {
    RCLCPP_ERROR(logger, "Can't control gripper");
  }

  return hardware_interface::return_type::OK;
}
}  // namespace open_manipulator_hardware
}  // namespace robotis

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  robotis::open_manipulator_hardware::OpenManipulatorManipulationSystemHardware,
  hardware_interface::SystemInterface)
