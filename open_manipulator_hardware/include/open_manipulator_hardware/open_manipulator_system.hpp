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

#ifndef open_manipulator_HARDWARE__open_manipulator_SYSTEM_HPP_
#define open_manipulator_HARDWARE__open_manipulator_SYSTEM_HPP_

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "open_manipulator_hardware/usb_device.hpp"
#include "open_manipulator_hardware/opencr.hpp"
#include "open_manipulator_hardware/u2d2.hpp"
#include "open_manipulator_hardware/visibility_control.h"

namespace robotis
{
namespace open_manipulator_hardware
{
class OpenManipulatorManipulationSystemHardware
  : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(OpenManipulatorManipulationSystemHardware);

  open_manipulator_HARDWARE_PUBLIC
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  open_manipulator_HARDWARE_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  open_manipulator_HARDWARE_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  open_manipulator_HARDWARE_PUBLIC
  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  open_manipulator_HARDWARE_PUBLIC
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  open_manipulator_HARDWARE_PUBLIC
  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  open_manipulator_HARDWARE_PUBLIC
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  uint8_t id_;
  std::string usb_port_;
  std::string usb_device_type_;
  uint32_t baud_rate_;
  uint8_t heartbeat_;

  std::array<int32_t, ARM_JOINTS> joint_profile_acceleration_;
  std::array<int32_t, ARM_JOINTS> joint_profile_velocity_;

  int32_t gripper_acceleration_;
  int32_t gripper_velocity_;

  std::unique_ptr<UsbDevice> usbdevice_;

  std::array<double, ARM_JOINTS> dxl_joint_state_positions_;
  std::array<double, ARM_JOINTS> dxl_joint_state_velocities_;
  std::array<double, ARM_JOINTS> dxl_joint_command_positions_;

  double dxl_gripper_state_position_;
  double dxl_gripper_state_velocity_;
  double dxl_gripper_command_position_;
};
}  // namespace open_manipulator_hardware
}  // namespace robotis
#endif  // open_manipulator_HARDWARE__open_manipulator_SYSTEM_HPP_
