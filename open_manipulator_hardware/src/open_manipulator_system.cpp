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
#include <algorithm>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

#include "open_manipulator_hardware/open_manipulator_system.hpp"

namespace robotis
{
  namespace open_manipulator_hardware
  {
    auto logger = rclcpp::get_logger("open_manipulator");
    hardware_interface::CallbackReturn OpenManipulatorManipulationSystemHardware::on_init(
        const hardware_interface::HardwareInfo &info)
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

      for (size_t i = 0; i < ARM_JOINTS; i++)
      {
        joint_profile_acceleration_[i] = stoi(info_.hardware_parameters["dxl_joints_profile_acceleration"]);
        joint_profile_velocity_[i] = stoi(info_.hardware_parameters["dxl_joints_profile_velocity"]);
      }

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

      if (usbdevice_->open_port(usb_port_, baud_rate_))
      {
        RCLCPP_INFO(logger, "Succeeded to open port %s with baudrate %d", usb_port_.c_str(), baud_rate_);
      }
      else
      {
        RCLCPP_FATAL(logger, "Failed to open port %s with baudrate %d", usb_port_.c_str(), baud_rate_);
        return hardware_interface::CallbackReturn::ERROR;
      }

      int32_t model_number = usbdevice_->ping();
      RCLCPP_INFO(logger, "OpenCR Model Number %d", model_number);

      if (usbdevice_->is_connect_manipulator())
      {
        RCLCPP_INFO(logger, "Connected manipulator");
      }
      else
      {
        RCLCPP_FATAL(logger, "Not connected manipulator");
        return hardware_interface::CallbackReturn::ERROR;
      }

      for (const hardware_interface::ComponentInfo &joint : info_.joints)
      {
        // Open manipulator has exactly two states and one command interface on each joint
        if (joint.command_interfaces.size() != 1)
        {
          RCLCPP_FATAL(
              rclcpp::get_logger("OpenManipulatorManipulationSystemHardware"),
              "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
              joint.command_interfaces.size());
          return hardware_interface::CallbackReturn::ERROR;
        }

        if (joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
        {
          RCLCPP_FATAL(
              rclcpp::get_logger("OpenManipulatorManipulationSystemHardware"),
              "Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),
              joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY);
          return hardware_interface::CallbackReturn::ERROR;
        }

        if (joint.state_interfaces.size() != 2)
        {
          RCLCPP_FATAL(
              rclcpp::get_logger("OpenManipulatorManipulationSystemHardware"),
              "Joint '%s' has %zu state interface. 2 expected.", joint.name.c_str(),
              joint.state_interfaces.size());
          return hardware_interface::CallbackReturn::ERROR;
        }

        if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
        {
          RCLCPP_FATAL(
              rclcpp::get_logger("OpenManipulatorManipulationSystemHardware"),
              "Joint '%s' have '%s' as first state interface. '%s' expected.", joint.name.c_str(),
              joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
          return hardware_interface::CallbackReturn::ERROR;
        }

        if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
        {
          RCLCPP_FATAL(
              rclcpp::get_logger("OpenManipulatorManipulationSystemHardware"),
              "Joint '%s' have '%s' as second state interface. '%s' expected.", joint.name.c_str(),
              joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
          return hardware_interface::CallbackReturn::ERROR;
        }
      }

      RCLCPP_INFO(logger, "Manipulator init successful");

      return hardware_interface::CallbackReturn::SUCCESS;
    }

    std::vector<hardware_interface::StateInterface>
    OpenManipulatorManipulationSystemHardware::export_state_interfaces()
    {
      std::vector<hardware_interface::StateInterface> state_interfaces;

      for (size_t i = 0; i < ARM_JOINTS; i++)
      {
        RCLCPP_INFO(logger, "Exporting state interfaces for %s", info_.joints[i].name.c_str());
        state_interfaces.emplace_back(
            hardware_interface::StateInterface(
                info_.joints[i].name, hardware_interface::HW_IF_POSITION, &dxl_joint_state_positions_[i]));
        state_interfaces.emplace_back(
            hardware_interface::StateInterface(
                info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &dxl_joint_state_velocities_[i]));
      }

      for (size_t i = ARM_JOINTS; i < ARM_JOINTS + GRIPPER_JOINTS_ROS; i++)
      {
        // use single gripper state for both gripper joints
        RCLCPP_INFO(logger, "Exporting state interfaces for %s", info_.joints[i].name.c_str());
        state_interfaces.emplace_back(
            hardware_interface::StateInterface(
                info_.joints[i].name, hardware_interface::HW_IF_POSITION, &dxl_gripper_state_position_));
        state_interfaces.emplace_back(
            hardware_interface::StateInterface(
                info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &dxl_gripper_state_velocity_));
      }

      return state_interfaces;
    }

    std::vector<hardware_interface::CommandInterface>
    OpenManipulatorManipulationSystemHardware::export_command_interfaces()
    {
      std::vector<hardware_interface::CommandInterface> command_interfaces;

      for (size_t i = 0; i < ARM_JOINTS; i++)
      {
        RCLCPP_INFO(logger, "Exporting command interfaces for %s", info_.joints[i].name.c_str());
        command_interfaces.emplace_back(
            hardware_interface::CommandInterface(
                info_.joints[i].name, hardware_interface::HW_IF_POSITION, &dxl_joint_command_positions_[i]));
      }

      for (size_t i = ARM_JOINTS; i < ARM_JOINTS + GRIPPER_JOINTS_ROS; i++)
      {
        // use single gripper command position for both gripper joints
        RCLCPP_INFO(logger, "Exporting command interfaces for %s", info_.joints[i].name.c_str());
        command_interfaces.emplace_back(
            hardware_interface::CommandInterface(
                info_.joints[i].name, hardware_interface::HW_IF_POSITION, &dxl_gripper_command_position_));
      }

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
      usbdevice_->set_joint_profile_acceleration(joint_profile_acceleration_);
      usbdevice_->set_joint_profile_velocity(joint_profile_velocity_);

      RCLCPP_INFO(logger, "Set profile acceleration and velocity to gripper");
      usbdevice_->set_gripper_profile_acceleration(gripper_acceleration_);
      usbdevice_->set_gripper_profile_velocity(gripper_velocity_);
      usbdevice_->set_gripper_current();

      RCLCPP_INFO(logger, "System starting");
      usbdevice_->play_sound(opencr::SOUND::ASCENDING);

      // set goal to current state, so the arm won't move
      usbdevice_->get_joint_positions(dxl_joint_command_positions_);
      usbdevice_->get_gripper_position(dxl_gripper_command_position_);

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

      if (usbdevice_->read_all() == false)
      {
        RCLCPP_WARN(logger, "Failed to read all control table");
      }

      usbdevice_->get_joint_positions(dxl_joint_state_positions_);
      usbdevice_->get_joint_velocities(dxl_joint_state_velocities_);

      size_t joint_i;
      for (joint_i = 0; joint_i < ARM_JOINTS; joint_i++)
      {
        RCLCPP_INFO(logger, "Joint name: %s current pos: %f vel: %f", info_.joints[joint_i].name.c_str(), dxl_joint_state_positions_[joint_i], dxl_joint_state_velocities_[joint_i]);
      }

      usbdevice_->get_gripper_position(dxl_gripper_state_position_);
      usbdevice_->get_gripper_velocity(dxl_gripper_state_velocity_);

      joint_i++;
      RCLCPP_INFO(logger, "Gripper name: %s current pos: %f vel: %f", info_.joints[joint_i].name.c_str(), dxl_gripper_state_position_, dxl_gripper_state_velocity_);

      return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type OpenManipulatorManipulationSystemHardware::write(
        const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
    {
      RCLCPP_INFO_ONCE(logger, "Start to write manipulator commands");

      usbdevice_->send_heartbeat(heartbeat_++);

      if (!usbdevice_->set_joint_positions(dxl_joint_command_positions_))
      {
        RCLCPP_ERROR(logger, "Can't control joints");
      }

      size_t joint_i;
      for (joint_i = 0; joint_i < ARM_JOINTS; joint_i++)
      {
        RCLCPP_INFO(logger, "Joint name: %s goal pos: %f", info_.joints[joint_i].name.c_str(), dxl_joint_command_positions_[joint_i]);
      }

      if (!usbdevice_->set_gripper_position(dxl_gripper_command_position_))
      {
        RCLCPP_ERROR(logger, "Can't control gripper");
      }

      joint_i++;
      RCLCPP_INFO(logger, "Gripper name: %s goal pos: %f", info_.joints[joint_i].name.c_str(), dxl_gripper_command_position_);

      return hardware_interface::return_type::OK;
    }
  } // namespace open_manipulator_hardware
} // namespace robotis

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
    robotis::open_manipulator_hardware::OpenManipulatorManipulationSystemHardware,
    hardware_interface::SystemInterface)
