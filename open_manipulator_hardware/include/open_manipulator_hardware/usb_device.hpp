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

#ifndef open_manipulator_HARDWARE__USBDEVICE_HPP_
#define open_manipulator_HARDWARE__USBDEVICE_HPP_

#include <stdlib.h>
#include <array>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "open_manipulator_hardware/dynamixel_sdk_wrapper.hpp"
#include "open_manipulator_hardware/opencr_control_table.hpp"
#include "open_manipulator_hardware/opencr_definitions.hpp"

namespace robotis
{
  namespace open_manipulator_hardware
  {

    static constexpr size_t ARM_JOINTS = 4;
    // there are 2 gripper joints controlled from 1 dynamixel
    static constexpr size_t GRIPPER_JOINTS_DXL = 1;
    static constexpr size_t GRIPPER_JOINTS_ROS = 2;

    class UsbDevice
    {
    public:
      virtual ~UsbDevice() {};

      virtual bool open_port(const std::string &usb_port, const uint32_t &baud_rate) = 0;

      virtual uint16_t ping() = 0;

      virtual bool is_connect_manipulator() = 0;

      virtual void play_sound(uint8_t sound) const = 0;

      virtual void joints_torque(uint8_t onoff) const = 0;

      virtual bool read_all() = 0;

      virtual void get_joint_positions(std::array<double, ARM_JOINTS> &joint_positions) = 0;
      virtual void get_joint_velocities(std::array<double, ARM_JOINTS> &joint_velocities) = 0;
      virtual bool set_joint_positions(std::array<double, ARM_JOINTS> &joint_goals) = 0;
      virtual bool set_joint_profile_acceleration(
          const std::array<int32_t, ARM_JOINTS> &acceleration) = 0;
      virtual bool set_joint_profile_velocity(const std::array<int32_t, ARM_JOINTS> &velocity) = 0;

      virtual void get_gripper_position(double &meter) = 0;
      virtual void get_gripper_velocity(double &meter_per_second) = 0;
      virtual bool set_gripper_position(double meter) = 0;
      virtual bool set_gripper_profile_acceleration(const int32_t &acceleration) = 0;
      virtual bool set_gripper_profile_velocity(const int32_t &velocity) = 0;

      virtual bool set_home_pose() = 0;
      virtual bool set_init_pose() = 0;
      virtual bool set_zero_pose() = 0;

      virtual bool set_gripper_current() = 0;

      virtual bool open_gripper() = 0;
      virtual bool close_gripper() = 0;
      virtual bool init_gripper() = 0;

      virtual void send_heartbeat(const uint8_t &count) = 0;
    };
  } // namespace open_manipulator_hardware
} // namespace robotis
#endif // open_manipulator_HARDWARE__USBDEVICE_HPP_
