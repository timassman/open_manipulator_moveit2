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
class UsbDevice
{
public:
  virtual ~UsbDevice() { };

  virtual int32_t get_data(const uint16_t & address, const uint16_t & length) = 0;

  virtual bool open_port(const std::string & usb_port) = 0;
  virtual bool set_baud_rate(const uint32_t & baud_rate) = 0;

  virtual uint16_t ping() = 0;

  virtual bool is_connect_manipulator() = 0;

  virtual void play_sound(uint8_t sound) const = 0;

  virtual void joints_torque(uint8_t onoff) const = 0;

  virtual bool read_all() = 0;

  virtual std::array<double, 4> get_joint_positions() = 0;
  virtual std::array<double, 4> get_joint_velocities() = 0;

  virtual double get_gripper_position() = 0;
  virtual double get_gripper_velocity() = 0;

  virtual bool set_joint_positions(const std::vector<double> & radians) = 0;
  virtual bool set_joint_profile_acceleration(
    const std::array<int32_t, 4> & acceleration) = 0;
  virtual bool set_joint_profile_velocity(const std::array<int32_t, 4> & velocity) = 0;

  virtual bool set_gripper_position(const double & meters) = 0;
  virtual bool set_gripper_profile_acceleration(const int32_t & acceleration) = 0;
  virtual bool set_gripper_profile_velocity(const int32_t & velocity) = 0;

  virtual bool set_home_pose() = 0;
  virtual bool set_init_pose() = 0;
  virtual bool set_zero_pose() = 0;

  virtual bool set_gripper_current() = 0;

  virtual bool open_gripper() = 0;
  virtual bool close_gripper() = 0;
  virtual bool init_gripper() = 0;

  virtual void send_heartbeat(const uint8_t & count) = 0;

  virtual void write_byte(const uint16_t & address, uint8_t data) = 0;
  virtual uint8_t read_byte(const uint16_t & address) = 0;
};
}  // namespace open_manipulator_hardware
}  // namespace robotis
#endif  // open_manipulator_HARDWARE__USBDEVICE_HPP_
