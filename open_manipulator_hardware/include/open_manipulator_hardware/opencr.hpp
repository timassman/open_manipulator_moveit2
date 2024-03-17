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

#ifndef open_manipulator_HARDWARE__OPENCR_HPP_
#define open_manipulator_HARDWARE__OPENCR_HPP_

#include <stdlib.h>
#include <array>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "open_manipulator_hardware/dynamixel_sdk_wrapper.hpp"
#include "open_manipulator_hardware/opencr_control_table.hpp"
#include "open_manipulator_hardware/opencr_definitions.hpp"

#include "open_manipulator_hardware/usb_device.hpp"

namespace robotis
{
namespace open_manipulator_hardware
{
class OpenCR : public UsbDevice
{
public:
  explicit OpenCR(const uint8_t & id);
  virtual ~OpenCR();

  int32_t get_data(const uint16_t & address, const uint16_t & length) override
  {
    int32_t data = 0;
    uint8_t * p_data = reinterpret_cast<uint8_t *>(&data);

    std::lock_guard<std::mutex> lock(buffer_m_);
    switch (length) {
      case 1:
        p_data[0] = data_[address + 0];
        break;

      case 2:
        p_data[0] = data_[address + 0];
        p_data[1] = data_[address + 1];
        break;

      case 4:
        p_data[0] = data_[address + 0];
        p_data[1] = data_[address + 1];
        p_data[2] = data_[address + 2];
        p_data[3] = data_[address + 3];
        break;

      default:
        p_data[0] = data_[address + 0];
        break;
    }

    return data;
  }

  bool open_port(const std::string & usb_port) override;
  bool set_baud_rate(const uint32_t & baud_rate) override;

  uint16_t ping() override;

  bool is_connect_manipulator() override;

  void play_sound(uint8_t sound) const override;

  void joints_torque(uint8_t onoff) const override;

  bool read_all() override;

  std::array<double, 4> get_joint_positions() override;
  std::array<double, 4> get_joint_velocities() override;

  double get_gripper_position() override;
  double get_gripper_velocity() override;

  bool set_joint_positions(const std::vector<double> & radians) override;
  bool set_joint_profile_acceleration(
    const std::array<int32_t, 4> & acceleration) override;
  bool set_joint_profile_velocity(const std::array<int32_t, 4> & velocity) override;

  bool set_gripper_position(const double & meters) override;
  bool set_gripper_profile_acceleration(const int32_t & acceleration) override;
  bool set_gripper_profile_velocity(const int32_t & velocity) override;

  bool set_home_pose() override;
  bool set_init_pose() override;
  bool set_zero_pose() override;

  bool set_gripper_current() override;

  bool open_gripper() override;
  bool close_gripper() override;
  bool init_gripper() override;

  void send_heartbeat(const uint8_t & count) override;

  void write_byte(const uint16_t & address, uint8_t data) override;
  uint8_t read_byte(const uint16_t & address) override;

private:
  bool set_joints_variables(
    const uint16_t & address,
    const std::array<int32_t, 4> & variables);

  bool set_gripper_variables(
    const uint16_t & address, const int32_t & variables);

  std::unique_ptr<DynamixelSDKWrapper> dxl_sdk_wrapper_;

  uint8_t data_[CONTROL_TABLE_SIZE];
  uint8_t data_buffer_[CONTROL_TABLE_SIZE];

  std::mutex buffer_m_;
};
}  // namespace open_manipulator_hardware
}  // namespace robotis
#endif  // open_manipulator_HARDWARE__OPENCR_HPP_
