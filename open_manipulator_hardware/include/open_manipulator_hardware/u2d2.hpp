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

#ifndef open_manipulator_HARDWARE__U2D2_HPP_
#define open_manipulator_HARDWARE__U2D2_HPP_

#include <stdlib.h>
#include <array>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "open_manipulator_hardware/usb_device.hpp"
#include "open_manipulator_x_libs/open_manipulator_x.hpp"

#define JOINT_DYNAMIXEL "joint_dxl"
#define TOOL_DYNAMIXEL  "tool_dxl"

namespace robotis
{
namespace open_manipulator_hardware
{
class U2d2 : public UsbDevice
{
public:
  explicit U2d2();
  virtual ~U2d2();

  bool open_port(const std::string & usb_port, const uint32_t & baud_rate) override;

  uint16_t ping() override;

  bool is_connect_manipulator() override;

  void play_sound(uint8_t sound) const override;

  void joints_torque(uint8_t onoff) const override;

  bool read_all() override;

  std::array<double, 4> get_joint_positions() override;
  std::array<double, 4> get_joint_velocities() override;

  double get_gripper_position() override;
  double get_gripper_velocity() override;

  bool set_joint_positions(std::vector<double> & radians) override;
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

private:
  std::unique_ptr<OpenManipulatorX> open_manipulator_x_;

  std::vector<JointValue> joint_value_;
  std::vector<JointValue> tool_value_;

  std::vector<uint8_t> dxl_id_ = {11, 12, 13, 14, 15};
};
}  // namespace open_manipulator_hardware
}  // namespace robotis
#endif  // open_manipulator_HARDWARE__U2D2_HPP_
