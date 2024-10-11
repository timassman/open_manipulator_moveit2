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

#include "open_manipulator_hardware/u2d2.hpp"

#include "rclcpp/rclcpp.hpp"

#include <unistd.h>
#include <algorithm>
#include <memory>
#include <string>
#include <vector>

namespace robotis
{
  namespace open_manipulator_hardware
  {
    U2d2::U2d2()
    {
      joint_value_present_.resize(ARM_JOINTS);
      joint_value_goal_.resize(ARM_JOINTS);

      tool_value_present_.resize(GRIPPER_JOINTS_DXL);
      tool_value_goal_.resize(GRIPPER_JOINTS_DXL);

      open_manipulator_x_ = std::make_unique<OpenManipulatorX>();
    }

    U2d2::~U2d2()
    {
      open_manipulator_x_->disableAllActuator();
    }

    bool U2d2::open_port(const std::string &usb_port, const uint32_t &baud_rate)
    {
      bool sim = false;
      open_manipulator_x_->init_open_manipulator_x(sim, usb_port, std::to_string(baud_rate));
      return true;
    }

    uint16_t U2d2::ping()
    {
      return 0; // not available on U2D2
    }

    bool U2d2::is_connect_manipulator()
    {
      return read_all();
    }

    void U2d2::play_sound(uint8_t sound) const
    {
      (void)sound;
      // not available on U2D2
    }

    void U2d2::joints_torque(uint8_t onoff) const
    {
      (void)onoff;
      // not needed on U2D2
    }

    bool U2d2::read_all()
    {
      joint_value_present_ = open_manipulator_x_->receiveAllJointActuatorValue();
      tool_value_present_ = open_manipulator_x_->receiveAllToolActuatorValue();

      return true;
    }

    void U2d2::get_joint_positions(std::array<double, ARM_JOINTS> &joint_positions)
    {
      for (uint8_t i = 0; i < joint_positions.size(); i++)
      {
        joint_positions[i] = joint_value_present_.at(i).position;
      }
    }

    void U2d2::get_joint_velocities(std::array<double, ARM_JOINTS> &joint_velocities)
    {
      for (uint8_t i = 0; i < joint_velocities.size(); i++)
      {
        joint_velocities[i] = joint_value_present_.at(i).velocity;
      }
    }

    void U2d2::get_gripper_position(double &meter)
    {
      double radian = tool_value_present_.at(0).position;

      meter = radian * opencr::grippers::RAD_TO_METER;
    }

    void U2d2::get_gripper_velocity(double &meter_per_second)
    {
      double radian_per_second = tool_value_present_.at(0).velocity;

      meter_per_second = radian_per_second * opencr::grippers::RAD_TO_METER;
    }

    bool U2d2::set_joint_positions(std::array<double, ARM_JOINTS> &joint_goals)
    {
      for (uint8_t i = 0; i < joint_goals.size(); i++)
      {
        joint_value_goal_[i].position = joint_goals[i];
      }
      return open_manipulator_x_->sendAllJointActuatorValue(joint_value_goal_);
    }

    // source: open_manipulator/open_manipulator_x_libs/src/open_manipulator_x.cpp
    bool U2d2::set_joint_profile_acceleration(const std::array<int32_t, ARM_JOINTS> &acceleration)
    {
      // Set joint actuator parameter
      STRING gripper_dxl_opt_arg[2];
      void *p_gripper_dxl_opt_arg = &gripper_dxl_opt_arg;
      gripper_dxl_opt_arg[0] = "Profile_Acceleration";

      for (uint8_t i = 0; i < acceleration.size(); i++)
      {
        gripper_dxl_opt_arg[1] = std::to_string(acceleration[i]);
        std::vector<uint8_t> id_array = {dxl_id_[i]};
        open_manipulator_x_->setJointActuatorMode(JOINT_DYNAMIXEL, id_array, p_gripper_dxl_opt_arg);
      }

      return true;
    }

    bool U2d2::set_joint_profile_velocity(const std::array<int32_t, ARM_JOINTS> &velocity)
    {
      // Set joint actuator parameter
      STRING gripper_dxl_opt_arg[2];
      void *p_gripper_dxl_opt_arg = &gripper_dxl_opt_arg;
      gripper_dxl_opt_arg[0] = "Profile_Velocity";

      for (uint8_t i = 0; i < velocity.size(); i++)
      {
        gripper_dxl_opt_arg[1] = std::to_string(velocity[i]);
        std::vector<uint8_t> id_array = {dxl_id_[i]};
        open_manipulator_x_->setJointActuatorMode(JOINT_DYNAMIXEL, id_array, p_gripper_dxl_opt_arg);
      }

      return true;
    }

    bool U2d2::set_gripper_position(double meter)
    {
      double radian = meter / opencr::grippers::RAD_TO_METER;
      tool_value_goal_[0].position = radian;
      return open_manipulator_x_->sendAllToolActuatorValue(tool_value_goal_);
    }

    bool U2d2::set_gripper_profile_acceleration(const int32_t &acceleration)
    {
      // Set gripper actuator parameter
      STRING gripper_dxl_opt_arg[2];
      void *p_gripper_dxl_opt_arg = &gripper_dxl_opt_arg;
      gripper_dxl_opt_arg[0] = "Profile_Acceleration";
      gripper_dxl_opt_arg[1] = std::to_string(acceleration);
      open_manipulator_x_->setToolActuatorMode(TOOL_DYNAMIXEL, p_gripper_dxl_opt_arg);

      return true;
    }

    bool U2d2::set_gripper_profile_velocity(const int32_t &velocity)
    {
      // Set gripper actuator parameter
      STRING gripper_dxl_opt_arg[2];
      void *p_gripper_dxl_opt_arg = &gripper_dxl_opt_arg;
      gripper_dxl_opt_arg[0] = "Profile_Velocity";
      gripper_dxl_opt_arg[1] = std::to_string(velocity);
      open_manipulator_x_->setToolActuatorMode(TOOL_DYNAMIXEL, p_gripper_dxl_opt_arg);

      return true;
    }

    bool U2d2::set_init_pose()
    {
      std::array<double, ARM_JOINTS> init_pose = {0.0, -1.57, 1.37, 0.26};
      return set_joint_positions(init_pose);
    }

    bool U2d2::set_zero_pose()
    {
      std::array<double, ARM_JOINTS> zero_pose = {0.0, 0.0, 0.0, 0.0};
      return set_joint_positions(zero_pose);
    }

    bool U2d2::set_home_pose()
    {
      std::array<double, ARM_JOINTS> home_pose = {0.0, -1.05, 0.35, 0.70};
      return set_joint_positions(home_pose);
    }

    bool U2d2::set_gripper_current()
    {
      // unclear how to do this with open_manipulator_x_libs
      return true;
    }

    bool U2d2::open_gripper()
    {
      return set_gripper_position(0.01);
    }

    bool U2d2::close_gripper()
    {
      return set_gripper_position(-0.01);
    }

    bool U2d2::init_gripper()
    {
      return set_gripper_position(0.0);
    }

    void U2d2::send_heartbeat(const uint8_t &count)
    {
      (void)count;
      // not available on U2D2
    }
  } // namespace open_manipulator_hardware
} // namespace robotis
