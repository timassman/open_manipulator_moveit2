#!/usr/bin/env python3
#
# Copyright 2020 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Authors: Hye-jong KIM

import os
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import xacro


def generate_launch_description():

    ld = LaunchDescription()

    use_sim = LaunchConfiguration('use_sim')
    declare_use_sim = DeclareLaunchArgument(
        'use_sim',
        default_value='true',
        description='Start robot in Gazebo simulation.')
    ld.add_action(declare_use_sim)

    # Robot description
    robot_description_config = xacro.process_file(
        os.path.join(
            get_package_share_directory("turtlebot3_manipulation_description"),
            "urdf",
            "turtlebot3_manipulation.urdf.xacro",
        )
    )
    robot_description = {"robot_description": robot_description_config.toxml()}

    # Robot description Semantic config
    robot_description_semantic_path = os.path.join(
        get_package_share_directory("turtlebot3_manipulation_moveit_config"),
        "config",
        "turtlebot3_manipulation.srdf",
    )
    try:
        with open(robot_description_semantic_path, "r") as file:
            robot_description_semantic_config = file.read()
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None

    robot_description_semantic = {
        "robot_description_semantic": robot_description_semantic_config
    }

    # Get parameters for the Servo node
    servo_yaml_path = os.path.join(
        get_package_share_directory("turtlebot3_manipulation_moveit_config"),
        "config",
        "moveit_servo.yaml",
    )
    try:
        with open(servo_yaml_path, "r") as file:
            servo_params = {"moveit_servo": yaml.safe_load(file)}
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None

    # Launch as much as possible in components
    servo_node = Node(
        package="moveit_servo",
        executable="servo_node_main",
        parameters=[
            {'use_gazebo':use_sim}
            servo_params,
            robot_description,
            robot_description_semantic,
        ]
    )
    ld.add_action(servo_node)

    return ld
