################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.5)
project(open_manipulator_hardware)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

################################################################################
# Find ament packages and libraries for ament and system dependencies
################################################################################
find_package(ament_cmake REQUIRED)
find_package(dynamixel_sdk REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(open_manipulator_msgs REQUIRED)
find_package(open_manipulator_x_libs REQUIRED)
find_package(robotis_manipulator REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/dynamixel_sdk_wrapper.cpp
  src/opencr.cpp
  src/u2d2.cpp
  src/open_manipulator_system.cpp
)

ament_target_dependencies(${PROJECT_NAME}
  dynamixel_sdk
  hardware_interface
  pluginlib
  rclcpp
  geometry_msgs
  open_manipulator_msgs
  open_manipulator_x_libs
  robotis_manipulator
  sensor_msgs
  std_msgs
  trajectory_msgs
)

target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

pluginlib_export_plugin_description_file(hardware_interface open_manipulator_hardware.xml)

################################################################################
# Install
################################################################################
install(DIRECTORY include/
  DESTINATION include
)

install(TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

################################################################################
# Macro for ament package
################################################################################
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})

ament_export_dependencies(
  dynamixel_sdk
  hardware_interface
  pluginlib
  rclcpp
  geometry_msgs
  open_manipulator_msgs
  open_manipulator_x_libs
  robotis_manipulator
  sensor_msgs
  std_msgs
  trajectory_msgs
)

ament_package()
