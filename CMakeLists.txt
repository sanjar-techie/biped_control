cmake_minimum_required(VERSION 3.0.2)
project(biped_control)

# Use C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find required packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  dynamixel_controllers
  ros_control
  ros_controllers
  tf2
  tf2_ros
  visualization_msgs
  trajectory_msgs
  cmake_modules
)

# Find Eigen
find_package(Eigen3 REQUIRED)

# Find yaml-cpp
find_package(yaml-cpp REQUIRED)

# Declare catkin package
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES 
    biped_core
    biped_hardware
  CATKIN_DEPENDS
    roscpp
    std_msgs
    sensor_msgs
    dynamixel_controllers
    ros_control
    ros_controllers
    tf2
    tf2_ros
    visualization_msgs
    trajectory_msgs
  DEPENDS
    EIGEN3
    YAML_CPP
)

# Include directories
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIRS}
)

# Declare core library
add_library(biped_core
  src/biped.cpp
  src/utils/transforms.cpp
  src/kinematics/forward_kinematics.cpp
  src/visualization/biped_visualizer.cpp
)

# Declare hardware interface library
add_library(biped_hardware
  src/hardware/dynamixel_interface.cpp
  src/hardware/motor_controller.cpp
)

# Declare executables
add_executable(walking_node src/nodes/walking_node.cpp)
add_executable(trajectory_node src/nodes/trajectory_node.cpp)
add_executable(hardware_interface_node src/nodes/hardware_interface_node.cpp)

# Specify libraries to link executable targets against
target_link_libraries(biped_core
  ${catkin_LIBRARIES}
  ${EIGEN3_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
)

target_link_libraries(biped_hardware
  biped_core
  ${catkin_LIBRARIES}
)

target_link_libraries(walking_node
  biped_core
  biped_hardware
  ${catkin_LIBRARIES}
)

target_link_libraries(trajectory_node
  biped_core
  biped_hardware
  ${catkin_LIBRARIES}
)

target_link_libraries(hardware_interface_node
  biped_core
  biped_hardware
  ${catkin_LIBRARIES}
)

# Install targets
install(TARGETS 
  biped_core
  biped_hardware
  walking_node
  trajectory_node
  hardware_interface_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install headers
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hpp"
  PATTERN ".svn" EXCLUDE
)

# Install launch and config files
install(DIRECTORY 
  launch/
  config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

# Add testing
if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_biped test/test_biped.cpp)
  target_link_libraries(test_biped biped_core ${catkin_LIBRARIES})

  catkin_add_gtest(test_kinematics test/test_kinematics.cpp)
  target_link_libraries(test_kinematics biped_core ${catkin_LIBRARIES})
endif()