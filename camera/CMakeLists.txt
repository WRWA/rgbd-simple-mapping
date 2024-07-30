cmake_minimum_required(VERSION 3.8)
project(camera)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find required packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(PCL REQUIRED)
find_package(realsense2 REQUIRED)
find_package(OpenCV 4.5 REQUIRED)

# Include directories
include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
  ${cv_bridge_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${realsense2_INCLUDE_DIRS}
)

# Create the executable
add_executable(l515_node src/l515_node.cpp)

# Specify libraries to link a library or executable target against
target_link_libraries(l515_node
  ${rclcpp_LIBRARIES}
  ${sensor_msgs_LIBRARIES}
  ${cv_bridge_LIBRARIES}
  ${PCL_LIBRARIES}
  ${realsense2_LIBRARY}
)

# Add dependencies
ament_target_dependencies(l515_node
  rclcpp
  sensor_msgs
  cv_bridge
  PCL
  realsense2
)

# Install the executable
install(TARGETS
  l515_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()