cmake_minimum_required(VERSION 3.8)
project(slam)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 의존성 찾기
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(Eigen3 REQUIRED)

# PCL 설정
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# slam_node 실행 파일 추가
add_executable(slam_node src/slam_node.cpp)

# 의존성 링크
ament_target_dependencies(slam_node
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  pcl_conversions
  tf2_ros
  tf2_eigen
  Eigen3
)

# PCL 라이브러리 링크
target_link_libraries(slam_node
  ${PCL_LIBRARIES}
)

# 실행 파일 설치
install(TARGETS
  slam_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()