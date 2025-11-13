cmake_minimum_required(VERSION 3.8)
project(assignment_1_07)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(apriltag_msgs REQUIRED)

# build goal_selector node
add_executable(goal_selector src/goal_selector.cpp)
ament_target_dependencies(goal_selector
  rclcpp
  tf2
  tf2_ros
  tf2_geometry_msgs
  geometry_msgs
  apriltag_msgs
)

# install executables
install(TARGETS
  goal_selector
  DESTINATION lib/${PROJECT_NAME}/
)

# install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

# install configuration files
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/
)


ament_package()
