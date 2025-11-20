cmake_minimum_required(VERSION 3.8)
project(assignment_1_07)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(apriltag_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)

# generate service files
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/GetGoal.srv"
  DEPENDENCIES geometry_msgs
)

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

# build nav2_orchestrator node
add_executable(nav2_orchestrator src/nav2_orchestrator.cpp)
ament_target_dependencies(nav2_orchestrator 
  rclcpp
  std_srvs
  nav2_msgs
)

# build goal_sender node
add_executable(goal_sender src/goal_sender.cpp)
ament_target_dependencies(goal_sender 
  rclcpp 
  rclcpp_action 
  geometry_msgs 
  nav2_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

# install executables C++
# build cancel_nav2_goal node
add_executable(cancel_nav2_goal src/cancel_nav2_goal.cpp)
ament_target_dependencies(cancel_nav2_goal 
  rclcpp 
  rclcpp_action 
  nav2_msgs
)

# install executables
install(TARGETS
  goal_selector
  goal_sender
  nav2_orchestrator
  cancel_nav2_goal
  DESTINATION lib/${PROJECT_NAME}/
)

# Install Python modules
#ament_python_install_package(${PROJECT_NAME})
# Install Python executables
install(PROGRAMS
  assignment_1_07/Detection_Lidar.py
  assignment_1_07/corridor_detector.py
  assignment_1_07/Corridor_Controller.py
  DESTINATION lib/${PROJECT_NAME}
)

# install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

# install configuration files
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/
)

# link generated service files
rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")

# link the target libraries
target_link_libraries(goal_selector "${cpp_typesupport_target}")
target_link_libraries(goal_sender "${cpp_typesupport_target}")

ament_package()
