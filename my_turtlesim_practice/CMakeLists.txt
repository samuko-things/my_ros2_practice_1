cmake_minimum_required(VERSION 3.8)
project(my_turtlesim_practice)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(turtlesim REQUIRED)
find_package(my_robot_interfaces REQUIRED)

add_executable(turtle_move_to_goal src/turtle_move_to_goal.cpp)
ament_target_dependencies(turtle_move_to_goal 
  rclcpp
  std_msgs
  std_srvs
  geometry_msgs
  turtlesim
  my_robot_interfaces
)

add_executable(turtle_add_goal_pose src/turtle_add_goal_pose.cpp)
ament_target_dependencies(turtle_add_goal_pose 
  rclcpp
  std_msgs
  std_srvs
  geometry_msgs
  turtlesim
  my_robot_interfaces
)

install(TARGETS
  turtle_move_to_goal
  turtle_add_goal_pose
  DESTINATION lib/${PROJECT_NAME})

# Install launch files.
# install(DIRECTORY
# config
# launch
# urdf
# world
# DESTINATION share/${PROJECT_NAME}/
#)

ament_package()
