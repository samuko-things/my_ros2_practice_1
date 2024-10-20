cmake_minimum_required(VERSION 3.8)
project(my_cpp_practice)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(my_robot_interfaces REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

add_executable(robot_news_station src/robot_news_station.cpp)
ament_target_dependencies(robot_news_station rclcpp example_interfaces)

add_executable(smart_phone src/smart_phone.cpp)
ament_target_dependencies(smart_phone rclcpp example_interfaces)

add_executable(number_publisher src/number_publisher.cpp)
ament_target_dependencies(number_publisher rclcpp example_interfaces)

add_executable(number_counter src/number_counter.cpp)
ament_target_dependencies(number_counter rclcpp example_interfaces)

add_executable(reset_counter_client src/reset_counter_client.cpp)
ament_target_dependencies(reset_counter_client rclcpp example_interfaces)

add_executable(add_two_ints_server src/add_two_ints_server.cpp)
ament_target_dependencies(add_two_ints_server rclcpp example_interfaces)

add_executable(add_two_ints_client_no_oop src/add_two_ints_client_no_oop.cpp)
ament_target_dependencies(add_two_ints_client_no_oop rclcpp example_interfaces)

add_executable(add_two_ints_client src/add_two_ints_client.cpp)
ament_target_dependencies(add_two_ints_client rclcpp example_interfaces)

add_executable(hw_status_publisher src/hw_status_publisher.cpp)
ament_target_dependencies(hw_status_publisher rclcpp my_robot_interfaces)

add_executable(led_state_server src/led_state_server.cpp)
ament_target_dependencies(led_state_server rclcpp my_robot_interfaces)

add_executable(battery_client src/battery_client.cpp)
ament_target_dependencies(battery_client rclcpp my_robot_interfaces)


install(TARGETS
  robot_news_station
  smart_phone
  number_publisher
  number_counter
  reset_counter_client
  add_two_ints_server
  add_two_ints_client_no_oop
  add_two_ints_client
  hw_status_publisher
  led_state_server
  battery_client
  DESTINATION lib/${PROJECT_NAME})


ament_package()
