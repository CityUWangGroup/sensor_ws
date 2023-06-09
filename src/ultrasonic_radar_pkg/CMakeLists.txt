cmake_minimum_required(VERSION 3.8)
project(ultrasonic_radar_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# add message path
include_directories(
  ../../install/sensor_driver_msgs/include
)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(serial REQUIRED)
find_package(sensor_driver_msgs REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

add_executable(ultrasonic_radar_all_node src/ultrasonic_radar6_node.cpp)
ament_target_dependencies(ultrasonic_radar_all_node rclcpp std_msgs serial sensor_driver_msgs)
install(TARGETS
  ultrasonic_radar_all_node
  DESTINATION lib/${PROJECT_NAME}
)
  
add_executable(ultrasonic_radar_1_node src/ultrasonic_radar_com1_node.cpp)
ament_target_dependencies(ultrasonic_radar_1_node rclcpp std_msgs serial sensor_driver_msgs)
install(TARGETS
  ultrasonic_radar_1_node
  DESTINATION lib/${PROJECT_NAME}
)
add_executable(ultrasonic_radar_2_node src/ultrasonic_radar_com2_node.cpp)
ament_target_dependencies(ultrasonic_radar_2_node rclcpp std_msgs serial sensor_driver_msgs)
install(TARGETS
  ultrasonic_radar_2_node
  DESTINATION lib/${PROJECT_NAME}
)
add_executable(ultrasonic_radar_3_node src/ultrasonic_radar_com3_node.cpp)
ament_target_dependencies(ultrasonic_radar_3_node rclcpp std_msgs serial sensor_driver_msgs)
install(TARGETS
  ultrasonic_radar_3_node
  DESTINATION lib/${PROJECT_NAME}
)
add_executable(ultrasonic_radar_4_node src/ultrasonic_radar_com4_node.cpp)
ament_target_dependencies(ultrasonic_radar_4_node rclcpp std_msgs serial sensor_driver_msgs)
install(TARGETS
  ultrasonic_radar_4_node
  DESTINATION lib/${PROJECT_NAME}
)
add_executable(ultrasonic_radar_5_node src/ultrasonic_radar_com5_node.cpp)
ament_target_dependencies(ultrasonic_radar_5_node rclcpp std_msgs serial sensor_driver_msgs)
install(TARGETS
  ultrasonic_radar_5_node
  DESTINATION lib/${PROJECT_NAME}
)
add_executable(ultrasonic_radar_6_node src/ultrasonic_radar_com6_node.cpp)
ament_target_dependencies(ultrasonic_radar_6_node rclcpp std_msgs serial sensor_driver_msgs)
install(TARGETS
  ultrasonic_radar_6_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})
