cmake_minimum_required(VERSION 3.8)
project(control_logic_bt)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -std=c++17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(bt_node src/bt_main.cpp)

target_include_directories(bt_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Collega le librerie. Nota: BT::behaviortree_cpp_v3 gestisce ZMQ internamente
ament_target_dependencies(bt_node
  rclcpp
  behaviortree_cpp_v3
  sensor_msgs
  std_msgs
)

install(TARGETS bt_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

ament_package()