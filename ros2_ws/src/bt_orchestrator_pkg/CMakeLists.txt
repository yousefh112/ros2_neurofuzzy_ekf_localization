cmake_minimum_required(VERSION 3.8)
project(bt_orchestrator_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(rclcpp REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)

add_executable(bt_brain src/main.cpp)
ament_target_dependencies(bt_brain rclcpp behaviortree_cpp_v3)

install(TARGETS
  bt_brain
  DESTINATION lib/${PROJECT_NAME}
)

# Optional: Install the XML folder so it copies to the install directory
install(DIRECTORY bt_xml/
  DESTINATION share/${PROJECT_NAME}/bt_xml
)






# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)

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

ament_package()
