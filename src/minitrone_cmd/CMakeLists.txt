cmake_minimum_required(VERSION 3.8)
project(minitrone_cmd)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(minitrone_interfaces REQUIRED)

add_executable(minitrone_position_cmd src/position_cmd.cpp)
ament_target_dependencies(minitrone_position_cmd rclcpp minitrone_interfaces)

add_executable(minitrone_attitude_sweep_cmd src/attitude_sweep_cmd.cpp)
ament_target_dependencies(minitrone_attitude_sweep_cmd
  rclcpp
  minitrone_interfaces
)

add_executable(minitrone_external_wrench_cmd src/external_wrench_cmd.cpp)
ament_target_dependencies(minitrone_external_wrench_cmd
  rclcpp
  minitrone_interfaces
)

install(TARGETS
  minitrone_attitude_sweep_cmd
  minitrone_external_wrench_cmd
  DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS minitrone_position_cmd
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/minitrone_position_teleop.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME minitrone_position_teleop
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
