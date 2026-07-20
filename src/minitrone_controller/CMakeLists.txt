cmake_minimum_required(VERSION 3.8)
project(minitrone_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(minitrone_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)

# minitrone_wrench_controller
add_executable(minitrone_wrench_controller src/wrench_controller.cpp)
ament_target_dependencies(minitrone_wrench_controller rclcpp minitrone_interfaces std_msgs)
target_link_libraries(minitrone_wrench_controller Eigen3::Eigen)

# minitrone_admittance_controller
add_executable(minitrone_admittance_controller src/admittance_controller.cpp)
ament_target_dependencies(minitrone_admittance_controller rclcpp minitrone_interfaces std_msgs)
target_link_libraries(minitrone_admittance_controller Eigen3::Eigen)

# passive-aligning hybrid force/motion filter
add_executable(passive_aligning_controller src/minitrone_passive_aligning_controller.cpp)
ament_target_dependencies(
  passive_aligning_controller
  rclcpp
  rcl_interfaces
  minitrone_interfaces
  std_msgs
)
target_link_libraries(passive_aligning_controller Eigen3::Eigen)

# minitrone_allocator_controller
add_executable(minitrone_allocator_controller src/allocator_controller.cpp)
ament_target_dependencies(minitrone_allocator_controller rclcpp minitrone_interfaces)
target_link_libraries(minitrone_allocator_controller Eigen3::Eigen)

# wrench_observer
add_executable(minitrone_first_wrench_observer src/first_wrench_observer.cpp)
ament_target_dependencies(minitrone_first_wrench_observer rclcpp minitrone_interfaces)
target_link_libraries(minitrone_first_wrench_observer Eigen3::Eigen)

# second_wrench_observer
add_executable(minitrone_second_wrench_observer src/second_wrench_observer.cpp)
ament_target_dependencies(minitrone_second_wrench_observer rclcpp minitrone_interfaces)
target_link_libraries(minitrone_second_wrench_observer Eigen3::Eigen)

# minitrone_ekf_state_estimator
add_executable(minitrone_ekf_state_estimator src/ekf_state_estimator.cpp)
ament_target_dependencies(minitrone_ekf_state_estimator rclcpp minitrone_interfaces std_msgs)
target_link_libraries(minitrone_ekf_state_estimator Eigen3::Eigen)

install(TARGETS
  minitrone_wrench_controller
  minitrone_admittance_controller
  passive_aligning_controller
  minitrone_allocator_controller
  minitrone_first_wrench_observer
  minitrone_second_wrench_observer
  minitrone_ekf_state_estimator
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
