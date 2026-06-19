cmake_minimum_required(VERSION 3.8)
project(trajectory_generator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O3)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(eigen3_cmake_module REQUIRED)

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
add_executable(foot_trajectory_gen src/foot_trajectory_gen.cpp)
ament_target_dependencies(foot_trajectory_gen rclcpp trajectory_msgs sensor_msgs)

add_executable(phase_generator src/phase_gen.cpp)
ament_target_dependencies(phase_generator rclcpp trajectory_msgs sensor_msgs)

add_executable(cmd_control_go2 src/cmd_control_go2.cpp)
ament_target_dependencies(cmd_control_go2 rclcpp trajectory_msgs sensor_msgs geometry_msgs)

add_executable(velocity_estimator src/velocity_estimator.cpp)
ament_target_dependencies(velocity_estimator rclcpp trajectory_msgs sensor_msgs geometry_msgs nav_msgs)

add_executable(jacobian_validator src/jacobian_validator.cpp)
target_link_libraries(jacobian_validator Eigen3::Eigen)
ament_target_dependencies(jacobian_validator rclcpp)

add_executable(mpc_trajectory_gen src/mpc_trajectory_gen.cpp)
target_link_libraries(mpc_trajectory_gen Eigen3::Eigen)
ament_target_dependencies(mpc_trajectory_gen rclcpp trajectory_msgs sensor_msgs nav_msgs geometry_msgs)

install(
  DIRECTORY include/ src/
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
  foot_trajectory_gen
  phase_generator
  cmd_control_go2
  velocity_estimator
  mpc_trajectory_gen
  jacobian_validator
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
