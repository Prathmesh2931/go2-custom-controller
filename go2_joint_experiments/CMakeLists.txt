cmake_minimum_required(VERSION 3.8)
project(go2_joint_experiments)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O3)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(eigen3_cmake_module REQUIRED)

# Cross-package dependencies
find_package(ament_index_cpp REQUIRED)
find_package(go2_mpc_controller REQUIRED)

# --- FIX: We must explicitly find Pinocchio here to satisfy the static math library! ---
find_package(pinocchio REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

add_executable(trajectory_inspector src/trajectory_inspector.cpp)
ament_target_dependencies(trajectory_inspector rclcpp trajectory_msgs sensor_msgs)

add_executable(stand_pose_publisher src/stand_pose_publisher.cpp)
ament_target_dependencies(stand_pose_publisher rclcpp trajectory_msgs sensor_msgs)

add_executable(static_stand_balancer src/static_stand_balancer.cpp)
target_link_libraries(static_stand_balancer Eigen3::Eigen)
ament_target_dependencies(static_stand_balancer rclcpp trajectory_msgs sensor_msgs)

add_executable(dynamic_stand_balancer src/dynamic_stand_balancer.cpp)
target_link_libraries(dynamic_stand_balancer Eigen3::Eigen)
ament_target_dependencies(dynamic_stand_balancer rclcpp trajectory_msgs sensor_msgs)

add_executable(cartesian_stand_balancer src/cartesian_stand_balancer.cpp)
target_link_libraries(cartesian_stand_balancer Eigen3::Eigen)
ament_target_dependencies(cartesian_stand_balancer rclcpp trajectory_msgs sensor_msgs)

add_executable(pure_pd_tracker src/pure_pd_tracker.cpp)
# --- FIX: We explicitly link pinocchio::pinocchio to the final executable ---
target_link_libraries(pure_pd_tracker Eigen3::Eigen pinocchio::pinocchio)
ament_target_dependencies(pure_pd_tracker 
  rclcpp 
  sensor_msgs 
  std_msgs
  ament_index_cpp
  go2_mpc_controller
)

add_executable(robot_model_validation src/robot_model_validation.cpp)
target_link_libraries(robot_model_validation Eigen3::Eigen pinocchio::pinocchio)
ament_target_dependencies(robot_model_validation 
  rclcpp 
  ament_index_cpp
  go2_mpc_controller
)

install(TARGETS
  trajectory_inspector
  stand_pose_publisher
  static_stand_balancer
  dynamic_stand_balancer
  cartesian_stand_balancer
  pure_pd_tracker
  robot_model_validation
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/ src/ 
  DESTINATION share/${PROJECT_NAME}
)

ament_package()