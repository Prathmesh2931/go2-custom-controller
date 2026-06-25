cmake_minimum_required(VERSION 3.8)
project(go2_mpc_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O3)
endif()

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)

set(ROS2_DEPENDENCIES
  rclcpp
  sensor_msgs
  std_msgs
  geometry_msgs
  nav_msgs
  trajectory_msgs
  tf2 
  tf2_geometry_msgs
)

foreach(pkg IN LISTS ROS2_DEPENDENCIES)
  find_package(${pkg} REQUIRED)
endforeach()

include_directories(include)


add_library(mpc_math_lib 
  src/mpc_solver.cpp
  src/robot_dynamics.cpp
)
target_link_libraries(mpc_math_lib Eigen3::Eigen qpOASES)


add_executable(dynamic_tracker src/dynamic_tracker.cpp)
target_link_libraries(dynamic_tracker mpc_math_lib) # Links the math library!
ament_target_dependencies(dynamic_tracker ${ROS2_DEPENDENCIES})


add_executable(debug_math src/debug_math.cpp)
target_link_libraries(debug_math mpc_math_lib) # Links the math library!
# Notice we DO NOT link ROS2 dependencies here, making it super fast to run.

# Install Rules
install(TARGETS
  dynamic_tracker
  debug_math
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()