cmake_minimum_required(VERSION 3.8)
project(go2_mpc_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O3)
endif()

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(pinocchio REQUIRED)

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

# 1. Create the Library
add_library(mpc_math_lib 
  src/mpc_solver.cpp
  # src/robot_dynamics.cpp
  src/robot_model.cpp
)
target_link_libraries(mpc_math_lib Eigen3::Eigen qpOASES pinocchio::pinocchio)

target_include_directories(mpc_math_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# 2. Create the Executables
add_executable(dynamic_tracker src/dynamic_tracker.cpp)
target_link_libraries(dynamic_tracker mpc_math_lib) 
ament_target_dependencies(dynamic_tracker ${ROS2_DEPENDENCIES})

add_executable(debug_math src/debug_math.cpp)
target_link_libraries(debug_math mpc_math_lib) 

# --- THE FIX: INSTALL THE LIBRARY ---
# Libraries must be installed to the global lib destination so ament_export can find them!
install(TARGETS mpc_math_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install the Executables
install(TARGETS
  dynamic_tracker
  debug_math
  DESTINATION lib/${PROJECT_NAME}
)

# Install the Headers
install(DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Export for other packages (like go2_joint_experiments)
ament_export_include_directories(include)
ament_export_libraries(mpc_math_lib)
# Explicitly export the dependencies that your headers might expose
ament_export_dependencies(Eigen3 pinocchio)

ament_package()