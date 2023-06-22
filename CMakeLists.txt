cmake_minimum_required(VERSION 3.8)
project(dbot_moveit)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(moveit REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_visual_tools REQUIRED)

add_executable(dbot_moveit src/dbot_moveit.cpp)
target_include_directories(dbot_moveit PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(dbot_moveit PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  dbot_moveit
  "moveit"
  "moveit_ros_planning_interface"
  "rclcpp"
  "moveit_visual_tools"
)

add_executable(dbot_rb_model_and_state src/dbot_rb_model_and_state.cpp)
target_include_directories(dbot_rb_model_and_state PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(dbot_rb_model_and_state PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  dbot_rb_model_and_state
  "moveit"
  "moveit_ros_planning_interface"
  "rclcpp"
  "moveit_visual_tools"
)

add_executable(dbot_planning_scene src/dbot_planning_scene.cpp)
target_include_directories(dbot_planning_scene PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(dbot_planning_scene PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  dbot_planning_scene
  "moveit"
  "moveit_ros_planning_interface"
  "rclcpp"
  "moveit_visual_tools"
)

add_executable(dbot_motion_planning src/dbot_motion_planning.cpp)
target_include_directories(dbot_motion_planning PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(dbot_motion_planning PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  dbot_motion_planning
  "moveit"
  "moveit_ros_planning_interface"
  "rclcpp"
  "moveit_visual_tools"
  "pluginlib"
)

install(
  TARGETS 
  dbot_moveit
  dbot_rb_model_and_state
  dbot_planning_scene
  dbot_motion_planning
  DESTINATION 
  lib/${PROJECT_NAME}
)

# Install folders
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

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
