cmake_minimum_required(VERSION 3.8)
project(bt_talk_2021)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

add_compile_options(-g)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(laser_lines REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(BT_Talk_2021 
  src/BT_Talk_2021.cpp
  src/calculateNearestWallGoal.cpp
  src/canSeeACloseWall.cpp
  src/closePointSubscriber.cpp
  src/isNearAWall.cpp
  src/moveToPose.cpp
  )
target_include_directories(BT_Talk_2021 PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${rclcpp_INCLUDE_DIRS}
  )
target_compile_features(BT_Talk_2021 PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  BT_Talk_2021
  "behaviortree_cpp_v3"
  "geometry_msgs"
  "laser_lines"
  "nav2_msgs"
  "rclcpp"
  "rclcpp_action"
  "rclcpp_components"
  "tf2"
  "tf2_geometry_msgs"
  "tf2_ros"
)

install(TARGETS BT_Talk_2021
  DESTINATION lib/${PROJECT_NAME})

# Install config files.
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
)
# Install launch files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

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
