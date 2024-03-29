cmake_minimum_required(VERSION 3.8)
project(slam_utils)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclpy REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(rclcpp REQUIRED)

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

add_executable(occupacy_mean src/occupacy_mean.cpp)
ament_target_dependencies(occupacy_mean rclcpp nav_msgs)

add_executable(map_pose_listener src/map_pose_listener.cpp)
ament_target_dependencies(map_pose_listener rclcpp nav_msgs tf2_ros)

add_executable(pose_cov_listener src/pose_cov_listener.cpp)
ament_target_dependencies(pose_cov_listener rclcpp geometry_msgs)

add_executable(marker_array_listener src/marker_array_listener.cpp)
ament_target_dependencies(marker_array_listener rclcpp visualization_msgs)

install(TARGETS occupacy_mean map_pose_listener pose_cov_listener marker_array_listener
  DESTINATION lib/${PROJECT_NAME})

# install(TARGETS map_pose_listener
#   DESTINATION lib/${PROJECT_NAME})


ament_package()
