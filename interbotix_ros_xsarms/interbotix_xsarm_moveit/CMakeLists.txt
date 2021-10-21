cmake_minimum_required(VERSION 2.8.3)
project(interbotix_xsarm_moveit)

find_package(catkin REQUIRED)

catkin_package()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)


set (ROS_DEPENDENCIES
ament_index_cpp
rclcpp
moveit
moveit_msgs
moveit_ros_planning_interface)

add_executable(moveit_client src/moveit_client.cpp)
ament_target_dependencies(moveit_client ${ROS_DEPENDENCIES})
install(TARGETS
moveit_client
DESTINATION lib/${PROJECT_NAME})



install(DIRECTORY
  launch config
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
