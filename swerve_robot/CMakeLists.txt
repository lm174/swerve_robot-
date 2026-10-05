cmake_minimum_required(VERSION 3.10)
# 必须第一时间定义项目名，放在所有find_package前面！
project(swerve_robot)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  tf
  controller_manager
  joint_state_controller
  position_controllers
  velocity_controllers
  gazebo_ros
  gazebo_ros_control
  xacro
)
catkin_package(
  CATKIN_DEPENDS roscpp geometry_msgs std_msgs nav_msgs sensor_msgs tf controller_manager joint_state_controller position_controllers velocity_controllers gazebo_ros gazebo_ros_control xacro
)
include_directories(
${catkin_INCLUDE_DIRS}
)
# 控制节点，文件名改为 swerve_controller.cpp
add_executable(swerve_control_node src/swerve_controller.cpp)
target_link_libraries(swerve_control_node ${catkin_LIBRARIES})
# 里程计节点（暂时注释）
add_executable(swerve_odom_node src/swerve_odom.cpp)
target_link_libraries(swerve_odom_node ${catkin_LIBRARIES})
install(TARGETS
  swerve_control_node
  swerve_odom_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
