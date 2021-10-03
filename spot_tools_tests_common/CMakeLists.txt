cmake_minimum_required(VERSION 2.8.3)
project(spot_tools_tests_common)
find_package(catkin REQUIRED COMPONENTS
  rospy
  asa_ros_msgs
  std_msgs
)
catkin_python_setup()
catkin_package()
