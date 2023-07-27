CMAKE_MINIMUM_REQUIRED(VERSION 2.8.3)
PROJECT(ur3_gazebo)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  rospy
  std_msgs
  sensor_msgs
)

catkin_python_setup()

catkin_package()
include_directories()

