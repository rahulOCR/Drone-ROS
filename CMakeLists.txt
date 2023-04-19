cmake_minimum_required(VERSION 2.8.3)
project(obs)

find_package(catkin REQUIRED COMPONENTS
  mavros
  std_msgs
  geometry_msgs
  mavros_msgs
  rospy
  sensor_msgs
  roscpp
)


catkin_python_setup()

