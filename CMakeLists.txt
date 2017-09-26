cmake_minimum_required(VERSION 2.8.3)
project(criutils)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  rosbash
  rospy
  sensor_msgs
  tf
)

catkin_python_setup()

catkin_package()

# Tests
catkin_add_nosetests(tests/test_rviz.py)
