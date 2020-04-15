cmake_minimum_required(VERSION 2.8)
project(rover_s_node)

find_package(catkin REQUIRED)
catkin_package()

catkin_python_setup()

catkin_install_python(PROGRAMS nodes/driver-control.py
  nodes/rover-control.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)