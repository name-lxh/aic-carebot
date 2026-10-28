cmake_minimum_required(VERSION 3.0.2)
project(carebot_face)

find_package(catkin REQUIRED)

catkin_package()

catkin_install_python(PROGRAMS
  nodes/face_gate.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
