cmake_minimum_required(VERSION 3.0.2)
project(carebot_iot)

find_package(catkin REQUIRED)

catkin_package()

catkin_install_python(PROGRAMS
  nodes/iot_bridge.py
  nodes/iot_serial_bridge.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
