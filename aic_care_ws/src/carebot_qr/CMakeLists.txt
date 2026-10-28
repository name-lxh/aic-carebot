cmake_minimum_required(VERSION 3.0.2)
project(carebot_qr)

find_package(catkin REQUIRED)

catkin_package()

# Python 节点，无需编译。仅安装脚本以便 rosrun 直接调用。
catkin_install_python(PROGRAMS
  nodes/qr_detector.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
