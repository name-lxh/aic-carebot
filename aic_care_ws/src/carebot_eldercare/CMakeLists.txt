cmake_minimum_required(VERSION 3.0.2)
project(carebot_eldercare)

find_package(catkin REQUIRED)

catkin_package()

catkin_install_python(PROGRAMS
  nodes/mission_manager.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(PROGRAMS
  scripts/preflight_check.py
  scripts/sim_stubs.py
  scripts/build_report.py
  scripts/rehearsal.py
  scripts/run_competition.sh
  scripts/run_rehearsal.sh
  scripts/run_recording.sh
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
