cmake_minimum_required(VERSION 3.0.2)
project(carebot_nav_kit)

find_package(catkin REQUIRED)

catkin_package()

# ---------------------------------------------------------------------------
#  Python 节点
# ---------------------------------------------------------------------------
catkin_install_python(PROGRAMS
  nodes/waypoint_nav.py
  nodes/nav_check.py
  nodes/odom_monitor.py
  nodes/set_initial_pose.py
  nodes/fake_sensors.py
  nodes/sim_precheck.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# ---------------------------------------------------------------------------
#  Python 脚本（标定工具）
# ---------------------------------------------------------------------------
catkin_install_python(PROGRAMS
  scripts/calibrate_linear.py
  scripts/calibrate_angular.py
  scripts/make_placeholder_map.py
  scripts/verify_nav_config.py
  scripts/missing_pkg_notice.py
  scripts/make_stub_packages.py
  scripts/nav_launch.py
  scripts/sim_arena.py
  scripts/make_sim_map.py
  scripts/utf8_stdio.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# ---------------------------------------------------------------------------
#  Shell 脚本
# ---------------------------------------------------------------------------
install(PROGRAMS
  scripts/save_map.sh
  scripts/check_hardware.sh
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# ---------------------------------------------------------------------------
#  配置、启动文件、rviz 配置、地图
# ---------------------------------------------------------------------------
install(DIRECTORY launch config rviz maps docs
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)




