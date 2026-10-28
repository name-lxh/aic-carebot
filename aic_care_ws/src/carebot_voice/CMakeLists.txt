cmake_minimum_required(VERSION 3.0.2)
project(carebot_voice)

find_package(catkin REQUIRED)

catkin_package()

catkin_install_python(PROGRAMS
  nodes/voice_bridge.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# ---------------------------------------------------------------------------
# 注意：scripts/xf_tts_cli.cpp 不在这里编译。
#
# 它需要链接讯飞 SDK 的 libmsc.so 以及 robot_voice 包内的头文件，
# 而这些资源属于组委会提供的 robot_voice 包，不在本包内。
# 请在机器人上用配套脚本一次性编译：
#
#     ./scripts/build_xf_tts_cli.sh ~/omniWheelCareRobot/rosCode/src/robot_voice
#
# 编译产物路径再配到 voice_bridge 的 ~tts_bin 参数（见 launch/voice.launch）。
# ---------------------------------------------------------------------------

install(PROGRAMS
  scripts/build_xf_tts_cli.sh
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
