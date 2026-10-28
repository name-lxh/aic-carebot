cmake_minimum_required(VERSION 3.0.2)
project(carebot_iot_msgs)

# ---------------------------------------------------------------------------
#  纯消息包：只定义服务，不含可执行节点
#
#  ⚠️ 为什么单独建包而不是塞进 carebot_iot：
#     消息/服务定义必须在**编译期**生成，任何依赖它的包都要
#     在 package.xml 里声明依赖。如果定义和业务代码混在一起，
#     别的包想用这个服务就得依赖整个 carebot_iot（连带它的
#     HomeAssistant 客户端、pyserial 等运行期依赖）——
#     耦合太重，而且循环依赖很容易踩到。
# ---------------------------------------------------------------------------

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
)

add_service_files(
  FILES
  Control.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs
)
