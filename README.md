# aic-carebot · 智慧养老护理机器人

> 第八届 AIC 算法应用赛道（智慧养老）参赛机器人：**哈基米-LXH**
> 基于 ROS Kinetic + Ubuntu 16.04 + 三轮全向底盘，实现「人脸核验 → 二维码解析巡视顺序 →
> 房间巡视（环境/烟雾/漏水/灯光窗帘/心率呼吸）→ 语音交互 → 老人跌倒检测与呼救 → 返回值班点」全流程。

<p align="left">
  <img alt="ROS" src="https://img.shields.io/badge/ROS-Kinetic-22314E">
  <img alt="Python" src="https://img.shields.io/badge/Python-2.7%20%2F%203.9-yellow">
  <img alt="License" src="https://img.shields.io/badge/License-MIT-green">
</p>

---

## 一、这是什么

一台面向居家/机构养老场景的护理机器人，在仿真与实车各跑通过完整流程：

| 能力 | 实现方式 |
|---|---|
| 二维码识别巡视顺序 | 三点扫描 + 后台持续解帧（poller），实测 0.4~20 s 命中 |
| 人脸身份核验 | Haar 检测 + LBPH 识别，12 帧多帧投票，最多三轮复核 |
| 房间巡视 | AMCL 定位 + move_base（GlobalPlanner + DWA），枢纽往返式动线 |
| 环境感知 | HomeAssistant REST 读取光照/温度/湿度/烟雾/水浸 |
| 家居控制 | 电灯、窗帘、风扇（开关/回读校验，窗帘 16 次回读窗口）|
| 心率/呼吸 | ESP32 UDP 广播 `seq,hr,br,people,ts`，卧室一站双播报 |
| 语音交互 | **Vosk 离线识别** + 语法受限 + 一次唤醒会话模型（每条指令续 20 s）|
| 语音播报 | **离线 WAV 查表**（134 句单音色），数字/单位片段拼接，零网络 |
| 跌倒检测 | YOLOv5 + 时序证据确认；未识别则自动复检一次 |
| 呼救流程 | 播报 + 开灯/开窗帘 + 模拟拨号音 + 104 室广播 + 退回值班点 |
| 安全 | `vel_watchdog`：0.4 s 无速度指令持续发零，防底盘锁存速度失控 |

### 实车成绩（竞赛场地，组委会设备）

```
总用时 300.9 s   失败 0   21 段导航全成功   卡住恢复 1 次
二维码 322° 命中   人脸通过   心率 76 次每分 + 呼吸 18 次每分
语音 5 条指令全中   跌倒 9 帧确认 → 呼救流程完整播出
```

---

## 二、目录结构

```
aic-carebot/
├── aic_care_ws/src/           # 自研 ROS 包（Python 2.7）
│   ├── carebot_eldercare/     # 主任务节点（预留）
│   ├── carebot_qr/            # 二维码检测服务（pyzbar + zbar 兜底）
│   ├── carebot_face/          # 人脸检测/识别服务
│   ├── carebot_iot/           # HomeAssistant 桥（设备控制 + 环境读取）
│   ├── carebot_iot_msgs/      # IoT 服务消息定义
│   ├── carebot_nav_kit/       # 地图 / move_base 配置 / 导航辅助
│   └── carebot_voice/         # 语音相关（TTS 备用链路）
├── tools/                     # 实车脚本（py2）
│   ├── m36_mission_v5.py      # ★ 全流程任务状态机（核心）
│   ├── voice_dialog.py        # 语音会话（会话模型 / 5 轮模型可切换）
│   ├── asr_relay.py           # Vosk 识别结果 → /voice_words
│   ├── tts_play.py            # 离线播报（WAV 查表 + 片段拼接）
│   ├── fall_detect.py         # 跌倒检测（YOLOv5 worker 封装）
│   ├── vel_watchdog.py        # 速度看门狗
│   ├── m40_fast_start.sh      # 一键起栈（12 s 全绿）
│   ├── camera_warmup.py       # 相机预热
│   ├── intent_match.py        # 意图匹配（相似度 0.80 + 关键字约束）
│   └── ...
├── vosk/
│   ├── vosk_asr.py            # Vosk 离线识别（py3 + arecord 直采）
│   └── voicecache/            # 离线播报缓存工具与句表
│       ├── build_voice_cache.py   # 批量生成 WAV（需联网一次）
│       ├── check_tts.py           # 静态核对每句能否拼出
│       ├── phrases.txt            # 134 条播报句
│       └── vosk_grammar.json      # 识别语法（38 字符 + 22 词）
├── fall/
│   ├── factory_fall_logic.py  # 时序证据（单人/位移/连续 1 s）
│   └── factory_fall_worker.py # YOLOv5 推理 worker（py3）
└── docs/                      # 指令库、语音方案、跌倒方案、技术报告
```

> ⚠️ 模型 `best.pt`（14 MB）与离线语音 WAV 未入库，见下文「资源准备」。

---

## 三、快速开始

### 3.1 环境

| 项 | 版本 |
|---|---|
| OS | Ubuntu 16.04 |
| ROS | Kinetic |
| Python | 2.7（ROS 侧）、3.9（Vosk / YOLO 侧）|
| 底盘 | 三轮全向（STM32 串口，115200）|
| 传感器 | 激光雷达、Astra 深度相机、ESP32 心率呼吸模块 |
| 网关 | HomeAssistant（Zigbee 灯/窗帘/插座/传感器）|

### 3.2 起栈

```bash
# 1) 编译自研包
cd ~/catkin_ws && catkin_make && source devel/setup.bash

# 2) 准备凭据（不要提交进仓库）
printf '%s' '<你的 HomeAssistant 长期令牌>' > ~/.ha_token && chmod 600 ~/.ha_token

# 3) 一键起栈 + 跑全流程
bash ~/start.sh
```

`start.sh` 会：清理残留 → 起 master 与全部节点（雷达/底盘/IMU/TF/AMCL/move_base/相机/二维码/人脸/IoT/语音）→ 等就绪 → 发起点位姿 → 启动全流程任务。

### 3.3 只起栈不跑任务

```bash
bash ~/start.sh --no-mission
```

### 3.4 资源准备

| 资源 | 说明 |
|---|---|
| 跌倒模型 `best.pt` | YOLOv5 权重（约 14 MB），**未随仓库分发**；放到 `fall/factory_fall_assets/best.pt`，或用自有数据训练同结构模型 |
| Vosk 模型 | `vosk-model-small-cn-0.22`（约 66 MB），从 [Vosk 模型库](https://alphacephei.com/vosk/models) 下载解压 |
| 离线播报 WAV | 首次联网执行 `python3 vosk/voicecache/build_voice_cache.py` 生成，之后完全离线 |

---

## 四、关键设计

### 4.1 语音：为什么用 Vosk 而不是云端 ASR

云端/厂商离线 ASR（讯飞 BNF）在噪声下置信度窗口极窄（真实指令 8~11 分、噪声 0~7 分），
且封闭文法改一次要重装重启 22 s。Vosk 全离线、支持**语法受限识别**，把 14 条指令写成
grammar 后噪声几乎不可能凑出有效句，实测真指令置信度 0.99~1.00。

### 4.2 一次唤醒的会话模型

```
第 1 轮：车主动提问（不用唤醒）
唤醒一次 → 回“我在”（不占轮次）
每条有效指令 → 窗口续 20 s（不限轮次）
“结束语音交互” / 静默 20 s / 超 180 s / 车被推动 → 结束会话
```

相比固定 N 轮，避免了「喊了唤醒词但下一句没来得及说」导致的整段结束。

### 4.3 离线播报（单音色）

所有固定播报预生成为 WAV（按文本 MD5 命名），运行时：
整句命中 → 直接播；带数字 → `前缀 + 数字片段 + 单位` 拼接；多段用 `sox` 合并再播。
**espeak 兜底默认关闭**，避免出现第二种机械音色。

### 4.4 跌倒检测

```
枢纽点停下 → 相机转向起点 → 播“开始跌倒检测”
→ 15 s 观测（YOLOv5，约 0.5 s/帧）
→ 未确认 → 播“没有检测到摔倒，我再检测一次” → 再观测 15 s
→ 确认（或第 2 次按现场要求判成功）
   → “识别到摔倒行为” → 开灯 + 开窗帘 → “您没事吧” → “为您联系护工”
   → di…di…di（本地报警音）→ “这里是104室，有老人倒地”
   → “已联系护工，我将退回到值班点” → 返回起点
```

判据（时序证据）：单人、score ≥ 0.5、相邻帧中心位移 ≤ 0.30、`fall` 连续 ≥ 1.0 s。

### 4.5 导航调参（实测最快一版）

| 参数 | 值 | 说明 |
|---|---|---|
| 卡住判定 | 5 s + 5 s | 10 秒出结论（原 20 秒）|
| 到达容差 | 0.35 m | 减少最后几厘米磨蹭 |
| 脱困动作 | 先退 0.20 m，再转 ±40°，重发目标 | 位置不变时旋转无效，必须退出膨胀口袋 |

---

## 五、许可证

本项目采用 [MIT License](LICENSE)。

其中 `fall/factory_fall_worker.py` 与跌倒模型的接口逻辑参考合作队伍的实现，
模型权重不随仓库分发；若需商用请自行确认模型授权。

---

## 六、致谢

- Vosk / Alpha Cephei：离线中文语音识别
- 赛事组委会：场地设备与网关
