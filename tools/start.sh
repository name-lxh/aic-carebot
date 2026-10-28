#!/bin/bash
# ============================================================================
#  start.sh —— 一键启动（放在 ~ 下，车上直接 `bash ~/start.sh` 或 `./start.sh`）
# ============================================================================
#  做的事（全程并行，目标 30 s 内开始跑流程）：
#     1. 杀掉残留（含 astra 相机，避免抢 USB）
#     2. 起 master + 全部节点（雷达/底盘/IMU/TF/odom/滤波/地图/amcl/move_base/
#        相机/二维码/人脸/IOT/语音）
#     3. 等就绪（话题有数据 + TF 通了 + 服务在 + move_base 在线）
#     4. 自动发起点位姿 (0,0,0) 并立刻启动全流程任务
#
#  用法：
#     bash ~/start.sh              # 起栈 + 自动跑全流程
#     bash ~/start.sh --no-mission # 只起栈，不跑任务（调试用）
#     bash ~/start.sh --tap        # 起栈后进入"手动点单段"模式（不自动跑）
# ============================================================================
set -u
LXH=/home/ub/Hajimi/LXH

ARGS=""
for a in "$@"; do
    case "$a" in
        --no-mission|--tap) ;;
        *) ARGS="$ARGS $a" ;;
    esac
done

if [ "${1:-}" = "--no-mission" ] || [ "${1:-}" = "--tap" ]; then
    exec bash "$LXH/_tools/m40_fast_start.sh"
else
    exec bash "$LXH/_tools/m40_fast_start.sh" --mission
fi
