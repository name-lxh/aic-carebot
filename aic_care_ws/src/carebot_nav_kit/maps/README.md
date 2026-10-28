# 地图目录

本目录存放 `carebot_nav_kit` 使用的地图。

## 文件格式

地图由两个文件组成，缺一不可：

| 文件 | 内容 |
|---|---|
| `<名字>.pgm` | 占用栅格图像。黑=障碍（0），白=自由（254），灰=未知（205） |
| `<名字>.yaml` | 元数据：分辨率、原点、阈值 |

`yaml` 里的 `image:` 字段必须与实际 `pgm` 文件名一致，否则 `map_server` 加载失败。

## 命名约定

- 导航默认读取 **`mymap.yaml`**（`navigation.launch` 的 `map_file` 参数默认值）
- 每次用 `save_map.sh` 保存时会自动在 `history/` 下留一份带时间戳的备份
- 建议按用途命名，例如 `home_field.yaml`（赛题场地）、`lab.yaml`（实验室）

## 如何生成

```bash
# 1. 启动建图
roslaunch carebot_nav_kit mapping.launch

# 2. 另开终端遥控走遍场地
rosrun teleop_twist_keyboard teleop_twist_keyboard.py

# 3. 保存
rosrun carebot_nav_kit save_map.sh -n mymap
```

## 地图质量自检

保存后**务必**肉眼检查一下：

1. 用图像查看器打开 `.pgm`
2. 检查项：
   - [ ] 墙体连续闭合，没有断裂
   - [ ] 没有大片黑斑（那是把动态障碍物或机器人自身建进了地图）
   - [ ] 没有重影/双层墙（那是里程计漂移导致的对不齐）
   - [ ] 三个房间的门口是通的，没有被"堵死"
   - [ ] 地图整体是矩形（3m×3m 场地应该接近正方形）

3. 若发现问题：
   - **重影/扭曲** → 里程计漂移。先跑 `odom_monitor.py` 检查
   - **黑斑** → 建图时有人在场地里走动，或激光扫到了自己。重建，且确认 `laser_filter` 在跑
   - **墙体断裂** → 走得太快，gmapping 扫描匹配跟不上。放慢速度重建

## ⚠️ 重要提醒

**地图与点位必须配对。** 换一张地图后，`config/waypoints.yaml` 里所有坐标
都要重新标定。建议在地图 yaml 的注释里写清楚对应的点位文件版本。

## 为什么这个目录是空的

建图必须用**实际的机器人**在**实际的场地**上跑。本目录初始为空是正常的 ——
第一次上车后按上面流程建图即可。

如果需要在没有机器人的情况下测试导航栈的其他部分，可以用：
```bash
# 用一份手工构造的简单地图验证 launch 能否正常启动
rosrun carebot_nav_kit nav_check.py    # 静态检查
```
