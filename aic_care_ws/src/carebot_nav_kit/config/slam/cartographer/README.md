# cartographer 配置目录（占位）

本目录预留给 cartographer 方案。**当前为空是有意的** —— 本项目在 3m×3m 小场地上
实测 gmapping 已足够，且 cartographer 对 CPU 要求明显更高（树莓派/低配迷你主机
上跑不动）。

`mapping.launch` 里有一个 `slam:=cartographer` 分支会引用本目录下的
`carebot_2d.lua`。**在你确认要用 cartographer 之前，不要加 `slam:=cartographer` 参数。**

如需启用，需要准备：

- `carebot_2d.lua` —— cartographer 的 2D SLAM 配置
- `backpack_2d_imu.lua` 或类似 —— 若要用 IMU

并用 apt 安装：
```
sudo apt install ros-kinetic-cartographer-ros
```

参考配置可从 `/opt/ros/kinetic/share/cartographer_ros/configuration_files/` 拷出来改。
关键改动点：
  · `tracking_frame` / `published_frame` / `odom_frame` 改成 base_footprint / odom
  · `num_laser_scans = 1`，`num_subdivisions_per_laser_scan = 1`
  · `TRAJECTORY_BUILDER_2D.min_range` / `max_range` 与雷达匹配
  · `use_odometry = true`

⚠️ 但请注意：**gmapping 在本赛题场景下已经够用且更省算力**，
   换 cartographer 的收益主要体现在大场地和强回环场景，本项目不太需要。
