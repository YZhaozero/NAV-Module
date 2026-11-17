# nav_cloud_self_filter

独立 ROS 2 功能包，复用 PCL `CropBox<PCLPointCloud2>`，删除机器人固定坐标框内的自体回波。不是去噪/体素化，不改变框外点、坐标系、强度、tag、line、逐点 timestamp 或 ROS Header 时间戳。输入必须已在配置的 `expected_frame` 中；不匹配会明确报错并拒绝该帧，不自动套用 TF。

## WOOSH .5 默认区域

`config/woosh_livox5.yaml`：单位 m，坐标 `base_footprint`，X [0.34, 0.48]、Y [-0.25, 0.25]、Z [0.30, 1.65]，边界包含在剔除框中。

此框根据 25 帧实测近距表面（X 0.365..0.451、Y -0.224..0.222 m）和照片中的立柱高度留余量设置。样本中约 38% 的点重合在安装点 [0.429,0,0.605]，与零距离回波经安装外参变换后的结果一致；框同时剔除这部分点及柱面点。该框只覆盖柱体朝向 .5 雷达的可见表面及安装点，并非整机精确碰撞模型。改装、移动雷达或更换机器人后需重新确认边界，不直接沿用此参数。

## 构建与独立启动

```bash
cd ~/nav-current  # 开发机为 /home/zy/ws/NAV-Module
source scripts/runtime_environment.sh
colcon build --symlink-install --packages-select nav_cloud_self_filter nav_app
source install/setup.bash
ros2 launch nav_cloud_self_filter livox5_self_filter.launch.py
```

输入 `/livox/lidar_192_168_128_5`，输出 `/nav/livox5/points_filtered`。独立启动仅处理已存在的话题；不启动传感器、建图或底盘。

默认 `woosh_native5.launch.py` 已自动启动此节点并给 FAST-LIVO2 接入输出。不要与默认建图重复启动同一个过滤器。参数页的“点云输入”仍指原始输入话题，先经过本包再进入估计器；IMU 和 /foo 相机保持原来的接线。

需要原始数据对照时：

```bash
ros2 launch nav_app woosh_native5.launch.py self_filter:=false
```

这将旁路 CropBox，仍经同一输出话题传输原始消息。也可用 `ros2 param set /nav_livox5_self_filter enabled false` 暂停裁剪；修改 min/max 或 expected_frame 需要编辑 YAML 并重新启动当前建图。边界参数为只读，以免参数已改、实际框未更新。节点每 5 秒输出输入/剔除/保留点数及耗时。

## 验证与使用限制

测试检查柱体点、零距离安装点被剔除，地面/框外点保留、原始字段和时间戳逐字节一致、空输出后能继续处理、坐标不匹配报错以及旁路功能。原始话题不变，已有 PCD/栅格/SC 不会被修改。

已经积累进 PCD 的自体轨迹不能靠新输入滤波自动擦除，需要重新建图；仅凭旧全局 PCD 无法可靠区分这些点与真实障碍物。照片和点云支持自体回波来源，但旧地图中的整个长条仍需重新采集同一路线做对照验收。

PCL 算法说明：https://pointclouds.org/documentation/classpcl_1_1_crop_box.html

## 2026-11-17 设备验证

- 开发机及 Jetson 构建通过；过滤器 4 项真实 ROS 节点测试、1 项嵌套启动参数回归与原建图配置 7 项回归通过。
- 105 帧实测数据共 1,743,552 点，保留 1,056,916 点、剔除 686,636 点；630,058 个安装点重合回波全部去除。
- 每帧对照原始消息：框外点记录、字段列表、PointCloud2 点步长和 Header 纳秒时间戳完全一致。
- Jetson 日志记录每帧约 1.2～1.4 ms；验证前后雷达驱动 PID 不变。验证没有修改用户已有地图，尚不代表沿原路线重新移动建图验收。

过滤后完整链路已验证：FAST-LIVO2 实际输入为 `/nav/livox5/points_filtered`，配对 IMU 和 /foo 相机接线不变，真实静止建图、VIO 及临时 PCD 保存成功；NAV 已重开并继承新包环境。独立启动的自定义配置参数名为 `filter_params_file`，避免与外层估计器的 `params_file` 冲突。
