# NAV-Module

ROS 2 Humble 机器人建图、点云定位与导航工作区。当前以 LiDAR / IMU 建图、Localizer 和 Nav2 为核心，支持 DLIO 与 FAST-LIVO2，并接入地图包、录包回放、日志诊断和独立 RTK；不引入 Gaussian Splatting 或步行引导业务。

## 目录与入口

保持现有工作区结构，ROS 包仍放在 `src/`：

| 目录 | 用途 |
|---|---|
| `src/nav_app/` | Qt 操作界面、建图和导航启动控制 |
| `src/nav_driver/` | Livox MID-360、RealSense 驱动 |
| `src/nav_slam/` | DLIO、FAST-LIVO2、视觉依赖 |
| `src/nav_localization/` | Localizer、Scan Context、手动重定位 |
| `src/nav_navgation/` | 现有 Nav2、SCAN-Planner，保留原目录拼写 |
| `src/nav_sensor_proc/` | 点云转 LaserScan |
| `src/nav_map/` | 本地地图 |
| `src/nav_rtk/` | 独立 RTK 串口/NTRIP、协议解析与 GNSS 监测 |

环境安装见 [部署说明](DEPLOYMENT_UBUNTU22.md)，界面操作见 [App 说明](src/nav_app/README.md)。本轮改动、接口、验证命令和现场验收边界见 [建图导航移植说明](NAVIGATION_MIGRATION.md)，第二批功能与改名记录见 [工具与 RTK 移植说明](OPERATIONS_MIGRATION.md)，最近状态见 [PROJECT_STATUS.md](PROJECT_STATUS.md)。

## 构建与运行

```bash
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
ros2 launch nav_app ar_nav.launch.py
```

“工具”中包含地图包、录包回放、日志、诊断与 RTK。RTK 的 ENU 仅作观测，不向导航发布 TF，也不代表已经完成 GNSS/LIO 融合。应用入口使用系统 Python 依赖，避免用户 NumPy 与系统 SciPy 混用。

默认保留 DLIO。进入“导航系统 → 里程计”选择 FAST-LIVO2；或在“建图”页选择 FAST-LIVO2。无相机标定的机器人先关闭视觉验证 LiDAR / IMU 链路，启用视觉前核对本机内外参、图像分辨率与时间。

建图与导航互斥启动。使用 Localizer 时，由其发布 `map → odom`，Nav2 只启动地图服务和导航节点。速度沿用现有 `/cmd_vel` 接口；定位失效会请求取消导航，但软件取消不等于底盘已经物理停止。

[工作区默认配置](colcon_defaults.yaml) 对裸 `colcon build` 生效，`./build.sh` 也使用同一配置。默认构建 App 的机器人运行依赖，Nav2 使用已安装的 Humble；工作区不再保存 Nav2 本体源码。Ninja 每次最多两路编译、一路链接，默认不编译测试。通过 `PCL_ROOT=/usr` 选择系统 PCL，路径由 CMake 按本机架构查找，不再要求手填 x86 路径。

需要额外的现有包时可显式选择，例如 `colcon build --packages-up-to icp_registration manual_relocalization_tool nav_app`。Nav2 的启动封装和参数入口保存在 `src/nav_app`，实际节点与默认参数由系统 `nav2_bringup` 提供；Qt 也可选择自定义参数 YAML。

工作区曾从 `GS-NAV-Module` 改名为 `NAV-Module`。旧 CMake 缓存不能搬到新路径继续使用；本次已将失效 `build/`、`install/` 备份到 `/tmp/nav-module-before-rebuild-20261112-Keb9rwmB/`，随后在原名称下重新生成。不要把备份中的缓存复制回来。
