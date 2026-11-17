# nav_rtk

ROS 2 Humble 独立 RTK 包，由固定来源 `nav-monorepo@aafb462ea8f7875205da888946d05c3d31bc8477` 的协议解析、串口桥接和 GNSS 监测合并而来。无 GS、App 或导航框架依赖。

- `nav_rtk/parser`：WTRTK/NMEA、接收机原始测量与 RTCM、串口运行时、NTRIP 和账号存储。
- `nav_rtk/bridge`：串口发现、ROS 桥接、历元合并、接收机指令目录与报文诊断。
- `nav_rtk/gnss`：GNSS 时间/质量检查、ENU、时间偏移估计。

## 使用

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
# 明确选择实际串口，搜索和测试不会自动打开硬件
ros2 launch nav_rtk rtk.launch.py port:=/dev/ttyUSB0 baudrate:=230400
ros2 launch nav_rtk monitor.launch.py fix_topic:=/rtk/fix
# CSV 列为 timestamp,speed；估计值不自动应用
ros2 run nav_rtk gnss_estimate_offset lio.csv gnss.csv
```

界面入口为 `nav_app → 工具 → RTK`，可以搜索串口、连接/断开、查看状态与原始数据、确认后发送接收机指令，并配置 NTRIP。连接会按 `config/rtk.yaml` 的 `record_raw_gnss` 设置管理接收机原始测量输出；这不是纯被动串口监听。

| 话题 | 类型 | 用途 |
|---|---|---|
| `/rtk/frame` | `std_msgs/String` JSON | WTRTK 帧 |
| `/rtk/status` | `std_msgs/String` JSON | 串口、差分、接收机和原始测量状态 |
| `/rtk/observation` | `std_msgs/String` JSON | 归一化历元观测与接收时间证据 |
| `/rtk/fix` | `sensor_msgs/NavSatFix` | 带接收机时间的定位结果 |
| `/rtk/enu` | `geometry_msgs/PointStamped` | 监测器的局部 ENU，首个有效观测为原点 |
| `/rtk/diagnostics` | `diagnostic_msgs/DiagnosticArray` | GNSS 质量、有效/拒绝计数 |

监测器默认最大时间年龄 1 秒、水平标准差 0.5 米、速度跳变阈值 15 米/秒；均是筛选门限，不是精度承诺。未知、过期或缺少有效质量证据的观测不可被解释为有效地图定位。不会发布 `map → odom` 或其他导航 TF，不自动融合 LIO，不自动套用时间偏移或坐标系对齐。

账号仍用 `~/.config/nav/ntrip.json`（0600）保存，以兼容既有数据。保存不会启用 NTRIP；服务商连接需明确请求。日志路径中的 `nav`、消息内部协议标识和旧文件名不是 ROS 包名，保留兼容。

验证采用协议样例、伪终端和隔离 ROS 域，283 项 RTK 回归通过。真实接收机、差分服务、天线杆臂、坐标基准和定位精度尚未现场验收。完整移植记录见 [第二批说明](../../OPERATIONS_MIGRATION.md)。
