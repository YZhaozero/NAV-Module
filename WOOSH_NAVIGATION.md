# WOOSH：NAV-Module 自主定位与规划

当前 WOOSH 导航使用本机 `.5` 雷达和配对 IMU，经 FAST-LIVO2 提供里程计、点云 Localizer 提供地图定位，再由 Nav2 计算路线。相机1继续显示桥接画面；按用户要求，导航预设已恢复开启视觉融合，保留开关，建图配置不变。底盘执行不接入，操作者使用手柄行走。

## 2026-11-18：GS 步行规划启动流程

进入导航后的启动顺序：确认/启动独立 `.5` 雷达服务 → FAST-LIVO2（雷达＋IMU＋相机）及机身滤波 → 自有 `/scan` → 点云 Localizer → `nav_app/walking_planner.launch.py`。实机 .5 预设选择「由 NAV 启动」时，结束导航会在算法退出后关闭 nav-livox5.service，下次进入导航重新采集。相机继续预览。

只规划后端沿用 GS 的拆分：仅启动 `map_server`、`planner_server`、`lifecycle_manager_walking`。不启动 controller、BT navigator、behavior、waypoint follower、velocity smoother、motion guard 或底盘适配器。保留 WOOSH 的机身轮廓和实时 scan，地图与规划器必须实际 Active，定位、里程计、scan 有效后才允许提交 ComputePathThroughPoses。

定位尚未完成或输入暂时中断时显示等待，暂停新的路线规划，保留进程与地图；输入恢复后重新就绪，不自动重发路线请求。算法进程意外退出仍报错，显式停止仍按反向顺序退出受管组件。AMCL 未启用。

使用：打开桌面 NAV → 选择地图包 → 进入导航 → 等待就绪（必要时点击“重新定位”）→ 在栅格地图选点 → 点击“规划路线” → 按显示路线用手柄行走。导航接入设置中的 `.5 雷达＋IMU + 相机融合` 为当前预设；“FAST-LIVO 视觉”默认开启，开关保留。恢复后的完整导航验收尚未通过，当前仍需排查 IMU 接收异常。

本地回归 82 项通过。第一轮视觉融合开启时未通过完整定位验收；同场地、用户确认车未移动时，关闭视觉融合后完成实际地图定位与路线规划（44 个点，约 1.11 m），无运动 action 调用，`/cmd_vel` 和 `/foo/nav_cmd_vel` 无发布者。该对照仅为关闭视觉时的历史验证，不能直接证明相机标定或同步问题的唯一根因，也不代表运动执行或定位精度验收。

第二次冷启动成功，运行 75 秒后再次规划得到 43 个点、约 1.11 m 的路线；期间曾有短暂定位失效，系统暂停规划并恢复，未停止整套进程。两次设备上打开导航系统页耗时约 163 ms、195 ms。这是使用真实传感器的离屏 Qt 验证，不等同于物理触摸、行驶精度或长期稳定性验收。

设备现有 `~/.config/NAV/NavigationAccess.conf` 已仅将 `profiles/woosh` 的 `fast_livo_vision` 恢复保存为 true，其他输入设置保持不变；已运行的 APP 需要重新打开以加载该设置。原设置保存为备份目录下 `NavigationAccess.conf.before`；原接入页源码为 `navigation_access.py.before`。完整结果分别为 `lio-planning-acceptance.json` 和 `lio-repeat-planning-acceptance.json`，失败的视觉融合首轮也保留在 `startup-acceptance.json`。

设备备份：`~/nav-deploy-backups/20261118-gs-walking-startup/`；源码主体在 `before.tar.gz`，后续接入设置源码与用户设置单独备份。回滚主体：停止 APP 后，在 `~/nav-current` 解包 `before.tar.gz`，恢复接入设置备份，再 `source scripts/runtime_environment.sh`、`colcon build --packages-select nav_app`，重新启动 APP。新增 walking launch/config 在旧代码下不再被调用。

以下日期段保留历史记录；关于完整 Nav2 启动及旧默认传感器的说明由本节替代。

## 2026-11-17：手柄行走、只规划路线与页面显示

当前 WOOSH APP 默认只计算路线：进入导航启动 FAST-LIVO、点云定位与 Nav2，地图上选点后点击“规划路线”，路线在当前大地图显示；由操作者用手柄驾驶。不会提交 NavigateThroughPoses 运动目标，底盘执行适配器继续关闭。WOOSH 启动参数 operating_mode=walking 用于这个只规划流程，Nav2 就绪只要求 planner_server，不把运动控制或导航任务服务作为规划前置条件。

相机在导航地图预览和 AR 预览中显示旋转 180°；同画面的路线叠加一起旋转。原始图像、CameraInfo、FAST-LIVO 输入和安装外参不变。

状态持续在现有独立 ROS 线程中查询，空闲后仍保持回执缓存新鲜；暂时等待回执的提示会在恢复后更新，不会自动重发规划请求。修复前“空闲一段时间后第一次点击”的场景未被上一轮持续验证覆盖，本次已增加对应回归与实机验证。

导航系统页不再进入时重放整段日志；仅将当前标签最近 3000 行按 150 ms 批量显示，后台不更新隐藏文本控件，完整日志继续写入原日志包。1500 批日志压力测试：本地进入页面约 1.22 s → 0.03 s；设备约 0.067 s，实际算法运行中页面切换约 0.25 s。这是本次测量值，不是全部运行情形的触摸延迟保证。

实机验证沿用用户当前地图 /home/nvidia/nav_maps/nav_map_20261117_10.yaml 和 nav_map_20261117_101756_555.pcd；空闲 12 秒后首次点击规划成功。目标 (1.45, 1.12) 得到 85 个点、约 2.19 m 的路径，保持大地图预览，未发送运动目标。相机转正和路线预览截图、planning-acceptance.json、性能与构建日志保存在 /home/nvidia/nav-deploy-backups/20261117-planning-ui/。

相关回归：本地及设备各 134 项通过，另 3 项地图/规划预览回归通过；原先已确认失效的 1 项旧控件测试仍排除。修复前源码备份为该设备目录下 source-before.tar.gz，原传感器与地图文件保留。APP 已恢复，操作步骤是选择现有地图 → 进入导航 → 地图选点 → 规划路线 → 用手柄行走。

## 2026-11-17：路线服务状态与传感器误停修复

本次确认了两类问题：原始图像把百兆链路压满，确实出现相机空档；界面繁忙时，状态与传感器回调也可能迟处理，导致把正常输入判成未就绪或断流。

- Nav2 GetState 回执、雷达 / 相机 / scan 的接收和定位质量接收复用现有独立 ROS 线程。图像转换仍使用原有有界 worker。启动超时、新鲜度、定位质量、真实断流停止和底盘执行限制保留，没有放宽阈值。
- 提示区分“等待 Nav2 状态回执”及具体未激活组件。启动、输入中断和主动停止原因保存到每次运行日志包的 navigation/stack.log。
- 底盘 ROS1 相机约 29.71 Hz，每帧 691200 字节，原始数据约 164 Mb/s；网口只有 100 Mb/s，实测原接收接近 97 Mb/s。仅限原始帧率的对照未通过持续验收，最终使用“5 Hz 限帧 → 无损 PNG → 一次跨网接收 → 开发板本地 raw 转发”。
- 相机链路：ROS1 /camera_1/color/image_raw → /nav/camera_1/image_throttled → /nav/camera_1/image/compressed → ROS2 /foo/nav/camera_1/image/compressed → /foo/camera_1/color/image_raw。算法与界面继续使用原 raw 话题，分辨率、原始 Header、RGB 像素和 CameraInfo 保留。真实同步采样 107 帧逐时间戳比对，像素哈希、编码、尺寸和坐标帧全部一致。
- 底盘原 bridge.yaml 未修改。相机1条目在启动时改用压缩传输，其它原桥接条目与 .3 IMU 独立桥接保留。相机硬件与雷达驱动未重启。
- 新增 nav-camera1.service 为共享 PNG 接收器；WOOSH APP 或原生 .5 启动时按需启动，停止导航保留接收器。不启用该服务或 .5 雷达服务的开机自启。ROS1 编码器独立加载 Noetic 环境，避免与桥接内 ROS2 同名库冲突。

设备安装接收器（已在当前设备完成）：

    cp deployment/nav-camera1.service ~/.config/systemd/user/
    systemctl --user daemon-reload
    systemctl --user start nav-camera1.service
检查：

    systemctl --user status nav-camera1.service nav-livox5.service

完整验证与失败对照均保存在 Jetson /home/nvidia/nav-deploy-backups/20261117-route-readiness/，没有把启动进程等同于有效导航。底盘桥接原文件保存在 /home/woosh/nav-backups/20261117-camera-bandwidth/autostart.sh.before 和 bridge.yaml.before；回滚时先停止导航与 nav-camera1.service，恢复桥接脚本和 APP 源码后重建 nav_app，仅重启桥接，不重启底盘或传感器驱动。操作前先保存当前工作。帧率降低后的视觉效果与移动导航尚需实车验收，本次未发送运动目标。

最终验证：本地与 Jetson 各 114 项相关回归通过，另 1 项既有 UI 测试被排除（使用修复前备份也会因 unknown_policy_combo 已不存在而失败）。nav_app 构建通过。真实 .5 + 配对 IMU + 相机1 + 自有 scan + 点云定位 + Nav2 静止验收通过：启动后运行约 160 秒未再自动停止；最后一分钟每 5 秒采样共 12 次，里程计与定位全部有效，规划器 / 控制器 / 导航任务均为 Active；最终网口平均接收约 44 Mb/s。/cmd_vel 与 /foo/nav_cmd_vel 无 NAV 发布者；未发送运动目标。验收进程结束后恢复桌面 APP。acceptance.json 与 png-integrity.json 为对应证据。统一原始运行代码回滚包为设备备份目录下的 nav-route-rollback-runtime.tar.gz；恢复旧链路时先停止 nav-camera1.service，恢复 APP 原始代码及底盘原始桥接脚本，再重建、重启 APP 和桥接。

## 2026-11-17：.5 雷达 + 自带相机导航预设

入口：设备与设置 → 导航接入 → 选择“WOOSH · .5 雷达 + 自带相机”→ 载入预设 → 保存接入设置。然后从地图包选择完整的 PCD/YAML 地图，再在首页进入导航。旧的 WOOSH 桥接预设继续使用 .3，可随时切回。

| 输入 | 预设选择 |
| --- | --- |
| 雷达点云 | `/livox/lidar_192_168_128_5`，开发板原生 ROS2 |
| 配对 IMU | `/livox/imu_192_168_128_5`，开发板原生 ROS2 |
| 自带相机1图像 | `/foo/camera_1/color/image_raw` |
| 相机1内参 | `/foo/camera_1/color/camera_info`，启动时读取实时 CameraInfo |
| 避障 scan | `/scan`，由 .5 点云自行生成 |
| 里程计 / 地图定位 / 导航 | FAST-LIVO2 / 点云 Localizer / Nav2 |

这套预设复用现有 `woosh_native5.yaml` 和 `woosh_native5.launch.py`：.5 驱动已把点云转换到 base_footprint，使用现有 30° 俯仰、[0.429, 0, 0.605] m 安装参数的逆变换接入物理 IMU，并保留 .5 的车身自滤波。导航额外启用真实采样时刻的传播里程计，按 .5 安装外参转换到 `/nav/woosh/odom`；不会套用 .3 的 IMU 原点。

处理链为 `.5 原始点云 → 车身自滤波 /nav/livox5/points_filtered → pointcloud_to_laserscan → /scan → Nav2`。投影坐标为 base_footprint，默认保留高度 0.05–1.50 m 的点，量程 0.45–10 m，扫描周期 0.1 s；高度和量程可在“导航系统 → LaserScan”调整。

雷达选择“由 NAV 启动”，相机继续“使用已有话题”，LaserScan 来源为“由 NAV 将点云转换”。进入导航时先启动 / 复用 `nav-livox5.service`，收到真实雷达数据后再启动 FAST-LIVO 和后续处理；服务启动失败或数据超时会中止并显示原因。已有雷达数据时跳过重复启动，停止导航保留共享采集服务，不修改开机自启状态。选择“使用已有话题”或回放数据时不会启动硬件。原有 .5 建图入口保持原来的驱动启动方式。

补充验证（雷达启动顺序）：本地和 Jetson 各 61 项导航启动相关测试通过；nav_app 构建通过。设备通过新控制器实际调用共享雷达服务，收到 3 帧真实点云和 31 条 IMU 后才继续；服务 PID 前后均为 4709，没有重复启动。下游 FAST-LIVO / Localizer / Nav2 使用模拟进程，未启动整套导航、未发送运动目标；没有停掉正在使用的雷达做断电冷启动测试。设备当前 .5 接入设置已改为“由 NAV 启动”，打开中的旧 APP 需保存工作、退出重开后生效。备份及测试证据：/home/nvidia/nav-deploy-backups/20261117-native5-autostart/。

“WOOSH 雷达安装”明确区分 .3 与 .5。切换安装配置时会清空高级页中上一次的 LIVO 自定义文件选择，采用对应雷达默认参数，原 YAML 文件保留；相机配置保留。原生 .5 预设当前使用 FAST-LIVO2，不允许直接套用 .3 的 DLIO 安装参数。

本次范围为新增接入预设、正确安装配置、自有 scan 生成及启动参数：已只读确认真实 .5 点云、IMU、相机1图像及内参有数据；相机为 640×360，原始静止 IMU 约 1 g。没有启动整套导航或发送运动目标，本次不代表之前的持续定位时延问题已解决。底盘执行适配器保持关闭，Nav2 输出仍与底盘速度入口隔离。

代码更新不会自动切换当前保存的接入配置。设备当前 APP 如仍打开，先保存正在进行的工作，退出程序后再从桌面重新打开，才能看到新选项。


验证记录：本地和 Jetson 各 58 项相关回归通过，两个包的构建及启动参数检查通过。真实 .5 数据短测产生 113 条 LaserScan，约 10.04 Hz，113 条均与原始点云时间戳对应，输出 frame 为 base_footprint。测试使用独立诊断话题，正式预设输出 /scan；测试处理节点已停止，当前 APP 和 nav-livox5.service 保持运行。本次备份和日志在设备 `/home/nvidia/nav-deploy-backups/20261117-native5-navigation-preset/`。

## 2026-11-16 FAST-LIVO IMU 与导航 TF 修复

- 原车 `.3` IMU 保持原始单位与 frame，经独立 `ros_bridge_imu` 进程传输，队列与 QoS depth 为 2000；其余原有桥接条目、话题名和数据内容保留。持久脚本为原车 `~/ros2_humble_ws/autostart.sh`，源码副本为 `deployment/woosh_bridge_autostart.sh`。
- FAST-LIVO 直接订阅 `/foo/livox/imu_192_168_128_3`，在 C++ 输入边界将加速度及协方差分别乘以 9.80665 和其平方，核对 `livox_frame`，再使用已标定的物理 IMU 坐标。角速度、原始采样时间不改。DLIO 的 `/nav/woosh/imu3_si` 转换节点保留；FAST-LIVO 不再经过该 Python 转发。
- 原生 IMU 接收和导航位姿输出各使用独立 DDS 参与者，避免与大消息共用接收/发布通道。导航启用 `imu_rate_odom:=true`：逐条积分已收到的 IMU，发布真实采样时刻的 50 Hz `/LIVO2/imu_propagate` 和唯一的动态 `odom → woosh_livox3_imu` TF。`woosh_odometry` 按安装外参转换到 `/nav/woosh/odom`，child 为 `base_link`，同时修正旋转、角速度杠杆臂及协方差；Nav2 使用此底盘里程计。
- Localizer 仍使用同一扫描坐标契约的 `/fast_livo/cloud_body` 与 `/aft_mapped_to_init`；没有给旧扫描改成当前时间。Localizer 仅在定位有效时以 20 Hz 发布当前的 `map → odom` 对齐。完整坐标链经物理 IMU 和静态安装关系连接到底盘及 `/foo/scan` 的 `laser_link`。
- 导航就绪需同时具备新鲜里程计、有效定位与对应 TF；首次自动匹配若因失效输入被取消，会重新收集观测。真正配准失败、人工取消和里程计坐标重置仍保留原有处理，未放宽 IMU 断档或定位阈值。
- 使用：导航接入选择“WOOSH · 桥接传感器 + FAST-LIVO”，里程计为 FAST-LIVO2，定位为点云 Localizer，scan 为 `/foo/scan`；从地图包选好同源 PCD/YAML 后在首页进入导航。不启动 AMCL、本地传感器驱动、点云转 scan 或底盘执行适配器。`enable_base_control` 仍为 false，规划速度保持隔离。

### 本次验证范围与剩余问题

本地及 Jetson 的针对性回归各 114 项通过；另排除 2 项：依赖已删除地图控件的旧测试，以及并行地图/轨迹页面工作中历史列表刷新测试。完整组合曾记录后者失败，未按全量通过汇报。本次未修改这两项页面行为。原生传输回归在两台机器上各完成两轮 40/40 消息检查，覆盖 IMU/大数据的 DDS 参与者隔离、重映射、仿真时钟与独立关闭；初始化观测窗口的七组原生回归通过。

使用现有 `906-261116` 地图包、不发送任何路线目标的约 3 分钟整套实测中，FAST-LIVO、点云 Localizer 和 Nav2 持续运行到结束，8 个 Nav2 生命周期节点均为 Active；完成 129 次有效地图匹配，最后 fitness 约 0.000474。`imu_gaps=0`、无跟踪暂停、传播输出无积压。`odom → base_link` 的 95% 延迟约 0.061 秒、最大 0.211 秒；`map → odom` 的 95% 延迟约 0.109 秒、最大 1.823 秒。原始验收数据在设备备份目录的 `acceptance-imu-tf-running-3min.json`。

**完整持续定位验收尚未通过**：该次脚本要求连续 60 秒定位有效，实际因点云观测年龄偶尔超过 1 秒而未达标（原始 JSON 的 passed=false）。未放宽新鲜度、IMU 断档或配准阈值。以上结果证明 IMU 与底盘 TF 修复的效果，不能作为可移动导航或长期稳定定位的通过证明。另一份“nav_map · 地形分析”地图初始全局匹配返回 `GLOBAL_GEOMETRY_AMBIGUOUS`；该地图未改动，需单独核对或由操作者提供定位先验。

独立单位/几何检查中，原始静止 IMU 约 0.994 g，估计器输入约 9.748 m/s²；1233 个同时间戳的底盘 Odometry/TF 数值比较最大误差约 4.4e-16，记录在 `native-direct-imu-contract.json`。验证未启动 AMCL，`/cmd_vel` 和 `/foo/nav_cmd_vel` 无 NAV 发布者。

单独桥接点云、再分离原生图像接收与 UDPv4 的试验未通过原生大消息接收验证，已撤回；未持久设置 UDPv4，也未删除共享内存资源。当前保留独立 IMU bridge、独立原生 IMU 接收与独立小消息位姿输出，地图对齐 TF 限为 20 Hz。

设备备份及全部实测日志：`/home/nvidia/nav-deploy-backups/20261116-fast-livo-imu-tf/`。原车 `bridge.yaml.bak.20261116-fast-livo-imu-tf` 和 `autostart.sh.bak.20261116-fast-livo-imu-tf` 保留。回退需要定向恢复对应源码并重新编译 `fast_livo`、`localizer`、`nav_app`；FAST-LIVO 的实现包含动态库，单独替换主可执行文件不是完整回退。

## 桌面操作

1. 双击“识动 NAV / NAV”。WOOSH 地图默认保存到 `/home/nvidia/nav_maps`，与源码 release 分开；如已设置 `NAV_MAP_DIR` 则使用该目录。
2. 进入“地图管理 → 地图包”。选中已有完整地图包，点击“用于导航”。原始 PCD 可通过“从 PCD 生成”准备同坐标的 YAML/PGM 和 Scan Context。
3. 返回系统首页，点击“进入导航”：打开导航页并自动启动 FAST-LIVO2 → 点云 Localizer → Nav2，无须再到导航系统页点击“按顺序启动”。WOOSH profile 不启动本地雷达或 RealSense 驱动，使用实时 bridge 话题并关闭仿真时钟。
4. 缺少完整地图包或建图尚未停止时，页面显示原因；启动中/运行中重复进入不创建第二套程序。“导航系统”页仍可查看参数、日志、停止或重试。等待里程计、定位和 Nav2 就绪。需要时使用重定位页的全局重定位或人工先验；定位质量未确认时不执行路线。
5. 在地图中设置目标、预览路径。底盘执行保持未启用；现有运动守卫仍要求真实反馈、定位质量和操作许可。
6. 停止导航系统后再进入建图，避免两个估计器同时发布同一 TF。

导航页不再独立猜测“最新 PCD”和“最新 YAML”是一套地图。必须从同一地图包选择，避免两份地图原点不同。

## 接口与坐标

- 输入：`/foo/livox/lidar_192_168_128_3`、`/foo/livox/imu_192_168_128_3`，由现有 bridge 提供。
- FAST-LIVO 直接读取原始 `.3` IMU，并在 C++ 中做 SI 适配；`/nav/woosh/imu3_si` 继续供 DLIO 使用。沿用 `woosh_fast_livo.yaml`、相机1 CameraInfo 和安装外参。
- Localizer：消费 `/fast_livo/cloud_body` 与 `/aft_mapped_to_init`，保留同一扫描时间戳和 IMU 机身坐标契约；发布 `map → odom`。
- FAST-LIVO2：发布 `odom → woosh_livox3_imu`，再通过已标定静态变换接到 `base_link → base_footprint`；不把 IMU 坐标点云简单改名为 base_link。导航系统高级页面仍可手动选 DLIO。
- 避障：直接使用 `/foo/scan`，补上原车 `base_footprint → laser_link` 高度 0.329 m 的静态关系。
- Nav2 轮廓引用原车当前八边形配置，覆盖约 0.58×0.55 m；不沿用 TurtleBot 默认圆。实际轮廓、驱动方式和运动学在启用执行前仍须核对。
- 规划仅允许已知区域；不会将未观测区域强制改为自由空间。
- 控制器和行为服务器输出 `/nav/woosh/controller_cmd_vel`，平滑后输出 `/nav/planner_cmd_vel`。没有直接输出到 `/cmd_vel`、`/foo/base_cmd_vel` 或原车 `/nav_cmd_vel`。

## 命令行验证

不能与界面已启动的导航/建图重复运行。在 Jetson 同一 DDS 域内：

```bash
source ~/nav-current/scripts/runtime_environment.sh
export ROS_DOMAIN_ID=0 OMP_NUM_THREADS=4 OPENBLAS_NUM_THREADS=1
ros2 launch nav_app woosh_navigation.launch.py \
  map:=/绝对路径/地图.yaml \
  pcd:=/绝对路径/同一地图.pcd
```

默认 `enable_base_control:=false`。可以调用 `/compute_path_through_poses` 做路径计算；不要把路径计算成功理解为底盘已获得执行许可。

## WOOSH 厂家执行适配器

`woosh_base_adapter` 预留对接 PDF 中的原生 ROS 2 SDK：

- 输出服务：`/woosh_robot/robot/Twist`，`woosh_robot_msgs/srv/Twist`。
- 反馈：`/woosh_robot/robot/PoseSpeed`、`DeviceState`、`Mode`。
- 将 Nav2 速度送入既有 `/nav/velocity_request → motion_guard → /nav/base_command`；只有通过守卫的命令才由适配器持续请求厂家服务。
- 急停位、控制模式及实际速度必须新鲜；失效时发送零速。服务响应只代表软件接收，不是电机动作确认。
- 当前仅支持前后速度和角速度；侧移请求被明确拒绝。厂家 SDK 的实际频率、许可模式、停止行为与底盘运动学尚未完成实机验证。

设备没有 `woosh_robot_agent`/`woosh_robot_msgs`。需要厂家提供适配 Ubuntu 22.04 ARM64、ROS 2 Humble 的安装包；目录中的 ROS 1 `woosh_msgs-1.1.8` 不能替代。当前不会自动安装或启动执行端。

后续显式启用还需给出厂家确认的 `required_control_mode`；默认 0 会拒绝启动适配器。即便适配器启动，运动守卫也默认未武装。当前阶段不要启用该参数。

## 地图质量限制

保存成功只证明文件完整，不等于地图几何正确。`woosh3-before-reopen-20261113-155022.pcd` 曾检测出约百万米至千万米级坐标，不能用于导航；其发散原因仍需单独诊断。不要把该图登记为可导航地图。

本次只用坐标范围正常、仍保留的真实小图验证模块链路。静止/小范围地图不能代替完整移动建图、通行区域、停止距离和长期定位稳定性验收。


## 2026-11-13 验证记录

- Jetson ARM64 上 `direct_lidar_inertial_odometry` 和 `nav_app` 编译通过；导航与界面相关 34 项测试通过。
- 原有世界点云保留。新增机身点云与扫描时刻里程计，消除了 Localizer 的 `cloud_odom_frame_mismatch`。实测 14530 点时间戳完全配对，重建世界坐标的最大误差约 1.4 微米。
- 使用 `/tmp/woosh-nav-acceptance` 中从真实雷达抓取的临时点云验证：全局重定位成功，匹配 fitness 约 0.00587；map_server、planner_server、controller_server、bt_navigator 均为 Active。
- 未启动厂家执行适配器。确认 `/cmd_vel` 与 `/foo/nav_cmd_vel` 无发布者；`/foo/base_cmd_vel` 仍仅由原有 bridge 发布，NAV 速度只进入 `/nav/planner_cmd_vel`。
- 路径规划验收尚未通过：单帧临时地图中已观测自由空间的最大边界距离只有 0.05 m，不能容纳原车轮廓。没有扩大自由区、放开未知区或降低定位阈值。
- 独立保存的 1703 点旧小图定位精匹配失败（fitness 约 0.466，大于 0.1），不能作为可用导航地图。
- 当前项目 `src/nav_map` 只剩旧 PGM/YAML 与 SC 索引，无配套 PCD。需保存一份完整且几何正常的新建图，再生成同源地图包。
- 本次验证进程均在结束后停止。设备界面使用 WOOSH profile；没有自动开始导航或建图。

以上属于静止传感器和模块联调，尚未验收真实路径计算、移动定位、避障及底盘执行。DLIO 在部分停止过程中仍需 launch 超时终止；停止后已确认没有残留估计器。


## 2026-11-14 FAST-LIVO 导航与自动启动

- 首页“进入导航”启动算法和 Nav2，不发送路径目标；途径点页“开始导航”调用路径预览。“开始引导”仍需要底盘反馈、控制链路和许可，当前未启用厂家执行适配器。
- 已只读核对底盘 `/move_base`：实际为 ROS1 `woosh_move_base`，不是 Nav2。把可对应的八边形 footprint、0.025 m padding、0.4 m/s² 线加速度、0.45 rad/s² 角加速度、0.05 m/0.03 rad 目标容差、8 Hz 控制频率、8×8 m/0.03 m 局部代价地图、2 m 膨胀和 scan 4.5 m/10 m 障碍/清除范围适配到 Nav2。保留原 0.20 m/s、0.40 rad/s 的较低速度上限，未复制厂商私有规划插件。
- 当前 Nav2 障碍输入为 `/foo/scan`；原车深度相机障碍点云尚未桥接，没有把这些输入配置成已可用。底盘 `/nav_cmd_vel` 由原车导航/跟随发布并由底盘驱动订阅；现有 bridge 未配置这一速度话题。NAV 速度仍隔离到 `/nav/planner_cmd_vel`，未改 bridge、未新增控制 remap。

- 部署验证：本地及 Jetson 各 75 项导航相关测试通过，另各 3 项 IMU 转换测试通过。真实地图按钮验证已自动拉起三套程序，但 Nav2 Active/有效定位验收未通过：整套负载下 Jetson 收到的原始 bridge IMU 出现约 0.5 秒断档，FAST-LIVO 触发 imu_gap 暂停。队列扩大实验未解决，已恢复原配置；没有降低阈值或把启动进程当作导航就绪。验收进程已停止，桌面界面已重新打开。


## 2026-11-17：.5 点云过滤与扫描通信修复

- 11 月 17 日版本曾为车身过滤器和扫描转换器强制启用 UDPv4。11 月 18 日实机逐帧核对发现，在同一窗口驱动发出 282 帧、过滤器发出 224 帧、FAST-LIVO 收到 192 帧。当前车身过滤器沿用驱动和 FAST-LIVO 的本机默认 DDS 传输；扫描转换器仍使用专用 UDPv4 配置。
- 11 月 18 日从桌面重启 .5 导航后，以同一首尾点云时间戳对齐 DDS 发布序号和 FAST-LIVO 接收回调计数。141.064 秒内三处均为 1,419 帧；132 个滑动 10 秒窗口为 9.998–10.101 Hz。设备验收记录保存在 `~/nav-deploy-backups/20261118-livox5-10hz.pP34rp/verification/results.txt`。
- APP 根据 `.5` 预设自动选择这个扫描入口，无需手动导出环境变量。输入仍为 `/nav/livox5/points_filtered`，输出仍为 `/scan`；高度、量程、坐标系和时间戳处理不变。
- 配置只作用于上述处理进程；雷达驱动、ROS1 bridge、FAST-LIVO 和其他机器人预设的通信环境不变。没有新增底盘运动控制。
- 通信修复与地图定位是两项独立验收。完整导航测试仍遇到定位匹配拒绝、点云与里程计时间不同步，不能据此宣称整套导航或路线规划已通过。
- 设备备份和验收日志：`~/nav-deploy-backups/20261117-scan-udp-fix/`。回退时，在 `~/nav-current` 解压其中的 `before.tar.gz`，再构建 `nav_cloud_self_filter` 和 `nav_app` 并重新打开 APP；新加的扫描入口和 XML 不再被旧入口引用。地图、接入设置和 bridge 配置不需要恢复或覆盖。
