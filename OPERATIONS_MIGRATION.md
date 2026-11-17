# 建图导航第二批移植：地图包、录包、日志和 RTK

## 2026-11-13 返回入口与按钮规范

各功能页统一使用右上角“返回”。传感器预览返回传感器管理，其余功能页返回导航主页；工具页及其所有嵌套页签共享一个返回入口，录包页不再重复显示返回。导航执行页的“退出导航”移动到同一角落，继续明确表示取消任务，普通返回则保留后台任务。

按钮按用途统一：页面跳转使用描边导航按钮，普通操作使用灰色按钮，主要操作使用蓝色按钮，停止/删除等危险操作使用红色按钮。所有类别统一各自的字体、边框及悬停/按下/禁用状态；原先看似文字的“工具”“建图视图”和未分类操作已具有按钮外观。栅格/点云切换仍作为独立的切换控件。

共享页头由 `src/nav_app/nav_app/ui/page_header.py` 实现，公共主题仍集中在 `NavigationWindow._style_sheet`，各页避免覆盖按钮样式。已修复初始化过程中桌面主题与小屏主题相互覆盖造成的字号差异。

验证：默认构建 18 包通过（`/tmp/nav-ui-controls-build.log`），562 项回归通过（`/tmp/nav-ui-controls-final-tests.log`）；真实 Qt/ROS 的 36 个页面与尺寸组合通过，覆盖 800×480、480×800 和 1280×900。返回位置、嵌套入口去重、返回不误停任务、跳转行为及同级按钮跨页/跨尺寸的一致性均已检查。截图见 `/tmp/nav-controls-final-*.png`，汇总 `/tmp/nav-controls-runtime-check.json`。本次不操作真实设备。


## 2026-11-13 Qt 界面补齐

当前品牌文案统一使用 NAV，现有窗口布局与全局 Qt 样式保持不变。新建地图、录包的默认前缀为 `nav_map`、`nav_bag`；已有数据名称及兼容通信/配置标识不改写。

录包回放页面新增“录包管理”页签，复用固定来源的只读扫描与删除模型，提供目录选择、搜索、按时间/大小/时长排序、详情、加载、校验和确认删除。stored 状态仍需校验，能够在列表中直接发起校验；活动录包、共享地图资产、目录链接和确认后变化的文件不会被直接删除。

诊断页面展示任务、遥测、定位、日志和资源，保留原始详情及 JSON 导出，增加包含当前应用状态的 ZIP 导出。资源采样与文件导出在后台执行，导出/删除进行中退出会等待操作完成。诊断不会把已过期的定位或遥测显示为有效。

录包设置提供原始 RGB 与 JPEG 95 选择；前者保留默认行为，后者明确标注有损。相机画面、窗口和页面标题已清理 GS 品牌；建图、导航、地图包、日志与 RTK 沿用既有入口和整体风格。

当前 `./build.sh` 18 包增量构建通过（7.04 秒，`/tmp/nav-ui-build.log`）；实际 Qt 导出按钮已验证诊断 ZIP 含应用状态和资源信息。

验证：555 项 App/RTK 回归通过（`/tmp/nav-ui-final-tests.log`）；JPEG 合成录制、落盘、校验及回放恢复通过，931 条消息、80 帧 Image、607 条 IMU（`/tmp/nav-recording-smoke-s49v29g0/result.json`）。实际 Qt/ROS 的 800×480 布局检查通过；无真实硬件操作。下文记录第二批最初移植结果。


## 范围与来源

工作区为 `/home/zy/ws/NAV-Module`。来源固定为 `gs-nav-monorepo@aafb462ea8f7875205da888946d05c3d31bc8477`，使用 `/tmp/nav-module-upstream-aafb462e` 快照，避免来源工作区后续切换分支影响本批内容。

本批接入现有 Qt 窗口的“工具”入口，包含地图包、录包回放、日志、诊断、RTK 五个页签。保留已有导航、建图、传感器和地图处理页面；不迁入 Gaussian Splatting、训练或 GS 关键帧录制业务。独立的 GNSS 接入不代表已经完成 GNSS/LIO 融合或室外绝对导航。

## 包名与目录

只调整原目录的名称，不重组 `src` 的层级；`nav_navgation` 沿用已有拼写。

| 原名称 | 当前名称 |
|---|---|
| `src/gs_nav_app` / ROS 与 Python 包 `gs_nav_app` | `src/nav_app` / `nav_app` |
| `src/gs_nav_driver` | `src/nav_driver` |
| `src/gs_nav_slam` | `src/nav_slam` |
| `src/gs_nav_localization` | `src/nav_localization` |
| `src/gs_nav_navgation` | `src/nav_navgation` |
| `src/gs_nav_sensor_proc` | `src/nav_sensor_proc` |
| `src/gs_nav_map` | `src/nav_map` |
| 来源 RTK 桥接、协议解析、GNSS 监测 | 单独 ROS 包 `src/nav_rtk` |

Livox、RealSense、DLIO、FAST-LIVO2、Localizer 和 Nav2 等第三方 ROS 包保留它们自身的包名。当前没有以 `gs` 开头的 ROS 包。启动命令、Python 导入、依赖、默认构建目录和依赖安装脚本均同步更新；旧 `gs_nav_app` 启动命令不再提供别名。

后续补齐批次已将新运行时环境变量、状态目录、录包清单、配置文件和 ROS 节点统一为 NAV 命名。当前配置文件为 `nav.yaml`，Qt 节点为 `/nav_qt_nav`。程序不会自动改写已有的历史数据；如需迁移，应先备份并按新路径显式导入。

改名前的源码和文档快照为 `/tmp/nav-module-stage2-before-20261112-huGsaTMn/source-and-docs.tar.gz`，当时可用的 `build`、`install` 也保存在该目录。改名后在当前根目录重新构建，未复用旧绝对路径 CMake 缓存。不要复制备份缓存回当前目录。

## 地图包

“工具 → 地图包”支持扫描目录、登记、从 PCD 生成、核对坐标、选择用于导航、改名和删除。清单为 `*.scene.json`，关联定位 PCD、导航 YAML/PGM、可选 `.sc.v2`、地图坐标系和核对状态；相对路径方便整体搬移，外部文件可以登记但删除地图包时保留。

登记后未核对的包不可直接用于导航。同名文件不会自动证明原点、方向或比例一致；“核对坐标”是操作者的明确确认，不会替代地图配准。选择“用于导航”会填入现有导航系统的 PCD、栅格和 SC 配置，加载点云预览，随后仍由操作者启动导航。

生成使用重力方向已对齐的 PCD，保留原 XYZ，不做坐标换轴。地面观测形成自由栅格，障碍和未知区域保持独立；只有缺少观测的空白区域不会被自动填成可通行。SC 索引通过当前 Localizer 工具生成并检查源 PCD 身份。生成失败会显示错误；地图生成尚未完成时，窗口要求等待后退出。自动生成结果仍需要现场检查通行范围与多层地形。

删除前显示具体文件并要求确认，重新检查文件身份；保留共享资产与地图目录外的文件。建图、导航、录包、回放、校验或地图保存尚未结束时，不允许修改地图包资产。

## 录包、校验与回放

“工具 → 录包回放”中刷新并选择原始话题，设置保存目录、SQLite3/MCAP、压缩、缓存和分片。支持只录 LiDAR/IMU；选择 RGB 时必须同时录制有效 CameraInfo。默认原始 RGB 保存；录制守护进程检查话题、时间戳、相机信息和资源余量，保存参数/配置快照与诊断证据。MCAP 需要本机已经安装相应存储插件。

停止录制时先让 rosbag2 写完缓存和分片，再保存落盘回执。此阶段不要断电；退出应用也会等待录包进程结束。`stored` 表示持久化收尾完成，不能当作逐条校验通过。加载相应录包后点击“校验所选录包”，执行独立离线审计；成功后清单状态才变为 `complete`。校验可暂停，原始数据保留。普通外部 rosbag 可做文件审计；没有本项目清单时不会伪造参数快照。

CLI 入口：

```bash
ros2 run nav_app audit_bag /absolute/path/to/bag
# 对本应用录制的包更新完整性回执
ros2 run nav_app audit_bag /absolute/path/to/bag --finalize
```

自动“回放建图/导航”沿用当前页面选择的算法及话题。先检查选中话题及映射是否覆盖算法输入；播放器以暂停模式启动，App/算法改用仿真时钟，输入订阅就绪后再请求继续播放。回放导航使用 `sensor_source=bag`，不启动现场传感器；回放期间禁止另起现场驱动。播放器停止会请求停止使用回放的算法。导航仍需预先选择匹配的地图。

FAST-LIVO2 自动回放从录包快照恢复外参、相机参数和时间补偿；关闭视觉时不要求相机内参快照，并保留当前运行控制参数；缺少有效快照时明确拒绝自动启动，需先人工核对离线标定。外部包可先普通回放，再在建图页人工设置经核对的配置。动态 TF、历史地图、规划路径和速度等派生结果不纳入自动算法回放的安全输入选择。原始时间戳保持不变。

## 日志和诊断

每次运行一个日志包，按设备、传感器、建图、导航、定位、录包回放分类。子进程输出通过有界队列异步写盘，界面显示写盘失败、队列丢弃和磁盘状态；未捕获异常仍继续送给原异常处理器。日志页支持刷新、导出 ZIP 和确认删除已结束的会话，活动会话不可删除。

默认目录 `~/.local/state/gs-nav/logs` 可用 `GS_NAV_LOG_DIR` 覆盖。“诊断”展示 FAST-LIVO2 心跳、输入时序、处理积压、Localizer 当前质量和进程状态，可导出 JSON；缺乏足够证据时保留不确定结论。资源快照只在明确导出时收集，不在界面定时器里扫描系统。

## RTK

[nav_rtk 说明](src/nav_rtk/README.md) 给出包内分层与话题。“工具 → RTK”支持搜索并明确选择串口、接收机状态/定位/方向/设备/基站、原始报文与命令、GNSS 质量监测和 NTRIP 设置。搜索只枚举，不打开串口；连接、断开和接收机指令都由操作者触发。任务进行期间阻止独占调试；接收机方向值不等于地图方向已校准。

NTRIP 保存账号不会启用服务；单独点击“启用差分”才请求连接，需自行填写服务商的主机、挂载点、账号、密码与坐标基准。账号以权限 0600 原子落盘，不通过命令行传递。未使用真实账号验证服务商连接。

GNSS 监测订阅 `/rtk/fix`，执行时间、状态、协方差及跳变检查，发布相对首个有效点的 ENU 与诊断。不会发布导航 TF，也不会覆盖 Localizer 的 `map → odom`。时间偏移估计只输出证据，不自动修改观测时间。

## 运行与验证

```bash
cd /home/zy/ws/NAV-Module
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
ros2 launch nav_app ar_nav.launch.py
# 需要实际串口时手动启动；勿在无接收机时照抄设备路径
ros2 launch nav_rtk rtk.launch.py port:=/dev/ttyUSB0 baudrate:=230400
ros2 launch nav_rtk monitor.launch.py fix_topic:=/rtk/fix
```

当前机器用户目录安装了 NumPy 2.2.6，系统 SciPy 1.8.0 使用系统 NumPy 1.21.5；混用会产生 ABI 错误。`qt_nav_node` 入口以当前 Python 的 `-s` 模式启动并设置 `PYTHONNOUSERSITE=1`，使用系统依赖，不卸载或改动全局/用户 Python 包。直接执行源码测试时也应设置该变量。

```bash
PYTHONNOUSERSITE=1 QT_QPA_PLATFORM=offscreen PYTHONDONTWRITEBYTECODE=1 \
  python3 -m pytest src/nav_app/test src/nav_rtk/test --import-mode=importlib -q
ROS_DOMAIN_ID=187 ROS_LOCALHOST_ONLY=1 PYTHONNOUSERSITE=1 \
  python3 src/nav_app/test/smoke_recording.py
# 同一隔离环境下，验证不录制相机
PYTHONNOUSERSITE=1 ROS_DOMAIN_ID=193 ROS_LOCALHOST_ONLY=1 \
  python3 src/nav_app/test/smoke_recording.py --lidar-only
```

| 检查 | 本批结果 |
|---|---|
| 当前根目录裸 `colcon build` | 18 包成功，退出码 0，冷构建 7 分 49 秒；日志 `/tmp/nav-stage2-build.log` |
| source 当前 install 后 `./build.sh` | 18 包成功，退出码 0，增量构建 6.93 秒；`/tmp/nav-stage2-incremental-build.log` |
| 当前安装入口 | 全部模块导入成功，两个 RTK launch 参数解析成功；实际 `ros2 run nav_app qt_nav_node` 使用 Python `-s` 启动并正常退出 |
| App + RTK 合并回归 | 533 项通过，54.29 秒；日志 `/tmp/nav-stage2-final-tests.log` |
| 纯 LiDAR/IMU 录制、落盘、校验、回放 | 519 条消息；回放收到 433 条 IMU；complete；`/tmp/nav-recording-smoke-1hdzkq2x/result.json` |
| 合成录制、落盘、逐条校验、回放 | 683 条消息；回放收到 428 条 IMU；状态 complete；`/tmp/nav-recording-smoke-yvp_8r12/result.json` |
| 无硬件 Qt/ROS 启动 | 新增五个页签可创建，800×480 布局检查通过；截图 `/tmp/nav-stage2-*-800x480.png` |
| RTK 串口 | 协议与伪终端测试通过；无真实串口、接收机或 NTRIP 服务操作 |

533 项包括 283 项 RTK 测试和 250 项应用测试，覆盖地图资产保护、未知区域语义、日志导出与活动会话保护、录包收尾、订阅就绪后恢复回放和界面接入。测试数据来自合成发布器或伪终端，不证明真实 GNSS 精度、真实相机/LiDAR 时间同步、地图精度或底盘运动闭环。此前 C++ 功能检查和 lint 限制仍见 [第一批说明](NAVIGATION_MIGRATION.md)。


## 2026-11-13 AR 显示补充移植

固定来源仍为 `aafb462ea8f7875205da888946d05c3d31bc8477`，复用 `ar_projection.py`、`navigation_trial_ar.py::route_chevrons`、`nav_math.py` 和方向几何；仅适配机器人 Path 的米制坐标与 Nav2 导航状态。代码落在既有 `src/nav_app`，不改变 src 层级。

- `nav_app/ar_projection.py`：地面参考系/配置高度贴地、图像时间戳 TF、相机畸变和缩放、可见范围与明确失效原因。
- `nav_app/navigation_display.py`：相对机器人朝向的方向、剩余路径距离、前方转弯与路线顺序箭头。转弯距离取首个连续转弯区间的峰值转角位置，避免依赖 Path 采样密度。
- `qt_nav_node.py` 与 `ar_nav_node.py`：共用显示几何；Qt 在每个新图像/路径上重新投影，横竖屏保留原色彩和按钮布局。相机失效撤下路线与实时提示，导航成功仍由 Nav2 action 结果决定。
- 诊断 JSON/ZIP 的应用状态中增加 `ar_projection`，可查看未显示路线的原因及地面来源。

默认 `./build.sh` 构建 18 包通过，6.74 秒（`/tmp/nav-ar-build.log`）；完整回归 587 项通过，AR 专项 30 项通过。真实 Qt/ROS 合成检查覆盖三个尺寸的 15 场景及独立输出时间戳，证据 `/tmp/nav-ar-smoke-93dindoj/result.json`，测试日志 `/tmp/nav-ar-final-tests.log`。配置含义及复现命令见 [App AR 说明](src/nav_app/README.md#ar-方向与路线2026-11-13)。

未修改相机内外参为某个假定机器人标定值：路线宽度 0.95 m 与原虚拟相机参数保留；新增地面高度 0.72 m 为可配置的缺省估计，使用此模式时界面提示核对现场。真实相机、时间同步、地面重合精度、坡面适用性和底盘运动尚未验收。

安装后的独立 AR 入口在隔离 ROS 域无硬件启动通过，并以 SIGINT 正常退出（退出码 0，`/tmp/nav-ar-installed-entry.log`）；修复了 Humble 已处理 SIGINT 后重复 shutdown 的报错，采用与现有 Qt 入口一致的关闭方式。
