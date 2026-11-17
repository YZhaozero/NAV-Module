# 当前项目状态

## 2026-11-16 FAST-LIVO IMU 与 TF 修复部署（持续定位仍待通过）

- 按用户要求使用 FAST-LIVO2 里程计、点云 Localizer 重定位和 Nav2，不使用 AMCL，不允许底盘运动。桥接 `.3` IMU 独立进程、可靠队列 2000；其余 bridge 配置条目保持一致。FAST-LIVO 直接读取原始 IMU，在 C++ 中做 9.80665 加速度缩放与 frame 核对，避免 Python 转发丢样/积压。
- 原生 IMU 接收、导航位姿输出独立 DDS 参与者。导航逐条积分 IMU，输出真实测量时刻的 50 Hz 传播位姿；按安装外参转换出 `/nav/woosh/odom` 底盘原点速度和协方差，Nav2 使用该消息。Localizer 保持扫描时刻的 cloud/odom 配对，有效 map→odom 对齐以 20 Hz 更新；首个匹配因无效输入被取消后允许重新收集观测，不修改真实匹配失败、人工取消及 epoch 重置处理。
- 两端各 114 项针对性 Python 回归通过，2 项既有/并行页面测试排除；完整组合的轨迹列表刷新失败已记录，未当作全量通过。原生传输回归两端各两轮 40/40、初始化窗口七组通过。构建与源码完整性另有设备日志。
- 真实地图 `906-261116` 静态运行约 3 分钟：三套程序一直运行到结束、8 个 Nav2 节点 Active、129 次地图匹配、imu_gaps=0。odom→base_link 延迟 P95=0.061 s、max=0.211 s。没有 AMCL、未下发目标，`/cmd_vel` 和 `/foo/nav_cmd_vel` 无发布者。
- **完整导航验收仍未通过**：连续 60 秒定位有效的要求因点云观测年龄偶尔超过 1 秒而失败；map→odom 最大延迟 1.823 s。未放宽阈值或伪造当前位姿。另一“地形分析”地图全局匹配为 GLOBAL_GEOMETRY_AMBIGUOUS，原始地图保留。详细边界、使用与回退见 `WOOSH_NAVIGATION.md` 最新章节。
- 分离点云桥接、分离图像接收与 UDPv4 试验没有通过，均已撤回。备份、原始失败与成功项证据：`/home/nvidia/nav-deploy-backups/20261116-fast-livo-imu-tf/`。本次保留其他聊天中的地图与 UI 工作。

## 2026-11-16 导航接入首版部署与接口实测

- 已定向部署到 Jetson 当前发布 `/home/nvidia/nav-releases/20261113-nav-module`；新增“设备与设置 → 导航接入”及导航系统页快捷入口，保留地图、标定、运行配置及设备已有两行高度选点逻辑。未修改原车 bridge 或网络。
- 设备已有 nav2_amcl；nav_app 最终构建通过（4.14 秒），设备 93 项接入/导航回归通过并正常退出（46.87 秒；沿用排除 1 项旧地图控件测试的范围）。部署测试时补齐外部输入超时后的 Nav2 行错误状态与具体缺失输入提示，本地同组 93 项通过（15.51 秒）。
- 真实外部 scan/odom 模式：收到 `/foo/scan` 和 `/foo/odom`，odom header=odom、child=base_footprint；ROS2 TF 图为空。未启动传感器驱动、LIO 或点云转换，Nav2 也因缺少 odom→base_link TF 按设计等待并超时，页面准确提示 `/foo/odom` 与 TF，随后释放测试订阅。该阻塞是实际输入契约未满足，不能记为 AMCL/Nav2 已 Active。
- 真实 WOOSH FAST-LIVO 模式：按配置只拉起 FAST-LIVO、点云 Localizer 和 Nav2；仍复现 tracking_suspended/imu_gap，未通过整套持续就绪验收。测试未调用路径/导航目标、初始位姿或底盘速度接口，确认 `/cmd_vel`、`/foo/nav_cmd_vel` 无 NAV 发布者；没有绕过 TF、定位或跟踪检查。
- 验收配置使用独立 test-settings.ini，未改用户接入设置；验收子进程已清理，桌面 nav-app.service 已恢复 active、NRestarts=0，WOOSH/woosh_fast_livo 配置正常，手机 HTTP 200。
- 设备回滚归档、文件清单、构建/测试日志与两组真实验收记录位于 `/home/nvidia/nav-deploy-backups/20261116-navigation-access/`；安装版截图已读取核对。下一步应补齐与底盘真实里程计、机身及 scan 外参一致的 ROS2 TF 链，再验收 AMCL/Nav2 就绪；不会用任意 TF 或放宽阈值伪造通过。


## 2026-11-16 可组合导航接入设置（本地初版，未部署）

- 新增导航接入页面，独立选择驱动、scan、里程计与地图定位来源，含 WOOSH bridge+FAST-LIVO、已有 scan/odom+AMCL、本机处理三类预设。按现有机器人 profile 保存设置，未保存的草稿不影响启动；运行期间禁止修改接入来源。
- 启动构建器按来源只生成所需 launch；支持外部里程计、外部 map→odom 和 AMCL，保留 FAST-LIVO/DLIO/点云 Localizer。对外部输入检查数据新鲜度及标准 TF 链，显示具体等待项；运行中只监控必要输入，外部数据失效停止受管导航，不停止共享驱动。保留原 start_guard 和运动许可边界。
- AMCL 与地图服务在定位生命周期中管理，补齐 scan/odom 参数与两点 `/initialpose` 发布；已有 scan/odom+AMCL 不再要求 PCD。通用 Nav2 输出也与底盘 `/cmd_vel` 隔离。本版要求已有 odom→base_link TF，不自动伪造或重命名坐标。
- 最终本地 92 项接入/导航相关测试通过并正常退出（15.20 秒），nav_app 构建通过（1.63 秒）；安装后的新页面与源码 SHA256 一致，两个 Nav2 launch 的 --show-args 新参数解析通过，未执行节点启动。
- 实际 Qt 预览检查 1600×900、800×480、480×800；小屏输入可滚动触达。预览 `/tmp/nav-access-preview-20261116/`，未连接机器人、未部署或发送运动指令。使用说明见 nav_app README“导航接入设置”。
- 现有 test_navigation_port 中依赖已移除 unknown_policy_combo 地图编辑控件的旧测试未计入本轮组合回归；未恢复或修改另一组地图编辑工作来绕过该问题。本轮保留所有其他在途本地改动。
- 修改前代码归档 `/tmp/nav-access-before-20261116-095404.tar.gz`，文档归档 `/tmp/nav-access-docs-before-20261116.tar.gz`。


## 2026-11-16 导航显示、重定位地图与雷达误停修复（已部署）

- 导航准备页和活动导航页固定使用原始点云坐标显示，取消继承地图编辑页的自动找平；编辑页原功能及地图源文件保留。
- 地图包“用于导航”同时读取 PCD 和配套 YAML/PGM，绑定本地栅格及 frame。重新定位页在 Nav2/map_server 未启动时也能显示所选地图，避免等待 `/map` 导致空白。
- 原设备 09:16 日志显示 Localizer 匹配 fitness=0.02683，09:16:49 Nav2 已 Active，随后整套进程被停止。健康监控此前硬编码订阅 `/livox/lidar/pointcloud`，与 WOOSH 实际 `/foo/livox/lidar_192_168_128_3` 不符；修复为 WOOSH 默认及导航配置中的实际点云话题，切换数据源时重建订阅并清空旧时间戳。保持 2 秒实际消息新鲜度判定和断流停止逻辑。
- 本地 nav_app 构建 1.86 秒、设备 4.65 秒通过；本地 82 项相关测试通过（12.96 秒），设备相同 82 项通过（41.72 秒）。覆盖原始坐标、地图包栅格与重定位绑定、切换实际雷达话题和失效检测。
- 设备真实地图包 `916-20261114` 验证：离线载入 301×208 栅格并在重定位页可见；导航截图显示“原始坐标”；雷达和相机输入 fresh=true。整套持续运行验收仍未通过：本次自动启动在整套负载下复现 FAST-LIVO imu_gap 暂停，Nav2 等待 odom TF 后超时，不是此前错误话题监控的误停。未降低任何跟踪/定位阈值，未发送目标或底盘速度。
- 原车 IMU 本次只读短测 1601 条/8 秒、最大间隔约 0.006 秒；ROS2 bridge 同期短测 1287 条/8 秒、最大间隔约 0.096 秒，说明启动时的间歇性断档尚未完成根因修复。该传输/负载问题单独保留，不能把本轮界面及监控修复视为导航已可行驶。
- 设备备份、构建/测试日志、真实验收 JSON 与截图：`/home/nvidia/nav-deploy-backups/20261116-navigation-fixes/`；本地备份 `/tmp/nav-navigation-fixes-before-20261116-092043.tar.gz`。部署时检测到本地另有未部署的地图预览改动，保留本地逻辑，设备 operations.py 仅定向合入本轮栅格绑定修复。


## 2026-11-14 FAST-LIVO 导航与首页自动启动（已同步 Jetson）

- WOOSH 导航默认 FAST-LIVO2，复用建图的 `.3` PointCloud2、配对 IMU SI 转换、相机1 CameraInfo 和安装外参；Localizer 使用 `/fast_livo/cloud_body` 与 `/aft_mapped_to_init`，Nav2 里程计同步切换。保留高级页面手动选 DLIO。
- 首页“进入导航”自动请求 FAST-LIVO2 → Localizer → Nav2 启动；返回导航页不重启已有任务。缺地图、已有建图、重复进入和启动失败有提示/保护；失败时归还本次启动占用。原 `NavigationStackController.start`（含 start_guard）AST 与修改前完全一致。
- 只读获取原车 ROS1 `/move_base` 参数和速度订阅关系，将可对应的 footprint、padding、加减速度、目标容差、代价地图与 scan 范围适配到 Nav2；保留原 0.20 m/s、0.40 rad/s 速度上限。bridge、厂家执行开关与控制 remap 未修改，未发布目标或速度命令。
- 本地 75 项导航/页面/互斥回归通过（13.56 秒），另 3 项 IMU 转换测试通过；Jetson nav_app 构建通过（4.17 秒），同组 75 项测试通过（41.35 秒）。真实地图包 `916-20261114` 的页面按钮实测自动拉起 FAST-LIVO2、Localizer 和 Nav2，但整套导航就绪验收未通过：FAST-LIVO 连续 IMU 时间戳断档触发 `imu_gap` 暂停，Localizer 未有效定位，Nav2 等待 odom/map TF。
- 原车 IMU 单独测量 1601 条/8 秒、最大间隔约 0.006 秒；bridge 独立可靠订阅 1600 条/8 秒同样连续。只启动 FAST-LIVO 的隔离测试约 19 秒未检测到超过 0.2 秒的断档；问题在整套负载下复现。IMU 队列 100→2000 的实验仍复现断档，已恢复原队列，不降低跟踪/定位阈值或伪造就绪。第二次整套实测中，Jetson 原始 bridge IMU 12968 条、13 个超过 0.2 秒的间隔，最大约 0.505 秒；转换后也存在同类间隔。验收子进程已清理，最终桌面服务重新启动为 active、NRestarts=0。
- 设备修改前备份 `/home/nvidia/nav-deploy-backups/20261114-fast-navigation/source-before.tar.gz`，同目录保留构建、测试和两次真实验收日志/JSON/截图；本地备份 `/tmp/nav-fast-navigation-before-20261114-142515.tar.gz`。具体入口和控制边界见 `WOOSH_NAVIGATION.md`。


## 2026-11-14 日志管理进入时自动加载（已部署 Jetson）

- 修复拆分工作区后进入“日志管理”未调用 refresh、列表和目录为空的问题；进入与再次进入均异步读取原日志目录，保留已有选择和运行中日志的操作保护。
- 增加真实日志包首次进入、再次进入及运行中/已结束选择权限的回归测试；本地 57 项通过（10.38 秒），Jetson 57 项通过（32.33 秒），设备 nav_app 构建通过（4.43 秒）。安装版本离屏读取设备原目录并显示 14 个日志包，截图 `/home/nvidia/nav-deploy-backups/20261114-log-entry/logs-fixed.png`。
- 用户建图运行期间只同步和构建；随后检测 FAST-LIVO 已停止且 NAV 已关闭，才启动修复后的桌面服务，WOOSH profile、服务 active 和 NRestarts=0 已核对。未删除、迁移或导出用户日志，未启动建图或底盘运动。设备更新前源码备份 `/home/nvidia/nav-deploy-backups/20261114-log-entry/source-before.tar.gz`，同目录保留构建与测试日志。


## 2026-11-14 设备设置分组与工具拆分（已部署 Jetson）

- 设备设置使用分组卡片和独立标题/说明，分为设备与导航、地图与采集、系统维护；保持蓝色主色。
- 地图包、地图处理、处理进度统一放入“地图管理”。原工具大页拆成地图、录包回放、日志、诊断和手机等独立工作区，原后台协调器改为 QObject，由主窗口统一持有，保留任务互斥与生命周期。
- 轨迹测试原实例移入“传感器数据”的第四个标签；RTK 仍在传感器页。处理进度、地图结果、重新定位诊断和退出保护改为对象路由，不再依赖旧工具标签序号。
- 44 项首页、目录跳转、RTK/轨迹归属、录包/诊断退出保护与单次退出清理测试通过并正常退出（9.53 秒）；本地 nav_app 构建通过。清理了本轮编辑遗留的重复退出处理，并核对修改类不存在重复方法。
- Qt 实际渲染检查 800×480、480×800、1280×900、1520×950；横屏 800×480 的七个设置入口完整显示，竖屏可触控滚动。预览 `/tmp/nav-workspaces-20261114/`，含设置、统一地图目录、传感器轨迹测试与录包页面。
- 用户确认设备恢复后，已将本轮页面、测试和说明同步到 Jetson 当前发布 `/home/nvidia/nav-releases/20261113-nav-module`，仅构建 nav_app（4.19 秒）；设备 44 项界面测试正常通过并退出（28.79 秒）。桌面 nav-app.service 已启动，单实例且重启计数为 0；WOOSH / woosh_dlio、相机1图像与内参参数正确，手机页面 HTTP 200。未启动建图、导航或底盘控制，传感器消息、算法、标定及地图/录包文件目录未修改。
- 设备部署前备份 `/home/nvidia/nav-deploy-backups/20261114-workspaces-120159/source-before.tar.gz`，同目录保存清单及构建、测试日志。本次经 Wi-Fi `192.168.110.119` 部署并验证 SSH 主机密钥；设备当前有线地址为 `192.168.128.100`，未修改网络配置。
- 本地修改前备份 `/tmp/nav-workspaces-before-20261114-090520.tar.gz`。使用说明见 `src/nav_app/README.md` 的“分类工作区”。


## 2026-11-14 统一系统首页与 RTK 入口

- 参考 `gs-nav-monorepo/gs-app/gs_nav_app/ui/handheld.py` 的任务首页结构，新增 `ui/system_home.py`：首页提供建图、导航准备、地图管理、设备设置和手机入口；设备设置集中导航系统、传感器、地图处理与工具。原导航工作区保留，重复工具入口收起以适配小屏。
- RTK 同一实例从工具标签移动到“设备与设置 → 传感器 → RTK”，保留串口、接收机监测、GNSS 质量、NTRIP、独占调试互斥和退出清理。未修改 ROS 话题、IMU 转换或算法配置。
- “开始建图”主按钮按用户色样使用 `#00A9DF`；首页和设备卡片加大触控尺寸；顶层页面返回系统首页，返回不停止后台建图。
- 本地 nav_app 构建通过（1.92 秒），Jetson nav_app 构建通过（4.53 秒）。新增首页与相关工具 37 项测试正常通过并退出（8.12 秒）；包含新首页、原 Qt 页面在内的 67 项断言通过，组合测试退出阶段仍挂起，已清理进程，未把该组合运行记为正常退出通过。
- 800×480、480×800、1280×900 首页按钮边界检查通过；实际 Qt 截图 `/tmp/nav-system-home-20261114/`，含首页、设置页、传感器 RTK 和导航准备页。
- Jetson 桌面入口已同步本次页面源码并启动；WOOSH profile、相机话题、单实例和手机 HTTP 200 已核对。界面启动不自动建图、导航或控制底盘。桌面设置与传感器通信保持原配置；RTK 接收质量和运动精度不是本次 UI 验收范围。
- 修改前本地备份 `/tmp/nav-system-home-before-20261114-084303.tar.gz`，设备备份 `/tmp/nav-system-home-before-20261114.tar.gz`。操作入口见 `src/nav_app/README.md` 的“系统首页与 RTK 入口”。


## 2026-11-13 AR 方向与路线显示移植

- 来源固定提交 `aafb462ea8f7875205da888946d05c3d31bc8477`；接入贴地投影、图像时刻 TF、CameraInfo 畸变/分辨率适配、屏幕内路线顺序箭头和米制方向提示。保留现有 NAV 主题、页面布局与 Nav2 任务控制。
- 方向支持前行、左/右转、转身、前方转弯及沿路径剩余距离；路线出视野时保留有效方向。偏离路线、定位失效、画面延迟时暂停提示。接近终点不代替 Nav2 成功结果。
- Qt 和独立 `ar_nav_node` 共用几何模块；有地面 TF 时重新贴地，没有地面 TF 时按配置高度估算并明确提示。未收到标定才允许 auto 示意投影，已有标定但 TF/模型无效时撤下路线。修正密集 Path 将转弯距离提前的问题。
- App/RTK 回归 587 项通过，57.46 秒；`/tmp/nav-ar-final-tests.log`。AR 专项 30 项通过；`/tmp/nav-ar-focused-tests.log`。
- 实际 Qt/ROS 合成检查：800×480、480×800、1280×900 下 15 个场景通过；独立 AR 输出收到 4 帧并保留原图时间戳。结果及截图 `/tmp/nav-ar-smoke-93dindoj/`。
- 默认 `./build.sh` 构建 18 包通过，6.74 秒（`/tmp/nav-ar-build.log`）；11 个 AR 源码/测试文件语法与空白检查、`git diff --check` 通过。修改前备份 `/tmp/nav-ar-before-20261113.tar.gz`。未连接真实相机、雷达、RTK 或底盘；实机贴地精度仍需本机标定与地面系验收。
- 安装后的 `ros2 run nav_app ar_nav_node --ros-args -p show_window:=false` 启动检查通过；修复 Humble 处理 SIGINT 后重复 shutdown，实际退出码 0，日志 `/tmp/nav-ar-installed-entry.log`。
- 参数、行为与复现命令见 [App AR 说明](src/nav_app/README.md#ar-方向与路线2026-11-13)。现有 src 结构未调整。

## 2026-11-13 页面返回与按钮一致性

- 六个功能页共用 `ui/page_header.py`：建图、导航系统、传感器管理/预览、地图处理、工具均在同一右上角位置放置返回按钮；传感器预览返回管理页，其余返回导航主页。工具内嵌录包页移除重复返回。
- 导航执行画面的“退出导航”也移动到右上角，保留取消任务的语义；普通页面返回不停止建图等后台任务。
- 统一导航、普通操作、主要操作、危险操作四类按钮的背景、边框、字体及悬停/按下/禁用样式；默认 QPushButton 也具有按钮外观。“工具”和“建图视图”等入口使用导航按钮样式。清理日志页及 RTK 控制台对公共按钮样式/字号的覆盖。
- 修复初始化时桌面主题再次覆盖响应式主题导致的字号不一致。横屏、竖屏、桌面往返切换时，同级按钮统一采用当前屏幕字号。
- 回归 562 项通过，57.54 秒；`/tmp/nav-ui-controls-final-tests.log`。实际 Qt/ROS 检查 800×480、480×800、1280×900 下 36 个页面组合通过；`/tmp/nav-controls-runtime-check.json`，截图 `/tmp/nav-controls-final-*.png`。
- 最终默认构建 18 包通过，日志 `/tmp/nav-ui-controls-build.log`；语法及差异空白检查通过。
- 修改前源码备份：`/tmp/nav-ui-controls-before-20261113.tar.gz`。验证未连接真实传感器、接收机或底盘。


## 2026-11-13 NAV 文案与相关 Qt 功能

- 主窗口与各功能页标题、相机画面和新文件默认名称清理 GS 品牌文案；主标题为 NAV。保留当前布局，全局 `_style_sheet` 与修改前 AST 一致。
- 从固定来源 `aafb462ea8f7875205da888946d05c3d31bc8477` 接入录包列表模型和 Qt 管理操作：目录、搜索/排序、详情、加载、校验和确认删除。删除时阻止并发采集，并重新核对文件身份。
- 诊断 Qt 面板展示任务、遥测、定位与资源状态，增加带当前应用状态的 ZIP 诊断包导出；过期数据明确显示失效。录包设置补齐原始 RGB/JPEG 95 选择。
- App/RTK 合并回归：555 项通过，55.46 秒；日志 `/tmp/nav-ui-final-tests.log`。JPEG 合成链路录制 931 条消息，回放恢复 80 帧 Image 与 607 条 IMU，落盘/校验 complete；结果 `/tmp/nav-recording-smoke-s49v29g0/result.json`。
- 当前 `./build.sh` 增量构建 18 包通过（7.04 秒），日志 `/tmp/nav-ui-build.log`；实际 Qt 诊断包导出按钮验证通过，应用状态与资源信息均写入 ZIP。
- 真实 Qt/ROS 无硬件启动检查：800×480 主界面、录包管理和诊断截图 `/tmp/nav-ui-20261113-*-800x480.png`。未连接真实传感器、接收机或底盘。
- 修改前 UI 源码备份：`/tmp/nav-ui-before-20261113.tar.gz`。旧数据和通信/配置标识保留；未移动 src 目录。


## 2026-11-12 第二批：工具、RTK 与去前缀

- 现有 `src` 层级保留，七个 `gs_nav_*` 模块目录改为 `nav_*`；App ROS/Python 包为 `nav_app`，新建独立 `src/nav_rtk`。已有用户地图、录包、日志和兼容配置路径保留。
- 现有界面新增“工具”：地图包管理、录包回放/离线校验、日志导出删除、诊断 JSON、RTK 串口与 NTRIP。未移植 GS 业务；GNSS 不向导航发布 TF。
- 原目录裸 `colcon build`：18 包通过，7 分 49 秒；改名前的代码/产物备份在 `/tmp/nav-module-stage2-before-20261112-huGsaTMn`。
- source 当前 install 后 `./build.sh`：18 包增量构建通过，6.93 秒；实际安装入口的 Python `-s` 启动、RTK launch 参数和正常退出验证通过。
- App 与 RTK 合并回归 533 项通过；合成录包 683 条消息，逐条校验完成，回放接收 428 条 IMU。纯 LiDAR/IMU 录包链路同样通过（519 条消息、433 条 IMU 回放）。真实 Qt/ROS 800×480 工具页检查通过。
- 应用入口隔离用户 Python site，避免本机 NumPy 2 与系统 SciPy ABI 冲突。真实接收机/NTRIP 服务、定位精度和机器人运动尚未验收。
- 操作、接口、测试命令和证据见 [第二批移植说明](OPERATIONS_MIGRATION.md)。下方第一批记录为历史状态。

## 2026-11-12 当前工作区默认构建修复

- 用户运行裸 `colcon build`，原 CMake 缓存仍指向已不存在的 `/home/zy/ws/GS-NAV-Module`；失败发生在缓存路径检查阶段。上一批 `/tmp` 构建通过不能代表该入口已验证。
- 失效构建和安装产物已备份至 `/tmp/nav-module-before-rebuild-20261112-Keb9rwmB/`，源码、地图和历史日志保留。
- 新增 `colcon_defaults.yaml`，让裸命令和 `./build.sh` 使用相同的 App 运行链路、系统 Humble Nav2/PCL、Ninja 编译池限制；不依赖手工补构建参数。
- 原目录裸 `colcon build` 冷构建成功：17 包、退出码 0，耗时 8 分 3 秒；随后 source 当前 install 再运行 `./build.sh`，17 包增量构建成功，耗时 5.88 秒。
- 当前 16 个 CMake 缓存无旧目录引用；App、FAST-LIVO2、Localizer 和两类驱动均解析到当前工作区 install。动态链接、launch 参数解析和合成建图保存通过（117 组精确时间戳配对，拒绝覆盖）。
- 本轮日志：`/tmp/nav-module-root-build.log`、`/tmp/nav-module-build-script.log`；合成结果：`/tmp/gs-nav-smoke-usqgrvld/result.json`。

## 2026-11-12 建图导航第一批移植

- 目标工作区：`/home/zy/ws/NAV-Module`，保留原 `src/`、包名与模块目录。
- 来源固定提交：`gs-nav-monorepo@aafb462ea8f7875205da888946d05c3d31bc8477`；源码读取期间来源分支发生变化，后续统一使用该提交快照。
- 已接入：栅格未知区域语义、FAST-LIVO2 导航输出与保存/检查点恢复、Localizer 跟踪质量与 SC v2、Nav2 外部定位启动、Livox/RealSense 时间处理。
- App 保留现有布局、DLIO 默认选项和 `/cmd_vel`；增加 FAST-LIVO2 配置、定位质量反馈与建图导航互斥。
- 本轮不引入 GS、RTK、手持步行引导及来源仓库设备专用标定；不改写原有用户修改的 Localizer YAML。
- 本机 17 个运行依赖包构建成功，116 项 App 回归、60 项 C++ 功能回归、合成建图保存及真实 Qt/ROS 无硬件启动通过；结果和日志入口见 [移植说明](NAVIGATION_MIGRATION.md)。
- 统一系统 PCL 1.12 与 vikit 的 Eigen 编译设置；验证产物位于 `/tmp/nav-migration`，原工作区 `build/`、`install/` 保留。
- 全量 lint 中 uncrustify 的来源格式差异和 xmllint 的在线 schema 下载限制尚未解决；未以功能测试通过替代完整 lint 通过。
- 后续批次：地图包登记、录包回放、集中日志与性能诊断；尚未在本轮接入。
- 实机验收尚未执行：机器人 TF/外参、传感器时差、建图与定位精度、底盘运动闭环和物理停止。
