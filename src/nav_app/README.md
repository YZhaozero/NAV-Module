# NAV Navigation App

一个 ROS 2 Humble 交互式导航界面：

- 订阅配置文件指定的彩色相机话题，作为全屏背景；
- 订阅导航路径 `nav_msgs/msg/Path`，在相机画面上绘制半透明导航带；
- 订阅导航 `map_server` 的 `/map` 栅格地图，在主地图和导航小窗中绘制全局路径；
- 使用 Qt 直接显示原始相机画面，导航带使用独立矢量图层绘制，不再压缩成 `/ar_image`；
- 地图支持添加任意数量的有序途径点，调用 Nav2 `/navigate_through_poses`；
- 支持查看导航反馈、剩余距离和取消当前导航；
- 支持加载 ASCII/binary/binary_compressed PCD，以及 ASCII/大小端 binary PLY 点云；
- 地图处理页以三维方式显示点云，支持鼠标旋转、平移、缩放和视角复位；
- 支持原始坐标/自动地面找平、XYZ 轴交换与翻转、透视/正交投影及 RGB/高度着色；
- 支持加载和保存 Nav2 PGM/YAML 栅格地图，并把 PCD/PLY 按高度范围转换为栅格；
- 支持用画笔修改栅格障碍/自由/未知区域，或对俯视点云添加、擦除点；
- 导航和地图处理使用两个独立工作区，地图编辑工具不会混入导航选点界面；
- 提供独立的传感器管理页，可配置、启动和停止 Livox MID360 与 RealSense D435i；
- 雷达、相机及合并日志实时显示在界面中，关闭软件时自动回收已启动的驱动进程；
- 传感器数据页可按消息类型选择 ROS 2 话题，监看三维点云、相机视频和 IMU 数值；
- 提供独立建图页，可启动/停止 DLIO 或 FAST-LIVO2、查看实时三维地图并调用服务保存 PCD；
- 导航设置页以地图为主、相机为辅，可在栅格地图与交互式三维点云之间即时切换；
- 有相机内参和 TF 时执行真实针孔投影，尚未标定时自动回退到可调地面投影。

## 建图导航移植（2026-11-12）

保留现有页面与 DLIO 默认后端，建图和“导航系统 → 里程计”均可选择 FAST-LIVO2。
未核对相机内外参时先关闭视觉，仅验证 LiDAR / IMU；启用视觉前配置本机标定和图像话题。
建图页新增“恢复中断建图”，可导出已落盘检查点。点云转栅格默认保留未知区域；人工确认覆盖后才选择将空白区域设为自由。

使用 Localizer 时不再额外启动 AMCL；定位质量过期会阻止新目标，并请求取消运行中的导航。
SC 新索引为 `.pcd.sc.v2`，原入口可重新生成，排队回执不再显示为重定位成功。
后续批次已加入 RTK、步行引导、运动保护和手机遥控；完整现状见 [功能补齐说明](../../FEATURE_MIGRATION_20261113.md)。软件取消仍不等于物理停止。
完整接口、构建与验证范围见 [移植说明](../../NAVIGATION_MIGRATION.md)。下方 DLIO 操作继续适用。

RTK 轨迹测试可加载地图包的栅格/点云，也可使用百度在线底图显示 GNSS 位置和轨迹。
在线底图需要系统组件 `python3-pyqt5.qtwebengine`。百度私有配置默认从权限为 600 的
`~/.config/nav/maps.env` 读取，模板见 `deployment/baidu/maps.env.example`；直接执行
`ros2 launch` 和设备服务启动使用同一份配置，不需要每次手工 `export`。组件或密钥缺失时，本地地图、轨迹记录和历史回看仍可使用。

## 无硬件预览

```bash
cd ~/ws/NAV-Module
colcon build --packages-select nav_app --symlink-install
source install/setup.bash
ros2 launch nav_app demo.launch.py
```

无桌面环境时关闭窗口，合成图仍会发布：

```bash
ros2 launch nav_app demo.launch.py show_window:=false
```

## 接入小车

先按实际话题修改 `config/nav.yaml`，然后运行：

```bash
ros2 launch nav_app ar_nav.launch.py
```

默认点云已随软件放在包内，不依赖原来的 `car_simple_sim` 路径。也可在启动时换成其他
PCD/PLY，或传入空值取消自动加载：

```bash
ros2 launch nav_app ar_nav.launch.py pointcloud_map_path:=/path/to/map.pcd
ros2 launch nav_app ar_nav.launch.py pointcloud_map_path:=""
```

该命令会立即打开 Qt 导航控制台；相机尚未发布图像时显示等待页面，收到第一帧后自动
切换到实时画面。设置页的小相机只显示预览，不显示距离状态卡。主地图每次点击会依次
追加一个途径点；右侧列表可单独选择某个途径点并重新选点或删除，删除后会自动重新
编号。规划路线后进入导航预览页，路线就绪后点击“开始导航”，显示相机、AR 路线、左下角
小地图及方向和剩余距离。手柄模式由用户控制小车行走，界面使用实时定位判断到达；机器人自动模式仍需明确点击“开始导航”确认任务。
到达最后一个目标点后显示“已到达目的地”，提示保持显示，点击“返回地图”结束本次导航。
成功、取消或失败时会清理路线和本次定位轨迹，当前位置标记继续更新；手动退出会取消当前任务。
如果通过 SSH 启动，需要保证图形转发可用并且环境中存在 `DISPLAY`。

## WOOSH 底盘遥控（Agent 0.0.7）

首页右上角“底盘遥控”，或“设备与设置 → 底盘遥控”进入。支持阿克曼、全向及差速车型，
可设置直线和转动速度上限，使用屏幕单摇杆或外接 USB/蓝牙手柄。选择车型只约束发送的速度，
不会修改底盘硬件配置。默认未选择车型、未允许遥控，打开页面和检查连接均不会切换模式或发送速度。

ARM64 设备先安装厂家 SDK 到用户目录（不执行 `.run` 中的 sudo 安装脚本）：

```bash
cd ~/nav-current   # 开发机可改为 ~/ws/NAV-Module
python3 deployment/install_woosh_agent.py /path/to/ros-humble-woosh-robot-agent_0.0.7-0jammy_arm64.run
source scripts/runtime_environment.sh
```

默认 SDK 目录为 `~/nav_dependencies/woosh-agent-0.0.7`，可用 `--prefix` 指定；
使用自定义目录时，启动应用前设置 `NAV_WOOSH_AGENT_ROOT`。桌面 NAV 启动脚本自动加载该环境。
外接手柄还需要 `ros-humble-joy`；设备上已安装。开发机的 x86 系统不能运行该 ARM64 Agent。

操作顺序：

1. 在“连接”填写底盘 IP，点击“启动 Agent”；已有 `/woosh_robot` Agent 时用“检查连接”。
   IP 只用于新 Agent；现有 Agent 使用其原启动地址。等待模式、急停和实测速度反馈。
2. 选择与实车一致的车型和速度上限；默认 `0.15 m/s`、`0.30 rad/s`，页面上限分别为 `0.5 m/s`、`1.0 rad/s`。
   自动/手动模式仅作为状态显示，无需申请或切换。导航后台运行时会阻止启用，请先在首页停止导航。
3. 切到“屏幕摇杆”，摇杆回中后勾选“允许遥控”。上推前进、下推后退；左右推转向。
   全向车可把横向动作切换为左右平移。阿克曼车不发送侧向速度或零速原地转动，倒车时转向角速度随行驶方向反向。
4. 外接手柄：打开“外接手柄”页后自动搜索，列表显示设备名称，无需知道序号。首次连接点击“开始标定”，
   放开按钮、摇杆回中后“记录中位”，按一下想用作“允许移动”的保持键并松开；按提示将前后摇杆向前推到底，
   回中后将转向摇杆向左推到底；全向车型还需标定另一个侧移轴。方向和中位会自动识别，完成后点击“保存标定”。
   下次进入此页会自动连接并按 SDL GUID 加载同型号手柄的映射、车型和速度，即使设备序号变化也无需重标。
   仍需先回中、松开保持键，再勾选“允许遥控”；移动时持续按住保持键。标定期间不允许运动，离开页面会断开本页驱动。
   驱动以 20 Hz 在 `/nav/manual_gamepad/joy` 发布独立输入，超过 0.3 秒无数据会取消遥控并请求零速。
   参数保存在 `~/.config/nav/gamepads.json`，可通过 `NAV_GAMEPAD_PROFILES` 指定文件；损坏的配置会明确报错，不会静默覆盖。
5. 松开屏幕摇杆/手柄保持键会请求零速；切换输入、离开页面、窗口失去焦点或关闭应用会取消遥控。
   红色“停止遥控 · 发零速”是显式零速请求；软件停止不替代车身急停。服务报错、状态过期及急停会取消遥控，需要重新主动允许。

厂家 PDF 第 32 页给出的运动入口是服务 `/woosh_robot/robot/Twist`（`woosh_robot_msgs/srv/Twist`），
需要连续请求；不是直接发布 `/cmd_vel`。字段 `arg.linear` 单位 m/s、前向为正；`arg.angular` 单位 rad/s、逆时针为正；
`arg.linear_y` 单位 m/s、左向为正，以 SDK 消息定义为准（PDF 对侧移方向的说明有笔误）。
页面以最高 20 Hz 发送，等待回执并在松手时优先请求零速；显示的“请求”和厂家“实测”速度分开，SDK 没有侧向实测字段。
全向平移合速度限制在所设直线速度内。后台建图可继续，现有 Nav2 输出与底盘执行开关保持原状。

验证命令（不向实车发送运动请求）：

```bash
source scripts/runtime_environment.sh
PYTHONPATH="$PWD/src/nav_app:$PYTHONPATH" QT_QPA_PLATFORM=offscreen python3 -m pytest -q \
  src/nav_app/test/test_gamepad.py src/nav_app/test/test_manual_drive.py src/nav_app/test/test_manual_drive_ui.py \
  src/nav_app/test/test_manual_drive_ros.py
```

策略和 UI 测试注入内存服务；SDK 合约测试使用独立 ROS domain 201、`/nav_manual_test` 模拟服务，
不连接实车速度服务。实车只验证了 Agent 模式/硬件/速度读取，实际行驶和实体手柄操控尚待现场验收。

### Jetson 上 Xbox 360 兼容手柄未识别（2026-11-17）

本设备 USB 能看到 `045e:028e`，但 `5.15.148-tegra` 内核缺少 xpad，USB 接口未绑定驱动，
因此 ROS 的手柄列表为空。已在 `~/nav_dependencies/xpad-5.15.148-tegra/` 使用匹配内核头文件
编译官方 Linux stable v5.15.148 的 `xpad.ko`；加载需要管理员权限。在 Jetson 终端执行：

```bash
sudo install -D -m 644 ~/nav_dependencies/xpad-5.15.148-tegra/xpad.ko /lib/modules/5.15.148-tegra/extra/xpad.ko
sudo depmod -a
sudo modprobe xpad
```

随后用 `ros2 run joy joy_enumerate_devices` 验证识别，再进入页面做实物标定。这些命令只安装输入驱动，
不发送底盘速度。驱动只适用于当前内核版本；升级内核后应使用新内核头文件重新编译。
源码：[Linux stable xpad.c](https://github.com/gregkh/linux/blob/v5.15.148/drivers/input/joystick/xpad.c)。

### P3 手柄可识别但按钮和摇杆不变化（2026-11-18）

这只 P3 布局手柄在 USB 上报 `045e:028e / Nintendo Co., Ltd. / XBOX 360 For Windows`。
xpad 能创建输入设备，但该兼容固件需要额外初始化；ROS 重复发布初始状态不能证明真实按键已经工作。
已实测 USB 初始化后，内核和 ROS 都能收到叉号键按下/松开及左摇杆变化。

一次性初始化（需要管理员 USB 写权限，不会访问底盘控制接口）：

```bash
sudo python3 ~/nav-current/deployment/p3_gamepad.py
```

为避免重新插拔或开机后再次执行，安装仅匹配该 VID/PID、厂家和产品字符串的 udev 规则：

```bash
sudo python3 ~/nav-current/deployment/p3_gamepad.py --install
```

安装程序把脚本复制到 root 管理的 `/usr/local/lib/nav/p3_gamepad.py`，规则为
`/etc/udev/rules.d/99-nav-p3-gamepad.rules`。安装时不重新初始化或重启当前手柄，下一次 USB 插入时生效。
不会覆盖内容不同的同名文件。需要撤销时删除这两个文件并执行 `sudo udevadm control --reload-rules`。
兼容请求参考：[xpad 的兼容初始化处理](https://github.com/paroj/xpad/blob/master/xpad.c)。

标定页已改为逐条处理输入，修复短按在界面刷新之间被松开覆盖的问题，并增加“最近按下”显示。
短按修复需重新打开 NAV 后加载；正在进行的标定可以先保存，无需为安装 udev 规则中断当前操作。

## 传感器管理

导航页右上角点击“传感器管理”进入独立的驱动控制页面。该页面直接运行本机 ROS 2
launch，不启动 HTTP 服务：

- “雷达”页启动 `livox_ros_driver2/msg_MID360_launch.py`，可设置输出消息格式、发布
  频率、`frame_id`、多话题模式和 MID360 JSON 配置路径；
- “相机”页启动 `realsense2_camera/d435i.launch.py`，可设置设备名称、命名空间、
  序列号、RGB/深度/IMU 数据流、同步、深度对齐、相机点云和 IMU 合并方式；
- 右侧日志可在“全部 / 雷达 / 相机”之间切换，启动失败、驱动输出和退出码都会实时
  显示；日志在页面切换后仍会保留，导航系统页产生的传感器诊断也会同步到这里。返回
  导航页面不会停止传感器，点击对应“停止”按钮或关闭整个软件会清理该 launch 的完整
  进程组，避免驱动子进程和发布话题残留。

点击传感器管理页右上角的“传感器数据”进入实时监看页面：

- “雷达点云”自动列出 `sensor_msgs/msg/PointCloud2` 和
  `livox_ros_driver2/msg/CustomMsg`，支持三维旋转、平移和缩放；
- “相机视频”自动列出 `sensor_msgs/msg/Image` 并显示原始视频帧；
- “IMU”自动列出 `sensor_msgs/msg/Imu`，显示 frame、时间戳、姿态四元数、角速度、
  线加速度和协方差；
- 三种选择框均可手动输入完整话题名称。返回传感器管理页时，会释放这些临时监看订阅，
  不影响已经启动的驱动。

传感器驱动随工作空间构建后，仍使用同一个入口打开界面：

```bash
source install/setup.bash
ros2 launch nav_app ar_nav.launch.py
```

## 导航系统启动

AR 导航主界面右上角的“导航系统”页面用于启动完整运行链路，默认顺序为：

1. Livox MID360 雷达；
2. RealSense D435i 相机；
3. 所选里程计（默认 DLIO，可选 FAST-LIVO2）；
4. PointCloud2 转 LaserScan；
5. 定位后端（默认点云 Localizer）；
6. 导航后端（默认 Nav2 Bringup）。

雷达或相机已经通过“传感器管理”启动，或者 ROS 图中已检测到对应数据话题时，会跳过
重复启动。传感器状态以最近收到的真实消息为准，不再仅以驱动进程或话题名称判断；未上电
时会停留在“等待数据”，超时后中止后续启动。启动参数已经拆分为“地图 / 定位 / 导航 /
DLIO / LaserScan / 通用”六个独立页签。地图页只管理地图资源；定位页可选择点云
Localizer、不启动定位或自定义定位 launch；导航页可选择 Nav2、SCAN-Planner 或自定义
导航 launch。默认组合仍是 Localizer + Nav2，并分别使用 PCD 定位地图和 YAML 栅格地图。
选择“不启动定位 + SCAN-Planner”时不强制要求这两种地图。日志按系统和六个组件分别
显示并跨页面保留；
任一中间 launch 异常退出都会中止启动队列。“停止整套系统”会回收软件当前管理的全部
六组进程，包括先在“传感器管理”页面启动的雷达和相机；每个 launch 都按独立进程组发送
INT、TERM、KILL 三级退出信号，父 launch 提前退出时也会继续清理其残留子进程。由外部
终端自行启动、并非本软件创建的进程不会被强制结束。

## 独立建图

导航页右上角点击“建图”进入 `NAV MAPPING STUDIO`。建图和导航、地图后处理分别处于
独立工作区；返回导航不会终止正在运行的建图任务，必须点击“停止建图”或关闭软件。
默认后端为 `direct_lidar_inertial_odometry`（DLIO），也可选择 FAST-LIVO2。以下为 DLIO 默认接口：

- 雷达输入只列出 `sensor_msgs/msg/PointCloud2`，IMU 输入只列出
  `sensor_msgs/msg/Imu`，也可以手动填写话题；
- 默认输入为 `/livox/lidar/pointcloud` 和 `/livox/imu`，默认实时地图为
  `/dlio/map_node/map`；
- 页面中央实时显示 DLIO 输出的三维点云，支持鼠标旋转、平移、缩放；
- 可选择 `use_sim_time` 和是否额外启动 RViz，右侧显示完整 launch 日志；
- “保存当前 PCD”调用 `/save_pcd`，按设置的体素大小和地图名称保存；文件名自动附加
  毫秒级时间，例如 `nav_map_20260918_123456_789.pcd`，连续保存不会覆盖旧地图；
- 在线建图 PCD 和地图处理页面保存的 PCD、Nav2 YAML/PGM，默认统一保存到工作空间的
  `~/nav_maps`（Jetson 为 `/home/nvidia/nav_maps`）；加载地图的文件窗口也默认从该目录打开；
- 保存前会检查是否已经收到有效地图点云，服务返回后还会验证目标文件真实存在且非空，
  最终完整路径会显示在页面和建图日志中。

如需修改统一地图目录，可设置 ROS 参数 `map_storage_dir`，或在启动软件前设置环境变量
`NAV_MAP_DIR`。在线建图页面仍可临时手动选择其他保存目录。

DLIO 本身要求 PointCloud2。若 Livox 驱动当前使用 `CustomMsg`，请先在“传感器管理”
中把雷达“输出格式”切到 `PointCloud2`，再启动建图。界面会阻止已知类型不匹配的话题，
避免进程正常启动却始终收不到点云。

建图算法入口集中在 `features/mapping.py` 的 `MAPPING_BACKENDS`。后续接入其他 SLAM 时，
注册包名、launch 文件、参数名、地图输出话题与保存适配器即可复用同一套页面、进程组
回收、日志和三维预览，不需要改动导航或地图处理页面。

## 代码结构

界面外壳不再直接实现新功能的 ROS 和进程逻辑，主要模块划分如下：

- `qt_nav_node.py`：导航 ROS 节点、主窗口和页面切换；
- `features/mapping.py`：建图后端定义、ROS 点云/保存适配器和建图控制器；
- `ui/mapping_page.py`：建图页面，只渲染控制器状态并转发用户操作；
- `features/sensors.py`：传感器驱动控制、启动参数生成和实时话题订阅；
- `features/navigation_backends.py`：定位与导航后端定义、参数校验和 launch 命令生成；
- `features/navigation_stack.py`：与具体定位/导航算法解耦的顺序启动和生命周期控制；
- `ros_launch_process.py`：所有功能共用的完整 launch 进程组启停与日志管理；
- `map_processing.py`：PCD/PLY、栅格转换及地图文件处理；
- `nav_math.py`：导航路径投影与几何计算。

新增功能时应建立自己的功能控制器；新增页面只依赖该控制器，不在主窗口中直接创建
ROS 订阅、服务客户端或子进程。这样算法、ROS 接口和 Qt 页面可以分别测试和替换。

接入新的定位或导航算法时，在 `features/navigation_backends.py` 中实现 `LaunchBackend`，
注册到 `LOCALIZATION_BACKENDS` 或 `NAVIGATION_BACKENDS`，再为它补充独立参数面板即可。
自定义 launch 入口也可以直接在界面填写包名、launch 文件和附加参数；定位参数支持
`{map}/{config}/{use_sim_time}`，导航参数支持 `{map}/{params}/{use_sim_time}` 占位符。

## 本地地图处理

地图处理完全在 Qt 进程内完成，不会启动 HTTP 服务。导航页右上角点击“地图处理”会
进入独立的 `NAV MAP STUDIO` 页面；该页面负责加载 `.pcd/.ply` 或 Nav2 `.yaml/.pgm`、点云
转栅格、地图画笔编辑和保存，页面内没有导航选点或启动导航按钮。点云转栅格时可设置
分辨率和保留的 Z 高度范围；“保存当前地图”会将点云写为 binary PCD，或将栅格写为
配套的 YAML/PGM 文件。点击“返回导航”回到相机与途径点导航页面。两个页面各自维护
点云/栅格显示模式，地图处理页切换显示类型不会改变导航小地图当前类型。

地图处理页切换到“点云地图”后，左键拖动旋转三维视角，右键或中键拖动平移，滚轮
放大缩小；双击点云区域或点击“重置三维视角”恢复默认等轴视角。导航设置页和开始导航
后的点云小窗也使用可交互三维视图：短按左键添加途径点，左键拖动旋转，右键拖动平移，
滚轮缩放；三维视图会叠加途径点、全局路径与机器人朝向。

普通点云显示默认使用“自动找平地面 + 透视 + 原始 RGB”。自动找平通过点云主平面估计
显示方向；没有 RGB 字段时自动回退到高度着色。姿态、轴顺序、反向开关、投影和颜色
都只影响三维预览，不会改写内存中的原始 XYZ。加载 PCD/PLY 后均可直接执行点云转
栅格；自动找平或轴变换启用时，转换会自动使用与预览一致的坐标副本，并把估计地面
设为 Z=0，因此界面的 Z 高度范围表示相对地面的高度。原始点云不会被旋转或覆盖。
保存当前点云时统一输出 binary PCD，并保留已加载的 RGB。只有切回完全未变换的显示
后才能选择原始 XYZ 转换。选择点云添加/擦除画笔时，界面会自动回到原始 XYZ 显示，
避免在显示变换下误编辑。

也可以直接通过 remap 临时接入，例如：

```bash
ros2 run nav_app ar_nav_node --ros-args \
  -p camera_topic:=/color/image_raw \
  -p camera_info_topic:=/color/camera_info \
  -p map_topic:=/map \
  -p path_topic:=/plan
```

## AR 方向与路线（2026-11-13）

导航执行页保持当前深色主题、全屏相机、左下角地图和右上角退出按钮。左上角状态卡显示前行、左/右转、转身、前方转弯及沿路径剩余距离；路线带内的白色箭头按规划点顺序指向前进方向。方向来自机器人相对路径的姿态，不从屏幕投影猜测，因此路线离开相机视野时仍可提示转向。

方向仅供显示，任务执行、取消、最终到达仍由 Nav2 action 结果管理。接近路径终点只显示“接近目的地”，不会自行判定导航成功；偏离路线时暂停方向和剩余距离。当前同时支持步行引导和受运动许可保护的机器人执行。

### 投影与时间

- `projection_mode: auto`：有 CameraInfo 时使用标定投影；尚未收到标定时使用示意投影，并明确显示“示意路线”。已有标定但图像时刻 TF 缺失、标定无效或镜头模型不支持时暂停路线，不伪装为标定成功。
- `projection_mode: calibrated`：只允许 CameraInfo + TF 投影，适合完成标定后的部署。
- `projection_mode: ground`：强制使用虚拟地面投影，适合初期联调。
- 投影与方向均查询 Image 原始时间戳对应的 TF；不会用最新姿态替代历史图像姿态。图像时间为零、早于当前时刻超过 0.5 秒、超前超过 0.05 秒，或接收后超过 0.5 秒未更新时，撤下路线与实时方向/距离。
- 标定投影支持 `plumb_bob` / `rational_polynomial` 的 0/4/5/8 项畸变参数，并按图像尺寸调整内参与像素中心。其他模型需要先提供受支持的标定图像。
- `ground_frame: base_footprint` 应表示当地地面平面，Z=0 为地面；将路径变换到该系后重新贴地，忽略 Path 中的高度。没有该 TF 时，以 map 水平面和 `ground_height_m` 估算地面，界面明确提示按配置高度投影。

| 参数 | 当前默认 | 用途 |
|---|---|---|
| `ground_height_m` | 0.72 m | 缺少地面 TF 时 base_frame 离地高度，必须按本机实测 |
| `ground_render_offset_m` | 0.015 m | 贴地后的显示抬升 |
| `route_near_clip_m` / `route_far_clip_m` | 0.45 / 15 m | 机身前向近裁剪 / 水平距离远裁剪 |
| `route_width_m` | 0.95 m | 保留当前路线带宽，不代表避障或通行宽度 |
| `camera_height_m` / `horizon_ratio` | 0.72 / 0.43 | 仅虚拟投影的相机高度/地平线 |

真实贴地显示需要图像时刻的 `路径坐标系 ↔ ground_frame ↔ base_frame ↔ 相机 optical frame` TF、正确 CameraInfo 及相机外参；fallback 使用 map 与机身变换。相机/ROS 时钟须一致。标定模式偏高或偏低时应核对外参和地面系，不能靠修改虚拟相机参数校准。

Qt 和独立 `ros2 run nav_app ar_nav_node` 共用 `ar_projection.py` 与 `navigation_display.py`。独立入口按输入 Path 显示，不发送导航目标；仍输出兼容话题 `/nav/ar_image`，可用 `-p show_window:=false` 关闭 OpenCV 窗口。Qt 直接绘制矢量图层，页面布局与控制入口保留。

合成验证命令（不启动硬件）：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
PYTHONNOUSERSITE=1 QT_QPA_PLATFORM=offscreen ROS_DOMAIN_ID=187 ROS_LOCALHOST_ONLY=1 \
  python3 src/nav_app/test/smoke_ar_display.py
```

该检查创建真实 Qt/ROS 节点，直接输入合成 Image、CameraInfo、Path 和带时间戳 TF，检查三个屏幕尺寸的前行/左右转/转身/过期画面及独立话题输出，结果保存至 `/tmp/nav-ar-smoke-*/result.json`。它验证显示和时间选择，不证明真实相机地面重合精度或底盘运动效果。

## 第二批工具与 RTK

当前 ROS/Python 包为 `nav_app`。主窗口“工具”提供地图包、录包回放/校验、日志、诊断和 RTK；独立接收机包为 `src/nav_rtk`。操作、数据兼容和验证见 [第二批移植说明](../../OPERATIONS_MIGRATION.md)。

## NAV 界面更新（2026-11-13）

保留现有主窗口布局和全局 Qt 主题，标题、相机画面文案和新建地图/录包默认名称使用 NAV/nav。

- 工具 → 录包回放 → 录包管理：添加目录、扫描、搜索/排序、详情、加载、离线校验与确认删除；未完成校验的 stored 录包可直接发起校验。
- 录制 rosbag：选择原始 RGB 或 JPEG 95；JPEG 是有损格式，回放时恢复原始 Image 话题及消息头。
- 工具 → 诊断：运行状态、输入/定位质量、设备资源、原始详情、JSON 和诊断包导出。过期的定位/遥测不会显示为有效。
- 地图包、建图、导航、传感器、日志与 RTK 继续使用既有页面入口和风格。

新运行时路径、ROS 节点/话题、配置和环境变量已统一为 NAV 命名；不会自动改写用户已有的历史数据。

## 返回与按钮规则（2026-11-13）

- 功能页右上角统一“返回”；工具内的所有页签共享这一入口。传感器预览返回传感器管理，其余返回导航主页。
- 普通返回保留后台任务。导航画面的“退出导航”明确结束当前导航，也位于右上角。
- 描边导航按钮用于页面跳转；灰色用于普通操作；蓝色用于主要操作；红色用于停止、删除等危险操作。同级按钮统一字体、边框和各交互状态。
- 共享页头使用 `ui/page_header.py`，按钮样式集中在主窗口主题中；横屏、竖屏和桌面布局使用对应的一套公共字号。

## 系统首页与 RTK 入口（2026-11-14）

启动 App 默认进入“建图与导航”系统首页：

- “开始建图 / 继续建图”：使用现有建图流程；WOOSH profile 保持 `.3` 雷达与配对 IMU。
- “进入导航”：打开原导航准备工作区，不直接下发运动指令；右上角“系统首页”返回。
- “地图管理”：打开地图包列表；“设备与设置”集中提供传感器、导航系统、地图处理、工具日志与手机入口。
- RTK 入口改为“设备与设置 → 传感器 → RTK”，包含接收机、GNSS 质量及 NTRIP 差分；工具页不再重复放置 RTK。移动的是同一实例，原有连接、差分账号、独占调试任务保护和退出清理保持不变。
- 建图、传感器、地图处理和工具的顶层返回按钮回到系统首页；普通返回不停止正在进行的建图或取消导航。

首页主按钮使用 `#00A9DF`，支持 800×480 横屏、480×800 竖屏和桌面尺寸。原有通用与 WOOSH 启动命令、桌面 NAV 图标继续使用。

## 分类工作区（2026-11-14，已部署 Jetson）

设备与设置改为有标题、说明和蓝色箭头的分组卡片：

- 设备与导航：传感器、导航系统、手机遥控。
- 地图与采集：地图管理、录包与回放。
- 系统维护：日志管理、系统诊断。

“地图管理”统一放置“地图包 / 地图处理 / 处理进度”三个标签。首页“地图管理”、建图处理进度、导入编辑地图均指向这一目录。原 NAV · 工具大页已拆开，录包回放、日志、诊断和手机各自使用独立页面。

轨迹测试入口改为“设备与设置 → 传感器 → 传感器数据 → 轨迹测试”；RTK 仍在传感器管理页。移动的是原有页面实例，轨迹记录、地图任务、录包回放、RTK 互斥和后台协调保持同一实例；普通返回不停止这些任务。录包删除、诊断导出、写盘期间退出会跳转到相应独立页面等待完成。

新版分类已同步到 Jetson 当前发布，nav_app 构建及设备 44 项界面测试通过。桌面 NAV 图标继续使用原启动入口，打开后即可进入上述分类；部署完成后已启动界面，未自动开始建图、导航或底盘控制。设备原配置、地图和录包目录保留，更新前源码备份位于 `/home/nvidia/nav-deploy-backups/20261114-workspaces-120159/source-before.tar.gz`。


## 导航接入设置（2026-11-16，已部署 Jetson）

入口为“设备与设置 → 导航接入”，或“导航系统”右上角“导航接入设置”。选择预设后可逐项修改，点击“保存接入设置”后用于下次导航启动；运行期间需先停止导航才能修改。机器人接入来源统一在新页面管理，旧导航页保留算法参数和运行日志，导航后端固定为 Nav2。

- WOOSH 桥接 + FAST-LIVO：不启动本地驱动，直接使用 `/foo/scan`，由 NAV 启动 FAST-LIVO2 与点云 Localizer。
- 已有 scan + 里程计：不启动驱动、LIO 或点云转 scan；从可配置的 LaserScan、Odometry 和 TF 获取输入，默认选择 Nav2 AMCL。这里 `/foo/odom` 是可编辑的候选输入，不代表已经完成底盘 TF 接入验证。
- 本机驱动与处理：启动 NAV 配置的雷达/相机驱动、LIO 与点云转 scan；支持与外部模块混合选择。FAST-LIVO 原始雷达输入与 PointCloud2/转换输入分别配置，以保留 CustomMsg 和 PointCloud2 的格式区别。

地图定位可选择点云 Localizer、AMCL 或外部定位 TF。AMCL 与 map_server 共用定位生命周期管理，Nav2 导航生命周期独立；AMCL 使用导航 YAML/PGM，不要求 PCD，并可在“重新定位”中用两点向 `/initialpose` 提交初始位置和方向。初始位姿不是运动目标。AMCL 自动全局重定位按钮本版不启用。

本版标准坐标链固定为 `map → odom → base_link`。使用已有里程计时，需要新鲜的 `Odometry`（header.frame_id 为 odom）和对应的 odom→base_link TF；本版不自动生成或改名 TF。外部 map→odom 必须与所选地图和里程计一致，不能仅因为 frame 名称相同就混用两个系统。切换接入来源不会修改原车 bridge。

启动前检查当前组合所需的 scan/odom/TF，缺失时显示具体等待项，超时停止受管程序；运行时继续监控实际必要输入。仅作预览的相机不阻塞导航。停止导航只停止本次拥有的驱动和算法，不接管外部服务。通用 Nav2 与 WOOSH Nav2 的速度输出均隔离为 `/nav/planner_cmd_vel`，底盘执行适配器和运动许可保持原配置。

接入设置按现有机器人 profile（woosh/default）保存到 Qt 的 NAV/NavigationAccess 配置；测试和预览通过 `NAV_ACCESS_SETTINGS` 指向临时文件，不改变用户配置。页面已检查 1600×900、800×480、480×800，预览在 `/tmp/nav-access-preview-20261116/`。已部署到 Jetson；设备构建与 93 项相关测试通过。真实 bridge 已验证 `/foo/scan` 和 `/foo/odom` 有数据，但 ROS2 未提供 TF，已有里程计子帧为 `base_footprint`，因此外部输入模式明确等待 odom→base_link TF 并在超时后清理，不启动 LIO/点云转换或 Nav2。FAST-LIVO 预设仍复现 imu_gap，整套导航就绪和底盘运动验收未通过。测试使用临时接入设置，未改原车 bridge 或用户配置；桌面服务已恢复，active、NRestarts=0。备份、脚本、JSON 与截图位于 `/home/nvidia/nav-deploy-backups/20261116-navigation-access/`。

## 地图触控与首页后台状态（2026-11-16）

地图管理 → 地图处理：视图上方提供“旋转 / 平移 / − / ＋ / 复位”。先选择旋转或平移，再单指拖动点云；缩放使用 − / ＋。设备当前 eGalax 触摸屏只上报一个触点，现场操作使用这组单指工具。鼠标保留左键旋转、右键/中键平移、滚轮缩放和双击复位。右侧参数区支持手指拖动滚动。

首页分别显示建图服务、导航服务和地图后台任务的状态。导航服务占用时，开始建图会留在首页并显示原因；点击“停止导航”并确认，等停止完成后再开始建图。“停止建图”也需要确认，保存期间该按钮禁用。需要保留地图时，先在建图页使用“保存并结束”。

地图后台占用时点击“查看/暂停”，会打开处理进度并选中正在运行的任务；暂停完成后再尝试建图。输入配置导致建图失败时，可从首页“建图参数”修正。

本轮版本已同步设备，本地与 Jetson 相关 124 项回归均正常通过并退出；触摸输入使用合成事件验证，实际屏幕手感仍需现场确认。部署前源码及构建、测试记录位于 `/home/nvidia/nav-deploy-backups/20261116-gestures-home-services/`。新版交互在重新打开 NAV 后生效。

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


## 导航结束后的路线清理（2026-11-18）

导航成功、取消、失败、手动退出或停止导航服务后，会统一清除地图页和导航页的规划路线、当前任务定位轨迹及相机 AR 叠加缓存。当前位置标记继续更新，地图文件、日志和录包保留。结束后的迟到路线数据不会重新画回；下一次被接受的任务从新的轨迹开始。

更新后需重新打开 NAV 一次，运行中的任务请先结束。部署前源码备份与验证记录位于 `/home/nvidia/nav-deploy-backups/20261118-navigation-overlays/`。

验证覆盖任务成功/取消/失败、返回前后刷新、停止服务、晚到数据及新任务。另两项旧 UI 用例（unknown_policy_combo 与原导航日志入口）已在修改前版本确认失败，本轮保留其他页面行为。
