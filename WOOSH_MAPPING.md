# 当前默认：本机 .5 + /foo 相机1（2026-11-16）

桌面 WOOSH 入口默认选择 **FAST-LIVO2 · 本机 .5 + 相机1**。首页点击“开始建图”即可，参数配置中仍保留原 .3 选项。

2026-11-17 新增默认自体滤波：原始 `/livox/lidar_192_168_128_5` → 独立 `nav_cloud_self_filter`（PCL CropBox）→ `/nav/livox5/points_filtered` → FAST-LIVO2。过滤代码、YAML 和独立启动文件全部放在 `src/nav_sensor_proc/nav_cloud_self_filter/`；仅建图入口增加接线，驱动、IMU、图像和 SC 生成保持原样。参数页“点云输入”仍表示滤波器的原始输入。

配置 `src/nav_sensor_proc/nav_cloud_self_filter/config/woosh_livox5.yaml` 中，`base_footprint` 坐标排除框为 X [0.34,0.48]、Y [-0.25,0.25]、Z [0.30,1.65] m，覆盖测得的柱面与雷达安装点上的零距离回波。原始话题不变；框外点字段和时间戳保留。独立使用、旁路与边界修改方法见同目录 README。`self_filter:=false` 可对照原始输入。

首次安装此新功能包后需重开 NAV，以继承新 ROS 包的环境；先保存并结束正在运行的旧建图及地图编辑。已保存 PCD 内的旧自体轨迹不会被新输入过滤自动清除，需要重新建图验证。


- 点云：`/livox/lidar_192_168_128_5`（本机 PointCloud2，`base_footprint`）。
- IMU：`/livox/imu_192_168_128_5`（本机原始 `livox_frame`）；FAST-LIVO2 内部把加速度 g 乘 9.80665，角速度保持 rad/s，不修改驱动消息。
- 图像：`/foo/camera_1/color/image_raw`；启动时读取同相机 `/foo/camera_1/color/camera_info` 实时内参。
- 本机雷达与网络相机之间保留 0.4 秒处理缓冲（`navigation.input_holdback_s`），不改写传感器时间戳；避免更晚到达的 `/foo` 图像全部被丢弃。仅此 `.5` 参数启用；关闭图像融合时不等待，仿真时钟回放也不启用该缓冲。
- `.5` 安装参数为 pitch=30°、位置 `[0.429, 0, 0.605]` m；新参数将已在 base 中的点云转回 .5 IMU 状态坐标，静态 TF 使用同一逆变换。相机外参沿用 base 到相机的标定。
- 真机启动时调用 `systemctl --user start nav-livox5.service`。服务已运行时复用，不重复启动；停止建图保留独立雷达服务。该服务的配置文件为 `~/nav-current/src/nav_driver/livox_ros_driver2/src/config/MID360_config.json`，当前仅采集 `.5`、host IP 为 `192.168.128.100`，输出 PointCloud2。回放 `use_sim_time:=true` 不启动硬件服务。
- 不启动底盘运动；保存路径、两种栅格处理方法及 SC 生成流程保持原样。

命令行入口（不要与界面重复运行）：

```bash
source ~/nav-current/scripts/runtime_environment.sh
export ROS_DOMAIN_ID=0 OMP_NUM_THREADS=4 OPENBLAS_NUM_THREADS=1
ros2 launch nav_app woosh_native5.launch.py rviz:=false
```

新设备使用此入口前需先部署 `nav-livox5.service` 及匹配的 `.5` 驱动配置。改变 `.5` 安装外参时必须同步更新 `woosh_native5.yaml` 和 `woosh_native5.launch.py`，不能只修改话题名。

本次设备验收：新旧后端 7 项 Python 回归通过；本机及 Jetson 的 `nav_app` / `fast_livo` 已构建，原相机延迟/丢帧连续性 C++ 回归通过。真实 `.5` + `/foo` 静止建图约 20 秒，收到 136 条里程计，预览 47,716 点，完成 10 次 VIO，最大单轴位置波动约 1.3 cm，底座 roll/pitch 接近 0°。测试 PCD 成功保存到设备 `/tmp/nav-native5-validation/`；原 `.5` 驱动 PID 不变。验收建图已停止，不启动底盘运动。该结果不代表移动建图精度或长期运行验收。

以下保留此前 `.3` 接入和历次功能的记录；其“默认”描述为当时状态。

# WOOSH .3 仅建图接入

本入口复用 `woosh@169.254.128.2` 的 ROS 1 bridge，不启动雷达/相机驱动、Nav2、运动守卫或底盘适配器，不发布速度指令。

## 输入与坐标

- 点云：`/foo/livox/lidar_192_168_128_3`，PointCloud2，约 10 Hz。源端已经应用安装外参，坐标为 `base_footprint`，保留逐点纳秒 `timestamp`，不再次旋转或平移点云。
- IMU：`/foo/livox/imu_192_168_128_3`，约 200 Hz。`woosh_imu` 将加速度乘以 9.80665、加速度协方差乘以其平方；时间戳和 rad/s 角速度不变，输出 `/nav/woosh/imu3_si`，坐标名 `woosh_livox3_imu`。
- 机器人驱动实际 IMU 回调直接复制原始数据，没有应用安装旋转。DLIO 用当前机器人 `mid360_config.json` 的 yaw=45°、平移 `[0.338, 0.190, 0.330]` m 将 IMU 换算到机身参考点。
- 本 profile 的 `base_link` 与源点云 `base_footprint` 取同一参考点，DLIO 发布二者的单位变换及 IMU 安装变换。地图在 `odom` 系。
- 相机：直接订阅 `/foo/camera_1/color/image_raw` 与 `/foo/camera_1/color/camera_info`，沿用真实 640×360 内参；仅复现机器人两段相机静态 TF。DLIO 不融合图像，本轮地图为几何点云。
- 不桥接 ROS 1 的动态 map/odom TF，不使用其他设备的 D435 标定。安装位置变化后应更新 `woosh_dlio.yaml` 和相机 TF。

## 本地构建

```bash
cd /home/zy/ws/NAV-Module
source scripts/runtime_environment.sh
colcon build --packages-select direct_lidar_inertial_odometry nav_app
source install/setup.bash
```

## 界面使用

当前 Jetson 已通过 `~/.config/nav/runtime.env` 设置 `NAV_SENSOR_PROFILE=woosh`、`ROS_DOMAIN_ID=0`。直接双击桌面“识动 NAV / NAV”图标，即打开 WOOSH 界面；进入“建图”后默认选中“DLIO · WOOSH .3”，点击开始才运行建图。图标本身不自动建图或启动底盘控制，手机入口保持启用。

原通用入口仍保留。取消 `NAV_SENSOR_PROFILE=woosh` 并恢复原域设置后，设备脚本继续使用原通用界面。

两台设备时间需同步。机器人 bridge 当前在 DDS 域 0；通用 profile 默认域 101；当前 Jetson 桌面已通过 runtime.env 切换为 WOOSH profile 的域 0。

```bash
# Jetson 上如已有通用界面，先停止，避免重复窗口。
systemctl --user stop nav-app.service
source ~/nav-current/scripts/runtime_environment.sh
export ROS_DOMAIN_ID=0
export OMP_NUM_THREADS=4 OPENBLAS_NUM_THREADS=1
ros2 launch nav_app woosh_app.launch.py fullscreen:=true
```

本地工作区中同样先 source 运行环境并设置 `ROS_DOMAIN_ID=0`，执行上述 launch 即可。

进入“建图”，默认算法为“DLIO · WOOSH .3”，确认点云和 IMU 为上述 `/foo` 原始话题，关闭仿真时钟，点击开始。它会一起启动 SI 转换、DLIO 和相机 TF；不再启动底盘或传感器驱动。点击保存地图，通过现有 DLIO SavePCD 保存；结束后点击停止建图。

## 无界面建图

不要与界面建图重复启动：

```bash
source ~/nav-current/scripts/runtime_environment.sh
export ROS_DOMAIN_ID=0 OMP_NUM_THREADS=4 OPENBLAS_NUM_THREADS=1
ros2 launch nav_app woosh_mapping.launch.py rviz:=false
```

默认启用静止时首帧建图，原地也能验收有效点云；静止地图不代表完成移动建图精度验收。

另开已 source 的终端（同样设置域 0）：

```bash
ros2 topic hz /nav/woosh/imu3_si
ros2 topic hz /dlio/odom_node/pointcloud/deskewed
ros2 service call /save_pcd direct_lidar_inertial_odometry/srv/SavePCD "{leaf_size: 0.10, save_path: '/home/nvidia/nav_maps/woosh3-first.pcd'}"
```

`/dlio/map_node/map` 按关键帧更新，静止时可能只发布首帧；应在启动前订阅，或通过 `/save_pcd` 验证地图，不能要求静止时持续刷新地图话题。Ctrl+C 停止命令行建图。原通用 launch 仍保留；`systemctl --user start nav-app.service` 当前会打开 WOOSH 桌面界面。要恢复通用桌面入口，需取消 runtime.env 中的 WOOSH profile 选择并恢复原 DDS 域后重启服务。

## 2026-11-13 验证记录

- 本机两包构建通过；Jetson 两包构建通过。新增 3 项 IMU 单测正常通过并退出。
- 建图/Qt 组合回归报告 35 项断言通过，但退出阶段挂起，已停止该测试进程；不把该组合运行记为完整通过。
- Jetson 域 0 真实静止建图约 35 秒：收到 229 帧去畸变点云、3368 条里程计和有效地图；静止末端位姿距起点约 3 mm。
- 原始 IMU 加速度模长约 0.99，SI 输出约 9.70～9.79 m/s²；新节点使用独立 IMU frame，时间戳不改写。
- 相机图像与 CameraInfo 实际收到；内参分辨率 640×360。新 UI profile 实际参数与默认 WOOSH 建图后端核对通过。
- 首图：Jetson `/home/nvidia/nav-maps/woosh3-first-20261113-142608.pcd`，54,717 字节。
- 修复正常 Ctrl+C 时重复 shutdown 后再次建图、停止与 UI profile 启停均通过；验收进程已停止。
- 验收时 `/cmd_vel`、`/nav/planner_cmd_vel`、`/nav/base_command` 无发布者；未启动底盘控制。该次验收时尚未切换设备服务；随后已将桌面默认入口切换到 WOOSH profile。
- 本轮只完成静止建图和保存链路，不代表移动覆盖、长时间稳定性或定位精度验收。

桌面入口切换复核：服务 active，NRestarts=0；实际界面使用域 0，mapping_backend=woosh_dlio，相机图像和内参订阅正确，手机 HTTP 200；没有自动启动 DLIO、Nav2 或底盘控制节点。

## 保存当前地图被误报资源占用的修复

2026-11-13 修复：运行中的建图持有 `mapping` 资源归属，保存当前地图属于同一任务，沿用该归属；不再以 `map_snapshot` 作为另一个任务去申请。其他导航、录包或后台任务的隔离不变。新增资源归属回归与保存回执回归合计 35 项通过。修复在重新打开界面后生效。

本次未中断用户正在运行的建图，通过 `/save_pcd` 先保全地图到 Jetson `src/nav_map/woosh3-preserved-20261113-154205.pcd`：6,402 点，205,085 字节。

## 点云显示与高度截取预览

- 建图页默认使用高度伪彩色，可切回原始 RGB/强度；支持 1～5 px 点大小。伪彩色只表示高度，不是相机实景颜色。实时地图解析及普通点云绘制上限从 12 万提高到 30 万点，超过上限只影响预览，不减少地图保存数据。
- “地图处理”加载 PCD 后，可继续在已编辑点云基础上调整保留高度。调整高度或开启截取预览后显示蓝色下界平面和橙色上界平面，区间内点云保持颜色，区间外变灰；“仅显示区间内点云”隐藏区间外预览。
- 可点击“点选下平面/点选上平面”，再在可见点云表面单击，把该点的高度填入对应边界。高度数值改变立即更新预览，不删除或改写原始点。
- 预览与“转换为栅格地图”共用转换坐标判断；自动找平、轴变换与原始 XYZ 选择一致。实际转换使用完整的当前编辑后点云，预览保留计数只统计绘制采样点。
- 本轮新增及相关回归 9 项通过；此前 Qt 选择回归有 8 项断言通过但退出挂起，已停止该测试进程。正常地图完成视觉验收，约 250 万点地图在本机显示约 27.7 万采样点，单次截图渲染约 0.14 秒。

另发现 `woosh3-before-reopen-20261113-155022.pcd` 内含数百万至数千万米坐标，而保存前保留日志中的位姿仍接近正常范围。该文件已保留，异常来源尚未定位到轨迹计算、点云累积或保存中的具体环节，不应将其直接视为有效导航地图；界面显示优化不修复这种源数据异常。

## PCD 伪彩色与栅格实际比例

- 普通 PCD 首次加载默认完整显示高度伪彩，不自动灰显高度区间外的点。调整高度、点选平面或主动开启“预览高度截取”后，才显示截取效果。
- 有 intensity 字段时可切换“强度伪彩色”；原始 RGB/灰度强度仍可手动切换，显示伪彩不会改写原文件颜色。
- 转栅格分辨率标为 m/格，并显示厘米；默认仍为 0.05 m/格，即每格 5 cm。屏幕缩放只改变绘制大小，不改变真实分辨率。栅格图左下角显示随缩放更新的物理比例尺。
- 原转换函数隐藏的 0.06 m 障碍膨胀，旧实现会在 5 cm 网格上向上取整到两格，使单个点最多变成 25 cm 宽。地图处理现在显式提供“障碍外扩”，默认 0，保留原始障碍轮廓；开启时按格中心真实米制距离决定范围，不再整格放大半径。
- 修改分辨率或外扩量后需要重新点击转换，已生成的图像不随参数自动重算。转换结果显示格数、米制宽高、每格尺寸和外扩量；导航安全外扩仍需在代价地图中单独配置。
- 本机显示、比例尺、外扩与保存等 23 项相关回归通过，真实正常 PCD 已完成伪彩和栅格比例尺视觉验收。


## 首页建图、轻量预览和自动生成导航地图（2026-11-13）

1. 首页点击“开始建图”，直接进入建图页并启动当前配置的算法；已有建图运行时只返回建图页，不重复启动。WOOSH 配置继续使用 .3 雷达及对应 IMU，不启动底盘控制。
2. 点云采用与 gs-nav-monorepo 行程页相同的原始 XYZ、正交投影和高度伪彩。实时预览最多 12 万点、最多 5 次/秒；保存地图使用建图后端的地图数据，不受预览限额影响。文件编辑仍可显示最多 30 万点；拖动时临时减少显示点，松开恢复。静止点云复用渲染结果，三维面板不再额外生成二维预览，隐藏导航页面不处理实时点云。
3. 建完点击点云区顶部“保存并结束”。收到成功保存回执后才结束建图，等进程退出后自动加入现有持久任务队列：识别地面 → 生成栅格 `.pgm/.yaml` → 生成并核对 `.pcd.sc.v2` → 登记 `.scene.json` 地图包。PCD 保存失败会保留当前建图供重试；处理失败保留 PCD，进入“处理进度 / 地图包”重试。
4. “保存当前 PCD”仍用于中途快照，不结束建图；“停止（不处理）”只停止进程，不触发保存或自动处理。
5. 自动处理保持原始坐标和尺度，采用参考项目的地面识别与栅格采样策略。请求网格 5 cm，点云稀疏时会选择更粗的有效网格，并在完成提示中显示原因；不会通过屏幕缩放伪装分辨率。默认处理单层环境，地面识别不准时仍可用地图处理页的高度区间/平面预览手动调整。

验证：本机及 Jetson 上 54 项相关回归通过。设备用此前正常的 6402 点实测 PCD，真正生成栅格、84 个候选位置的 SC 索引并通过缓存校验；界面保存成功状态到后台 worker、地图包登记链路通过（保存回执和建图退出在该独立测试中模拟，未重新启动实体建图）。该稀疏图自动选择 20 cm 网格，原始 PCD 未改。1280×720 离屏布局检查完成。

设备相同 28 万点、1000×650 视口的离屏对比：载入约 266 → 75 ms，静止重绘约 158 → 5 ms，拖动帧约 158 → 40 ms。这是显示基准，不代表移动建图精度或长期实时负载验收。此前源点云异常问题仍需单独定位。

另有未改动的任务协调器既有测试 `test_worker_exit_does_not_release_live_descendant` 在本机失败：测试轮询时任务尚未进入 snapshot，直接下标 None；未将本次显示及工作流改动扩大为调度器修复。


随后完成真实传感器闭环：独立 Qt/ROS 实例从首页启动 WOOSH .3，收到 2233 点 odom 地图预览；实际调用 DLIO 保存服务得到成功回执，PCD 为 `/tmp/nav-real-workflow-maps/nav_map_20261113_012951_405.pcd`（设备本地时区命名）。等建图退出后，持久任务 `c05b5af1f82b80e0a8d219c8095ee66c` 自动完成栅格、SC 和地图包登记，地面 Z≈0.0183 m。没有启动底盘控制，测试产物只在 `/tmp`；真实连续移动与长期负载仍待验证。另清除了界面模块末尾重复的 `main()` 调用，避免关闭后再次进入主程序。


## 建图工作页与参数页分离（2026-11-13）

- 首页“开始建图”进入新的全宽建图工作页，并启动当前配置；建图运行时再次进入只返回工作页，不重复启动。
- 日常工作页提供实时高度伪彩点云、重置视角、开始/停止、保存当前 PCD、保存并结束，以及处理进度/地图包入口。
- 点击右上“参数配置”进入原建图页，配置算法、雷达/IMU/相机话题、时钟、保存名称/目录/体素和点云显示参数，并查看日志或恢复中断地图。点击返回回到新工作页。
- 两页共用同一个 MappingController 和保存/自动处理状态。切换页面不停止、不重启建图；参数页期间不绘制隐藏点云，返回工作页显示最新预览。保存并结束后生成栅格和 SC 的流程保持一致。
- 新工作页 1280×720、800×480、480×800 下开始、保存结束、参数按钮均在窗口内，点云区域使用可用全宽；已进行离屏截图检查。


本轮本机及 Jetson 各 59 项相关回归通过；设备真实 .3 数据预览 2231 点，实际点击新工作页的参数入口并返回，确认建图进程继续运行、隐藏期间不更新绘制版本。随后从新页保存结束，后台完成 `.pgm/.yaml`、`.pcd.sc.v2` 和 `.scene.json`。独立测试地图包位于 `/tmp/nav-pages-real-maps/nav_map_20261113_014805_832-nav-302d79b3.scene.json`，稀疏原地观测选择 25 cm 网格；测试未启动底盘控制，未替换正式地图。


## 地图包文件选择与统一目录（2026-11-14）

- 在线建图、地图处理导出和地图包的默认目录统一为 `~/nav_maps`，本设备即 `/home/nvidia/nav_maps`。不再把运行地图默认写入源码 release 的 `src/nav_map`；显式设置的 `map_storage_dir` / `NAV_MAP_DIR` 及用户手动选择的输出目录仍有效。
- 设备原 `~/nav-maps` 内的 2 套完整地图（10 个文件）已复制至新目录，原件保留；PCD、栅格及 SC 的内容和关联校验通过。
- 进入“地图管理 → 地图包”，选中已有地图包即可查看所绑定的三个文件；“新地图包”清空表单。分别选择 PCD、SC `.sc.v2`、栅格 YAML，填写名称，核对坐标后点击“保存地图包”。修改已有包会保留地点等用户元数据；PCD/栅格变更会清除坐标确认与旧生成说明。
- PCD/栅格旁“查看”在当前页面预览；点云使用原始 XYZ 和高度伪彩，栅格保留真实分辨率和原点。“处理”直接进入地图处理页并加载对应文件；完整点云用于编辑，最多 30 万点的预览采样不替代完整源数据。
- SC 会校验格式、完整性及与所选 PCD 的匹配关系，失败不会保存错误组合。SC 可留空后续生成。PGM 可以预览、处理；登记导航地图包使用包含分辨率与原点的 YAML，PGM 由 YAML 引用。
- 查看与文件核对在后台执行；小屏点击查看后自动滚动到预览区域。旧的删除保护及坐标确认流程保留。

设备当前地址为 `nvidia@192.168.128.100`，已与原 `169.254.128.129` 的 SSH 主机密钥核对一致。使用实际 6859 点地图验证了点云/栅格预览和完整编辑跳转，显式 PCD/SC/栅格组合在 `/tmp/nav-library-explicit-bundle` 完成登记校验。本机及设备各 67 项相关回归通过；重开后运行中的 ROS 参数确认 `map_storage_dir=/home/nvidia/nav_maps`，服务 active、NRestarts=0、手机接口 HTTP 200；未启动底盘控制。


## 栅格粗糙对照与 WOOSH 降采样修正（2026-11-14）

已只读对照 `jgrobot@192.168.110.88` 的实际地图，参考设备未修改：

| 指标 | NAV 当前地图 | GS 参考地图 |
|---|---:|---:|
| PCD | `nav_map_20261113_022140_956.pcd` | `gs_map_20261113_171840_699.pcd` |
| 点数 | 6859 | 1384104 |
| 地面独立采样点 | 559 | 274640 |
| 地面近邻间距 P75 | 13.44 cm | 3.23 cm |
| 栅格尺寸 | 15 cm | 5 cm |

两图覆盖范围不同，总点数不能单独作为精度评价。对同一份 GS PCD 使用 NAV 的 `observed_floor_grid`，与 GS 已保存栅格逐格比较，差异为 0；问题不在复制了错误的参考投影算法。对参考 PCD 先施加 25 cm 三维体素平均后，仅剩 75414 点，地面间距 P75 约 24 cm，自动网格变为 25 cm，验证了过早稀疏化对细节的影响。

WOOSH 此前沿用通用 DLIO 的 `odom/preprocessing/voxelFilter/res=0.25` 和 `map/sparse/leafSize=0.25`。前者先稀疏配准及关键帧，后者再次稀疏累积地图；最终保存设 5 cm 不能恢复已删除的数据。本轮仅在 `woosh_dlio.yaml` 覆盖这两个参数为 `0.05`，原始单位、坐标和自动栅格策略保持当前配置。较细参数增加计算与存储量，持续采集负载尚未验收。

本机 YAML 解析及设备隔离 ROS_DOMAIN_ID=87 的原生 DLIO 参数查询通过，两个节点实际值均为 0.05。测试未接入真实传感器、未控制底盘、未重开当前界面。现有 PCD/PGM 未覆盖；下次开始建图会加载新参数，需要重新采集完整区域后保存并结束。当前生产 ROS2 图仅看到界面相关节点，未看到机器人桥接，尚未完成新参数下的移动采集质量验证。

配置备份：本机及设备 `/tmp/nav-woosh-grid-quality-before-20261114.yaml`；实际对照数据与图像位于本机 `/tmp/nav-grid-comparison/`。


## NVIDIA 直连三雷达与 WOOSH 对齐（2026-11-14）

已只读核对底盘 `woosh@192.168.128.2` 的运行参数：`xfer_format=0`、`multi_topic=1`、`publish_freq=10`、`frame_id=base_footprint`。底盘驱动没有安装源码；通过导出函数反汇编确认点云使用各台外参并从毫米转米，IMU 六轴原样输出，没有套用点云安装旋转。

NVIDIA 端 `nvidia@192.168.128.100` 已作以下对齐：

- 移除驱动中共享的 `extrinsic_global` IMU 旋转。各台 IMU 保留传感器坐标，角速度 rad/s、加速度 g、frame 为 `livox_frame`。
- 普通驱动默认 PointCloud2、独立话题、10 Hz、点云 frame 为 `base_footprint`。各台点云的安装旋转与平移只处理一次，IMU 的 SI 转换和安装外参继续由 SLAM 输入适配处理。
- WOOSH 界面“雷达”面板重开后也默认以上参数；其他 profile 的界面默认保持原样。RViz 独立显示三路点云。
- 三台点云为 `/livox/lidar_192_168_128_3`、`_4`、`_5`；IMU 为 `/livox/imu_192_168_128_3`、`_4`、`_5`，无 `/foo` 前缀。
- 新增回归测试经过生产 UDP 解析回调、逐台点云外参及 ROS 发布代码；仅替换 SDK observer 和 ROS 发送边界，不连接雷达。验证三路话题、三个安装姿态、六轴原始值和时间戳。设备 6 项 CTest 及本机 34 项导航/界面测试通过。

NVIDIA 的默认 JSON 仍为前次设置的三雷达配置，接收地址 `192.168.128.100`。启动直连驱动前，先协调同组雷达的采集主机，避免机器人、本机和 NVIDIA 同时配置数据目的地址。不要直接结束底盘的 required 雷达节点，它可能连带结束整套 bringup。本次没有启动直连驱动或改变正在运行的底盘采集。

```bash
source ~/nav-current/scripts/runtime_environment.sh
export ROS_DOMAIN_ID=0
ros2 launch livox_ros_driver2 msg_MID360_launch.py
```

需要直接用 NVIDIA 采集的 .3 数据跑 DLIO 时，在已有驱动的另一个终端运行：

```bash
source ~/nav-current/scripts/runtime_environment.sh
export ROS_DOMAIN_ID=0 OMP_NUM_THREADS=4 OPENBLAS_NUM_THREADS=1
ros2 launch nav_app woosh_mapping.launch.py \
  pointcloud_topic:=/livox/lidar_192_168_128_3 \
  imu_topic:=/livox/imu_192_168_128_3
```

以上仍是 .3 雷达配对 IMU 的 SLAM。现有桌面建图默认继续使用 bridge 的 /foo 输入；本次未改成三雷达融合，也未新增底盘滤波/虚拟激光扫描链。三路独立点云同为机身坐标，并不代表扫描时刻已同步；多雷达 SLAM 仍需同步、运动补偿和选定单路 IMU。不要将三路数据简单发布到同一 SLAM 输入，或直接使用硬编码合并话题的通用 FAST-LIVO 启动入口。

代码备份在本机及设备 `/tmp/woosh-multi-lidar-before-20261114.tar.gz`，RViz 原配置在 `/tmp/woosh-multi-lidar-before-display.rviz`。实测三路收数、移动建图质量仍待协调采集主机后验证。


## FAST-LIVO2 · WOOSH .3 + 相机1（2026-11-14）

当前 NAV 设备地址为 `nvidia@192.168.128.100`。本次新增独立 `woosh_fast_livo` 后端，WOOSH 桌面默认选中它；原 `DLIO · WOOSH .3` 以及既有 DLIO 配置保留。仅适配建图，不启动机器人驱动、Nav2 或底盘执行。

使用方式：

1. 首页“开始建图”启动 FAST-LIVO2；如需核对，在建图工作页进入“参数配置”，选择 **FAST-LIVO2 · WOOSH .3 + 相机1**。
2. 雷达 `/foo/livox/lidar_192_168_128_3`，IMU `/foo/livox/imu_192_168_128_3`，图像 `/foo/camera_1/color/image_raw`，默认开启视觉融合。实时内参来自同相机的 `/foo/camera_1/color/camera_info`；相机或标定不对时启动会明确报错。
3. 返回建图页观看点云。显示参数仍可选高度伪彩、强度伪彩或原始 RGB；真实 RGB 来自 FAST-LIVO2 相机融合输出，几何地图和 RGB 可见范围不同。
4. “保存当前 PCD”保存几何快照；“保存并结束”保存成功后结束建图，沿用已有后台栅格及 SC 索引生成流程。现有 SaveMap 持久 PCD 为几何/强度数据，RGB 为实时显示产物，不把几何 PCD 宣称为带相机 RGB 的文件。
5. 返回 DLIO 时只需在参数配置中改选 `DLIO · WOOSH .3`；两种算法不要同时运行。

无界面启动（不可与界面建图重复启动）：

```bash
cd ~/nav-current
source scripts/runtime_environment.sh
export ROS_DOMAIN_ID=0 OMP_NUM_THREADS=4 OPENBLAS_NUM_THREADS=1
ros2 launch nav_app woosh_fast_livo.launch.py rviz:=false
```

本机从 `/home/zy/ws/NAV-Module` 使用同一命令。临时验证激光惯性链路可显式传 `img_en:=0`；该模式不构成三传感器视觉融合验收。构建命令：

```bash
source scripts/runtime_environment.sh
CMAKE_BUILD_PARALLEL_LEVEL=2 colcon build --symlink-install \
  --packages-select fast_livo nav_app --parallel-workers 1 --cmake-args -DBUILD_TESTING=OFF
```

适配内容：

- FAST-LIVO 原生新增 `preprocess.lidar_type=8`，读取 WOOSH PointCloud2 的 xyz/intensity/tag/line/float64 timestamp；支持未对齐的 26 字节点和带行填充的组织点云。逐点绝对纳秒时间减原始扫描头时间后转毫秒，保留真实采样间隙，稳定排序供去畸变使用；错误字段、单位或 frame 会报错，不用接收时间伪造点时间。
- 机身点云坐标保持 `base_footprint`，不重复应用驱动已经施加的 yaw=45° / [0.338,0.190,0.330] m。`woosh_fast_livo.yaml` 的 extrinsic_R/T 将该虚拟雷达坐标映射到物理 IMU。滤波里程计/去畸变 body cloud 使用 `woosh_livox3_imu`，通过实际逆安装 TF 关联 `base_link`、`base_footprint` 及相机，不只改 frame 名称。
- 复用现有 `woosh_imu` 输出 `/nav/woosh/imu3_si`，加速度 g→m/s²，角速度 rad/s、原始时间戳不变。FAST-LIVO 原有初始化重力归一化保留，不额外再乘一次 g。
- 相机 Rcl/Pcl 由机器人当前相机1两段静态 TF 推导，实时 CameraInfo 为 640×360、fx=fy≈359.763、零畸变；离线快照为 `woosh_camera1.yaml`。没有沿用其他设备 D435 的内参、外参或 0.1 s 图像偏移。时间偏移以 0 为起点，物理标定精度、精密时延和移动建图仍待实测。

验证：本机和 Jetson 的 Python 回归各 65 项通过；本机新增 WOOSH 点云/原 CustomMsg 点时间两项 C++ 回归通过。本机隔离域合成房间测试开启相机融合，得到 132 组同时间戳的点云/里程计、11 帧 RGB 地图，保存 PCD 成功且拒绝覆盖已有文件。合成测试不启动硬件或底盘，不作为实体精度验证。

实机当前数据条件：开始适配时 ROS1 .3 点云和 IMU 有发布者但无新消息，ROS1 相机图像有更新，ROS2 首轮探针仅收到 CameraInfo。用户确认另有终端在调试驱动，要求保留驱动状态；本次未重启、更改机器人驱动/bridge/网络配置。实体点云、IMU、图像收数与真实融合验收须待该调试恢复数据后进行，不能把已编译和合成成功写成真实传感器通过。


Jetson 追加验证：两个包构建成功；65 项 Python 与两项 C++ 回归通过。设备隔离域 228 中合成视觉融合得到 116 条 body 点云、116 条里程计、115 组精确时间戳配对、10 帧 RGB 地图，PCD 保存 24,251 字节且已有路径覆盖被拒绝。设备测试输出 `/tmp/nav-smoke-anz9ln4a`；本机测试输出 `/tmp/nav-smoke-kqadh6jm`。这些是隔离合成输入，未使用或改变正在调试的实体驱动。


用户确认已保存编辑后，已重开 NAV 用户服务：实际参数 `mapping_backend=woosh_fast_livo`，相机 image_raw/CameraInfo 均为 `/foo/camera_1/color/*`，服务 active、NRestarts=0、手机 HTTP 200。未自动启动实体建图。启用后的最终只读 4 秒探针得到 cloud=0、imu=0、image=0、camera_info=20；真实融合仍待并行驱动调试恢复传感器数据。


## 已加载栅格与编辑拖动诊断（2026-11-14）

用户截图来自直接加载 `916-20261114` 的 YAML/PGM。已通过原主机密钥重新连接设备 `192.168.110.119`。实际文件为 `nav_map_20261114_140548_637-nav-5e1990cb.yaml`，208×301 格、5 cm/格、negate=0；对应 PCD 有 85903 点，地面独立采样 8509 点、近邻间距 P75≈2.50 cm。本次地图日志启动的是 FAST-LIVO2，不能沿用上一张 DLIO 小样本的稀疏原因解释这张图。

由对应 PCD、元数据地面高度 -0.289582 m 和当前自动转换算法重建，结果与保存的 PGM 逐格差异为 0，原 PCD SHA256 与地图包记录一致。自动转换采用地面以上 0.20～2.10 m 的障碍投影和 0.10 m 外扩，桌椅等高度内物体会投成平面障碍。取消外扩、保持其他数据与坐标不变时，黑格由 8061 降为 4225，白格由 1668 增为 2335。截图右侧 0.00 m 和高度范围是手动重新转换的输入，不是已加载文件的历史生成参数；界面已补充说明。

另存同坐标对照：`/home/nvidia/nav_maps/916-20261114-no-inflation-comparison.yaml` 及其 PGM。原图和地图包未覆盖；对照只取消障碍外扩，不将缺少观测的区域补成自由。

编辑跳动已分别复现并修复：画笔状态双击曾无条件执行 fit_view，导致缩放 2→1、平移归零；切换工具时旧平移手势未结束，后续画笔移动可继续拖动地图。现在切换工具结束旧手势，画笔双击继续绘制，仅浏览/平移模式双击复位。

本机及设备地图相关 59 项回归通过；设备追加确认 5 项编辑/加载像素回归通过，确认黑白类别与原 PGM 一致。一次更宽范围运行记录为 81 passed / 1 failed（传感器进页测试），未修改该项传感器逻辑；该共享工作区在此期间还有其他传感器代码更新，保留其改动。本轮已按用户确认保存后重开 NAV，服务 active、NRestarts=0；无底盘运动控制。


## 地图包自动预览（2026-11-16）

- 原页面选择地图包只填写路径、切换预览标签也不读取文件，必须再点击“查看”；已修复为选中地图包自动加载当前标签，切换点云/栅格标签自动加载对应文件。首次进入会选择首个地图包，未完成的新地图包表单会保留。
- 选择 PCD/栅格文件或完成路径输入后直接预览。“查看”仍可强制重读；切回已加载标签复用预览。预览由独立的单个后台任务读取，快速切换只显示最后选择的文件，读取失败显示文件路径与原因并清除旧预览。
- 预览不更改地图编辑数据、不保存地图；“处理”仍加载完整点云。预览点云读取上限 30 万点。
- 本机及 Jetson 各 81 项相关回归通过，覆盖自动进入、标签切换、缓存、快速选择、错误文件、草稿保留及原有编辑/地图库行为。设备实际 `916-20261114` 文件验证：自动显示 85903 点、301×208 栅格（5 cm/格），不点击“查看”，编辑器内地图保持未加载。
- 设备部署首次测试发现缺少 preview_worker；核对后 build/nav_app/nav_app 实际为源码链接，问题是运行文件内容尚未一致。按内容校验同步后，本机/设备 operations.py SHA256 均为 ac3fab20fae02a7498ff51c3a6787def9d56fe4760c60bbf2e3c1203a3ca4e5b，回归通过。当前界面进程启动时间晚于同步，HTTP200。不能仅因导入路径含 build 就判断是独立旧副本。

截图（本机）：`/tmp/nav-auto-preview-cloud.png`、`/tmp/nav-auto-preview-grid.png`。设备原地图未改动；本轮未启动建图或底盘运动。
