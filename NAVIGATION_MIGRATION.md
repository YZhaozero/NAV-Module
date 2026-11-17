# 建图导航移植说明

> 本文记录第一批及其默认构建修复；第二批已加入工具/RTK 并将模块和 App 包改为 `nav_*`，当前入口与验证见 [第二批移植说明](OPERATIONS_MIGRATION.md)。

日期：2026-11-12。本文记录第一批代码移植、接口和验收方法，不将源码构建或合成测试视为实机导航验收。

## 目标与成功标准

工作区继续采用 `src/`、`build/`、`install/`、`log/`，不移动原有模块，不改变 ROS 包名。保留现有 Qt 页面和 DLIO；接入独立于 GS 业务的 FAST-LIVO2、点云定位及 Nav2 改进。

本轮完成标准：相关 C++ 包可构建，App 原有回归及新增接口回归通过；合成输入能产生同时间戳的机身点云与里程计、保存地图并拒绝覆盖；Localizer 的排队回执不被当作定位成功，过期定位不能提交新导航任务。

## 来源与边界

来源仓库：`/home/zy/ws/GITREPO/gs-nav-monorepo`。
固定提交：`aafb462ea8f7875205da888946d05c3d31bc8477`。检查期间来源分支从此前评估版本切换到了 `test/module2-main-fusion`，本轮以固定提交快照统一迁移，未切换任一共享仓库分支。

只移植下表及其直接编译/运行依赖；保留原许可证。未迁入来源仓库的 GS 重建/训练/辅助定位、RTK、百度路线、步行引导、手机遥控、私有 DDS 配置及设备专用标定。

| 来源 | 当前目录 | 本轮内容 |
|---|---|---|
| `fast-livo2/` | `src/nav_slam/fast_livo/` | 导航点云/里程计、内存和输出队列、地图服务、检查点、恢复工具及相关测试 |
| `third_party/rpg_vikit/vikit_common/CMakeLists.txt` | `src/nav_slam/rpg_vikit/vikit_common/` | 去掉强制 native 指令集以统一 Eigen ABI，构建产物留在构建目录 |
| `localization/localizer/` | `src/nav_localization/localizer/` | 异步匹配、跟踪质量、输入时间/坐标检查、SC v2、全局匹配及有界求解 |
| `drivers/livox_ros_driver2/` | `src/nav_driver/livox_ros_driver2/` | 设备时钟映射、点云帧边界、双格式及有界发布 |
| `drivers/realsense_ros/` | `src/nav_driver/realsense_ros/` | 逐帧采集时间、深度重复发布修正及配套有界发布代码 |
| `gs-app/` 的建图/导航部分 | `src/nav_app/` | 地图转换、建图保存、算法选择、Nav2 启动、定位质量和必要回归 |

Nav2 本体源码不保留，运行时复用 ROS 2 Humble 安装的 Nav2；本项目只保留启动封装、运行依赖和参数入口。SCAN-Planner 与 pointcloud_to_laserscan 继续使用现有包和目录。

原有未提交的部署说明、历史日志、App/RealSense README 与 Localizer YAML 不被整文件覆盖；Localizer 的地图路径、匹配阈值以及传感器/相机原有标定和时间补偿继续保留。

## 本轮行为

### 地图转换

`pointcloud_to_grid()` 默认将空白栅格设为未知，只将观测到的障碍/明确自由证据写入对应状态，障碍和膨胀优先。界面默认“空白区域保留未知”。人工已确认覆盖时可明确选择“已确认覆盖：空白区域设为自由”；不能由点云空洞推断可通行。支持大点云预览抽样，完整保存与预览使用独立数据。

### FAST-LIVO2 与 DLIO

默认仍为 DLIO。在“导航系统 → 里程计”或“建图 → 建图算法”选择 FAST-LIVO2。建图页和导航页均可关闭视觉；本轮新增入口默认不启用未经本机复核的视觉融合。FAST-LIVO2 雷达输入使用 `CustomMsg`，Livox 的格式 4 同时提供 CustomMsg 和 PointCloud2。独立建图前仍需启动真实传感器或外部回放输入。

切换导航里程计时，Localizer、LaserScan 和 SCAN-Planner 的点云与里程计契约一起切换。建图与导航互斥，启动依赖真实传感器/里程计与导航 action 就绪；异常退出会收尾本次拥有的依赖进程。

地图预览限制内存，完整地图使用分片落盘。保存回执与提交凭证关联到本次请求，旧请求不能完成新保存；地图名带时间戳，已有输出不覆盖。建图进程退出时保留落盘检查点。建图页“恢复中断建图”选择对应的 `capture-*` 目录，将已完成检查点恢复为新的 PCD；未落盘末段不能恢复。

### Localizer 与 SC

Localizer 发布独立质量状态，包含匹配与输入年龄、请求编号及完成结果。App 使用其判断定位有效性：服务接受排队请求时继续等待最终结果；过期质量或输入不能当作有效地图定位。定位失效会拒绝新目标，并对已运行任务请求取消。

SC 使用 `.pcd.sc.v2`，包含 PCD SHA-256、固定描述子维度、生成参数和负载摘要。旧 `.sc` 不作为 v2 使用；可在原“生成 SC 地图”入口重新生成。生成工具默认半径 80 m、网格 2 m，显式缓存与当前 Localizer 配置不符时拒绝使用。原 YAML 的参数值保持不变。

### Nav2 与底盘

选择点云 Localizer / 自定义外部定位时，新入口仅启动 map_server 与 Nav2 navigation，避免额外启动 AMCL。选择“不启动定位 + Nav2”时继续由 Nav2 完整 bringup 启动 AMCL。

本项目保留标准 `/cmd_vel`，没有迁入来源的 `/gs_nav/planner_cmd_vel → MotionGuard → BaseAdapter` 链。软件取消请求不证明底盘物理停止；底盘看门狗、急停、速度约束和独立里程计反馈仍由机器人本身负责，并需现场验证。

## 时间与坐标契约

| 数据 | 接口 | 坐标 / 时间 |
|---|---|---|
| 去畸变机身点云 | `/fast_livo/cloud_body` | `base_link`；与同批里程计一致 |
| 世界点云 | `/fast_livo/cloud_world` | `odom`；不能当作机身点云 |
| 里程计 | `/aft_mapped_to_init` | `odom → base_link` |
| 几何地图预览 | `/fast_livo/map` | `odom` |
| 完整覆盖的上色预览 | `/fast_livo/map_preview` | `odom`；无相机颜色的几何仍保留 |
| 地图保存 | `/fast_livo/save_map` | `fast_livo/srv/SaveMap`，绝对 PCD 路径和体素尺寸 |
| 定位质量 | `/localizer/localizer_node/quality` | JSON 版本 1，ROS 测量时间及匹配/输入年龄 |
| 地图定位 TF | `map → odom` | Localizer 或 AMCL，只有一个发布者 |
| 机器人速度 | `/cmd_vel` | 沿用现有底盘接口 |

FAST-LIVO2 本配置以 LiDAR 内置 IMU 作为机身参考。真实机器人的 `base_link` 如果有不同原点/方向，需要先适配并核对 TF；不能仅改话题 frame 标签。实机内外参、LiDAR 驱动安装变换、相机分辨率和时间补偿必须配套。现有相机时间补偿未因移植而自动改零。

实机使用系统时钟；回放应统一启用 `use_sim_time` 并发布 `/clock`。App、独立 TF 监听与当前算法使用一致时钟。

## 构建与检查

当前工作区默认入口：

```bash
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

根目录 `colcon_defaults.yaml` 由已安装的 colcon-defaults 插件加载。默认扫描保留的机器人模块目录，构建 `nav_app` 及其运行依赖，Nav2 复用系统包；`./build.sh` 同样使用这些设置。默认关闭 CMake 测试，使用 Ninja 编译池限制每个包两路编译、一路链接，并按顺序构建各包。依赖脚本原本已包含 `ninja-build` 和 colcon 扩展，无新增系统安装。

PCL 通过 `PCL_ROOT=/usr` 和 CMP0074 的 NEW 行为选择系统安装，CMake 自行解析架构目录；其他本地库（例如 TEASER++）仍按原路径查找。命令行若显式传入 `--cmake-args`，会替换该默认参数列表；应保留需要的系统 PCL 和并行设置。选择额外的已扫描定位包可使用 `--packages-up-to`。工作区不再提供可单独构建的 Nav2 源码。

### 原目录缓存修复（本轮追加）

用户 2026-11-12 16:58 的命令为 `colcon build`。其 CMakeCache 中的 `CMAKE_CACHEFILE_DIR` / `CMAKE_HOME_DIRECTORY` 仍是 `/home/zy/ws/GS-NAV-Module`，与当前 `/home/zy/ws/NAV-Module` 不符，因此在 C++ 编译开始前失败。该失败与先前单独记录的 lint 限制不是同一项检查。

此前保留旧 `build/`、`install/` 却仅验证 `/tmp` 构建，是默认入口验收遗漏。本轮将两者整体保存在 `/tmp/nav-module-before-rebuild-20261112-Keb9rwmB/` 后重新生成原目录；未删除或移动 `src/`、地图与历史日志。旧缓存不可复制回新路径。原目录实际验证结果：

| 检查 | 结果 | 证据 |
|---|---|---|
| 根目录裸 `colcon build` | 17 包成功，退出码 0，冷构建 8 分 3 秒 | `/tmp/nav-module-root-build.log` |
| source 当前 install 后 `./build.sh` | 17 包成功，增量构建 5.88 秒 | `/tmp/nav-module-build-script.log` |
| CMake 缓存 | 当前 16 个缓存无旧 `GS-NAV-Module` 根路径引用 | 本次原目录检查 |
| 安装产物 | 五个核心包前缀均为当前工作区 install；Python、SaveMap、动态库导入成功，无 PCL 1.14 或缺失动态库 | 本次原目录检查 |
| App launch | `ros2 launch nav_app ar_nav.launch.py --show-args` 成功解析 | `/tmp/nav-module-workspace-check/launch-args.log` |
| 当前 install 合成建图保存 | 117 组精确时间戳配对、12 次地图消息、保存 28,555 字节，拒绝重复覆盖 | `/tmp/gs-nav-smoke-usqgrvld/result.json` |

这些检查使用隔离 ROS 域和合成数据，未启动真实雷达、相机或底盘。终端中已有编译警告保留；成功依据是退出码 0 和最终 Summary 无失败包。

本轮使用独立 `/tmp/nav-migration/{build,install,log}` 检查，避免移动、清理或复用原工作区旧绝对路径缓存。重现命令：

```bash
source /opt/ros/humble/setup.bash
CMAKE_BUILD_PARALLEL_LEVEL=2 MAKEFLAGS=-j2 colcon --log-base /tmp/nav-migration/log build \
  --base-paths src/nav_app src/nav_driver src/nav_slam \
    src/nav_localization src/nav_sensor_proc src/nav_navgation/SCAN-Planner-Ros2 \
  --build-base /tmp/nav-migration/build --install-base /tmp/nav-migration/install \
  --executor sequential --packages-up-to nav_app \
  --packages-ignore fast_gicp ndt_omp_ros2 scancontext_ros2 \
  --allow-overriding realsense2_camera realsense2_camera_msgs \
  --cmake-args -DBUILD_TESTING=ON -DCMAKE_BUILD_TYPE=Release \
    -DPCL_DIR=/usr/lib/$(gcc -dumpmachine)/cmake/pcl
```

App 回归（ROS Python 和 PyQt5 依赖已安装）：

```bash
source /opt/ros/humble/setup.bash
PYTHONPATH="$PWD/src/nav_app:$PYTHONPATH" QT_QPA_PLATFORM=offscreen \
  python3 -m pytest src/nav_app/test -q -p no:cacheprovider
```

合成建图接口检查不启动物理传感器：

```bash
source /tmp/nav-migration/install/setup.bash
ROS_DOMAIN_ID=187 ROS_LOCALHOST_ONLY=1 \
  python3 src/nav_slam/fast_livo/test/smoke_navigation.py
```

中断地图也可命令行恢复，输入为某次会话的检查点目录，输出必须是尚不存在的绝对路径：

```bash
ros2 run fast_livo recover_map /path/to/capture-session /path/to/recovered.pcd
```

### 本次验证结果

验证平台为本机 x86_64 / Ubuntu 22.04 / ROS 2 Humble。下表是第一批 `/tmp` 验证记录；随后原工作区构建缓存已按上节单独备份重建，二者的验收范围分别记录。

| 检查 | 本次结果 | 证据 |
|---|---|---|
| App 运行依赖构建 | 17 包成功；Nav2 复用系统 Humble | `/tmp/nav-migration/log/` |
| App 回归，含实际 ROS TF 投递/时钟/重映射 | 116 项通过 | `/tmp/nav-migration/python-final.xml` |
| FAST-LIVO2 C++ 回归 | 45 项通过，串行执行 | `/tmp/nav-migration/fast-livo-final-serial.log` |
| Localizer 功能回归 | 9 项通过 | `/tmp/nav-migration/build/localizer/Testing/Temporary/LastTest.log` |
| Livox 驱动回归 | 5 项通过 | `/tmp/nav-migration/build/livox_ros_driver2/Testing/Temporary/LastTest.log` |
| RealSense 发布回归 | 1 项通过 | `/tmp/nav-migration/build/realsense2_camera/Testing/Temporary/LastTest.log` |
| 合成建图与保存 | 117 组精确时间戳配对、12 次地图消息，保存 28,523 字节，拒绝重复覆盖 | `/tmp/gs-nav-smoke-lbac6o4v/result.json` |
| 临时安装的真实 Qt/ROS App | 无硬件启动、渲染与退出成功 | `/tmp/nav-migration/app-mapping-800x480.png` |
| 小屏界面 | 800×480 建图控件无横向溢出，里程计设置可纵向滚动 | `/tmp/nav-migration/app-navigation-800x480.png` |
| 包清单及补丁 | 三个变更 package.xml 经 catkin_pkg 校验；按原换行格式检查补丁空白通过 | 本次本地检查 |

构建中发现并修复了本机 PCL 1.12 / 1.14 混合链接，以及旧 vikit_common 强制 native 指令集造成的 Eigen ABI 不一致。最终使用系统 PCL 1.12，FAST-LIVO2 的动态链接中不再出现 PCL 1.14；vikit 也采用配套编译设置。两套 PCL 并存的机器应显式设置上面的 `PCL_DIR`，不修改系统安装来掩盖问题。

FAST-LIVO2 集成用例使用固定 ROS 服务名。并行执行时曾发生故障后保存用例串扰；独立复核及整组 `ctest --test-dir /tmp/nav-migration/build/fast_livo --output-on-failure -j1` 通过。此类用例使用隔离 `ROS_DOMAIN_ID`、`ROS_LOCALHOST_ONLY=1` 且串行执行。

**全量 lint 并未全部通过。** Localizer 的 uncrustify 对来源代码报告格式差异，xmllint 因无法下载 `http://download.ros.org/schema/package_format3.xsd` 失败；未为此格式化整批算法文件或修改系统。其余四项 lint 当轮通过，变更包清单另经本地 catkin_pkg 解析校验通过。上述 60 项 C++ 功能测试不包含这六项 lint。

编译仍有旧第三方代码的警告；本轮功能验证不代表消除了全部历史警告。测试日志和产物为临时路径，清理 `/tmp` 后需按命令重建。

## 现场验收与后续批次

尚未开展实机：确认传感器与机器人 TF、时间、相机标定；回放同一段真实数据核对建图/定位；确认 `map → odom` 发布者唯一；在有底盘反馈的条件下验收目标提交、取消和物理停止；测量长时间内存、地图完整性、重定位与导航误差。

后续独立批次为地图包登记与切换、录包回放、集中日志和性能诊断。本轮未将这些能力标为已完成。
