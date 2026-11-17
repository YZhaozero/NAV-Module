# NAV Ubuntu 22.04 部署说明

本文档面向已经安装好 Ubuntu 22.04 和 ROS 2 Humble 的设备。工作空间可以放在任意路径，以下命令不要求用户名为 `zy`，也不要求目录固定为 `~/ws/NAV-Module`。

## 1. 部署结构

建议把项目和第三方依赖分开：

```text
任意位置/NAV-Module/       # 本项目源码及 build/install/log
~/nav_dependencies/
├── src/                     # Sophus、Livox-SDK2、TEASER++ 源码
├── build/                   # 上述原生库的构建目录
├── install/                 # 上述原生库的用户级安装前缀
├── ros2_ws/                 # Scan Context、FastGICP、NDT-OMP underlay
└── setup.bash               # 统一环境入口
```

安装脚本会用 `apt` 安装 Ubuntu/ROS 二进制包，并按项目要求添加 BorgLab 的 GTSAM 4.2 PPA。GTSAM 属于系统级安装；从源码构建的 Sophus、Livox-SDK2、TEASER++ 等库不会写入 `/usr/local`，也不会写入当前 NAV 工作空间。

部署脚本固定或指定的关键第三方依赖为：

| 依赖 | 固定版本 | 用途 |
| --- | --- | --- |
| Livox-SDK2 | `v1.2.5` | `livox_ros_driver2` 的硬依赖 |
| Sophus | `1.22.10` | FAST-LIVO2 和 vikit 的硬依赖 |
| TEASER++ | `e2f6f17` | `localizer` 全局点云配准的硬依赖 |
| GTSAM | BorgLab PPA `4.2` | `localizer` 位姿图、iSAM2 和 SC 生成工具的硬编译依赖 |
| Scan Context ROS 2 | 仓库内适配版本 | 外部 ROS underlay；定位数据库/兼容工具 |
| FastGICP、NDT-OMP | 仓库内适配版本 | 独立定位算法包，当前默认 Localizer 不直接链接它们 |

说明：当前 `localizer` 已在自身源码中实现 Scan Context 描述子、数据库加载和匹配，并未链接 `scancontext_ros2` 动态库。外部构建该包是为了保留完整定位工具链和后续数据库工具兼容。`localizer_node` 和 `generate_sc_tool` 都直接链接 GTSAM 与 TEASER++，因此即使运行时暂不启用位姿图，也必须在编译前安装 GTSAM。

若项目用于商业交付，还应单独复核各第三方项目许可证。仓库内 Scan Context README 标注了 CC BY-NC-SA 4.0 条款，不能只因为脚本能够编译就默认满足商业使用条件。

## 2. 一键安装依赖

在项目根目录执行：

```bash
./scripts/install_dependencies.sh
```

脚本会调用 `sudo apt-get`、`sudo add-apt-repository`、`rosdep` 和 `git`，因此需要管理员密码及可访问 Ubuntu、ROS、Launchpad 和 GitHub 的网络。默认使用全部 CPU 核心；Jetson 或内存较小的设备建议限制并行数：

```bash
./scripts/install_dependencies.sh --jobs 2
```

如需换依赖目录：

```bash
./scripts/install_dependencies.sh --deps-root "$HOME/robot_libs/nav"
export NAV_DEPS_ROOT="$HOME/robot_libs/nav"
```

后续只重编源码、不再运行 apt/rosdep：

```bash
./scripts/install_dependencies.sh --skip-apt --jobs 2
```

`--skip-apt` 会同时跳过 GTSAM PPA 配置和 GTSAM 安装，只能在已经成功安装 `libgtsam-dev`、`libgtsam-unstable-dev` 的机器上使用。

脚本可重复运行。它只同步 `~/nav_dependencies/ros2_ws/src` 中由本项目管理的三个包；如果指定目录已经存在但没有 `.nav_dependencies` 标记，脚本会停止，避免覆盖用户文件。

### 手动安装 GTSAM 4.2

如果不使用一键脚本，必须先按以下顺序安装 GTSAM：

```bash
sudo apt update
sudo apt install software-properties-common

sudo add-apt-repository ppa:borglab/gtsam-release-4.2
sudo apt update

sudo apt install libgtsam-dev libgtsam-unstable-dev
```

然后确认两个开发包的来源和版本：

```bash
apt-cache policy libgtsam-dev libgtsam-unstable-dev
dpkg-query -W 'libgtsam*'
```

候选版本应来自 `ppa.launchpadcontent.net/borglab/gtsam-release-4.2`。不要再额外安装 `ros-humble-gtsam`：项目的 `package.xml` 虽然声明了 `gtsam`，但一键脚本会让 `rosdep` 跳过该键，统一使用 BorgLab PPA 版本，避免两套 GTSAM 头文件、CMake 配置或动态库相互覆盖。

GTSAM 安装到系统标准路径，CMake 会直接通过 `find_package(GTSAM REQUIRED)` 找到它，不需要把 GTSAM 路径加入 `~/nav_dependencies/setup.bash`。

## 3. 一键编译工作空间

依赖部署完成后执行：

```bash
./build.sh
```

`build.sh` 会自动加载 ROS 2 Humble 和 `~/nav_dependencies/setup.bash`，并忽略工作空间内的 `fast_gicp`、`ndt_omp_ros2`、`scancontext_ros2` 副本，实际使用外部 underlay。编译产物仍位于本项目的 `build/`、`install/`、`log/`。

如果这个工作空间以前已经编译过上述三个包，`install/fast_gicp`、`install/ndt_omp_ros2`、`install/scancontext_ros2` 可能仍是旧产物。迁移到外部 underlay 时先把这三个精确目录移动到工作空间之外备份，再运行 `./build.sh`；不要删除整个 `install/`，除非确定不需要保留其他构建结果。

内存较小的设备可执行：

```bash
./build.sh --parallel-workers 2
```

需要刷新 CMake 缓存时：

```bash
./build.sh --cmake-clean-cache --parallel-workers 2
```

只有离线调试、明确希望在主工作空间构建三套随仓库提供的第三方 ROS 包时，才使用：

```bash
NAV_BUILD_BUNDLED_THIRDPARTY=1 ./build.sh
```

不要使用 `sudo colcon build`，否则构建产物会变成 root 所有，后续普通用户无法增量编译。

每个新终端运行程序前需要加载两层环境：

```bash
source "${NAV_DEPS_ROOT:-$HOME/nav_dependencies}/setup.bash"
source ./install/setup.bash
```

如果工作空间不在当前目录，把第二行换成该工作空间的绝对路径。不要把主工作空间 `install/setup.bash` 永久写死到 `~/.bashrc`，否则换分支或清理工作空间后容易污染下一次编译环境。

## 4. Livox MID-360 网络配置（每台机器必须核对）

仓库默认配置为：

- 上位机有线网卡：`192.168.1.100`；
- 雷达：`192.168.1.161`；
- 数据端口：`56100`～`56501` 范围内的配置端口。

这两个 IP 不是通用值。换工控机、Jetson、网卡或雷达后，需要同时设置主机网卡和 JSON，不能只改其中一个。

先给连接雷达的有线网卡配置静态 IPv4。假设接口名为 `enp3s0`、主机地址为 `192.168.1.100/24`：

```bash
ip -br address
sudo ip address add 192.168.1.100/24 dev enp3s0
sudo ip link set enp3s0 up
ping -c 3 192.168.1.161
```

`ip address add` 重启后会失效，量产部署请用 Ubuntu“网络”设置或 Netplan 固化，并避免给无线网卡和雷达网卡配置重叠子网。

不要直接把每台设备的 IP 提交回公共配置。建议复制一份机器专用配置：

```bash
mkdir -p "$HOME/.config/nav"
cp src/nav_driver/livox_ros_driver2/src/config/MID360_config.json \
  "$HOME/.config/nav/MID360_config.json"
vim "$HOME/.config/nav/MID360_config.json"
```

需要修改：

- `host_net_info` 下所有非空的 `*_ip`：连接雷达的本机网卡 IP；
- `lidar_configs[].ip`：雷达实际 IP；
- 多雷达时，每台雷达使用独立 `lidar_configs` 项并检查端口冲突；
- `extrinsic_parameter`：雷达安装姿态和位置。当前示例 `pitch` 为 `30.0` 度，不能默认认为适用于新车。

单独验证雷达：

```bash
ros2 launch livox_ros_driver2 msg_MID360_launch.py \
  user_config_path:="$HOME/.config/nav/MID360_config.json" \
  xfer_format:=4
ros2 topic hz /livox/lidar/pointcloud
ros2 topic hz /livox/imu
```

`xfer_format:=4` 发布 `sensor_msgs/msg/PointCloud2`，适合当前 DLIO、LaserScan 和默认导航链路；FAST-LIVO2 的 `mid360_live.launch.py` 使用 `CustomMsg` 时需要 `xfer_format:=1`。不要让同一个雷达驱动被界面和命令行重复启动。

若能 `ping` 通但没有数据，依次检查：JSON 中主机 IP、雷达 IP、网卡路由、防火墙和端口占用。首次联调可临时检查 `sudo ufw status`，正式设备应按安全策略放行 JSON 中使用的 UDP 端口，而不是长期关闭防火墙。

## 5. RealSense D435i

相机应连接 USB 3.x 端口。先验证系统是否识别：

```bash
lsusb | rg -i 'intel|realsense'
ros2 launch realsense2_camera d435i.launch.py
ros2 topic list | rg '^/camera/'
```

无权限或设备反复断开时，优先检查 librealsense udev 规则、USB 线材、供电和 USB 带宽。多相机部署时在界面或 launch 参数中填写序列号，不要仅依赖自动发现顺序。

## 6. 定位、地图、GTSAM 和 Scan Context 配置

`src/nav_localization/localizer/config/localizer.yaml` 当前仍在 `default_map_path` 中保存了开发机绝对路径。通过 NAV 界面启动定位时，所选 PCD 会通过 `map:=...` 覆盖它，所以推荐在“导航系统 → 地图”中选择实际 PCD。

命令行启动示例：

```bash
ros2 launch localizer localizer_launch.py \
  map:="/absolute/path/to/localization_map.pcd" \
  cloud_topic:=/dlio/odom_node/pointcloud/deskewed \
  odom_topic:=/dlio/odom_node/odom
```

如果需要保存本机专用的定位参数，复制 YAML 后修改 `default_map_path`、SC 阈值和采样分辨率，再显式传入配置：

```bash
mkdir -p "$HOME/.config/nav"
cp src/nav_localization/localizer/config/localizer.yaml \
  "$HOME/.config/nav/localizer.yaml"
vim "$HOME/.config/nav/localizer.yaml"

ros2 launch localizer localizer_launch.py \
  config_path:="$HOME/.config/nav/localizer.yaml" \
  map:="/absolute/path/to/localization_map.pcd"
```

当前版本不再从 `sc_db/poses.txt + data/*.sc` 目录加载定位数据库。启用 `use_scan_context: true` 后，Localizer 会使用与 PCD 同目录、同文件名追加 `.sc` 后缀的二进制缓存，例如：

```text
/data/maps/site_a.pcd
/data/maps/site_a.pcd.sc
```

首次加载 PCD 时，如果缓存不存在，Localizer 会根据 `sc_grid_resolution` 和 `sc_max_radius` 自动生成并写入；若缓存读取失败，也会重新生成。地图目录必须对运行 NAV 的普通用户可写，否则应提前离线生成缓存。仓库中的 `src/nav_localization/localizer/sc_db/` 属于上一版格式，当前默认加载路径不会使用它。

更新后的代码还提供了离线 SC 生成工具，输入 PCD 后会在同目录生成 `<地图文件>.pcd.sc`：

```bash
ros2 run localizer generate_sc_tool /absolute/path/to/localization_map.pcd
```

工具发现目标 `.sc` 已存在时会直接跳过。PCD 内容发生变化，或修改 `sc_grid_resolution`、`sc_max_radius` 等影响数据库结构的启动参数后，应先把旧 `.sc` 移走备份，再重新生成，避免地图与缓存不匹配。

`generate_sc_tool` 与 `localizer_node` 共用定位源码，因此也会链接 GTSAM。当前 GTSAM 位姿图管理器采用 iSAM2，并已进入编译链路；运行时位姿图开关目前默认为关闭，且优化结果尚未直接应用到 TF，但这不影响 GTSAM 已成为必装的编译和动态链接依赖。

定位还需注意：

- PCD 地图与实时点云必须使用一致的尺度、坐标约定和外参；
- `map_frame`、`local_frame`、DLIO 的 `odom`/点云话题必须一致；
- TEASER++ 全局配准依赖有效几何特征，空旷或重复结构场景需要重新调 `global_voxel_size`、`global_feature_radius`；
- `use_sim_time` 真机运行为 `false`，AR launch 和默认 YAML 均已使用系统时钟。回放或仿真时显式传入 `use_sim_time:=true`。

## 7. 首次启动建议顺序

不要第一次就直接启动整套系统。按下列顺序确认，能快速定位问题：

1. 启动 MID-360，确认 `/livox/lidar/pointcloud` 和 `/livox/imu` 有稳定频率；
2. 启动 D435i，确认彩色、深度和 IMU 话题；
3. 启动 DLIO，确认 `/dlio/odom_node/odom` 与去畸变点云；
4. 启动 Localizer，检查地图成功加载、TEASER++/Scan Context 日志和 TF；
5. 启动 Nav2，确认地图、TF、LaserScan、代价地图和 `/cmd_vel`；
6. 最后启动图形界面统一管理。

图形界面入口：

```bash
ros2 launch nav_app ar_nav.launch.py
```

远程 SSH 启动 GUI 时需要可用的 `DISPLAY`/X11 转发；无图形环境只适合分别启动后端节点。

## 8. 编译与运行自检

```bash
ros2 pkg prefix livox_ros_driver2
ros2 pkg prefix localizer
ros2 pkg prefix scancontext_ros2
ros2 pkg prefix fast_livo

dpkg-query -W 'libgtsam*'
ldd install/localizer/lib/localizer/localizer_node | rg 'not found' || true
ldd install/localizer/lib/localizer/generate_sc_tool | rg 'not found' || true
```

`livox_ros_driver2`、`localizer`、`fast_livo` 应指向当前项目 `install`，`scancontext_ros2` 应指向 `~/nav_dependencies/ros2_ws/install`。`dpkg-query` 应列出 `libgtsam-dev` 和 `libgtsam-unstable-dev`；两次 `ldd` 均不应输出 `not found`。如果提示找不到 `libteaser_registration.so` 或 Livox SDK 动态库，通常是当前终端漏掉了：

```bash
source "${NAV_DEPS_ROOT:-$HOME/nav_dependencies}/setup.bash"
source ./install/setup.bash
```

常见编译问题：

| 现象 | 处理 |
| --- | --- |
| `Could not find GTSAM` / `GTSAMConfig.cmake` | 按本文添加 BorgLab 4.2 PPA并安装两个 GTSAM 开发包，然后用 `--cmake-clean-cache` 重编 |
| `libgtsam.so` 或 `libgtsam_unstable.so` 找不到 | 用 `apt-cache policy` 检查 PPA 来源；若混装了 `ros-humble-gtsam`，先确认没有其他项目依赖它，再移除冲突版本、重装 PPA 包并运行 `sudo ldconfig` |
| `add-apt-repository: command not found` | 先安装 `software-properties-common` |
| `Could not find teaserpp` | 先运行依赖脚本，并确认 `CMAKE_PREFIX_PATH` 包含外部 `install` |
| `Could not find Sophus` | 同上；不要混用系统中其他不兼容 Sophus 版本 |
| `livox_lidar_api.h` 或 SDK 库找不到 | 检查外部 `install/include`、`install/lib`，重新加载 `setup.bash` |
| 构建进程被系统杀死 | 使用 `--parallel-workers 1` 或 `2`，Jetson 同时确认 swap/内存 |
| 编译时出现旧工作空间路径 | 新终端重新执行 `./build.sh --cmake-clean-cache`，不要 source 旧工作空间 |
| 真机节点全部等待时间 | 关闭 `use_sim_time`，并确认没有等待不存在的 `/clock` |

## 9. 部署检查清单

- [ ] Ubuntu 22.04、ROS 2 Humble 架构与目标设备一致；
- [ ] BorgLab GTSAM 4.2 PPA 已配置，两个 GTSAM 开发包已安装且未混装 `ros-humble-gtsam`；
- [ ] `~/nav_dependencies/setup.bash` 存在且可加载；
- [ ] 主机网卡 IP 与 MID-360 JSON 完全一致；
- [ ] 雷达 IP、端口、安装外参已按本车修改；
- [ ] 真机使用 `PointCloud2` 的链路已设 `xfer_format:=4`；
- [ ] RealSense USB、权限、序列号已验证；
- [ ] PCD 定位地图和 Nav2 YAML/PGM 栅格地图没有混用；
- [ ] Localizer 的 `<定位地图>.pcd.sc` 与当前 PCD/SC 参数匹配，或已明确关闭 `use_scan_context`；
- [ ] 真机 `use_sim_time=false`；
- [ ] TF 树包含 `map -> odom -> base_link` 及传感器外参；
- [ ] `/cmd_vel` 接口、底盘控制和急停策略已单独验证后再联调整套导航。

## 10. NVIDIA 169.254.128.129 部署（2026-11-13）

目标：`nvidia@169.254.128.129`，Jetson L4T R36.4.4，aarch64，Ubuntu 22.04.5，ROS 2 Humble。

该设备使用 ROS 仓库提供的 ARM64 `ros-humble-gtsam` 4.2.0、`ros-humble-sophus` 和 `ros-humble-librealsense2`，与前文通用脚本的 BorgLab PPA 路线二选一，不混装两套 GTSAM。Livox-SDK2 `v1.2.5` 和 TEASER++ `e2f6f17` 在用户目录原生编译。

部署目录：

```text
/home/nvidia/nav-releases/20261113-nav-module/  # 当前源码和 ARM64 构建产物
/home/nvidia/nav-current                      # 服务使用的版本链接
/home/nvidia/nav_dependencies/                # 用户级原生依赖
/home/nvidia/.config/nav/                     # 跨版本保留的设备设置
```

管理员准备脚本为 `deployment/jetson-system-setup.sh`，只适用于此设备的 `nvidia` 用户。它安装缺失的二进制依赖，并将 `nvidia` 加入 `dialout`。`ttyTHS1/2` 原有权限为 `root:dialout 0660`；脚本使用 ACL 让已经登录的桌面会话立即获得读写权限，重新登录或重启后由持久的 `dialout` 组成员关系授权。

串口验证已在普通 `nvidia` 用户下完成：分别使用 `O_RDWR | O_NOCTTY | O_NONBLOCK` 打开并立即关闭 `ttyTHS1` 和 `ttyTHS2`，未发送数据、未修改波特率或串口参数。该检查只证明访问权限，不证明 RTK 接收数据正常。

设备手动启动（先停止用户服务，避免重复实例）：

```bash
systemctl --user stop nav-app.service
~/nav-current/scripts/start_device_app.sh
```

设备服务和该入口默认使用 `ROS_DOMAIN_ID=101`；需要与独立启动的底盘或传感器节点通信时，这些节点也要使用同一个域。

桌面启动和状态检查：

```bash
systemctl --user start nav-app.service
systemctl --user status nav-app.service
journalctl --user -u nav-app.service -n 80 --no-pager
```

本次为首次安装，没有可回滚的旧 NAV release。后续升级由 `scripts/device_service.py install` 记录前一版本，可使用 `rollback` 恢复。默认启用图形会话登录后的自动启动；是否免登录进入桌面仍取决于设备自己的登录配置。

本次实际验收结果：

- 目标机冷构建：18 包完成，22 分 18 秒，退出码 0；
- 目标机 RTK 解析、串口发现、启动脚本、户外转换与图像缓冲：131 项回归通过；
- Localizer、SC 工具、TEASER 工作进程、FAST-LIVO、DLIO、Livox 驱动的动态库检查无缺失；
- Qt、RTK、FAST-LIVO SaveMap 和 DLIO SavePCD 安装模块均可导入；
- 物理 X11 显示器 `:1` 已显示 `NAV 导航控制台`，日志出现 `Qt navigation ready`；
- `nav-app.service` 已启用，状态 `active/running`，验收时 `NRestarts=0`；
- `/nav_qt_nav` 的 `use_sim_time` 实际查询为 `False`；
- 手机入口 `http://169.254.128.129:8765/` 返回 HTTP 200，配对码在设备“工具 → 手机遥控”页显示；
- 普通用户 `nvidia` 打开并关闭 `ttyTHS1/2` 再次成功；
- 本机与目标 release 源码文件已按校验和核对，Nav2 本体源码未上传。

该设备目前的用户 journal 没有条目，应用日志可在 `/home/nvidia/.ros/log/` 下查看。部署完成时 Qt 日志为 `python3_17329_1794539634540.log`；下次启动会生成新文件。

本次未执行真实雷达建图、相机标定、RTK 接收精度或底盘运动验收。

## 11. 显示与 RTK 搜索修复（2026-11-13）

设备实际输出为 DP-1 `1920×1080 @ 60 Hz`，Qt 逻辑尺寸同为 `1920×1080`，缩放比为 `1.0`，逻辑 DPI 为 `96`；普通桌面可用区域为 `1850×1053`。本轮按这些实测尺寸适配，没有调整系统输出分辨率。

- 原程序固定以 `1500×900` 窗口启动，自动全屏只对短边小于 700 像素生效；新增 `fullscreen` launch 参数，设备入口显式使用 `fullscreen:=true`，实测窗口覆盖 `1920×1080+0+0`。
- 主页增加“全屏/退出全屏”按钮，所有页面均可使用 `F11` 切换、`Esc` 返回窗口模式；原有主题、页面布局及小屏模式保留。
- RTK 页创建时自动列出可用串口，显示结果数量与路径；刷新保留仍存在的选项，设备移除后清除选择。默认不选择或打开设备，需要用户按接线选择后点击“连接”。
- 串口选择框默认使用下拉选择，避免触摸时弹出系统键盘遮挡诊断页；需要输入其他绝对路径时，显式勾选“手动输入”。目标机实际发现 `/dev/ttyTHS1` 和 `/dev/ttyTHS2`。
- GDM 已由设备管理员启用 `AutomaticLoginEnable=true`、`AutomaticLogin=nvidia`；`desktop_login_ready` 实际检查为 `true`，用户服务保持启用。没有为验证而重启设备。
- 按用户选择，`nvidia` 桌面的空闲延时设为 `0`，自动锁屏及变暗关闭，交流/电池自动休眠均设为 `nothing`；当前 X11 屏保和 DPMS 关闭。原设置备份于 `~/.local/state/nav/updates/display-power-before-20261113/`。

验证：本机与 Jetson 各 34 项界面/串口回归通过；另有 3 项原有小屏布局回归通过。设备真实 X11 屏幕上的主页、建图、导航、传感器、地图和 RTK 六页均为 `1920×1080`，RTK 底部状态可见，F11/Esc 切换通过。验收结果和截图位于 `/tmp/nav-display-review-an0i7llz/`，不连接硬件的显示测试窗口已经关闭。

后续实际运行截图也确认 `/dev/ttyTHS1 @ 230400` 已接收 WTRTK 报文；此观察只确认串口通信，未验证 GNSS 定位精度。

修改前的界面文件备份为 `~/.local/state/nav/updates/before-display-20261113-113421.tar.gz`。

## 12. 本机代码增量更新（2026-11-13）

将本机新增的轨迹地图页面、网页地图组件、地图私有配置加载器和百度地图检查工具同步至设备的现有 release；本轮未修改算法实现。`src/nav_map`、设备标定、录包和用户运行设置保持原样。

- 更新前代码备份：`~/.local/state/nav/updates/before-code-update-20261113-maps.tar.gz`；
- 本次代码暂存：`~/.local/state/nav/updates/nav-code-update-20261113-maps/`；
- 设备管理员安装新增依赖 `python3-pyqt5.qtwebengine`；
- 小车原先没有 `~/.config/nav/maps.env`，本次同步本机私有配置，权限保持 `600`；已有设备配置不覆盖，密钥不写入源码或更新日志；
- 本机与 Jetson 各 46 项地图配置、轨迹页面与工具页回归通过；
- Jetson 默认 18 包增量构建通过，耗时 13.8 秒；
- QtWebEngine 在设备实际图形会话中加载内嵌 HTML 与 `NAVBaidu` / `NAVTrajectory` 脚本通过，测试未提交 GNSS 位置或调用在线地图 API；
- 当前图形会话由设备启动脚本识别为 `:0`，NAV 实际窗口为 `1920×1080+0+0`，运行参数 `fullscreen=True`；
- 服务状态 `active/running`，验收时 `NRestarts=0`，手机 HTTP 入口返回 200；
- 自动登录检查继续为 `true`，空闲延时仍为 `0`、自动锁屏仍为 `false`。

本次没有执行真实建图或底盘导航运动。地图私有配置通过离线格式检查，在线 API 授权及真实地图位置显示不在本次渲染组件验收范围内。
