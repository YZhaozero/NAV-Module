# grid_map_pcl 局部地形对照版

用于现有 PCD 的离线效果评估。调用官方 `grid_map_pcl::GridMapPclLoader` 提取每个 XY 单元最低点簇的平均高程，再检查局部坡度、粗糙度、台阶连续性，导出 Nav2 的 YAML/PGM。
官方算法：[ANYbotics/grid_map](https://github.com/ANYbotics/grid_map/tree/humble/grid_map_pcl)。

PCD 的 XYZ 不做旋转或平移；高程图通过每格实际 XY 坐标导出，避免矩阵轴方向和地图原点错位。工具只读取源 PCD，离线入口拒绝覆盖已存在的对照输出。NAV 现已提供两种可选栅格方法；SC 仍按原流程处理。

## NAV 界面使用

- **建图页**的“结束后栅格方法”或**建图参数页**的“栅格方法”可选择“原有地面投影”与“局部高程通行性（grid_map_pcl）”。两处选择同步，初始保留原有方法。
- 点击“保存并结束”时固定本轮选择；PCD 保存成功、建图进程退出后，后台自动生成该方法的栅格和原有 SC。处理中再切换选择只影响下一轮。
- 自动任务与地图包标题标注“原有投影”或“地形分析”，任务和检查点按方法区分，避免对比时复用错栅格。
- **地图处理页**有“转换栅格 · 原有投影”与“转换栅格 · 局部高程通行性”两个按钮。旧方法保留原参数；新方法分析完整编辑后点云并自动确定地面参考，使用当前转换坐标和栅格分辨率。它不使用旧方法的全局高度切片。
- 新方法后台计算，进度和错误显示在转换按钮旁；计算失败或期间修改了地图，不替换现有编辑结果。转换完成后仍需点击“保存当前地图”。
- 两种自动生成方法均继续调用原 `scan_context()`，使用同一输入 PCD 对应的 `.pcd.sc.v2` 缓存；切换栅格方法不改变 SC 算法、参数和缓存校验规则。

## 构建

系统有 `ros-humble-grid-map-pcl` 时，直接使用 ROS 前缀即可。设备无 sudo 时，本次将官方 Debian 包解压到独立目录，未安装/升级系统软件：

```bash
nav_grid_deps="$HOME/nav_dependencies/grid_map"
mkdir -p "$nav_grid_deps/debs"
cd "$nav_grid_deps/debs"
apt-get download ros-humble-filters ros-humble-grid-map-core ros-humble-grid-map-cv ros-humble-grid-map-msgs ros-humble-grid-map-ros ros-humble-grid-map-pcl
for nav_deb in *.deb; do dpkg-deb -x "$nav_deb" "$nav_grid_deps"; done
```

在 NAV-Module 根目录运行：

```bash
source /opt/ros/humble/setup.bash
export NAV_GRID_PREFIX="$HOME/nav_dependencies/grid_map/opt/ros/humble"
cmake -S tools/grid_map_preview -B build/nav_grid_map_preview   -DCMAKE_BUILD_TYPE=Release   -DCMAKE_PREFIX_PATH="$NAV_GRID_PREFIX;/opt/ros/humble"   -DPCL_DIR="/usr/lib/$(dpkg-architecture -qDEB_HOST_MULTIARCH)/cmake/pcl"   -DPython3_EXECUTABLE=/usr/bin/python3
cmake --build build/nav_grid_map_preview -j1
```

显式使用 Ubuntu 的 PCL，避免本机 `/usr/local` 中其他版本与官方 ROS 库混用。

## 生成对照

```bash
source /opt/ros/humble/setup.bash
export NAV_GRID_PREFIX="$HOME/nav_dependencies/grid_map/opt/ros/humble"
export LD_LIBRARY_PATH="$NAV_GRID_PREFIX/lib:$LD_LIBRARY_PATH"
export PYTHONPATH="$PWD/src/nav_app:$PYTHONPATH"
export OPENBLAS_NUM_THREADS=1 PYTHONNOUSERSITE=1
/usr/bin/python3 -s tools/grid_map_preview/preview.py   /home/nvidia/nav_maps/nav_map_20261114_140548_637.pcd   /home/nvidia/nav_maps/916-grid-map-preview.yaml   --extractor build/nav_grid_map_preview/nav_grid_map_pcl_export
```

输出：`.yaml`、`.pgm`、`.analysis.npz` 和 `.analysis.json`。在 NAV“地图处理 → 加载 YAML/PGM”中打开新 YAML 查看。对照图未登记为正式导航地图包。

## 当前几何假设

- 高程单元 20 cm，输出栅格 5 cm；同一高程单元内的白色格代表局部地面面片，精度不等于 5 cm 的独立观测。
- 稀疏点簇可由一个实测点形成，但需通过邻域平面和地面连续性检查。参考地面高度仅用于启动地面连通分量，不是全图的薄高度切片；可用 `--ground-z` 指定参考。
- 单楼层：参考高度以上 1.5 m 的高程单元不作为该楼层地面锚点，避免天花板参与地面插值。
- 默认坡度上限 20°、相邻地面高度变化上限 10 cm、平面残差上限 4 cm。这些是本版效果评估参数，尚未按具体底盘验证。
- 只在实测高程组成的短三角形内插值，默认三角形最长边不超过 45 cm，不向大面积未知区域扩展。NPZ 中区分 `measured` 和 `interpolated`。
- 局部地面以上 12 cm～1.5 m 的实测点投影为黑色；缺少邻近地面时，当前楼层内有足够竖直跨度的实测结构也保留为障碍。高处回波本身不证明下方地面可通行。
- JSON 记录源 PCD 哈希、全部参数、计数和运行时间；`field_validated=false` 表示未做实机通行验收。

验收重点是指定通道的连通情况、实际墙体保留和坐标重合，不能只比较白色像素数。只有合并 PCD 时，缺地面且缺原始观测位姿的通道可能继续是灰色。

## 2026-11-16 首版验证

- 设备 PCD：`/home/nvidia/nav_maps/nav_map_20261114_140548_637.pcd`，85,903 点；SHA256 为 `31ce0672d322104f67c50b437139364f084995223cb4436e5f2b9a94e5836de8`。
- 已生成 `/home/nvidia/nav_maps/916-gridmap-v1-20261116.yaml` 及对应 PGM、NPZ、JSON、对照 PNG。输出 372×344 格、5 cm/格；白色 3,505 格、黑色 3,788 格、灰色 120,675 格。设备耗时约 1.1 秒。
- 本机和设备分别通过 `test_terrain_grid.py` 与 `test_map_processing.py` 共 20 项测试；设备独立临时 Qt 窗口已验证“加载 YAML/PGM”按钮与三态渲染。未重启当前 NAV、未控制底盘。
- 原始 PCD 和正式地图 YAML 的哈希保持不变；新增文件不是正式导航地图包。
- 同世界坐标对比已另存 `916-gridmap-v1-20261116-comparison.png`；图中 A/B 是用户截图箭头的近似位置。两处仍有未知区域，尚未通过通道完全连通的效果验收。
- 按截图定位的近似圆形区域（半径 35 cm）读取原始点云：上方主要为约 2.66～3.73 m 高处回波，下方少量地面回波与高处回波混合；这些计数不是精确人工标定，不能据此断言现场不可通行。缺少地面和每帧观测位姿的空白不能只靠此版高程图确认为自由空间。
