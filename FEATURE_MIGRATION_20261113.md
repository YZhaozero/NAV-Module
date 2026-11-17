# 建图导航功能补齐移植说明

日期：2026-11-13  
目标仓库：`/home/zy/ws/NAV-Module`  
参考快照：`/home/zy/ws/GITREPO/gs-nav-monorepo` 的 `aafb462ea8f7875205da888946d05c3d31bc8477`

## 范围和成功标准

本批只迁移机器人建图、定位、导航、RTK、运维和手机控制能力，不迁移三维重建、训练、模型查看或模型任务接口。保持当前 `src/nav_app`、`src/nav_rtk`、`src/nav_slam` 等目录层级，不重组工作空间。

完成标准：

1. ROS/Python 包、节点、话题、环境变量和新数据路径统一使用 NAV 命名；
2. Qt 保持现有布局与主题，新入口继续使用公共页头和按钮样式；
3. 路线只能在 Nav2 生命周期、定位质量和运动许可均满足时执行；
4. 后台地图任务、录包、回放和诊断互斥关系可见，退出时等待持久化操作；
5. 根目录 `colcon build` 成功，功能回归按模块通过；
6. 合成测试不冒充真实机器人、真实 RTK 或真实相机验收。

## 已迁移内容

### 定位、导航与运动安全

- 两点人工重定位、全局重定位、进度、取消、超时和诊断导出；
- Localizer 粗/精匹配阈值、分辨率和迭代次数的高级设置；
- Nav2 lifecycle readiness，规划预览和执行分离，取消结果需要确认；
- `motion_guard` 与 `ros2_base_adapter`，使用 `/nav/*` 话题；底盘适配器默认不自动启动；
- 步行引导和机器人执行共用路线预览，机器人模式仍需运动许可；
- AR 图像按消息时间等待对应 TF，使用有容量和字节上限的显示缓冲；Qt 每轮 ROS 事件处理同时限制回调数和时间预算；
- FAST-LIVO 关键进程退出即关闭 launch，不自动反复拉起。

### 地图、录包和后台任务

- 地图包登记、生成、核对、切换、删除和 SC 索引；
- 持久化地图任务队列、进度、取消、重试和前台算法资源租约；
- rosbag 录制、回放、审计、录包库、JPEG 图像归档和诊断导出；
- 2D 地图缩放、平移、跟随朝向和运动轨迹；3D 点云保留既有交互；
- FAST-LIVO 中断检查点和恢复入口。

### RTK、地理路线和手机端

- `src/nav_rtk` 独立包负责串口桥接、协议解析、NTRIP、GNSS 监测和时间新鲜度判断；
- RTK 轨迹录制、冻结地理对准、GNSS/里程计/定位对比、回放和 ZIP 导出；
- RTK 轨迹页可切换百度在线底图与本地地图对比，加载地图包的栅格/点云，并在历史回看中恢复随记录保存的地图快照；
- 户外状态机、地理地图、GNSS/LIO 连续性、室外/室内分段路线、路口辅助；
- 手机配对、Qt 画面遥控、室外地图和路线页面；默认关闭监听，通过参数或“工具 → 手机遥控”开启；
- 手机端已删除模型任务、模型查看和关键帧建模入口。

Scan Context 缓存文件头仍识别 `GSSCC002`。这是已有 `.sc` 文件的二进制格式版本，不是包名、节点名或界面品牌；保留该标识以兼容已经生成的地图缓存。

手机服务参数：

```bash
ros2 launch nav_app ar_nav.launch.py \
  phone_remote_enabled:=true \
  phone_remote_bind:=0.0.0.0 \
  phone_remote_port:=8765
```

百度地图私有配置默认从 `~/.config/nav/maps.env` 读取，模板为 `deployment/baidu/maps.env.example`。直连使用 `NAV_BAIDU_BROWSER_AK`、`NAV_BAIDU_SERVER_AK` 和可选的 `NAV_BAIDU_SERVER_SK`；网关模式使用 `NAV_MAP_GATEWAY` 与 `NAV_MAP_TOKEN`。未配置网络地图服务时，本地建图和室内 Nav2 不受影响。

### 设备启动和回滚

- `scripts/runtime_environment.sh`：只 source ROS Humble、可选依赖 underlay 和当前工作空间；
- `scripts/start_device_app.sh`：寻找可用 X11 会话并启动 NAV Qt，默认开启手机入口；
- `scripts/device_service.py`：检查、安装和回滚 `nav-app.service`，维护 `~/nav-current`；
- `scripts/desktop_autologin.py`：可选的一次性 GDM 自动登录设置和恢复；
- `deployment/nav-app.service`：用户级图形会话服务模板。

安装或回滚会切换用户目录中的 release 链接并重启用户服务，不能在开发机上把“检查脚本通过”当作已经部署。

## 主要接口

| 用途 | 接口 |
|---|---|
| 规划预览 | `/compute_path_through_poses` |
| 机器人执行 | `/navigate_through_poses` |
| 规划速度输入 | `/nav/planner_cmd_vel` |
| 运动许可/状态 | `/nav/motion_permit`、`/nav/motion_status` |
| 底盘请求/反馈 | `/nav/base_command`、`/nav/base_feedback` |
| RTK 原始观测 | `/rtk/observation` |
| 户外定位状态 | `/nav/outdoor/localization` |
| 地理导航状态 | `/nav/navigation/localization` |
| 手机绝对定位 | `/nav/phone_absolute_solution` |

## 验证结果

- 根目录 `colcon build`：18 包成功，退出码 0；
- 户外连续导航、地理地图、手机定位、路线辅助和 HTTP 边界：392 项通过；
- Qt 地图、工具页、手机遥控、跟随显示和启动配置专项：101 项通过；
- 手机绝对定位与连续性：84 项通过；
- 设备启动、桌面入口和回滚配置：11 项通过；
- 传感器 launch 进程组停止测试单独通过。

一次把全部测试放入同一 pytest 进程时，旧 Qt 用例创建的窗口和非守护后台协调器累积，进程在后半段提前退出且没有生成失败报告。因此本批按功能分组运行，不宣称单进程全量测试通过。系统 Python 可用；用户目录 NumPy 2 与 Ubuntu 22.04 系统 SciPy ABI 不兼容，源码测试应设置 `PYTHONNOUSERSITE=1`。

## 未完成的实机验收

- 未连接真实底盘执行运动，也未验证急停、速度限制和物理停止距离；
- 未验证真实 MID360、相机外参、图像时间同步和 AR 地面重合精度；
- 未连接真实 RTK 接收机、NTRIP 服务或手机 GNSS 桥，未给出定位精度结论；
- 未在目标设备安装或切换 `nav-app.service`，只验证脚本逻辑和 launch 解析；
- 百度路线需要设备自身合法的 API 配置和联网条件。

现场验收前先核对 TF 唯一发布者、传感器时间、地图坐标、底盘反馈和运动许可；不得因为界面显示“取消完成”就推断物理机器人已经停止。
