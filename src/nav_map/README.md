# NAV maps

此目录保留随源码分发的地图示例。运行时生成的 PCD、YAML、PGM、SC 和地图包
默认放在 `~/nav_maps`，Jetson 上为 `/home/nvidia/nav_maps`。
默认文件名包含毫秒级时间戳，避免覆盖旧地图。

可通过 ROS 参数 `map_storage_dir` 或环境变量 `NAV_MAP_DIR` 修改默认位置。
