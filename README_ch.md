# WiFi室内定位 (wifi_loc)

这是一个基于WiFi信号强度(RSSI)进行室内定位的ROS2功能包。它通过分析接收到的WiFi AP信号，结合建筑物的OSM地图信息（包含房间边界和AP位置），估计机器人的位置（楼层和二维坐标）。

## 1. 所需依赖

### ROS2 依赖
*   `rclpy`: ROS2 Python客户端库
*   `std_msgs`: 标准ROS2消息类型
*   `geometry_msgs`: 几何相关的ROS2消息类型
*   `rss`: 自定义消息包，用于传输RSSI数据 (`RssData`) 和WiFi定位结果 (`WifiLocation`)。**注意：** 您需要确保 `rss` 功能包已编译。

### Python 库
*   `python3-shapely`: 用于处理几何对象（如多边形、点）
*   `numpy`: 用于数值计算
*   `matplotlib`: 用于结果可视化（可选）

### 构建工具
*   `ament_python`

您可以使用 `rosdep` 安装大部分依赖：
```bash
# 确保 rosdep 已初始化并更新
# sudo rosdep init
# rosdep update
rosdep install --from-paths src --ignore-src -r -y
# 可能需要手动安装 shapely
sudo apt-get update
sudo apt-get install -y python3-shapely
```

## 2. 与 localization_using_area_graph 功能包的关系

*   `wifi_loc` 接收原始的WiFi扫描数据（`/rss` 话题），进行处理和定位计算。
*   计算得到的WiFi定位结果（包含估计的x, y坐标和楼层信息）通过 `/WiFiLocation` 话题发布。
*   `localization_using_area_graph` 功能包订阅 `/WiFiLocation` 话题，将WiFi定位结果作为全局定位的初始猜测或参考信息，特别是用于救援模式(`bRescueRobot=true`)下的初始位置确定。

数据流： `/rss` -> `wifi_loc` -> `/WiFiLocation` -> `localization_using_area_graph`

## 3. 数据准备

*   **OSM地图文件 (.osm):**
    *   包含建筑物的楼层、房间边界（多边形）以及WiFi接入点(AP)的真实位置信息。
    *   默认路径：`wifi_loc/map/shanghaitech_d2_1_2F_3F.osm`。可以通过launch文件参数 `osm_file_path` 指定。
*   **Rosbag 文件 (.db3/.mcap):**
    *   必须包含 `/rss` 话题，消息类型为 `rss/msg/RssData`。该消息包含机器人扫描到的周围WiFi AP的MAC地址和对应的信号强度(RSSI)列表。
    *   可以通过launch文件参数 `bag_path` 指定rosbag的路径。
*   **(可选) 估计的AP位置文件 (estimated_positions.json):**
    *   如果不想使用OSM文件中的真实AP位置，可以提供一个JSON文件，包含估计的AP位置。
    *   默认路径：`wifi_loc/wifi_loc/data/estimated_positions.json`。
    *   通过launch文件参数 `use_true_ap_positions` 控制是否使用此文件（`false` 表示使用估计位置）。

## 4. 编译与运行

### 编译
```bash
# 确保 rss 功能包已编译 (如果它在同一工作空间)
# colcon build --packages-select rss
colcon build --symlink-install --packages-select wifi_loc
source install/setup.bash
```

### 运行
使用提供的launch文件启动节点并播放rosbag：
```bash
ros2 launch wifi_loc robot_loc_with_bag.launch.py bag_path:=<your_rosbag_directory_path> [use_true_ap_positions:=true/false] [use_sim_time:=true]
```
*   `bag_path`: 指定包含 `/rss` 话题的rosbag **目录** 路径。
*   `use_true_ap_positions`: (可选, 默认为 `true`) 设置为 `false` 时，使用 `estimated_positions.json` 中的估计AP位置；设置为 `true` 时，使用OSM文件中的真实AP位置。
*   `use_sim_time`: (可选, 默认为 `true`) 使用rosbag中的时间戳。

## 5. 输入与输出

### 输入
*   **ROS2 话题:**
    *   `/rss` (`rss/msg/RssData`): 订阅此话题以接收原始WiFi扫描数据。
*   **文件:**
    *   OSM地图文件 (路径由 `osm_file_path` 参数指定)。
    *   (可选) 估计的AP位置JSON文件 (当 `use_true_ap_positions` 为 `false` 时)。
*   **ROS2 参数:**
    *   `use_sim_time`: 是否使用仿真时间。
    *   `bag_path`: Rosbag目录路径 (主要用于日志记录和真值对比)。
    *   `osm_file_path`: OSM地图文件路径。
    *   `use_true_ap_positions`: 是否使用真实的AP位置。

### 输出
*   **ROS2 话题:**
    *   `/WiFiLocation` (`rss/msg/WifiLocation`): 发布基于WiFi计算得到的机器人位置估计（x坐标, y坐标, 楼层）。消息包含时间戳和用于定位的AP列表。该话题具有 `TRANSIENT_LOCAL` 的QoS设置，确保后来的订阅者能收到最后发布的消息。
*   **日志:**
    *   输出节点初始化、数据处理、楼层判断、定位计算等过程信息。
*   **(可选) 可视化:**
    *   代码中包含使用 `matplotlib` 进行可视化的部分，可能在特定条件下生成包含AP位置、房间边界和定位结果的图像。