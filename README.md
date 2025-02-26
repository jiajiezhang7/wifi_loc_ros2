# WiFi 室内定位系统

本项目提供了一个基于 WiFi 信号强度 (RSSI) 的室内定位系统，使用 ROS2 框架实现。系统由两个主要功能包组成：`rss` 和 `wifi_loc`。

## 功能包介绍

### rss

`rss` 功能包定义了系统中使用的自定义消息类型，用于 WiFi 信号数据的传输和位置信息的发布。

#### 消息类型

- **RssData**: 包含 WiFi 扫描的原始数据，包括时间戳、MAC 地址、频率和信号强度
- **RssDatum**: 单个 RSSI 数据点
- **WifiLocation**: 定位结果，包含经度、纬度和楼层信息

### wifi_loc

`wifi_loc` 功能包实现了 WiFi 定位的核心算法，提供了从 RSSI 数据计算位置的功能。

#### 节点

- **robot_loc**: 主要的定位节点，订阅 RSSI 数据，计算并发布位置信息

## 使用方法

### 先决条件

1. ROS2 环境（已测试于 ROS2 Iron）
2. Python 3.8+
3. 依赖库：numpy, shapely

### 构建

```bash
cd ~/wifi_ws
colcon build --packages-select rss wifi_loc
source install/setup.bash
```

### 运行

#### 使用 launch 文件启动（推荐）

```bash
ros2 launch wifi_loc robot_loc_with_bag.launch.py
```

这将启动 `robot_loc` 节点并播放预先录制的 rosbag 数据。

#### 手动启动

```bash
# 终端 1：启动 robot_loc 节点
ros2 run wifi_loc robot_loc

# 终端 2：播放 rosbag
ros2 bag play /home/jay/wifi_ws/rosbag/95 --clock
```

### 参数配置

`robot_loc` 节点支持以下参数：

- **osm_file_path**: OSM 地图文件路径（默认：包安装目录下的地图文件）
- **use_true_ap_positions**: 是否使用 AP 的真实位置（默认：true）

可以在启动时设置这些参数：

```bash
ros2 run wifi_loc robot_loc --ros-args -p use_true_ap_positions:=false
```

## 数据流

1. `/rss` 话题接收 WiFi 信号强度数据（RssData 类型）
2. `robot_loc` 节点处理这些数据，计算位置
3. 计算结果发布到 `/WifiLocation` 话题（WifiLocation 类型）

## 先验数据要求

系统需要以下先验数据才能正常工作：

1. **OSM 地图文件**: 包含建筑物结构和 AP 位置信息的 OpenStreetMap 格式文件
2. **AP 位置数据**: 可以在 OSM 文件中提供，或者通过估计算法生成
3. **RSSI 数据**: 通过 rosbag 提供，或者实时采集
4. **估计位置文件**: 如果使用估计的 AP 位置 (`use_true_ap_positions=false`)，需要在 `wifi_loc/data` 目录中包含 `estimated_positions.json` 文件，该文件包含 MAC 地址到估计位置的映射

## 工作流程

1. `robot_loc` 节点启动时加载 OSM 文件，获取 AP 位置信息
2. 节点订阅 `/rss` 话题，接收 RSSI 数据
3. 收集到 10 个数据点后，节点取消订阅并开始处理数据
4. 处理过程包括：
   - 计算每个 MAC 地址的平均 RSSI 值
   - 将 RSSI 值转换为距离
   - 使用三边测量法估计位置
   - 考虑建筑物结构约束优化结果
5. 计算结果以 WifiLocation 消息发布到 `/WifiLocation` 话题

## 注意事项

- 系统当前设计为收集 10 个数据点后进行一次性定位，不会持续更新位置
- 定位精度依赖于 AP 位置信息的准确性和 RSSI 到距离转换模型的精确度
- 在使用估计的 AP 位置时，定位精度可能会降低
- 如果设置 `use_true_ap_positions=false`，必须确保 `wifi_loc/data/estimated_positions.json` 文件存在且格式正确，否则节点将无法正常工作
- `estimated_positions.json` 文件应该包含 MAC 地址到位置坐标的映射，格式为：
  ```json
  {
    "xx:xx:xx:xx:xx:xx": [latitude, longitude, altitude],
    ...
  }
  ```

## 未来改进

- 实现连续定位功能
- 添加粒子滤波等高级定位算法
- 改进 RSSI 到距离的转换模型
- 添加可视化工具