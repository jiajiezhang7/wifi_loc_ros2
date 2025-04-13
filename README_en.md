# WiFi Indoor Localization (wifi_loc)

This is a ROS2 package for indoor localization based on WiFi signal strength (RSSI). It estimates the robot's location (floor and 2D coordinates) by analyzing received WiFi AP signals in conjunction with building OSM map information (including room boundaries and AP locations).

## 1. Dependencies

### ROS2 Dependencies
*   `rclpy`: ROS2 Python client library
*   `std_msgs`: Standard ROS2 message types
*   `geometry_msgs`: Geometry-related ROS2 message types
*   `rss`: Custom message package for transmitting RSSI data (`RssData`) and WiFi localization results (`WifiLocation`). **Note:** Ensure the `rss` package is compiled.

### Python Libraries
*   `python3-shapely`: For handling geometric objects (e.g., polygons, points)
*   `numpy`: For numerical computations
*   `matplotlib`: For result visualization (optional)

### Build Tools
*   `ament_python`

You can install most dependencies using `rosdep`:
```bash
# Ensure rosdep is initialized and updated
# sudo rosdep init
# rosdep update
rosdep install --from-paths src --ignore-src -r -y
# Manual installation of shapely might be required
sudo apt-get update
sudo apt-get install -y python3-shapely
```

## 2. Relationship with localization_using_area_graph Package

*   `wifi_loc` receives raw WiFi scan data (from the `/rss` topic) and performs processing and localization calculations.
*   The calculated WiFi localization result (including estimated x, y coordinates and floor information) is published via the `/WiFiLocation` topic.
*   The `localization_using_area_graph` package subscribes to the `/WiFiLocation` topic and uses the WiFi localization result as an initial guess or reference information for global localization, especially for initial position determination in rescue mode (`bRescueRobot=true`).

Data Flow: `/rss` -> `wifi_loc` -> `/WiFiLocation` -> `localization_using_area_graph`

## 3. Data Preparation

*   **OSM Map File (.osm):**
    *   Contains information about building floors, room boundaries (polygons), and the true locations of WiFi Access Points (APs).
    *   Default path: `wifi_loc/map/shanghaitech_d2_1_2F_3F.osm`. Can be specified via the `osm_file_path` launch file parameter.
*   **Rosbag File (.db3/.mcap):**
    *   Must contain the `/rss` topic with message type `rss/msg/RssData`. This message includes the MAC addresses and corresponding signal strengths (RSSI) of surrounding WiFi APs scanned by the robot.
    *   The path to the rosbag can be specified via the `bag_path` launch file parameter.
*   **(Optional) Estimated AP Positions File (estimated_positions.json):**
    *   If you prefer not to use the true AP positions from the OSM file, you can provide a JSON file containing estimated AP locations.
    *   Default path: `wifi_loc/wifi_loc/data/estimated_positions.json`.
    *   Use the `use_true_ap_positions` launch file parameter to control whether this file is used (`false` means use estimated positions).

## 4. Compilation and Execution

### Compilation
```bash
# Ensure the rss package is compiled (if in the same workspace)
# colcon build --packages-select rss
colcon build --symlink-install --packages-select wifi_loc
source install/setup.bash
```

### Execution
Use the provided launch file to start the node and play the rosbag:
```bash
ros2 launch wifi_loc robot_loc_with_bag.launch.py bag_path:=<your_rosbag_directory_path> [use_true_ap_positions:=true/false] [use_sim_time:=true]
```
*   `bag_path`: Specify the **directory** path containing the rosbag with the `/rss` topic.
*   `use_true_ap_positions`: (Optional, default: `true`) Set to `false` to use estimated AP positions from `estimated_positions.json`; set to `true` to use true AP positions from the OSM file.
*   `use_sim_time`: (Optional, default: `true`) Use timestamps from the rosbag.

## 5. Inputs and Outputs

### Inputs
*   **ROS2 Topics:**
    *   `/rss` (`rss/msg/RssData`): Subscribed to receive raw WiFi scan data.
*   **Files:**
    *   OSM map file (path specified by `osm_file_path` parameter).
    *   (Optional) Estimated AP positions JSON file (used when `use_true_ap_positions` is `false`).
*   **ROS2 Parameters:**
    *   `use_sim_time`: Whether to use simulation time.
    *   `bag_path`: Rosbag directory path (primarily for logging and ground truth comparison).
    *   `osm_file_path`: Path to the OSM map file.
    *   `use_true_ap_positions`: Whether to use true AP positions.

### Outputs
*   **ROS2 Topics:**
    *   `/WiFiLocation` (`rss/msg/WifiLocation`): Publishes the robot's estimated position (x coordinate, y coordinate, floor) based on WiFi calculations. The message includes a timestamp and the list of APs used for localization. This topic has a `TRANSIENT_LOCAL` QoS setting, ensuring late subscribers receive the last published message.
*   **Logs:**
    *   Outputs information about node initialization, data processing, floor determination, localization calculation, etc.
*   **(Optional) Visualization:**
    *   The code includes sections using `matplotlib` for visualization, which may generate images showing AP positions, room boundaries, and localization results under specific conditions.