#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from wifi_loc.utils.Xmlparser import OsmDataParser
from wifi_loc.utils.read_pickle import RssData as PickleRssData, RssDatum
from rss.msg import RssData, WifiLocation
from collections import Counter
import numpy as np
from wifi_loc.utils.util import rssi_to_distance, load_estimated_positions
from wifi_loc.utils.opter import PointEstimator
from shapely.geometry import Polygon, Point, LineString
import os
from ament_index_python.packages import get_package_share_directory

class RobotLocalizer(Node):
    def __init__(self):
        super().__init__('robot_loc')  # ROS2中需要先调用父类的初始化
        
        # 创建订阅者（ROS2风格）
        self.rss_subscription = self.create_subscription(
            RssData,
            'rss',
            self.callback_rss,
            10  # QoS profile depth
        )
        
        # 创建位置发布者
        self.location_publisher = self.create_publisher(
            WifiLocation,
            'WifiLocation',
            10  # QoS profile depth
        )
        
        # 初始化数据
        self.raw_rss = []
        
        # 获取包路径并设置默认OSM文件路径
        package_share_dir = get_package_share_directory('wifi_loc')
        default_osm_path = os.path.join(package_share_dir, 'map', 'shanghaitech_d2_1_2F_3F.osm')
        
        # OSM文件路径
        self.declare_parameter('osm_file_path', default_osm_path)
        self.osm_file_path = self.get_parameter('osm_file_path').get_parameter_value().string_value
        
        # 控制是否使用AP真实位置的参数
        self.declare_parameter('use_true_ap_positions', True)
        self.use_true_ap_positions = self.get_parameter('use_true_ap_positions').get_parameter_value().bool_value
        
        # 创建OSM解析器实例
        self.parser = OsmDataParser(self.osm_file_path)
        self.parser.parse()
        
        # 获取解析后的数据
        self.ap_to_position, self.ap_level, self.target_ap, self.way_data, self.all_mac = self.parser.get_data()
        
        self.polygons = []
        for way, way_level in self.way_data:
            if len(way) > 2 and Polygon(way).is_valid:
                poly = Polygon(way)
                self.polygons.append((poly, way_level))
                
        # 加载估计的AP位置(从json中读取)
        self.estimated_AP_positions = load_estimated_positions()
        
        # 创建logger
        self.get_logger().info('Robot localizer node has been initialized')

    def callback_rss(self, msg):
        """
        ROS2风格的回调函数，处理接收到的RSS数据
        """
        try:
            if len(msg.data) > 0:
                self.raw_rss.append(msg)
                self.get_logger().info(f'Received RSS data, current data points: {len(self.raw_rss)}')
                
                # 当收集到10个数据点时
                if len(self.raw_rss) >= 10:
                    collected_data = self.raw_rss.copy()
                    # 不再销毁订阅者，而是取消订阅
                    self.destroy_subscription(self.rss_subscription)
                    self.rss_subscription = None  # 将引用设为 None
                    self.process_rss_data(collected_data)
                    
        except Exception as e:
            self.get_logger().error(f'Error in callback_rss: {str(e)}')

    def process_rss_data(self, rss_data):
        """处理收集到的RSS数据"""
        # 创建字典存储每个MAC地址对应的RSSI值列表
        mac_rssi_dict = {}
        
        # 遍历所有收集的数据
        for rss_data in self.raw_rss:
            for i, mac in enumerate(rss_data.mac_address):
                if mac not in mac_rssi_dict:
                    mac_rssi_dict[mac] = []
                # 将该MAC地址的RSSI值添加到列表中
                mac_rssi_dict[mac].extend(rss_data.data[i].rss)
        
        # 计算每个MAC地址的RSSI平均值
        mac_avg_rssi = {}
        for mac, rssi_list in mac_rssi_dict.items():
            mac_avg_rssi[mac] = sum(rssi_list) / len(rssi_list)
            self.get_logger().info(f'MAC: {mac}, Average RSSI: {mac_avg_rssi[mac]:.2f}')
        
        # 将 mac_avg_rssi 按 RSSI 值降序排序并取前 50 项
        mac_avg_rssi = dict(sorted(mac_avg_rssi.items(), key=lambda x: x[1], reverse=True)[:50])
        
        self.get_logger().info(f'Top MAC addresses and their average RSSI values: {mac_avg_rssi}')
        
        positions = []
        rssis = []
        
        # 这是AP的真值 - 用它定位会很准
        self.get_logger().info(f'Known AP positions count: {len(self.ap_to_position)}')
        # 这是AP的估计值 - 用它定位会一般
        self.get_logger().info(f'Estimated AP positions count: {len(self.estimated_AP_positions)}')
        self.get_logger().info(f'Using {"true" if self.use_true_ap_positions else "estimated"} AP positions for calculation')
        
        for mac, avg_rssi in mac_avg_rssi.items():
            if self.use_true_ap_positions and mac in self.ap_to_position:
                self.get_logger().info(f'MAC address {mac} found in known list (using true position)')
                positions.append([self.ap_to_position[mac][0], self.ap_to_position[mac][1], 2])
                rssis.append(avg_rssi)
            elif not self.use_true_ap_positions and mac in self.estimated_AP_positions:
                self.get_logger().info(f'MAC address {mac} found in estimated list (using estimated position)')
                positions.append([self.estimated_AP_positions[mac][0], self.estimated_AP_positions[mac][1], 2])
                rssis.append(avg_rssi)
            else:
                self.get_logger().warn(f'MAC address {mac} not found in {"known" if self.use_true_ap_positions else "estimated"} list')
        
        if len(positions) >= 3:
            # 计算初始猜测位置（使用已知AP位置的平均值）
            initial_lat = np.mean([pos[0] for pos in positions])
            initial_lon = np.mean([pos[1] for pos in positions])
            initial_alt = np.mean([pos[2] for pos in positions])
            
            initial_guess = [initial_lat, initial_lon, initial_alt]
            
            # 将RSSI值转换为距离
            distances = np.array([rssi_to_distance(rssi, A=-38.85890085025037, n=2.221716321548527) for rssi in rssis])
            
            # 创建PointEstimator实例并估计位置
            estimator = PointEstimator(positions, distances, self.polygons)
            result = estimator.estimate_point(initial_guess=initial_guess)
            
            if result is not None:
                self.get_logger().info(f'Estimated position: {result.x}')
                
                # 创建并发布 WifiLocation 消息
                location_msg = WifiLocation()
                location_msg.latitude = float(result.x[0])   # 纬度
                location_msg.longitude = float(result.x[1])  # 经度
                location_msg.floor = int(round(result.x[2])) # 楼层
                
                # 发布位置消息
                self.location_publisher.publish(location_msg)
                self.get_logger().info('Published location to /WifiLocation topic')
            else:
                self.get_logger().error('Position estimation failed')

def main(args=None):
    rclpy.init(args=args)
    
    robot_localizer = RobotLocalizer()
    
    # 添加启动消息
    robot_localizer.get_logger().info("wifi_loc start work...")
    
    try:
        rclpy.spin(robot_localizer)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        robot_localizer.get_logger().error(f"意外错误: {e}")
    finally:
        # 确保在销毁节点前清理所有订阅
        try:
            robot_localizer.destroy_node()
        except Exception as e:
            print(f"Error during node destruction: {e}")
        
        # 只在rclpy还在运行时执行关闭操作
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception as e:
            print(f"Error during rclpy shutdown: {e}")

if __name__ == '__main__':
    main()
