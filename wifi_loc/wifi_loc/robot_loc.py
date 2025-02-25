#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from wifi_loc.utils.Xmlparser import OsmDataParser
from wifi_loc.utils.read_pickle import RssData, RssDatum
from wifi_loc_interfaces.msg import RssData4Ros
from collections import Counter
import numpy as np
from wifi_loc.utils.util import rssi_to_distance, load_estimated_positions
from wifi_loc.utils.opter import PointEstimator
from shapely.geometry import Polygon, Point, LineString

class RobotLocalizer(Node):
    def __init__(self):
        super().__init__('robot_loc')  # ROS2中需要先调用父类的初始化
        
        # 创建订阅者（ROS2风格）
        self.rss_subscription = self.create_subscription(
            RssData4Ros,
            'rss',
            self.callback_rss,
            10  # QoS profile depth
        )
        
        # 初始化数据
        self.raw_rss = []
        
        # OSM文件路径
        self.declare_parameter('osm_file_path', '/home/jay/wifi_ws/src/wifi_loc/map/shanghaitech_d2_1_2F_3F.osm')
        self.osm_file_path = self.get_parameter('osm_file_path').get_parameter_value().string_value
        
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
                
        # 加载估计的AP位置
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
                    self.rss_subscription.destroy()  # ROS2中使用destroy而不是unregister
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
        
        self.get_logger().info(f'Known AP positions count: {len(self.ap_to_position)}')
        self.get_logger().info(f'Estimated AP positions count: {len(self.estimated_AP_positions)}')
        
        for mac, avg_rssi in mac_avg_rssi.items():
            if mac in self.ap_to_position:
                self.get_logger().info(f'MAC address {mac} found in known list')
                positions.append([self.ap_to_position[mac][0], self.ap_to_position[mac][1], 2])
                rssis.append(avg_rssi)
            else:
                self.get_logger().warn(f'MAC address {mac} not found in known list')
        
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
            else:
                self.get_logger().error('Position estimation failed')

def main(args=None):
    rclpy.init(args=args)
    
    robot_localizer = RobotLocalizer()
    
    try:
        rclpy.spin(robot_localizer)
    except KeyboardInterrupt:
        pass
    finally:
        # 清理节点
        robot_localizer.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
