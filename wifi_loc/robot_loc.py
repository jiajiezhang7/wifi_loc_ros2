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
        
        # 控制是否使用AP真实位置的参数（已经写入了localization_using_area_graph的run.launch.py中）
        self.declare_parameter('use_true_ap_positions', False)
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
                
        # 初始化polygon_edges用于可视化
        self.polygon_edges = {'1':[],'2':[], '3':[]}
        for polygon, poly_level in self.polygons:
            exterior_coords = list(polygon.exterior.coords)
            for i in range(len(exterior_coords) - 1):
                edge = LineString([exterior_coords[i], exterior_coords[i + 1]])
                self.polygon_edges[poly_level].append(edge)
                
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
                
                # 当收集到5个数据点时(修改WiFi定位的时间就需要修改这里)
                if len(self.raw_rss) >= 5:
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
            self.get_logger().debug(f'MAC: {mac}, Average RSSI: {mac_avg_rssi[mac]:.2f}')
        
        # 计算每个MAC地址所在的楼层
        mac_floors = {}
        for mac in mac_avg_rssi:
            if mac in self.ap_level:
                floor = int(self.ap_level[mac])
                mac_floors[mac] = floor
            else:
                mac_floors[mac] = None

        floor_counts = Counter([floor for floor in mac_floors.values() if floor is not None])
        self.get_logger().info(f'每个楼层检测到的MAC地址数量: {dict(floor_counts)}')

        # 确定最可能的楼层（检测到的MAC地址最多的楼层）
        most_probable_floor = floor_counts.most_common(1)[0][0] if floor_counts else 2
        self.get_logger().info(f'最可能的楼层: {most_probable_floor}')

        # 将 mac_avg_rssi 按 RSSI 值降序排序
        mac_avg_rssi = dict(sorted(mac_avg_rssi.items(), key=lambda x: x[1], reverse=True))

        positions = []
        rssis = []
        AP_positions = self.ap_to_position if self.use_true_ap_positions else self.estimated_AP_positions

        self.get_logger().debug(f'Using {"true" if self.use_true_ap_positions else "estimated"} AP positions for calculation')

        # 按AP分组（忽略最后一个字符）
        ap_groups = {}
        for mac, avg_rssi in mac_avg_rssi.items():
            if mac in AP_positions and int(self.ap_level[mac]) == most_probable_floor:
                # 使用MAC地址除最后一个字符作为分组键
                ap_group_key = mac[:-1]
                if ap_group_key not in ap_groups:
                    ap_groups[ap_group_key] = {
                        'pos': [],
                        'rssis': []
                    }
                ap_groups[ap_group_key]['pos'].append([AP_positions[mac][0], AP_positions[mac][1], 2])
                ap_groups[ap_group_key]['rssis'].append(avg_rssi)

        # 计算每个分组的平均位置和RSSI
        for ap_key, group_data in ap_groups.items():
            positions_array = np.array(group_data['pos'])
            pos_data = np.mean(positions_array, axis=0)  # 返回[x_avg, y_avg, z_avg]
            avg_group_rssi = sum(group_data['rssis']) / len(group_data['rssis'])

            positions.append([pos_data[0], pos_data[1], pos_data[2]])
            rssis.append(avg_group_rssi)

        positions_tuples = [tuple(pos) for pos in positions]
        self.get_logger().info(f'位置总数: {len(positions)}, 唯一位置数: {len(set(positions_tuples))}')
        
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
                
                # 计算room_id（使用前两个AP位置的平均值）
                if len(positions) >= 2:
                    first_two_positions = np.array(positions[:2])
                    room_pos = np.mean(first_two_positions, axis=0)[:2]  # 返回[x_avg, y_avg]
                    self.get_logger().info(f'Room position: {room_pos}')
                else:
                    room_pos = [result.x[0], result.x[1]]  # 如果AP数量不足，使用定位结果
                
                # 创建并发布 WifiLocation 消息
                location_msg = WifiLocation()
                # latitude 纬度 -- x[1],  longitude 经度 -- x[0]
                location_msg.latitude = float(result.x[1])
                location_msg.longitude = float(result.x[0])
                location_msg.altitude = float(result.x[2]) * 3.2
                location_msg.floor = most_probable_floor

                # 为了在AGLoc中确定房间ID，从而进一步缩小采样粒子的范围，因此这里需要提供房间的经纬度
                location_msg.room_long = float(room_pos[0])
                location_msg.room_lat = float(room_pos[1])

                
                # 发布位置消息
                self.location_publisher.publish(location_msg)
                self.get_logger().info('Published location to /WifiLocation topic')
                
                # 可视化定位结果
                self.visualize_localization_result(positions, rssis, result.x, room_pos=room_pos)
            else:
                self.get_logger().error('Position estimation failed')

    def visualize_localization_result(self, positions, rssis, result, room_pos=None):
        """可视化定位结果、检测到的AP位置及其RSSI信号值"""
        try:
            # 导入matplotlib并设置非交互式后端
            import matplotlib
            matplotlib.use('Agg')  # 使用非交互式后端
            import matplotlib.pyplot as plt
            from datetime import datetime
            
            # 创建一个新图形
            fig, ax = plt.subplots(figsize=(10, 8))
            
            # 绘制多边形（地图边界）
            for edge in self.polygon_edges[str(2)]:
                edge_x, edge_y = edge.xy
                ax.plot(edge_x, edge_y, linewidth=1, linestyle='solid')
            
            # 绘制定位点
            ax.scatter(result[0], result[1], color='red', s=100, marker='*', label='Location Result')
            ax.annotate(f'Result ({result[0]:.2f}, {result[1]:.2f})', 
                    (result[0], result[1]), 
                    textcoords="offset points", 
                    xytext=(0,10), 
                    ha='center')
            
            # 绘制房间位置点（如果提供了room_pos）
            if room_pos is not None:
                ax.scatter(room_pos[0], room_pos[1], color='green', s=100, marker='o', label='Room Position')
                ax.annotate(f'Room ({room_pos[0]:.2f}, {room_pos[1]:.2f})', 
                        (room_pos[0], room_pos[1]), 
                        textcoords="offset points", 
                        xytext=(0,-20), 
                        ha='center')
                
                # 绘制从房间位置到定位结果的连线
                ax.plot([room_pos[0], result[0]], [room_pos[1], result[1]], 'r--', alpha=0.5, linewidth=2, label='Room-Result Distance')
            
            # 绘制检测到的AP位置并显示RSSI值
            for i, (pos, rssi) in enumerate(zip(positions, rssis)):
                ax.scatter(pos[0], pos[1], color='blue', s=50, alpha=0.7)
                ax.annotate(f'AP{i+1}: {rssi:.1f} dBm', 
                        (pos[0], pos[1]), 
                        textcoords="offset points", 
                        xytext=(0,-15), 
                        ha='center', 
                        fontsize=8)
                
                # 绘制从AP到定位结果的连线
                ax.plot([pos[0], result[0]], [pos[1], result[1]], 'g--', alpha=0.3)
            
            # 添加图例和标题
            ax.legend()
            ax.set_title('WiFi Positioning Result')
            ax.set_xlabel('Longitude')
            ax.set_ylabel('Latitude')
            
            # 保持纵横比一致
            ax.set_aspect('equal')
            
            # 生成带时间戳的文件名
            timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
            fig_path = f'/home/jay/AGLoc_ws/figs_wifi_loc/localization_result_{timestamp}.png'
            
            # 保存图像
            plt.savefig(fig_path, dpi=300, bbox_inches='tight')
            
            # 关闭图形，释放内存
            plt.close(fig)
            
            self.get_logger().info(f'定位结果可视化已保存到 {fig_path}')
            
        except Exception as e:
            self.get_logger().error(f'可视化定位结果时出错: {str(e)}')

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
