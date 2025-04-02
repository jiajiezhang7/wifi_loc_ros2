#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from wifi_loc.utils.Xmlparser import OsmDataParser
from wifi_loc.utils.read_pickle import RssData as PickleRssData, RssDatum
from rss.msg import RssData, WifiLocation
from collections import Counter
import numpy as np
import copy
from wifi_loc.utils.util import rssi_to_distance, load_estimated_positions
from wifi_loc.utils.util import calculate_precise_distance, find_largest_polygon, find_smallest_room_polygon
from wifi_loc.utils.opter import PointEstimator
from shapely.geometry import Polygon, Point, LineString
from shapely.ops import nearest_points
import os
import json
from datetime import datetime
from ament_index_python.packages import get_package_share_directory
import matplotlib
import matplotlib.pyplot as plt
import matplotlib.patches as patches

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
        
        # 创建位置发布者，使用与particle_generator兼容的QoS设置
        from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy
        
        # 创建与particle_generator兼容的QoS配置
        reliable_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        self.location_publisher = self.create_publisher(
            WifiLocation,
            'WifiLocation',
            reliable_qos
        )
        
        # 初始化数据
        self.raw_rss = []
        
        # 保存最近一次的WiFi定位结果
        self.latest_location_msg = None
        
        # 创建定时器，定期重发最新的WiFi定位结果（确保订阅者能接收到）
        self.republish_timer = self.create_timer(1.0, self.republish_location)
        
        # 创建存储定位结果的列表
        self.localization_results = []
        
        # 获取rosbag名称
        self.declare_parameter('bag_path', '')
        bag_path = self.get_parameter('bag_path').get_parameter_value().string_value
        # 提取bag名称 - 使用目录的basename作为bag名称
        self.bag_name = os.path.basename(bag_path) if bag_path else 'unknown_bag'
        self.get_logger().info(f'使用bag: {self.bag_name}')
        
        # 获取包路径并设置默认OSM文件路径
        package_share_dir = get_package_share_directory('wifi_loc')
        default_osm_path = os.path.join(package_share_dir, 'map', 'shanghaitech_d2_1_2F_3F.osm')
        
        # OSM文件路径
        self.declare_parameter('osm_file_path', default_osm_path)
        self.osm_file_path = self.get_parameter('osm_file_path').get_parameter_value().string_value
        
        # 控制是否使用AP真实位置的参数
        self.declare_parameter('use_true_ap_positions', False)
        self.use_true_ap_positions = self.get_parameter('use_true_ap_positions').get_parameter_value().bool_value
        
        # 添加调试输出，显示参数的值
        if self.use_true_ap_positions:
            self.get_logger().info('使用真实AP位置进行定位计算')
        else:
            self.get_logger().info('使用估计的AP位置进行定位计算')
        
        # 创建OSM解析器实例
        self.parser = OsmDataParser(self.osm_file_path)
        self.parser.parse()
        self.parser.extract_polygon_edges()
        
        # 获取解析后的数据
        self.ap_to_position, self.ap_level, self.target_ap, self.way_data, self.all_mac, self.polygons, self.polygon_edges = self.parser.get_data()
                
        # 加载估计的AP位置(从json中读取)
        self.estimated_AP_positions = load_estimated_positions('/home/jay/AGLoc_ws/src/wifi_loc/wifi_loc/data/estimated_positions.json')
        
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

        self.get_logger().info(f'定位计算使用{"真实" if self.use_true_ap_positions else "估计的"}AP位置，共{len(AP_positions)}个AP')

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
        
        # 信号传播模型参数
        iter_num_total = 10  # 迭代次数
        ave_val = -25  # 墙体衰减值
        rssi_0_opt = -26.95154604523117  # 优化后的RSSI参考值
        n_opt = 2.8158686097356154  # 优化后的路径损耗指数
        
        # 查找最大面积的多边形（用于墙体检测）
        largest_area = find_largest_polygon(self.polygons)
        
        if len(positions) >= 3:
            # 计算房间ID（使用前两个AP位置的平均值）
            first_two_positions = np.array(positions[:2])
            room_id = np.mean(first_two_positions, axis=0)[:2]  # 返回[x_avg, y_avg]
            
            # 查找包含初始点的最小房间（用于空间约束）
            smallest_room = find_smallest_room_polygon(room_id, self.polygons, most_probable_floor)
            
            # 计算初始猜测位置
            initial_lat = np.mean([pos[0] for pos in positions])
            initial_lon = np.mean([pos[1] for pos in positions])
            initial_alt = most_probable_floor * 3.2
            
            initial_guess = [initial_lat, initial_lon, initial_alt]
            
            # 设置优化边界
            if smallest_room is not None:
                # 使用最小房间的边界作为优化限制
                minx, miny, maxx, maxy = smallest_room['bounds']
                # 对least_squares函数，边界格式需要是([lower_bounds], [upper_bounds])
                # 确保边界有正确的差距，避免上下界相等
                height_lower = initial_alt - 3.2
                height_upper = initial_alt + 1
                
                # 确保边界不相等
                if maxx - minx < 0.0001:
                    maxx = minx + 0.0001
                if maxy - miny < 0.0001:
                    maxy = miny + 0.0001
                if height_upper - height_lower < 0.0001:
                    height_upper = height_lower + 0.0001
                
                # 调整初始猜测点使其在边界内
                original_point = [initial_guess[0], initial_guess[1]]
                
                # 首先将初始猜测点限制在矩形边界内
                if initial_guess[0] < minx:
                    initial_guess[0] = minx + 0.0001  # 稍微偏移以避免在边界上
                    self.get_logger().info(f'初始猜测点经度小于下界，已调整到: {initial_guess[0]}')
                elif initial_guess[0] > maxx:
                    initial_guess[0] = maxx - 0.0001
                    self.get_logger().info(f'初始猜测点经度大于上界，已调整到: {initial_guess[0]}')
                
                if initial_guess[1] < miny:
                    initial_guess[1] = miny + 0.0001
                    self.get_logger().info(f'初始猜测点纬度小于下界，已调整到: {initial_guess[1]}')
                elif initial_guess[1] > maxy:
                    initial_guess[1] = maxy - 0.0001
                    self.get_logger().info(f'初始猜测点纬度大于上界，已调整到: {initial_guess[1]}')
                
                if initial_guess[2] < height_lower:
                    initial_guess[2] = height_lower + 0.0001
                    self.get_logger().info(f'初始猜测点高度小于下界，已调整到: {initial_guess[2]}')
                elif initial_guess[2] > height_upper:
                    initial_guess[2] = height_upper - 0.0001
                    self.get_logger().info(f'初始猜测点高度大于上界，已调整到: {initial_guess[2]}')
                
                # 如果有多边形信息，还需要检查点是否在多边形内
                if 'polygon' in smallest_room:
                    point = Point(initial_guess[0], initial_guess[1])
                    if not smallest_room['polygon'].contains(point):
                        # 找到从原始点到多边形边界的最近点（垂足）
                        original_point_shapely = Point(original_point[0], original_point[1])
                        boundary = smallest_room['polygon'].boundary
                        nearest_point = nearest_points(original_point_shapely, boundary)[1]
                        
                        # 将初始猜测点调整为边界上的最近点
                        initial_guess[0] = nearest_point.x
                        initial_guess[1] = nearest_point.y
                        self.get_logger().info(f'初始猜测点不在房间多边形内，已调整到边界最近点: ({nearest_point.x}, {nearest_point.y})')
                        
                        # 将边界点向多边形内部稍微偏移，确保在多边形内
                        centroid = smallest_room['polygon'].centroid
                        # 计算从边界点到中心的向量
                        vector_to_center = [centroid.x - nearest_point.x, centroid.y - nearest_point.y]
                        vector_length = np.sqrt(vector_to_center[0]**2 + vector_to_center[1]**2)
                        if vector_length > 0:
                            # 往中心方向偏移一小段距离
                            offset = 0.0001
                            initial_guess[0] += (vector_to_center[0] / vector_length) * offset
                            initial_guess[1] += (vector_to_center[1] / vector_length) * offset
                            
                            # 再次检查是否在多边形内
                            point = Point(initial_guess[0], initial_guess[1])
                            if not smallest_room['polygon'].contains(point):
                                # 如果仍然不在多边形内，使用多边形的中心点
                                initial_guess[0] = centroid.x
                                initial_guess[1] = centroid.y
                                self.get_logger().info(f'垂足偏移后仍不在多边形内，已调整到房间中心: ({centroid.x}, {centroid.y})')
                
                # 最后再次确认初始猜测点在边界内，并稍微偏移以避免在边界上
                initial_guess[0] = max(minx + 0.0001, min(initial_guess[0], maxx - 0.0001))
                initial_guess[1] = max(miny + 0.0001, min(initial_guess[1], maxy - 0.0001))
                initial_guess[2] = max(height_lower + 0.0001, min(initial_guess[2], height_upper - 0.0001))
                
                # 设置最终的优化边界，确保初始猜测点在边界内
                optimization_bounds = ([minx, miny, height_lower], [maxx, maxy, height_upper])
                self.get_logger().info(f'使用房间边界约束: {optimization_bounds}')
            else:
                # 默认边界
                optimization_bounds = None
                self.get_logger().info("未找到包含初始点的房间，使用默认边界")
            
            # 将RSSI值转换为距离
            distances = np.array([rssi_to_distance(rssi, A=rssi_0_opt, n=n_opt) for rssi in rssis])
            
            # 创建PointEstimator实例并估计初始位置
            estimator = PointEstimator(positions, distances, self.polygons)
            result_one = estimator.estimate_point(initial_guess=initial_guess, bounds=optimization_bounds)
            result_init = result_one
            
            # 迭代定位过程
            for iter_num in range(1, iter_num_total):
                intersection_edge = []
                at_val = []
                temp_rssis = copy.deepcopy(rssis)
                flags = np.zeros(len(positions))
                
                # 检测每个AP信号是否穿过墙体
                for pos_num, pos_ap in enumerate(positions):
                    # 提取距离RP最近的边
                    ap_line = LineString([(pos_ap[0], pos_ap[1]), (result_one.x[0], result_one.x[1])])
                    closest_edge = None
                    closest_distance = float('inf')
                    
                    # 检查信号路径是否与墙体相交
                    for edge in self.polygon_edges[str(most_probable_floor)]:
                        if ap_line.intersects(edge):
                            ap_intersection = ap_line.intersection(edge)
                            ap_intersection_line_distance = calculate_precise_distance(
                                pos_ap[0], pos_ap[1], ap_intersection.x, ap_intersection.y
                            )
                            
                            if ap_intersection_line_distance < closest_distance:
                                closest_distance = ap_intersection_line_distance
                                closest_edge = edge
                    
                    # 计算closest_edge的长度（以米为单位）
                    if closest_edge is not None:
                        # 获取边的端点坐标
                        edge_coords = list(closest_edge.coords)
                        if len(edge_coords) >= 2:
                            start_point = edge_coords[0]  # (x, y)
                            end_point = edge_coords[-1]   # (x, y)
                            
                            edge_length_meters = calculate_precise_distance(
                                start_point[0], start_point[1], end_point[0], end_point[1]
                            )
                            
                            # 计算点到polygon边缘的实际距离（米）
                            if largest_area is not None and ap_intersection is not None:
                                # 找到多边形边界上最近的点
                                boundary = largest_area[most_probable_floor]['polygon'].boundary
                                nearest_point_on_boundary = nearest_points(ap_intersection, boundary)[1]
                                
                                # 计算实际地理距离（米）
                                distance_meters = calculate_precise_distance(
                                    ap_intersection.x, ap_intersection.y, 
                                    nearest_point_on_boundary.x, nearest_point_on_boundary.y
                                )
                                
                                # 如果墙体足够厚且长度足够长，考虑信号衰减
                                if distance_meters > 5 and edge_length_meters > 2:
                                    flags[pos_num] = 1
                                    intersection_edge.append(closest_edge)
                                    at_val.append(ave_val)
                                    temp_rssis[pos_num] = temp_rssis[pos_num] - ave_val
                
                # 根据墙体衰减情况过滤AP
                filter_dis = []
                filter_pos = []
                err_rssi = []
                
                for rssi_num, rssi in enumerate(temp_rssis):
                    dis = rssi_to_distance(rssi, A=rssi_0_opt, n=n_opt)
                    if flags[rssi_num] == 1 and dis < 60:
                        err_rssi.append(rssi)
                        filter_dis.append(dis)
                        filter_pos.append(positions[rssi_num])
                    elif flags[rssi_num] == 0 and dis < 20:
                        err_rssi.append(rssi)
                        filter_dis.append(dis)
                        filter_pos.append(positions[rssi_num])
                
                # 如果过滤后AP数量不足，使用所有AP
                if len(filter_dis) < 3:
                    filter_pos = positions
                    err_rssi = temp_rssis
                    distances = np.array([rssi_to_distance(rssi, A=rssi_0_opt, n=n_opt) for rssi in temp_rssis])
                    estimator = PointEstimator(positions, distances, self.polygons)
                    result = estimator.estimate_point(initial_guess=initial_guess, bounds=optimization_bounds)
                else:
                    # 使用过滤后的AP重新计算初始猜测
                    initial_lat = np.mean([pos[0] for pos in filter_pos])
                    initial_lon = np.mean([pos[1] for pos in filter_pos])
                    initial_alt = most_probable_floor * 3.2
                    
                    initial_guess = [initial_lat, initial_lon, initial_alt]
                    estimator = PointEstimator(filter_pos, filter_dis, self.polygons)
                    
                    # 估计待确定点的坐标
                    result = estimator.estimate_point(initial_guess=initial_guess, bounds=optimization_bounds)
                
                result_one = result
            
            self.get_logger().info(f'最终估计的坐标: {result_one.x}')
            
            if result_one is not None:
                self.get_logger().info(f'Estimated position: {result_one.x}')
                
                # 计算room_id（使用前两个AP位置的平均值）
                if len(positions) >= 2:
                    first_two_positions = np.array(positions[:2])
                    room_pos = np.mean(first_two_positions, axis=0)[:2]  # 返回[x_avg, y_avg]
                    self.get_logger().info(f'Room position: {room_pos}')
                else:
                    room_pos = [result_one.x[0], result_one.x[1]]  # 如果AP数量不足，使用定位结果
                
                # 计算Room Position和Final Position连线的中点
                mid_point = [
                    (room_pos[0] + result_one.x[0]) / 2,  # 经度中点
                    (room_pos[1] + result_one.x[1]) / 2,  # 纬度中点
                    result_one.x[2]  # 保持原来的高度
                ]
                self.get_logger().info(f'计算得到的中点坐标: {mid_point}')
                
                # 创建并发布 WifiLocation 消息
                location_msg = WifiLocation()
                # latitude 纬度 -- mid_point[1],  longitude 经度 -- mid_point[0]
                location_msg.latitude = float(mid_point[1])
                location_msg.longitude = float(mid_point[0])
                location_msg.altitude = float(mid_point[2])
                location_msg.floor = most_probable_floor

                # 为了在AGLoc中确定房间ID，从而进一步缩小采样粒子的范围，因此这里需要提供房间的经纬度
                location_msg.room_long = float(room_pos[0])
                location_msg.room_lat = float(room_pos[1])

                
                # 保存并发布位置消息
                self.latest_location_msg = location_msg
                self.location_publisher.publish(location_msg)
                self.get_logger().info('Published location to /WifiLocation topic')
                
                # 计算Room-Result距离
                room_result_distance = calculate_precise_distance(
                    room_pos[0], room_pos[1], result_one.x[0], result_one.x[1]
                )
                
                # 将定位结果保存到列表中
                result_data = {
                    'timestamp': datetime.now().strftime('%Y-%m-%d %H:%M:%S'),
                    'bag_name': self.bag_name,
                    'use_true_ap_positions': self.use_true_ap_positions,
                    'room_position': {'longitude': float(room_pos[0]), 'latitude': float(room_pos[1])},
                    'final_position': {'longitude': float(result_one.x[0]), 'latitude': float(result_one.x[1]), 'altitude': float(result_one.x[2])},
                    'midpoint_position': {'longitude': float(mid_point[0]), 'latitude': float(mid_point[1]), 'altitude': float(mid_point[2])},
                    'room_result_distance': float(room_result_distance),
                    'floor': int(most_probable_floor)
                }
                self.localization_results.append(result_data)
                
                # 将定位结果保存到JSON文件
                self.save_results_to_json()
                
                # 将中点传递给可视化函数
                self.visualize_localization_result(positions, rssis, result_one.x, room_pos=room_pos, 
                                                  smallest_room=smallest_room, mid_point=mid_point)
                
                # 可视化定位结果
                # # 先可视化初始定位结果 (已注释掉，只保留最终结果)
                # if result_init is not None and not np.array_equal(result_init.x, result_one.x):
                #     self.visualize_localization_result(positions, rssis, result_init.x, room_pos=room_pos, 
                #                                       smallest_room=smallest_room, is_init=True)
                # 只可视化最终定位结果
                # 这里不需要再调用可视化函数，因为已经在上面调用过了
            else:
                self.get_logger().error('Position estimation failed')

    def visualize_localization_result(self, positions, rssis, result, room_pos=None, smallest_room=None, is_init=False, mid_point=None):
        """可视化定位结果、检测到的AP位置及其RSSI信号值"""
        try:
            # 导入matplotlib并设置非交互式后端
            matplotlib.use('Agg')  # 使用非交互式后端
            
            # 创建一个新图形
            fig, ax = plt.subplots(figsize=(10, 8))
            
            # 绘制多边形（地图边界）
            for edge in self.polygon_edges[str(2)]:
                edge_x, edge_y = edge.xy
                ax.plot(edge_x, edge_y, linewidth=1, linestyle='solid')
            
            # 如果有最小房间边界，则绘制边界框
            if smallest_room is not None:
                minx, miny, maxx, maxy = smallest_room['bounds']
                width = maxx - minx
                height = maxy - miny
                self.get_logger().info(f'最小房间边界: {smallest_room["bounds"]}')
                # 创建一个矩形，表示房间边界
                room_rect = patches.Rectangle(
                    (minx, miny), width, height, 
                    linewidth=1.5,  # 减小线宽为原来的3的一半
                    edgecolor='orange', 
                    facecolor='none', 
                    label='Room Boundary',
                    linestyle='--'
                )
                
                # 添加矩形到图形中
                ax.add_patch(room_rect)
            
            # 绘制定位点
            result_label = 'Initial Position' if is_init else 'Final Position'
            result_color = 'blue' if is_init else 'red'
            ax.scatter(result[0], result[1], color=result_color, s=100, marker='*', label=result_label)
            # 不在点旁边标注，而是收集坐标信息稍后统一显示
            position_info = [f'{result_label}: ({result[0]:.6f}, {result[1]:.6f})']
            
            # 绘制房间位置点（如果提供了room_pos）
            if room_pos is not None:
                ax.scatter(room_pos[0], room_pos[1], color='green', s=100, marker='o', label='Room Position')
                # 收集坐标信息
                position_info.append(f'Room Position: ({room_pos[0]:.6f}, {room_pos[1]:.6f})')
                
                # 绘制从房间位置到定位结果的连线
                ax.plot([room_pos[0], result[0]], [room_pos[1], result[1]], 'r--', alpha=0.5, linewidth=2, label='Room-Result Distance')
                
                # 绘制中点（如果提供了mid_point）
                if mid_point is not None:
                    ax.scatter(mid_point[0], mid_point[1], color='goldenrod', s=150, marker='*', label='Midpoint Position')
                    # 收集坐标信息
                    position_info.append(f'Midpoint Position: ({mid_point[0]:.6f}, {mid_point[1]:.6f})')
            
            # 绘制检测到的AP位置并显示RSSI值
            for i, (pos, rssi) in enumerate(zip(positions, rssis)):
                ax.scatter(pos[0], pos[1], color='blue', s=50, alpha=0.7)
                ax.annotate(f'AP{i+1}: {rssi:.1f} dBm', 
                        (pos[0], pos[1]), 
                        textcoords="offset points", 
                        xytext=(0,-15), 
                        ha='center', 
                        # 修改AP标签的字体大小
                        fontsize=6)
                
                # 绘制从AP到定位结果的连线
                ax.plot([pos[0], result[0]], [pos[1], result[1]], 'g--', alpha=0.3)
            
            # 在右下角添加坐标信息
            coord_text = '\n'.join(position_info)
            # 修改坐标信息的字体大小
            ax.text(0.98, 0.05, coord_text, transform=ax.transAxes, fontsize=6,
                   verticalalignment='bottom', horizontalalignment='right',
                   bbox=dict(boxstyle='round', facecolor='white', alpha=0.8))
            
            # 添加图例和标题，放在右上角
            # 图例的字体大小：matplotlib提供了预设的字体大小选项，从大到小依次是：'xx-large', 'x-large', 'large', 'medium', 'small', 'x-small', 'xx-small'
            ax.legend(loc='upper right', fontsize='x-small')
            # 在标题中添加AP位置类型信息
            ap_type_text = 'True AP Positions' if self.use_true_ap_positions else 'Estimated AP Positions'
            title = f"{'Initial' if is_init else 'Final'} WiFi Positioning Result ({ap_type_text})"
            ax.set_title(title)
            ax.set_xlabel('Longitude')
            ax.set_ylabel('Latitude')
            
            # 保持纵横比一致
            ax.set_aspect('equal')
            
            # 生成带时间戳和rosbag名称的文件名
            timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
            result_type = 'initial' if is_init else 'final'
            
            # 创建保存目录
            figs_dir = '/home/jay/AGLoc_ws/figs_wifi_loc'
            os.makedirs(figs_dir, exist_ok=True)
            
            # 添加bag名称和AP位置类型到文件名中
            ap_type = 'true_ap' if self.use_true_ap_positions else 'est_ap'
            fig_path = f'{figs_dir}/{self.bag_name}_{ap_type}_localization_result_{result_type}_{timestamp}.png'
            
            # 保存图像
            plt.savefig(fig_path, dpi=300, bbox_inches='tight')
            
            # 关闭图形，释放内存
            plt.close(fig)
            
            self.get_logger().info(f'定位结果可视化已保存到 {fig_path}')
            
        except Exception as e:
            self.get_logger().error(f'可视化定位结果时出错: {str(e)}')
            
    def save_results_to_json(self):
        """将定位结果保存到JSON文件"""
        try:
            # 创建保存目录
            results_dir = '/home/jay/AGLoc_ws/results_wifi_loc'
            os.makedirs(results_dir, exist_ok=True)
            
            # 生成包含rosbag名称和AP位置类型的文件名
            timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
            ap_type = 'true_ap' if self.use_true_ap_positions else 'est_ap'
            filename = os.path.join(results_dir, f'{self.bag_name}_{ap_type}_localization_results_{timestamp}.json')
            
            # 将结果写入JSON文件
            with open(filename, 'w') as f:
                json.dump(self.localization_results, f, indent=4)
            
            self.get_logger().info(f'定位结果已保存到 {filename}')
        except Exception as e:
            self.get_logger().error(f'保存定位结果到JSON文件时出错: {str(e)}')
    
    def republish_location(self):
        """定期重发最新的WiFi定位结果，确保所有订阅者能接收到"""
        if self.latest_location_msg is not None:
            self.location_publisher.publish(self.latest_location_msg)
            self.get_logger().debug('重新发布WiFi定位结果到 /WifiLocation 话题')

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
