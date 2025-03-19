import rospy
from rss.msg import RssData
from osmAG.Xmlparser import OsmDataParser
from collections import Counter
import numpy as np
from osmAG.util import rssi_to_distance
from osmAG.opter import PointEstimator
from shapely.geometry import Polygon, Point, LineString
from osmAG.util import load_estimated_positions

class RobotLocalizer:
    def __init__(self):
        self.raw_rss = []
       
        # OSM文件路径
        self.osm_file_path = '/home/maxu/OsmAG_wifi/src/rss/map/shanghaitech_d2_1_2F_3F.osm'
        # 创建OSM解析器实例
        self.parser = OsmDataParser(self.osm_file_path)
        self.parser.parse()

        # Retrieve the parsed data
        self.ap_to_position, self.ap_level, self.target_ap, self.way_data, self.all_mac = self.parser.get_data()
        # print("sdlkjg",self.ap_level)
        # input("Press Enter to continue...")
        self.polygons = []
        for way , way_level in self.way_data:
            if len(way) > 2 and Polygon(way).is_valid:
                poly = Polygon(way)
                self.polygons.append((poly, way_level))
                
        self.polygon_edges = {'1':[],'2':[], '3':[]}
        for polygon, poly_level in self.polygons:
            exterior_coords = list(polygon.exterior.coords)
            for i in range(len(exterior_coords) - 1):
                edge = LineString([exterior_coords[i], exterior_coords[i + 1]])
                self.polygon_edges[poly_level].append(edge)
            
        self.estimated_AP_positions = load_estimated_positions()
        
        # 初始化ROS节点
        rospy.init_node('Robot_loc', anonymous=True)
        # 创建订阅者
        self.rss_sub = rospy.Subscriber('/rss', RssData, self.callback_rss)
        
        self.initialization_complete = True
    def callback_rss(self, new_rss):
        
        # 检查初始化是否完成
        if not self.initialization_complete:
            return
            
        if len(new_rss.data) > 0:
            self.raw_rss.append(new_rss)
            # rospy.loginfo("收到RSS数据，当前数据点数量: %d", len(self.raw_rss))
            
            # 当收集到10个数据点时
            if len(self.raw_rss) >= 10:
                collected_data = self.raw_rss.copy()
                self.rss_sub.unregister()
             
                self.process_rss_data(collected_data)
    
    def process_rss_data(self, collected_data):
        
        # 创建字典存储每个MAC地址对应的RSSI值列表
        mac_rssi_dict = {}
        
        # 遍历所有收集的数据
        for rss_data in self.raw_rss:
            for i, mac in enumerate(rss_data.mac_address):
                if mac not in mac_rssi_dict:
                    mac_rssi_dict[mac] = []
                mac_rssi_dict[mac].extend(rss_data.data[i].rss)
        
        # 计算每个MAC地址的RSSI平均值
        mac_avg_rssi = {}
        for mac, rssi_list in mac_rssi_dict.items():
            mac_avg_rssi[mac] = sum(rssi_list) / len(rssi_list)
            
        
            
        # 计算每个MAC地址所在的楼层
        mac_floors = {}
        for mac in mac_avg_rssi:
            # 检查MAC地址是否在已知AP列表中及其所在楼层
            if mac in self.ap_level:
                floor = int(self.ap_level[mac])
                mac_floors[mac] = floor  
            else:
                mac_floors[mac] = None  # 未知楼层
                
        floor_counts = Counter([floor for floor in mac_floors.values() if floor is not None])
        rospy.loginfo("每个楼层检测到的MAC地址数量: %s", dict(floor_counts))
        
            # 确定最可能的楼层（检测到的MAC地址最多的楼层）
        # TODO 这是楼层信息，我需要包装成自定义消息（WifiLocation.msg中的floor字段），并且发布出去
        most_probable_floor = floor_counts.most_common(1)[0][0] if floor_counts else 2
        rospy.loginfo("最可能的楼层: %d", most_probable_floor)
        

     
        
        # rospy.loginfo("已知的AP位置: %s", self.ap_to_position)
        # rospy.loginfo("已知的AP位置数量: %d", len(self.ap_to_position)) #  175
        # rospy.loginfo("已知的AP位置数量: %s", len(self.estimated_AP_positions)) # 84
        
        # sorted_macs = sorted(self.ap_to_position.keys())
        # rospy.loginfo("AP MAC addresses sorted alphabetically:")
        # for mac in sorted_macs:
        #     print(mac)
        
        positions=[]
        rssis = []
        AP_positions = self.estimated_AP_positions
        
         
        # 将 mac_avg_rssi 按 RSSI 值降序排序
        mac_avg_rssi = dict(sorted(mac_avg_rssi.items(), key=lambda x: x[1], reverse=True))
       
        # 按AP分组（忽略最后一个字符）
        ap_groups = {}
        for mac, avg_rssi in mac_avg_rssi.items():
            if mac in AP_positions and self.ap_level[mac] == most_probable_floor:
               
                # 使用MAC地址除最后一个字符作为分组键
                ap_group_key = mac[:-1]
                if ap_group_key not in ap_groups:
                    ap_groups[ap_group_key] = {
                        'pos': [],
                        'rssis': []
                    }

                ap_groups[ap_group_key]['pos'].append([AP_positions[mac][0], AP_positions[mac][1],2])
                ap_groups[ap_group_key]['rssis'].append(avg_rssi)
        
        
        for ap_key, group_data in ap_groups.items():
            positions_array = np.array(group_data['pos'])
            pos_data = np.mean(positions_array, axis=0)  # 返回[x_avg, y_avg, z_avg]
            avg_group_rssi = sum(group_data['rssis']) / len(group_data['rssis'])
            
            positions.append([pos_data[0], pos_data[1], pos_data[2]])
            rssis.append(avg_group_rssi)
    
                
        positions_tuples = [tuple(pos) for pos in positions]
        rospy.logerr("位置总数: %d, 唯一位置数: %d", len(positions), len(set(positions_tuples))) 
        

        rospy.loginfo("检测到的AP RSSI: %s", rssis)
        # self.raw_rss = []
        if len(positions) >= 3 :
            first_two_positions = np.array(positions[:2])
            # TODO room_id是long和lat，需要修改自定义消息类型WiFiLocation，再自定义更多字段room_long,room_lat，把这里的room_id发布出去
            room_id = np.mean(first_two_positions, axis=0)[:2]  # 返回[x_avg, y_avg, z_avg]
            
            
            initial_lat = np.mean([pos[0] for pos in positions])
            initial_lon = np.mean([pos[1] for pos in positions])
            initial_alt = np.mean([pos[2] for pos in positions])
            
            initial_guess = [initial_lat, initial_lon, initial_alt]
       
            distances = np.array([rssi_to_distance(rssi, A = -38.85890085025037, n = 2.221716321548527) for rssi in rssis])

            # 创建PointEstimator实例
            estimator = PointEstimator(positions, distances, self.polygons)
            
            # 估计待确定点的坐标
            result_one = estimator.estimate_point(initial_guess=initial_guess)
            
            # TODO 改成话题发布出去，
            rospy.loginfo("估计的坐标: %s", result_one.x)
                
    
            # 可视化定位结果
            # TODO maybe注释
            # self.visualize_localization_result(positions, rssis, result_one.x)
                
    def visualize_localization_result(self, positions, rssis, result):
        """可视化定位结果、检测到的AP位置及其RSSI信号值"""
        try:
            # 导入matplotlib并设置非交互式后端
            import matplotlib
            matplotlib.use('Agg')  # 使用非交互式后端
            import matplotlib.pyplot as plt
            
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
            
            # 添加图例和标题(使用英文避免字体问题)
            ax.legend()
            ax.set_title('WiFi Positioning Result')
            ax.set_xlabel('Longitude')
            ax.set_ylabel('Latitude')
            # ax.grid(True, linestyle='--', alpha=0.7)
            
            # 保持纵横比一致
            ax.set_aspect('equal')
            
            # 保存图像
            # TODO 路径
            fig_path = '/home/maxu/OsmAG_wifi/src/rss/fig/localization_result.png'
            plt.savefig(fig_path, dpi=300, bbox_inches='tight')
            
            # 关闭图形，释放内存
            plt.close(fig)
            
            rospy.loginfo("Positioning result visualization saved to %s", fig_path)
            
        except Exception as e:
            rospy.logerr("Error visualizing positioning result: %s", str(e))
        
    def run(self):
        rate = rospy.Rate(10)  # 10Hz
        while not rospy.is_shutdown():
            # 主循环逻辑
            rate.sleep()

if __name__ == "__main__":
    try:
        print("RobotLocalizer node started")
        localizer = RobotLocalizer()
        localizer.run()
    except rospy.ROSInterruptException:
        pass