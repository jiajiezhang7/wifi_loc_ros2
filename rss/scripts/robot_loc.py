
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
        # 初始化ROS节点
        rospy.init_node('Robot_loc', anonymous=True)
        # 创建订阅者
        self.rss_sub = rospy.Subscriber('/rss', RssData, self.callback_rss)
        # OSM文件路径
        self.osm_file_path = '/home/jay/wifi_ws/rss/map/shanghaitech_d2_1_2F_3F.osm'
        # 创建OSM解析器实例
        self.parser = OsmDataParser(self.osm_file_path)
        self.parser.parse()

        # Retrieve the parsed data
        self.ap_to_position, self.ap_level, self.target_ap, self.way_data, self.all_mac = self.parser.get_data()
   
        self.polygons = []
        for way , way_level in self.way_data:
            if len(way) > 2 and Polygon(way).is_valid:
                poly = Polygon(way)
                self.polygons.append((poly, way_level))
        
        self.estimated_AP_positions = load_estimated_positions()
                
    def callback_rss(self, new_rss):
        if len(new_rss.data) > 0:
            self.raw_rss.append(new_rss)
            rospy.loginfo("收到RSS数据，当前数据点数量: %d", len(self.raw_rss))
            
            # 当收集到10个数据点时
            if len(self.raw_rss) >= 10:
                collected_data = self.raw_rss.copy()
                self.rss_sub.unregister()
                self.process_rss_data(collected_data)
               
    
    def process_rss_data(self, rss_data):
        # 在这里处理收集到的RSS数据
        # TODO: 实现数据处理逻辑
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
            rospy.loginfo("MAC: %s, 平均RSSI: %.2f", mac, mac_avg_rssi[mac])
        
        # 将 mac_avg_rssi 按 RSSI 值降序排序并取前 10 项
        mac_avg_rssi = dict(sorted(mac_avg_rssi.items(), key=lambda x: x[1], reverse=True)[:50])
        
        rospy.loginfo("前 10 个 MAC 地址及其平均 RSSI 值: %s", mac_avg_rssi)
        
        
        positions = []
        distance_xy = []
        rssis = []
        target_ap_level = []
        
        # rospy.loginfo("已知的AP位置: %s", self.ap_to_position)
        rospy.loginfo("已知的AP位置数量: %d", len(self.ap_to_position))
        rospy.loginfo("已知的AP位置数量: %s", len(self.estimated_AP_positions))
        for mac, avg_rssi in mac_avg_rssi.items():
            if mac in self.ap_to_position:
                rospy.loginfo("MAC 地址 %s 在已知列表中", mac)
                positions.append([self.ap_to_position[mac][0], self.ap_to_position[mac][1], 2])
                rssis.append(avg_rssi)
                # target_ap_level.append(self.ap_level[mac])
            else:
                rospy.logwarn("MAC 地址 %s 不在已知列表中", mac)
                
                
        # self.raw_rss = []
        if len(positions) >= 3 :
          
            initial_lat = np.mean([pos[0] for pos in positions])
            initial_lon = np.mean([pos[1] for pos in positions])
            initial_alt = np.mean([pos[2] for pos in positions])
            
            initial_guess = [initial_lat, initial_lon, initial_alt]
       
            distances = np.array([rssi_to_distance(rssi, A = -38.85890085025037, n = 2.221716321548527) for rssi in rssis])

            # 创建PointEstimator实例
            estimator = PointEstimator(positions, distances, self.polygons)
            
            # 估计待确定点的坐标
            result_one = estimator.estimate_point(initial_guess=initial_guess)
            
            rospy.loginfo("估计的坐标: %s", result_one.x)
                
        
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