from collections import Counter

def filter_top_rssi(target_ap , rssi_num):
    target_ap_list = {}
    for tagap in target_ap:

        for mac in target_ap[tagap]['mac']:
            counter = Counter(target_ap[tagap]['mac'][mac])
            filtered_items = {key: count for key, count in counter.items()} #if count > rssi_num }
            max_count = max(filtered_items.values())
            if max_count < rssi_num:
                continue
            top_keys = [key for key, count in filtered_items.items() if count == max_count]
            top_one = int(sum(top_keys) / len(top_keys))
            # print(top_one)  # 输出 30

            # print(max_count , top_keys)
            # top_one = sorted(filtered_items, key=filtered_items.get, reverse=True)# [:1]
            # print(top_one)
            # if len(top_one) != 0:
            if tagap not in target_ap_list:
                
                target_ap_list[tagap] = target_ap_list.get(tagap, {})
                target_ap_list[tagap]['mac'] = {}
                target_ap_list[tagap]['level'] = target_ap[tagap]['level']
                
            # print(f"RP level: {target_ap[tagap]['RP']}")
            if target_ap[tagap]['RP'] == '48' and mac == '98:3F:60:34:FB:B0'.lower():
                print(f"top_one: {top_one} {max_count}")        
            target_ap_list[tagap]['mac'][mac] = top_one
            
    return target_ap_list    




from shapely.geometry import LineString
from pyproj import Geod
import math


def calculate_distance(lat1, lon1, lat2, lon2, ap_level, target_ap_level):
    # 创建一个Geod对象，用WGS84椭球
    geod = Geod(ellps='WGS84')
    
    # 使用Geod的inverse方法计算两个点之间的距离
    # 返回的结果包括方位角和反方位角，我们只需要距离
    _, _, distance = geod.inv(lon1, lat1, lon2, lat2)
    
    c = math.sqrt(distance**2 + (abs(ap_level-target_ap_level) + 3.2)**2)
    return c
   

def process_rssi_data(target_ap_list, ap_to_position, polygon_edges, ap_level, learn_level):
    lines = {}
    learn_no_wall_dis_rss = {'dis': [], 'rssi': []}
    learn_wall_dis_rss = {'dis': [], 'rssi': []}
    cout = 0
    for tagap in target_ap_list:
        # if target_ap_list[tagap]['level'] == learn_level:
            for it in ap_to_position:
                
                if target_ap_list[tagap]['level'] == ap_level[it]:
                    line = LineString([tagap, ap_to_position[it]])

                    flag = 1
                    for edge in polygon_edges[learn_level]:
                        if line.intersects(edge):
                            flag = 0

                    if it in target_ap_list[tagap]['mac']:
                        distance = calculate_distance(ap_to_position[it][1], ap_to_position[it][0], tagap[1], tagap[0], ap_level[it], target_ap_list[tagap]['level'])
                        ap_rssi = target_ap_list[tagap]['mac'][it]
                        if line not in lines:
                            lines[line] = {'rssi': 0, 'dis': 0}
                        lines[line]['rssi'] = ap_rssi
                        lines[line]['dis'] = distance

                    if it in target_ap_list[tagap]['mac']:
                        distance = calculate_distance(ap_to_position[it][1], ap_to_position[it][0], tagap[1], tagap[0],ap_level[it], target_ap_list[tagap]['level'])
                        ap_rssi = target_ap_list[tagap]['mac'][it]

                        learn_no_wall_dis_rss['dis'].append(distance)
                        learn_no_wall_dis_rss['rssi'].append(ap_rssi)
                        
                    if flag and it in target_ap_list[tagap]['mac']:
                        
                        distance = calculate_distance(ap_to_position[it][1], ap_to_position[it][0], tagap[1], tagap[0],ap_level[it], target_ap_list[tagap]['level'])
                        ap_rssi = target_ap_list[tagap]['mac'][it]

                        learn_wall_dis_rss['dis'].append(distance)
                        learn_wall_dis_rss['rssi'].append(ap_rssi)

                else:
                    pass
                    # first, second = sorted([int(target_ap_list[tagap]['level']) , int(ap_level[it])])
                    # # print(f"first: {first}, second: {second}")
                    # line = LineString([tagap, ap_to_position[it]])
                    # if it in target_ap_list[tagap]['mac']:
                    #     distance = calculate_distance(ap_to_position[it][1], ap_to_position[it][0], tagap[1], tagap[0], ap_level[it], target_ap_list[tagap]['level'])
                    #     ap_rssi = target_ap_list[tagap]['mac'][it]
                    #     if line not in lines:
                    #         lines[f"{first}-{second}"] = {'rssi': 0, 'dis': 0}
                    #     lines[f"{first}-{second}"]['rssi'] = ap_rssi
                    #     lines[f"{first}-{second}"]['dis'] = distance

                    
                    # print(target_ap_list[tagap]['level'] , ap_level[it])
                    
                    
    return lines, learn_no_wall_dis_rss, learn_wall_dis_rss


import numpy as np

def calculate_angle_and_normal(line, edge):
    line_vector = np.array(line.coords[1]) - np.array(line.coords[0])
    edge_vector = np.array(edge.coords[1]) - np.array(edge.coords[0])
    edge_normal = np.array([-edge_vector[1], edge_vector[0]])

    dot_product = np.dot(line_vector, edge_normal)
    magnitude_line = np.linalg.norm(line_vector)
    magnitude_normal = np.linalg.norm(edge_normal)
    
    cos_angle = dot_product / (magnitude_line * magnitude_normal)
    angle_rad = np.arccos(np.clip(cos_angle, -1.0, 1.0))
    angle_deg = np.degrees(angle_rad)
    
    edge_normal = edge_normal / magnitude_normal  # Normalize the normal vector
    
    return angle_deg, edge_normal



def process_wall_line(lines, polygon_edges, learn_level):
    intersections = []
    nums = []
    eq = {}
    # line 存储的是所有在真值库里的ap到Rp的线段
    
    for line in lines:
        if type(line) != str:
            line_intersections = []
            for edge in polygon_edges[learn_level]:
                if line.intersects(edge):
                    line_intersections.append(edge)

            if len(line_intersections) == 1:
                
                for edge in line_intersections:
                    if edge not in intersections:
                        intersections.append(edge)
                        eq[edge] = {'dis': [], 'rssi': [], 'at_val': 0, 'angle':[], 'bais': 0}
                        
                    eq[edge]['dis'].append(lines[line]['dis'])
                    eq[edge]['rssi'].append(lines[line]['rssi'])
                    angle, normal = calculate_angle_and_normal(line, edge)
                    eq[edge]['angle'].append(angle)
                    # print(angle)
        
            nums.append(len(line_intersections))
        else:
            first, second = line.split('-')
           
            if line not in eq:
                
                eq[line] = {'dis': [], 'rssi': [], 'at_val': 0, 'angle':[], 'bais': 0}
                
            eq[line]['dis'].append(lines[line]['dis'])
            eq[line]['rssi'].append(lines[line]['rssi'])
            # angle, normal = calculate_angle_and_normal(line, edge)
            eq[line]['angle'].append(angle)
                    # print(angle)
            # nums.append(1)
    
    return intersections, eq, nums



def rssi_to_distance(rssi, A=-28.879257951315253, n=2.6132845414003243):
    return 10 ** ((A - rssi) / (10 * n))

import os
import shutil

def clear_directory(directory):
    # 检查目录是否存在
    if os.path.exists(directory):
        # 遍历目录中的所有文件和子目录
        for filename in os.listdir(directory):
            file_path = os.path.join(directory, filename)
            # 如果是文件夹，递归删除
            if os.path.isdir(file_path):
                shutil.rmtree(file_path)
            # 如果是文件，直接删除
            else:
                os.remove(file_path)
        print(f"已清空目录: {directory}")
    else:
        print(f"目录 {directory} 不存在")
        
        
        
        
def calculate_distance_uncertainty(rssi_0, n, rssi_predicted, sigma_rssi_0, sigma_n):
    """
    计算距离的不确定性。

    参数：
    - rssi_0: float, 拟合出的 rssi_0 值
    - n: float, 拟合出的 n 值
    - rssi_predicted: float, 给定的预测 RSSI 值
    - sigma_rssi_0: float, rssi_0 的不确定性（标准差）
    - sigma_n: float, n 的不确定性（标准差）

    返回：
    - sigma_distance: float, 距离的不确定性
    """
    # 计算偏导数
    partial_rssi_0 = 10 ** ((rssi_0 - rssi_predicted) / (10 * n)) / (10 * n)
    partial_n = -((rssi_0 - rssi_predicted) * 10 ** ((rssi_0 - rssi_predicted) / (10 * n))) / (10 * n ** 2)
    
    # 计算距离的不确定性
    sigma_distance = np.sqrt((partial_rssi_0 * sigma_rssi_0) ** 2 + (partial_n * sigma_n) ** 2)
    
    return sigma_distance





import json
import numpy as np

def save_estimated_positions(estimated_positions, filename='/home/maxu/OsmAG_wifi/src/rss/src/osmAG/estimated_positions.json'):
    """保存估计的AP位置到json文件"""
    # 将numpy array转换为list
    serializable_dict = {}
    for key, value in estimated_positions.items():
        if isinstance(value, np.ndarray):
            serializable_dict[key] = value.tolist()
        else:
            serializable_dict[key] = value
            
    with open(filename, 'w') as f:
        json.dump(serializable_dict, f)
    print(f"已保存AP位置到: {filename}")

def load_estimated_positions(filename='/home/maxu/OsmAG_wifi/src/rss/src/osmAG/estimated_positions.json'):
    """从json文件读取AP位置"""
    with open(filename, 'r') as f:
        loaded_dict = json.load(f)
    
    # 将list转回numpy array
    result_dict = {}
    for key, value in loaded_dict.items():
        result_dict[key] = np.array(value)
    
    return result_dict

# 使用示例:
# 保存数据
# save_estimated_positions(estimated_AP_positions)

# # 读取数据
# loaded_positions = load_estimated_positions()