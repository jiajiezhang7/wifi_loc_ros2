from collections import Counter
import numpy as np
from shapely.geometry import LineString
from pyproj import Geod
import math
import os
import shutil
import json

def filter_top_rssi(target_ap, rssi_num):
    target_ap_list = {}
    for tagap in target_ap:
        for mac in target_ap[tagap]['mac']:
            counter = Counter(target_ap[tagap]['mac'][mac])
            filtered_items = {key: count for key, count in counter.items()}
            max_count = max(filtered_items.values())
            if max_count < rssi_num:
                continue
            top_keys = [key for key, count in filtered_items.items() if count == max_count]
            top_one = int(sum(top_keys) / len(top_keys))
            
            if tagap not in target_ap_list:
                target_ap_list[tagap] = target_ap_list.get(tagap, {})
                target_ap_list[tagap]['mac'] = {}
                target_ap_list[tagap]['level'] = target_ap[tagap]['level']
                
            target_ap_list[tagap]['mac'][mac] = top_one
            
    return target_ap_list    

def calculate_distance(lat1, lon1, lat2, lon2, ap_level, target_ap_level):
    geod = Geod(ellps='WGS84')
    _, _, distance = geod.inv(lon1, lat1, lon2, lat2)
    c = math.sqrt(distance**2 + (abs(ap_level-target_ap_level) + 3.2)**2)
    return c

def process_rssi_data(target_ap_list, ap_to_position, polygon_edges, ap_level, learn_level):
    lines = {}
    learn_no_wall_dis_rss = {'dis': [], 'rssi': []}
    learn_wall_dis_rss = {'dis': [], 'rssi': []}
    
    for tagap in target_ap_list:
        for it in ap_to_position:
            if target_ap_list[tagap]['level'] == ap_level[it]:
                line = LineString([tagap, ap_to_position[it]])

                flag = 1
                for edge in polygon_edges[learn_level]:
                    if line.intersects(edge):
                        flag = 0

                if it in target_ap_list[tagap]['mac']:
                    distance = calculate_distance(ap_to_position[it][1], ap_to_position[it][0], 
                                               tagap[1], tagap[0], ap_level[it], target_ap_list[tagap]['level'])
                    ap_rssi = target_ap_list[tagap]['mac'][it]
                    
                    if line not in lines:
                        lines[line] = {'rssi': 0, 'dis': 0}
                    lines[line]['rssi'] = ap_rssi
                    lines[line]['dis'] = distance

                    learn_no_wall_dis_rss['dis'].append(distance)
                    learn_no_wall_dis_rss['rssi'].append(ap_rssi)
                    
                    if flag:
                        learn_wall_dis_rss['dis'].append(distance)
                        learn_wall_dis_rss['rssi'].append(ap_rssi)
                        
    return lines, learn_no_wall_dis_rss, learn_wall_dis_rss

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
    
    edge_normal = edge_normal / magnitude_normal
    
    return angle_deg, edge_normal

def process_wall_line(lines, polygon_edges, learn_level):
    intersections = []
    nums = []
    eq = {}
    
    for line in lines:
        if isinstance(line, LineString):
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
            
            nums.append(len(line_intersections))
        else:
            first, second = line.split('-')
            if line not in eq:
                eq[line] = {'dis': [], 'rssi': [], 'at_val': 0, 'angle':[], 'bais': 0}
                
            eq[line]['dis'].append(lines[line]['dis'])
            eq[line]['rssi'].append(lines[line]['rssi'])
            eq[line]['angle'].append(0)  # Default angle for non-LineString cases
    
    return intersections, eq, nums

def rssi_to_distance(rssi, A=-28.879257951315253, n=2.6132845414003243):
    return 10 ** ((A - rssi) / (10 * n))

def clear_directory(directory):
    if os.path.exists(directory):
        for filename in os.listdir(directory):
            file_path = os.path.join(directory, filename)
            if os.path.isdir(file_path):
                shutil.rmtree(file_path)
            else:
                os.remove(file_path)
        print(f"Directory cleared: {directory}")
    else:
        print(f"Directory {directory} does not exist")

def calculate_distance_uncertainty(rssi_0, n, rssi_predicted, sigma_rssi_0, sigma_n):
    """
    Calculate the uncertainty in distance estimation.
    
    Parameters:
    - rssi_0: float, fitted rssi_0 value
    - n: float, fitted n value
    - rssi_predicted: float, predicted RSSI value
    - sigma_rssi_0: float, uncertainty in rssi_0 (standard deviation)
    - sigma_n: float, uncertainty in n (standard deviation)
    
    Returns:
    - sigma_distance: float, uncertainty in distance
    """
    d = rssi_to_distance(rssi_predicted, rssi_0, n)
    ln10 = np.log(10)
    
    # Partial derivatives
    dd_drssi0 = -d * ln10 / (10 * n)
    dd_dn = -d * ln10 * (rssi_0 - rssi_predicted) / (10 * n * n)
    
    # Error propagation
    sigma_distance = np.sqrt((dd_drssi0 * sigma_rssi_0)**2 + (dd_dn * sigma_n)**2)
    
    return sigma_distance

def save_estimated_positions(estimated_positions, filename=None):
    """Save estimated AP positions to a JSON file"""
    if filename is None:
        filename = os.path.join(os.path.dirname(__file__), 'estimated_positions.json')
    
    # Convert data to JSON serializable format
    serializable_positions = {}
    for key, value in estimated_positions.items():
        if isinstance(key, tuple):
            str_key = f"{key[0]},{key[1]}"
        else:
            str_key = str(key)
        serializable_positions[str_key] = value
    
    with open(filename, 'w') as f:
        json.dump(serializable_positions, f)

def load_estimated_positions(filename=None):
    """加载估计的AP位置，按优先级在多个位置查找文件"""
    import os
    import json
    import numpy as np
    
    # 如果提供了明确的文件名，直接使用
    if filename is not None:
        if os.path.exists(filename):
            try:
                with open(filename, 'r') as f:
                    estimated_positions = json.load(f)
                    # 将列表转换回numpy数组
                    for key, value in estimated_positions.items():
                        if isinstance(value, list):
                            estimated_positions[key] = np.array(value)
                    return estimated_positions
            except Exception as e:
                print(f"警告：从指定路径 {filename} 加载文件时出错: {str(e)}")
    
    # 可能的文件位置列表
    current_dir = os.path.dirname(os.path.abspath(__file__))
    possible_locations = [
        os.path.join(current_dir, 'estimated_positions.json'),  # 当前目录
        os.path.join(os.path.dirname(current_dir), 'data', 'estimated_positions.json'),  # wifi_loc/data 目录
        os.path.join(os.path.dirname(os.path.dirname(current_dir)), 'data', 'estimated_positions.json'),  # 包根目录的data
        '/home/jay/wifi_ws/src/wifi_loc/data/estimated_positions.json'  # 完整路径
    ]
    
    # 尝试每个位置
    for path in possible_locations:
        if os.path.exists(path):
            try:
                print(f"找到估计位置文件: {path}")
                with open(path, 'r') as f:
                    estimated_positions = json.load(f)
                    # 将列表转换回numpy数组
                    for key, value in estimated_positions.items():
                        if isinstance(value, list):
                            estimated_positions[key] = np.array(value)
                    return estimated_positions
            except Exception as e:
                print(f"警告：从 {path} 加载文件时出错: {str(e)}")
    
    # 如果所有位置都找不到文件，返回空字典
    print("警告：在所有位置都找不到估计位置文件，将使用空字典")
    return {}
