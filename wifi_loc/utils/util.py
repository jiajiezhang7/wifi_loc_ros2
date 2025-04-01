from collections import Counter
from shapely import Point
from shapely.geometry import LineString
from pyproj import Geod
import math
from geopy.distance import geodesic
import numpy as np
# from torch import le
# 计算AP到交点的距离
def calculate_precise_distance(lon1, lat1, lon2, lat2):
    """
    使用PyProj的Geod类计算两点间的距离
    
    参数:
        lat1, lon1: 第一个点的纬度和经度
        lat2, lon2: 第二个点的纬度和经度
        
    返回:
        距离，单位为米
    """
    # 使用WGS84椭球体
    geod = Geod(ellps='WGS84')
    
    # 计算测地线距离
    _, _, distance = geod.inv(lon1, lat1, lon2, lat2) # 纬度，经度，纬度，经度
    
    return distance

def calculate_distance(lon1, lat1, lon2, lat2, ap_level, target_ap_level):
    # 创建一个Geod对象，用WGS84椭球
    geod = Geod(ellps='WGS84')
    
    # 使用Geod的inverse方法计算两个点之间的距离
    # 返回的结果包括方位角和反方位角，我们只需要距离
    _, _, distance = geod.inv(lon1, lat1, lon2, lat2)
    
    c = math.sqrt(distance**2 + (abs(ap_level-target_ap_level) + 3.2)**2)
    return c
   
def filter_top_rssi(target_ap , rssi_num):
    target_ap_list = {}
    for tagap in target_ap:

        for mac in target_ap[tagap]['mac']:
            counter = Counter(target_ap[tagap]['mac'][mac])
            most_common_n = counter.most_common(1)  
            most_common_element = most_common_n[0][0]   
            most_common_count = most_common_n[0][1]  
            if most_common_count < rssi_num:
                continue
          
            if tagap not in target_ap_list:
                
                target_ap_list[tagap] = target_ap_list.get(tagap, {})
                target_ap_list[tagap]['mac'] = {}
                target_ap_list[tagap]['level'] = target_ap[tagap]['level']
                
            target_ap_list[tagap]['mac'][mac] = most_common_element
            
    return target_ap_list    


def process_rssi_data(target_ap_list, ap_to_position, polygon_edges, ap_level, learn_level):
    lines = {}
    learn_no_wall_dis_rss = {'dis': [], 'rssi': []}
    learn_wall_dis_rss = {'dis': [], 'rssi': []}
    
    for tagap in target_ap_list:
        # 只处理指定楼层的数据
        if target_ap_list[tagap]['level'] != learn_level:
            continue
            
        for mac in ap_to_position:
            # 如果AP和接收点在同一楼层
            if target_ap_list[tagap]['level'] == ap_level[mac]:
                # 创建AP到接收点的线段
                line = LineString([tagap, ap_to_position[mac]])
                
                # 检查线段是否穿墙
                has_wall = False
                for edge in polygon_edges[str(learn_level)]:
                    if line.intersects(edge):
                        has_wall = True
                        break
                
                # 如果该MAC地址在目标AP的MAC列表中
                if mac in target_ap_list[tagap]['mac']:
                    # 计算距离和获取RSSI值
                    distance = calculate_distance(
                        ap_to_position[mac][0], ap_to_position[mac][1], 
                        tagap[0], tagap[1], 
                        ap_level[mac], target_ap_list[tagap]['level']
                    )
                    ap_rssi = target_ap_list[tagap]['mac'][mac]
                    
                    
                    
                    # 添加到无墙数据集
                    if not has_wall:
                        learn_no_wall_dis_rss['dis'].append(distance)
                        learn_no_wall_dis_rss['rssi'].append(ap_rssi)
                    
                    # 如果没有墙，也添加到有墙数据集（这里逻辑可能有问题，应该是 if not has_wall）
                    if has_wall:
                        learn_wall_dis_rss['dis'].append(distance)
                        learn_wall_dis_rss['rssi'].append(ap_rssi)
                        
                        # 记录线段信息
                        if line not in lines:
                            lines[line] = {'rssi': 0, 'dis': 0}
                        lines[line]['rssi'] = ap_rssi
                        lines[line]['dis'] = distance
                else:
                    pass
                    # first, second = sorted([int(target_ap_list[tagap]['level']) , int(ap_level[mac])])
                    # # print(f"first: {first}, second: {second}")
                    # line = LineString([tagap, ap_to_position[mac]])
                    # if mac in target_ap_list[tagap]['mac']:
                    #     distance = calculate_distance(ap_to_position[mac][1], ap_to_position[mac][0], tagap[1], tagap[0], ap_level[mac], target_ap_list[tagap]['level'])
                    #     ap_rssi = target_ap_list[tagap]['mac'][mac]
                    #     if line not in lines:
                    #         lines[f"{first}-{second}"] = {'rssi': 0, 'dis': 0}
                    #     lines[f"{first}-{second}"]['rssi'] = ap_rssi
                    #     lines[f"{first}-{second}"]['dis'] = distance

                    
                    # print(target_ap_list[tagap]['level'] , ap_level[mac])
                    
                    
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
    
    edge_normal = edge_normal / magnitude_normal  # Normalize the normal vector
    
    return angle_deg, edge_normal


def process_wall_line(lines, polygon_edges, learn_level):
    attenuation_edges = []
    nums = []
    learn_parameter = {}
    # line 存储的是所有在真值库里的ap到Rp的线段
    
    for line in lines:
        if type(line) != str:
            line_intersections = []
            for edge in polygon_edges[str(learn_level)]:
                if line.intersects(edge):
                    line_intersections.append(edge)

            if len(line_intersections) == 1:
                
                for edge in line_intersections:
                    if edge not in attenuation_edges:
                        attenuation_edges.append(edge)
                        learn_parameter[edge] = {'dis': [], 'rssi': [], 'at_val': 0, 'angle':[], 'bais': 0}
                        
                    learn_parameter[edge]['dis'].append(lines[line]['dis'])
                    learn_parameter[edge]['rssi'].append(lines[line]['rssi'])
                    angle, normal = calculate_angle_and_normal(line, edge)
                    learn_parameter[edge]['angle'].append(angle)
          
        
            nums.append(len(line_intersections))
        else:
            first, second = line.split('-')
           
            if line not in learn_parameter:
                
                learn_parameter[line] = {'dis': [], 'rssi': [], 'at_val': 0, 'angle':[], 'bais': 0}
                
            learn_parameter[line]['dis'].append(lines[line]['dis'])
            learn_parameter[line]['rssi'].append(lines[line]['rssi'])
            # angle, normal = calculate_angle_and_normal(line, edge)
            learn_parameter[line]['angle'].append(angle)
                    # print(angle)
            # nums.append(1)
    
    return attenuation_edges, learn_parameter, nums

def calculate_polygon_area(polygon):
    """计算多边形的实际地理面积（平方米）"""
    from pyproj import Geod
    geod = Geod(ellps="WGS84")
    
    # 获取多边形的坐标
    coords = list(polygon.exterior.coords)
    
    # 使用测地线计算面积
    area, _ = geod.polygon_area_perimeter(
        lons=[coord[0] for coord in coords],
        lats=[coord[1] for coord in coords]
    )
    
    return abs(area)  # 返回绝对值，因为面积可能为负

def find_largest_polygon(polygons):
    """
    按楼层分别计算面积最大的polygon
    
    参数:
    polygons -- 多个polygon的列表，每个元素为(polygon, poly_level)格式
    
    返回:
    按楼层分组的字典，每层包含最大面积、对应polygon和楼层信息
    """
    # 创建按楼层分组的字典
    level_polygons = {}
    
    # 将多边形按楼层分组
    for polygon, poly_level in polygons:
        if poly_level not in level_polygons:
            level_polygons[poly_level] = []
        level_polygons[poly_level].append(polygon)
    
    # 存储每层最大多边形的结果
    result = {}
    
    # 遍历每个楼层，找出该层中面积最大的多边形
    for level, poly_list in level_polygons.items():
        largest_area = 0
        largest_polygon = None
        
        # 在当前楼层中找出面积最大的多边形
        for polygon in poly_list:
            area = calculate_polygon_area(polygon)
            if area > largest_area:
                largest_area = area
                largest_polygon = polygon
        
        # 将结果存入字典
        result[int(level)] = {
            'area': largest_area,
            'polygon': largest_polygon,
            'level': level
        }
        
    return result



def find_smallest_room_polygon(point, polygons, floor):
    """
    查找包含指定点的最小面积多边形
    
    Args:
        point: 坐标点 [x, y]
        polygons: 多边形字典
        floor: 楼层
    
    Returns:
        dict: 包含最小多边形信息的字典，如果没有找到则返回None
    """

    
        
    point_obj = Point(point[0], point[1])
    candidate_polygons = []
    
    # 将多边形按楼层分组
    for polygon, poly_level in polygons:
        # print(poly_level , floor,type(poly_level), type(floor)) # 2 2 <class 'str'> <class 'int'>
        # print(type(polygon)) <class 'shapely.geometry.polygon.Polygon'>
        if polygon.contains(point_obj) and int(poly_level) == floor:
            
            candidate_polygons.append({
                'polygon': polygon,
                'level': poly_level,
                'area': calculate_polygon_area(polygon),
                'bounds': polygon.bounds  # (minx, miny, maxx, maxy)
            })
        
   
           
    
    # 如果没有找到包含该点的多边形，返回None
    if not candidate_polygons:
        return None
    
    # 按面积排序，返回最小的
    smallest_polygon = min(candidate_polygons, key=lambda x: x['area'])
    return smallest_polygon  

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



