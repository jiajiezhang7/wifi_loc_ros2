import xml.etree.ElementTree as ET
import matplotlib.pyplot as plt
from shapely.geometry import Polygon, Point, LineString
from shapely.ops import nearest_points
import osmium
# import geopandas as gpd
import matplotlib.cm as cm  # Import colormap
import numpy as np
from scipy.optimize import least_squares

from pyswarm import pso  # 导入粒子群优化库
from scipy.optimize import curve_fit
from ipywidgets import interact, IntSlider
from geopy.distance import geodesic
from geopy.distance import distance
from geopy.point import Point
from scipy.optimize import minimize
import math
import geopy.distance
from rss.msg import RssData, RssDatum
import seaborn as sns
import copy
import logging
import os
# 读取pickle文件
from collections import Counter
from pyproj import Geod
from osmAG.colorlogger import colorlogger
from osmAG.Xmlparser import OsmDataParser
from osmAG.util import filter_top_rssi
from osmAG.util import calculate_distance
from osmAG.util import process_rssi_data
from osmAG.util import calculate_angle_and_normal
from osmAG.util import process_wall_line
from osmAG.opter import RSSIOptimizer
from osmAG.opter import PointEstimator
from osmAG.util import rssi_to_distance
from osmAG.util import clear_directory
from osmAG.util import calculate_distance_uncertainty
from osmAG.visAp import RSSIVisualizer
from osmAG.util import save_estimated_positions

if __name__ == "__main__":
       
    # Load and parse the OSM file
    # osm_file = '/home/maxu/OsmAG_wifi/src/rss/map/shanghaitech_d2.osm'  # Adjust the path to your uploaded file
    
    
    logger = colorlogger('/home/maxu/OsmAG_wifi/src/rss/log/', log_name='log.txt')

    osm_file_path = '/home/maxu/OsmAG_wifi/src/rss/map/shanghaitech_d2_1_2F_3F.osm'

    # Create an instance of the OsmDataParser
    parser = OsmDataParser(osm_file_path)

    # Parse the OSM file
    parser.parse()

    # Retrieve the parsed data
    ap_to_position, ap_level, target_ap, way_data, all_mac = parser.get_data()
    
    # for it in target_ap:
    #     for mac in target_ap[it]['mac']:
    #         logger.info(f"mac: {mac} , one loction num of mac : {target_ap[it]['mac'][mac]}")
    
    # target_ap
    sorted_mac = sorted(all_mac)

    # Print the sorted MAC addresses
    # for mac in sorted_mac:
    #     print(f"mac: {mac}")# print(f"all_mac: {all_mac}")
    # input("Press Enter to continue...")


    polygons = []
    for way , way_level in way_data:
        if len(way) > 2 and Polygon(way).is_valid:
            poly = Polygon(way)
            polygons.append((poly, way_level))

    # 提取每个多边形的边界并转换为线段
    polygon_edges = {'1':[],'2':[], '3':[]}
    for polygon, poly_level in polygons:
        exterior_coords = list(polygon.exterior.coords)
        for i in range(len(exterior_coords) - 1):
            edge = LineString([exterior_coords[i], exterior_coords[i + 1]])
            polygon_edges[poly_level].append(edge)
        

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    # Plot the Polygon
    for polygon, poly_level in polygons:
        x, y = polygon.exterior.xy
        zs = np.ones_like(x) * 3.2 * float(poly_level)
        ax.plot(x, y, zs)

    # # Reference Points
    # tagap_levels = np.array([float(target_ap[tagap]['level']) * 3.2 for tagap in target_ap])
    # tagap_coords = np.array([[tagap[0], tagap[1], level] for tagap, level in zip(target_ap, tagap_levels)])
    # ax.scatter(tagap_coords[:, 0], tagap_coords[:, 1], tagap_coords[:, 2], color='red', s=20, marker='o', label='Reference Point')

    # # AP Points
    # ap_coords = np.array([[ap_to_position[it][0], ap_to_position[it][1], ap_level[it] * 3.2] for it in ap_to_position])
    # colors = ['blue' for _ in ap_coords]  # Assuming all AP points are blue for simplicity
    # ax.scatter(ap_coords[:, 0], ap_coords[:, 1], ap_coords[:, 2], color=colors, s=30, marker='o', label='AP')
        
        
    # # 设置背景颜色为完全透明6
    # ax.set_facecolor((0, 0, 0, 0))
    # ax.set_facecolor('white')

    # # 去除栅格线
    # ax.grid(False)

    # # 去除坐标轴上的刻度线和标签
    # ax.set_xticks([])
    # ax.set_yticks([])
    # ax.set_zticks([])
    # ax.set_axis_off()
    # ax.legend()
    # # plt.savefig('/home/maxu/Desktop/fir/visualization.pdf')
    # plt.show()
    # input("Press Enter to continue...") 
    

    target_ap_list = filter_top_rssi(target_ap, 15)
          
          
    learn_level =  '1'     
    # line : ap to rp
    lines, learn_no_wall_dis_rss, learn_wall_dis_rss = process_rssi_data(target_ap_list, ap_to_position, polygon_edges, ap_level, learn_level)
    
    # fig2, ax2 = plt.subplots()

    # ax2.scatter(learn_no_wall_dis_rss['rssi'], learn_no_wall_dis_rss['dis'])

    # ax2.set_title('Distance vs RSSI')
    # ax2.set_xlabel('Distance')
    # ax2.set_ylabel('RSSI')
    # plt.show()
    
    # intersections is line with wall
    intersections, eq, nums = process_wall_line(lines, polygon_edges, learn_level)
      
    logger.info(f"Number of intersections: {set(nums)}")    
    
    
    with_wall_optimizer = RSSIOptimizer(learn_wall_dis_rss, eq)
    with_wall_result, sigma_rssi_0, sigma_n = with_wall_optimizer.optimize_no_wall()
    with_wall_rssi_0_opt, with_wall_n_opt = with_wall_result.x
    
    # print(f"RSSI_0: {with_wall_rssi_0_opt}, n: {with_wall_n_opt}")
    optimizer = RSSIOptimizer(learn_no_wall_dis_rss, eq)
    no_wall_result, sigma_rssi_0, sigma_n = optimizer.optimize_no_wall()
    rssi_0_opt, n_opt = no_wall_result.x
    # with_wall_rssi_0_opt, with_wall_n_opt = no_wall_result.x
    print(f"RSSI_0: {rssi_0_opt}, n: {n_opt}")
    eq,ave_val = optimizer.optimize_wall_attenuation(no_wall_result.x[0], no_wall_result.x[1])
    print(f"ave_val: {ave_val}")
    # input("Press Enter to continue...")
    #print(len(eq.keys()))    35
    sample_max = 0
    for edge in eq:
        if len(eq[edge]['rssi']) > sample_max:
            sample_max = len(eq[edge]['rssi'])
    
    print(f"sample_max : {sample_max} {sample_max * 0.1}")
    
    fig2, ax2 = plt.subplots()
    colors = plt.cm.jet(np.linspace(0, 1, len(polygon_edges[learn_level])))

    for edge in polygon_edges[learn_level]:
        edge_x, edge_y = edge.xy
        ax2.plot(edge_x, edge_y, linewidth=1, linestyle='solid')
      
                                        

    colors = plt.cm.jet(np.linspace(0, 1, len(intersections)))

    # Plot the intersections (edges) with different colors
    for edge, color in zip(intersections, colors):
        edge_x, edge_y = edge.xy
        ax2.plot(edge_x, edge_y, color=color, linewidth=1, linestyle='solid')
        ax2.text(edge.centroid.x, edge.centroid.y, f'{eq[edge]["at_val"]:.2f}', fontsize=10, ha='right')  # annotate each dot with RSSI value
        # ax2.text(edge.centroid.x, edge.centroid.y, f'{len(eq[edge]["dis"])}', fontsize=10, ha='right')  # annotate each dot with RSSI value
    
    # for it in ap_to_position:
    #     if ap_level[it] == int(learn_level):
    #         ax2.scatter(ap_to_position[it][0], ap_to_position[it][1], color='red', s=20, marker='o')
    
    
    # for target in target_ap_list:
    #     # print(type(target_ap_list[target]['level']),type(learn_level))
    #     if target_ap_list[target]['level'] == int(learn_level):
    #         ax2.scatter(target[0], target[1], color='blue', s=20, marker='o')
    #         # ax2.text(target[0], target[1], f'{target_ap_list[target]["mac"]}', fontsize=8, ha='right')
    
    
    # ax2.legend()
    # ax2.set_aspect('equal')
    # # plt.savefig(f"/home/maxu/OsmAG_wifi/src/rss/fig/000.pdf", format='pdf')
    # plt.show() 
    # input("Press Enter to continue...")
    
    
    
    # clear_directory("/home/maxu/OsmAG_wifi/src/rss/fig/1F/")  
    # visualizer = RSSIVisualizer(ap_to_position, ap_level, target_ap_list, rssi_0_opt, n_opt, polygon_edges)
    # visualizer.visualize(rssi_vis_level=1)

    # clear_directory("/home/maxu/OsmAG_wifi/src/rss/fig/2F/")  
    # visualizer = RSSIVisualizer(ap_to_position, ap_level, target_ap_list, rssi_0_opt, n_opt, polygon_edges)
    # visualizer.visualize(rssi_vis_level=2)
    
    # clear_directory("/home/maxu/OsmAG_wifi/src/rss/fig/3F/")  
    # visualizer = RSSIVisualizer(ap_to_position, ap_level, target_ap_list, rssi_0_opt, n_opt, polygon_edges)
    # visualizer.visualize(rssi_vis_level=3)

    positions_xy = []
    lab2 = 0
    item = 0
    # two_dy , two_dx = central_node_coords
    err_distance = []
  
    distance_error = []
    clear_directory("/home/maxu/OsmAG_wifi/src/rss/fig/3D_result/")
    clear_directory("/home/maxu/OsmAG_wifi/src/rss/fig/2D_result/")
    total_num = 0
    traverse_num = 0
    not_traverse_num = 0
    loc_level = 2
    iter_num_total = 20
    err_distance = []
    err_distance_without_wall = []  
    estimated_positions = []
    true_positions = []
    distance_uncertainty = []
    mac_list = ['9c:b2:e8:92:94:60','9c:b2:e8:92:94:61','9c:b2:e8:92:94:62','9c:b2:e8:92:94:63','9c:b2:e8:92:94:65']
    
    
    estimated_AP_positions = {}
    # 过滤掉不知道真值的AP
    for it in ap_to_position:
        
        # if  ap_level[it] == loc_level:
        if ap_level[it] != 1 :   # and it == "9C:B2:E8:92:70:C0".lower()
         
            positions = []
            distance_xy = []
            rssis = []
            target_ap_level = []
            for tagap in target_ap_list :
                # print(type(tagap))
                
                if it in target_ap_list[tagap]['mac']:
      
                    # if rssi_to_distance(target_ap_list[tagap]['mac'][it], A = rssi_0_opt, n=n_opt)**2-(3.2)**2 > 0 : 
                        # distance_xy.append(np.sqrt(rssi_to_distance(target_ap_list[tagap]['mac'][it], A = rssi_0_opt,n=n_opt)**2-(3.2)**2))
                        distance_xy.append(calculate_distance(ap_to_position[it][1], ap_to_position[it][0], tagap[1], tagap[0], ap_level[it], target_ap_list[tagap]['level']))
                        positions.append([tagap[0], tagap[1], 3.2 * target_ap_list[tagap]['level']])
                        target_ap_level.append(target_ap_list[tagap]['level'])
                        rssis.append(target_ap_list[tagap]['mac'][it])
              
            itr_num = 1            
            if len(positions) >= 3 :
                
                # 确定AP的楼层
                counter = Counter(target_ap_level)
                filtered_items = {key: count for key, count in counter.items()} #if count > rssi_num }
                one_ap_level = max(filtered_items.values())
                
                
                floor = f"1-2"
                print(len(set(target_ap_level)))
                
                
                initial_lat = np.mean([pos[0] for pos in positions])
                initial_lon = np.mean([pos[1] for pos in positions])
                initial_alt = np.mean([pos[2] for pos in positions])
                
                initial_guess = [initial_lat, initial_lon, initial_alt]
                # print(f"Initial guess: {initial_guess}") # Initial guess: [121.59106583794335, 31.179509175523336, 6.400000000000001]
               
                # 已知的距离
                # distances = np.array([rssi_to_distance(rssi, A = with_wall_rssi_0_opt, n = with_wall_n_opt) for rssi in rssis])
           
                distances = np.array([rssi_to_distance(rssi, A = rssi_0_opt, n = n_opt) for rssi in rssis])

                # polygons 的作用是防止AP出楼层
                estimator = PointEstimator(positions, distances, polygons)
                result_one = estimator.estimate_point(initial_guess=initial_guess)
                
                # 可视化第一次结果
                fig_fl3, ax_fl3 = plt.subplots()

                # Plot the Polygon
                for edge in polygon_edges[str(3)]:
                        edge_x, edge_y = edge.xy
                        ax_fl3.plot(edge_x, edge_y, linewidth=1, linestyle='solid')
                
                ax_fl3.scatter(result_one.x[0], result_one.x[1], c='green', s=30, marker='o', label='Estimated Position')
                ax_fl3.text(result_one.x[0], result_one.x[1], f"{itr_num}", fontsize=8, ha='right')  # annotate each dot with RSSI value
                ax_fl3.scatter(ap_to_position[it][0], ap_to_position[it][1], color='blue', s=100, marker='o', label='True Position')
                
                # ax_fl3.set_aspect('equal')
                # plt.show()

               
               
                # temp_rssis = copy.deepcopy(rssis)
                for iter_num in range(1, iter_num_total):
                    
                    intersection_edge = []
                    at_val = []
                    temp_rssis = copy.deepcopy(rssis)
                    flags = np.zeros(len(positions))
                    for it_num, pos_ap in enumerate(positions):
                        
                        # if one_ap_level == target_ap_level[it_num]:
                        
                            # 提取距离RP 最近的边
                            ap_line = LineString([(pos_ap[0],pos_ap[1]), (result_one.x[0], result_one.x[1])])
                            closest_edge = None
                            closest_distance = float('inf')
                            for edge in polygon_edges[str(loc_level)]:
                                if ap_line.intersects(edge):
                                
                                    ap_intersection = ap_line.intersection(edge)                                
                                    ap_intersection_line_distance = calculate_distance(pos_ap[1],pos_ap[0], ap_intersection.y,ap_intersection.x,0,0) #y('d', [31.17944676845, 31.17951629596])
                                    
                                    if ap_intersection_line_distance < closest_distance:
                                        closest_distance = ap_intersection_line_distance
                                        closest_edge = edge
                                        
                            # if closest_edge != None:            
                            #     line_closest_distance = float('inf')
                            #     closest_line = closest_edge
                            #     for edge in polygon_edges[learn_level]:
                            #         line_distance = closest_line.distance(edge)  
                            #         if line_distance < line_closest_distance:
                            #             closest_edge = edge
                            #             line_closest_distance = line_distance
                            #             print(f"line_distance: {line_distance}")
                                
                                    
                            if closest_edge in eq:
                                flags[it_num] = 1
                                intersection_edge.append(closest_edge)
                                at_val.append(eq[closest_edge]['at_val'])
                                if type(closest_edge) != str:
                                    angle, normal = calculate_angle_and_normal(ap_line, closest_edge)
                                if eq[closest_edge]['at_val'] > 0:
                                    temp_rssis[it_num] = temp_rssis[it_num] + (eq[closest_edge]['at_val']) # / np.abs(np.cos(np.radians(angle)) ) # + eq[closest_edge]['bais']
                                else:
                                    temp_rssis[it_num] = temp_rssis[it_num] - (eq[closest_edge]['at_val'])  #/ np.abs(np.cos(np.radians(angle)) ) # - eq[closest_edge]['bais']
                                # temp_rssis[it_num] = temp_rssis[it_num] - ave_val
                        # else:
                        #     continue
                        #     print(eq[floor]['at_val'])
                        #     if eq[floor]['at_val'] > 0:
                        #         temp_rssis[it_num] = temp_rssis[it_num] + (eq[floor]['at_val']) # / np.abs(np.cos(np.radians(angle)) ) # + eq[closest_edge]['bais']
                        #     else:
                        #         temp_rssis[it_num] = temp_rssis[it_num] - (eq[floor]['at_val'])  #/ np.abs(np.cos(np.radians(angle)) ) # - eq[closest_edge]['bais']     
                    
                    
                    filter_dis = []
                    filter_pos = []
                    err = []
                    err_with_wall = []
                    err_without_wall = []
                    err_rssi = []  
                    err_rssi_without_wall = [] 
                    for it_num, rssi in enumerate(temp_rssis):
                        dis = rssi_to_distance(rssi, A = rssi_0_opt, n = n_opt)
                        if flags[it_num] == 1 and dis < 40 :
                            err_rssi.append(rssi)
                            filter_dis.append(dis)
                            filter_pos.append(positions[it_num])
                            err.append(distance_xy[it_num])
                            err_with_wall.append({distance_xy[it_num], dis})  
                        elif flags[it_num] == 0 and dis < 20 :
                            err_rssi_without_wall.append(rssi)
                            err_rssi.append(rssi)
                            filter_dis.append(dis)
                            filter_pos.append(positions[it_num])
                            err.append(distance_xy[it_num])
                            # err_without_wall.append({distance_xy[it_num], dis})
                            err_without_wall.append(abs(distance_xy[it_num] - dis))
                        
                            
                    # distances = np.array([rssi_to_distance(rssi, A = rssi_0_opt, n = n_opt) for rssi in temp_rssis])
                    # estimator = PointEstimator(positions, distances,polygons)
                    if len(filter_dis) < 3:
                        continue
                        input(len(filter_dis))
                        
                        
                    estimator = PointEstimator(filter_pos, filter_dis ,polygons)
                
                    # 估计待确定点的坐标
                    result = estimator.estimate_point(initial_guess=initial_guess)
                    
                    error = geodesic((result.x[1], result.x[0]), (result_one.x[1], result_one.x[0])).meters
                    result_one = result  
                    
                    itr_num += 1
                    # ax_fl3.scatter(result.x[0], result.x[1], c='green', s=30, marker='o', label='Estimated Position')
                    # ax_fl3.text(result.x[0], result.x[1], f"{itr_num}", fontsize=8, ha='right')  # annotate each dot with RSSI value 
                    
                    
                    fig_fl_iter, ax_fl_iter = plt.subplots()

                    # Plot the Polygon
                    for edge in polygon_edges[str(3)]:
                            edge_x, edge_y = edge.xy
                            ax_fl_iter.plot(edge_x, edge_y, linewidth=1, linestyle='solid')
                            
                    for ap_tag, dis in zip(filter_pos,filter_dis):
                        ax_fl_iter.scatter(ap_tag[0], ap_tag[1], color='blue', s=30, marker='o', label='True Position')
                        # ax_fl_iter.text(ap_tag[0], ap_tag[1], f"{target_ap_list[(ap_tag[0], ap_tag[1])]['mac'][it]}", fontsize=8, ha='right')  
                        ax_fl_iter.text(ap_tag[0], ap_tag[1], f"{dis:.1f}", fontsize=8, ha='right')  

                    ax_fl_iter.scatter(result.x[0], result.x[1], c='green', s=30, marker='o', label='Estimated Position')
                    ax_fl_iter.scatter(ap_to_position[it][0], ap_to_position[it][1], color='black', s=100, marker='o', label='True Position')
                    ax_fl_iter.text(result.x[0], result.x[1], f"{itr_num}", fontsize=8, ha='right')  # annotate each dot with RSSI value 

                    if error < 0.5:
                        break
             
                    
                if it in ap_to_position.keys():
                    # ax_fl3.scatter(result.x[0], result.x[1], c='red', s=30, marker='o', label='Estimated Position')
                    # ax_fl3.text(result.x[0], result.x[1], f"{itr_num}", fontsize=8, ha='right')  # annotate each dot with RSSI value 
                    
                    
                    
                    # ax_fl3.set_aspect('equal')
                    # ax_fl_iter.set_aspect('equal')
                    # plt.show()
                    
                    estimated_AP_positions[it] = [result.x[0], result.x[1], result.x[2]]
                    print(f"Estimated Position: {result.x}")
                    point1 = (result.x[1], result.x[0])  # 注意经纬度顺序，通常是(纬度, 经度)
                    point2 = (ap_to_position[it][1], ap_to_position[it][0])
                    if len(err_without_wall) > 0:
                        err_distance_without_wall.extend(err_without_wall)
                        # print(len(err_without_wall))
                    
                    distance_error.append(abs(np.array(err) - filter_dis))  
                    distance_uncertainty.append(np.array([calculate_distance_uncertainty(rssi_0_opt, n_opt, rssi, sigma_rssi_0, sigma_n) for rssi in err_rssi_without_wall])) 

                    # print(f"distance error: {type(np.array(distance_xy) - distances)}")
                    distance2 = np.sqrt((ap_level[it] * 3.2 -result.x[2])**2 + geodesic(point1, point2).meters**2)  # 输出单位为米
                    err_distance.append(distance2)

                    estimated_positions.append([result.x[0], result.x[1], result.x[2]])
                    true_positions.append([ap_to_position[it][0], ap_to_position[it][1], 3.2 * ap_level[it]])
                    
                    
                    if len(set(target_ap_level)) > 1:
                        traverse_num += 1
                        # print(f"num floor {len(set(target_ap_level))} height err : {ap_level[it] * 3.2 -result.x[2]}")
                        # estimator.visualize_spheres(polygons, target_ap_level, ap_to_position, ap_level, it, result, path = "/home/maxu/OsmAG_wifi/src/rss/fig/3D_result/")
                    if len(set(target_ap_level)) == 1:
                        not_traverse_num += 1
                        # estimator.visualize_2D(polygons, target_ap_level, ap_to_position, ap_level, it, result, distances, temp_rssis, path = "/home/maxu/OsmAG_wifi/src/rss/fig/2D_result/")
                    total_num += 1
                
                else:
                   
                    pass
                    

                
    
    save_estimated_positions(estimated_AP_positions)
    
    estimated_positions_np = np.array(estimated_positions)
    true_positions_np = np.array(true_positions)
    # print(f"Estimated Position: {estimated_positions}")
    # plt.show()
    colors = plt.cm.jet(np.linspace(0, 1, len(estimated_positions_np)))
    
    
    fig_fl, ax_fl = plt.subplots()

    # Plot the Polygon
    for edge in polygon_edges[str(learn_level)]:
            edge_x, edge_y = edge.xy
            ax_fl.plot(edge_x, edge_y, linewidth=1, linestyle='solid')
    
    ax_fl.scatter(estimated_positions_np[:, 0], estimated_positions_np[:, 1], c='red', s=30, marker='o', label='Estimated Position')
    ax_fl.scatter(true_positions_np[:, 0], true_positions_np[:, 1], c='blue', s=30, marker='o', label='True Position')
    
    for est_pos, true_pos in zip(estimated_positions_np, true_positions_np):
        ax_fl.plot([est_pos[0], true_pos[0]], [est_pos[1], true_pos[1]], 'gray', linestyle='--', linewidth=0.5)

    ax_fl.set_aspect('equal')

    ax.scatter(estimated_positions_np[:, 0], estimated_positions_np[:, 1], estimated_positions_np[:, 2], c='red', s=30, marker='o', label='Estimated Position')
    ax.scatter(true_positions_np[:, 0], true_positions_np[:, 1], true_positions_np[:, 2], c='blue', s=30, marker='o', label='True Position')
    
    # 设置背景颜色为完全透明
    ax.set_facecolor((0, 0, 0, 0))
    # ax.set_facecolor('white')

    # 去除栅格线
    ax.grid(False)

    # 去除坐标轴上的刻度线和标签
    # ax.set_xticks([])
    # ax.set_yticks([])
    # ax.set_zticks([])
    ax.set_axis_off()
    ax.legend()
    # plt.savefig('/home/maxu/Desktop/fir/visualization.pdf')
    
    # Total number: 133, Traverse number: 28, Not traverse number: 105
    print(f"Total number: {total_num}, Traverse number: {traverse_num}, Not traverse number: {not_traverse_num}")
    average = sum(err_distance) / len(err_distance)

    # 输出平均数
    print("平均数:", average)
    # plt.show()
   
   
    # # 设置图形的风格
    # sns.set(style="whitegrid")

    # # 绘制直方图
    # plt.figure(figsize=(12, 6))
    # plt.subplot(1, 2, 1)
    # # sns.histplot(err_distance, bins=15)
    # distance_error = np.concatenate(distance_error)
    # sns.histplot(distance_error, bins=15)
    # plt.title('Histogram of Errors')

    sns.set(style="whitegrid")

    # 绘制直方图
    plt.figure(figsize=(12, 6))
    plt.subplot(1, 2, 1)
    # sns.histplot(err_distance, bins=15)
    distance_error = np.concatenate(distance_error)
    
    print(f"distance_error mean: {np.mean(distance_error)}") # distance_error mean: 13.48420400232641

    
    
    hist_data, bins, patches = plt.hist(distance_error, bins=15, alpha=0.6, color='b', edgecolor='black')

    # 计算每个bin的标准差
    bin_centers = 0.5 * (bins[:-1] + bins[1:])  # 计算每个bin的中心
    bin_std = []  # 存储每个bin的标准差

    for i in range(len(bins) - 1):
        # 选择属于当前bin的所有数据
        bin_data = distance_error[(distance_error >= bins[i]) & (distance_error < bins[i+1])]
        # 计算当前bin的数据标准差
        bin_std.append(np.std(bin_data))

    
    print(f"bin_std: {bin_std}")
    # bin_std: [1.7949180522791324, 1.8266479336490407, 2.1095725768217783, 1.8090629060128427, 1.833428306343827, 1.6118996486795953, 1.814716534143821, 1.774642308055855, 1.2550216603305295, 1.2952143952540272, 0.0, 1.7929982112945215, nan, 0.6991886563499062, 0.8073579335287349]

    # 将标准差绘制成误差条
    plt.errorbar(bin_centers, hist_data, yerr=bin_std, fmt='none', ecolor='red', capsize=5)

        
    
    
    # sns.histplot(distance_error, bins=15, stat='count', kde=True)
    # plt.title('Histogram of Errors with Standard Deviation')

    # # 计算每个 bin 的标准差
    # bin_counts, bin_edges = np.histogram(distance_error, bins=15)
    # bin_centers = 0.5 * (bin_edges[1:] + bin_edges[:-1])
    # bin_std = np.std(distance_error)

    # # 绘制标准差
    # for count, x in zip(bin_counts, bin_centers):
    #     plt.errorbar(x, count, yerr=bin_std, fmt='o', color='r')

 

    # 绘制箱线图
    plt.subplot(1, 2, 2)
    sns.boxplot(distance_error)
    # sns.boxplot(err_distance)
    plt.title('Boxplot of Errors')

    # 显示图形
    plt.tight_layout()
    
    
    

    # 绘制直方图
    plt.figure(figsize=(12, 6))
    plt.subplot(1, 2, 1)
    # sns.histplot(err_distance, bins=15)
    err_distance_without_wall = np.array(err_distance_without_wall)
    hist_data, bins, patches = plt.hist(err_distance_without_wall, bins=15, alpha=0.6, color='b', edgecolor='black')

    # 计算每个bin的标准差
    bin_centers = 0.5 * (bins[:-1] + bins[1:])  # 计算每个bin的中心
    bin_std = []  # 存储每个bin的标准差

    for i in range(len(bins) - 1):
        # 选择属于当前bin的所有数据
        bin_data = err_distance_without_wall[(err_distance_without_wall >= bins[i]) & (err_distance_without_wall < bins[i+1])]
        # 计算当前bin的数据标准差
        bin_std.append(np.std(bin_data))

    
    print(f"bin_std: {bin_std}")
    # bin_std: [1.7949180522791324, 1.8266479336490407, 2.1095725768217783, 1.8090629060128427, 1.833428306343827, 1.6118996486795953, 1.814716534143821, 1.774642308055855, 1.2550216603305295, 1.2952143952540272, 0.0, 1.7929982112945215, nan, 0.6991886563499062, 0.8073579335287349]

    # 将标准差绘制成误差条
    plt.errorbar(bin_centers, hist_data, yerr=bin_std, fmt='none', ecolor='red', capsize=5)

        
    
    
    # sns.histplot(distance_error, bins=15, stat='count', kde=True)
    # plt.title('Histogram of Errors with Standard Deviation')

    # # 计算每个 bin 的标准差
    # bin_counts, bin_edges = np.histogram(distance_error, bins=15)
    # bin_centers = 0.5 * (bin_edges[1:] + bin_edges[:-1])
    # bin_std = np.std(distance_error)

    # # 绘制标准差
    # for count, x in zip(bin_counts, bin_centers):
    #     plt.errorbar(x, count, yerr=bin_std, fmt='o', color='r')

 

    # 绘制箱线图
    plt.subplot(1, 2, 2)
    sns.boxplot(err_distance_without_wall)
    # sns.boxplot(err_distance)
    plt.title('Boxplot of Errors')

    # 显示图形
    plt.tight_layout()
    
    
    
  
    

    # indices = range(len(distance_error))  # 假设每个点按顺序排列
    # distance_uncertainty = np.concatenate(distance_uncertainty)
    # # 创建散点图
    # plt.figure(figsize=(10, 6))
    # plt.scatter(indices, distance_error, color='blue', label='Distance Error', alpha=0.7)
    # plt.scatter(indices, distance_uncertainty, color='red', label='Distance Uncertainty', alpha=0.7)

    # # 图表标题和标签
    # plt.title('Distance Error and Uncertainty')
    # plt.xlabel('Index')
    # plt.ylabel('Value')
    # plt.legend()
    # plt.grid(True)
    # plt.show()
    
    
    
    
    
    
    
    # print(f"Error: {err_distance}")     
 
    # input('sldjfkdskl')
    
    
    
    # plt.show()
    
