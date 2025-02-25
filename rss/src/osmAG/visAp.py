import numpy as np
import matplotlib.pyplot as plt

class RSSIVisualizer:
    def __init__(self, ap_to_position, ap_level, target_ap_list, rssi_0_opt, n_opt, polygon_edges):
        self.ap_to_position = ap_to_position
        self.ap_level = ap_level
        self.target_ap_list = target_ap_list
        self.rssi_0_opt = rssi_0_opt
        self.n_opt = n_opt
        self.polygon_edges = polygon_edges

    def rssi_to_distance(self, rssi, A, n):
        return 10 ** ((A - rssi) / (10 * n))

    def visualize(self, rssi_vis_level=1):
        for it in self.ap_to_position:
            if self.ap_level[it] == rssi_vis_level:
                
                positions = []
                distance_xy = []
                rssis = []
                for tagap in self.target_ap_list:
                    # print(type(self.target_ap_list[tagap]['level']),type(rssi_vis_level))
                    if it in self.target_ap_list[tagap]['mac'] and self.target_ap_list[tagap]['level'] == rssi_vis_level:
                        # print('save')
                        # distance = self.rssi_to_distance(self.target_ap_list[tagap]['mac'][it], A=self.rssi_0_opt, n=self.n_opt)
                        # if distance**2 - (3.2)**2 > 0:
                        #     distance_xy.append(np.sqrt(distance**2 - (3.2)**2))
                        positions.append(tagap)
                        rssis.append(self.target_ap_list[tagap]['mac'][it])

                fig_rssi, ax_rssi = plt.subplots()

                # Plot the Polygon
                for edge in self.polygon_edges[str(rssi_vis_level)]:
                    edge_x, edge_y = edge.xy
                    ax_rssi.plot(edge_x, edge_y, linewidth=1, linestyle='solid')

                ax_rssi.scatter(self.ap_to_position[it][0], self.ap_to_position[it][1], color='blue', s=100, marker='o', label='True Position')

                if len(positions) >= 3:
                    for pos in positions:
                        if it in self.ap_to_position.keys():
                            ax_rssi.scatter(pos[0], pos[1], color='red', marker='x', label='Center')
                            ax_rssi.text(pos[0], pos[1], f"{self.target_ap_list[pos]['mac'][it]}", fontsize=8, ha='right')  # annotate each dot with RSSI value

                    ax_rssi.set_aspect('equal')
                    
                    plt.savefig(f"/home/maxu/OsmAG_wifi/src/rss/fig/{rssi_vis_level}F/{it}.pdf", format='pdf')  
  
  
  
  
  
  # for it in target_ap_mac:
    
    # rssi_vis_level = 1
    # for it in ap_to_position:
        
    #     if  ap_level[it] == rssi_vis_level :
            
    #         positions = []
    #         distance_xy = []
    #         rssis = []
    #         for tagap in target_ap_list:
   
    #             if it in target_ap_list[tagap] and target_ap_list[tagap]['level'] == str(rssi_vis_level):
      
    #                 if rssi_to_distance(target_ap_list[tagap][it], A = rssi_0_opt, n=n_opt)**2-(3.2)**2 > 0 : 
    #                     distance_xy.append(np.sqrt(rssi_to_distance(target_ap_list[tagap][it], A = rssi_0_opt,n=n_opt)**2-(3.2)**2))
    #                     positions.append(tagap)
    #                     rssis.append(target_ap_list[tagap][it])
              
    #         fig_rssi, ax_rssi = plt.subplots()

    #         # Plot the Polygon
    #         for edge in polygon_edges[str(rssi_vis_level)]:
    #                 edge_x, edge_y = edge.xy
    #                 ax_rssi.plot(edge_x, edge_y, linewidth=1, linestyle='solid')
            
    #         ax_rssi.scatter(ap_to_position[it][0], ap_to_position[it][1], color='blue', s=100, marker='o', label='True Position')
        
    #         if len(positions) >= 3 and len(distance_xy) >= 3:
                
    #             for pos, dist in zip(positions, distance_xy):
                  
    #                 if it in ap_to_position.keys():
    #                     ax_rssi.scatter(pos[0], pos[1], color='red', marker='x', label='Center')
    #                     ax_rssi.text(pos[0], pos[1], f'{target_ap_list[pos][it]}', fontsize=8, ha='right')  # annotate each dot with RSSI value

                
    #             ax_rssi.set_aspect('equal')
    #             plt.savefig(f"/home/maxu/OsmAG_wifi/src/rss/fig/{rssi_vis_level}F/{it}.pdf", format='pdf')             
        
        
        
    # rssi_vis_level = 3
    # for it in ap_to_position:
        
    #     if  ap_level[it] == rssi_vis_level :
            
    #         positions = []
    #         distance_xy = []
    #         pos_levels = []  
    #         rssis = []
    #         for tagap in target_ap_list:
                
    #             # if target_ap_list[tagap]['level'] != str(rssi_vis_level):
    #             #     print(f"orgin level: {target_ap_list[tagap]['level']}  new level: {rssi_vis_level}")
   
    #             if it in target_ap_list[tagap]:
      
    #                 if rssi_to_distance(target_ap_list[tagap][it], A = rssi_0_opt, n=n_opt)**2-(3.2)**2 > 0 : 
    #                     distance_xy.append(np.sqrt(rssi_to_distance(target_ap_list[tagap][it], A = rssi_0_opt,n=n_opt)**2-(3.2)**2))
    #                     positions.append(tagap)
    #                     rssis.append(target_ap_list[tagap][it])
    #                     pos_levels.append(target_ap_list[tagap]['level'])
              
    #         print(f"pos_levels: {pos_levels}")  
    #         fig_cross = plt.figure(figsize=(10, 7))
    #         ax_cross = fig_cross.add_subplot(111, projection='3d')

    #         # Plot the Polygon
    #         for polygon, poly_level in polygons:
    #             x, y = polygon.exterior.xy
    #             zs = np.ones_like(x) * 3.2 * float(poly_level)
    #             ax_cross.plot(x, y, zs)

            
    #         ax_cross.scatter(ap_to_position[it][0], ap_to_position[it][1], rssi_vis_level * 3.2, color='blue', s=100, marker='o', label='True Position')
        
    #         if len(positions) >= 3 and len(distance_xy) >= 3:
                
    #             for pos, rssi, pos_level in zip(positions, rssis, pos_levels):
                    
                   
    #                 if it in ap_to_position.keys():
    #                     print(f"orgin level: {target_ap_list[tagap]['level']}  new level: {rssi_vis_level}")
    #                     ax_cross.scatter(pos[0], pos[1], float(pos_level) * 3.2, color='red', marker='x', label='Center')
    #                     ax_cross.text(pos[0], pos[1], float(pos_level) * 3.2, f'{rssi}', fontsize=8, ha='right')
            
    #             # plt.show()
    #             # 设置背景颜色为完全透明
    #             ax_cross.set_facecolor((0, 0, 0, 0))
    #             ax_cross.view_init(elev=20, azim=-96)  # 例如，设置仰角为20度，方位角为30度

    #             # ax.set_facecolor('white')

    #             # 去除栅格线
    #             ax_cross.grid(False)

    #             # 去除坐标轴上的刻度线和标签
    #             # ax.set_xticks([])
    #             # ax.set_yticks([])
    #             # ax.set_zticks([])
    #             ax_cross.set_axis_off()
    #             if len(set(pos_levels)) > 1:
    #                 plt.savefig(f"/home/maxu/OsmAG_wifi/src/rss/fig/{rssi_vis_level}F_cross/{it}.pdf", format='pdf')
    #             # plt.savefig(f"/home/maxu/OsmAG_wifi/src/rss/fig/{rssi_vis_level}F_cross/{it}.pdf", format='pdf')             
        
