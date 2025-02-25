import numpy as np
from scipy.optimize import minimize, least_squares
from scipy.optimize import curve_fit

class RSSIOptimizer:
    def __init__(self, learn_no_wall_dis_rss, eq):
        self.learn_no_wall_dis_rss = learn_no_wall_dis_rss
        self.eq = eq
        self.d0 = 1.0  # 参考距离 d0

    def signal_model(self, distance, rssi_0,n ):
        return rssi_0 - 10 * n * np.log10(distance)



    def optimize_no_wall(self):
        initial_params = [-40, 2.0]  # 初始猜测的rssi_0和n的值
        distances = np.array(self.learn_no_wall_dis_rss['dis'])
        rssi_measured = np.array(self.learn_no_wall_dis_rss['rssi'])
        no_wall_result = minimize(self.loss_function, initial_params, args=(distances, rssi_measured))
        
        popt, pcov = curve_fit(self.signal_model, distances, rssi_measured)
        rssi_0, n = popt  # 拟合的rssi_0和n
        sigma_rssi_0, sigma_n = np.sqrt(np.diag(pcov))  # 参数的不确定性（标准差）

        print(f"拟合的 rssi_0: {rssi_0}, 不确定性: {sigma_rssi_0}")
        print(f"拟合的 n: {n}, 不确定性: {sigma_n}")
        
        return no_wall_result, sigma_rssi_0, sigma_n
    
    def loss_function(self, params, distances, rssi_measured):
        rssi_0, n = params
        rssi_predicted = rssi_0 - 10 * n * np.log10(distances)
        return np.mean((rssi_predicted - rssi_measured) ** 2)

    def optimize_wall_attenuation(self, L_d0, n):
        ave_val = []
        for eq_key in self.eq:
           
            # if len(self.eq[eq_key]['dis']) <= 1:
            #     continue
            distances = self.eq[eq_key]['dis']
            signal_strengths = self.eq[eq_key]['rssi']
            initial_guess = self.eq[eq_key]['at_val']
            # initial_guess = [self.eq[eq_key]['at_val'], self.eq[eq_key]['bais']]
            angle = self.eq[eq_key]['angle']
          
            result = least_squares(self.residuals, initial_guess, args=(distances, signal_strengths, L_d0, self.d0, n, angle))
            if len(distances) >= 7:
                ave_val.append(result.x[0])
            self.eq[eq_key]['at_val'] = result.x[0]
            # self.eq[eq_key]['bais'] = result.x[1]
            # print(f"Wall attenuation for : {result.x[0]}  bais: {result.x[1]}")
          
        return self.eq, np.mean(ave_val)

    def residuals(self, initial_guess, distances, signal_strengths, L_d0, d0, n, angle):
        """
        定义残差函数
        """
        # w, bais = initial_guess
        w = initial_guess
        res = []
        for i in range(len(distances)):
            # a_i
            # predicted_signal = L_d0 - 10 * n * np.log10(distances[i] / d0) +  ( w / np.abs(np.cos(np.radians(angle))))
            
            # b_i
            predicted_signal = L_d0 - 10 * n * np.log10(distances[i] / d0) + w
            
            # a_i and b_i
            # predicted_signal = L_d0 - 10 * n * np.log10(distances[i] / d0) +  ( w / np.abs(np.cos(np.radians(angle)))) + bais
            res.append(predicted_signal - signal_strengths[i])
        return np.array(res).flatten()

  
   

  
        # distances = eq[eq_key]['dis']
        # signal_strengths = eq[eq_key]['rssi']
        # angle = eq[eq_key]['angle']
        # lb = [-50]  # 下界
        # ub = [0]  # 上界
        # # 使用PSO进行优化
        # xopt, fopt = pso(objective_function, lb, ub, args=(distances, signal_strengths,angle, L_d0, d0, n),swarmsize=100, maxiter=100)
        # print("优化后的墙壁衰减值:", xopt)
        # print("优化的目标函数值:", fopt)
        # eq[eq_key]['at_val'] = xopt[0]
        # ax.text(eq_key.centroid.x, eq_key.centroid.y, f'{xopt[0]:.2f}', fontsize=8, ha='right')  # annotate each dot with RSSI value


import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from geopy.distance import distance
from pyproj import Geod
import math
from scipy.spatial import ConvexHull


class PointEstimator:
    def __init__(self, known_points, distances, polygons):
        """
        初始化PointEstimator类
        
        :param known_points: 已知点的三维坐标列表，格式为 [[lon1, lat1, alt1], [lon2, lat2, alt2], ...]
        :param distances: 每个已知点与待确定点的距离，格式为 [distance1, distance2, ...]
        """
        self.known_points = np.array(known_points)
        self.distances = np.array(distances)
        self.polygons = polygons
        self.min_lon, self.min_lat, self.max_lon, self.max_lat = self.find_min_max_coordinates()
        self.optimization_history = []

    def find_min_max_coordinates(self):
        min_lon, min_lat = float('inf'), float('inf')
        max_lon, max_lat = float('-inf'), float('-inf')

        for polygon, poly_level in self.polygons:
            x, y = polygon.exterior.xy
            min_lon = min(min_lon, min(x))
            max_lon = max(max_lon, max(x))
            min_lat = min(min_lat, min(y))
            max_lat = max(max_lat, max(y))

        return min_lon, min_lat, max_lon, max_lat

    def ap_residuals(self, params):
        lat, lon, height = params
        res = 0
        geod = Geod(ellps='WGS84')
        for (lat_i, lon_i, height_i), _distance in zip(self.known_points, self.distances):
            _, _, distance2 = geod.inv(lat, lon, lat_i, lon_i)
            
            height_err = (height_i - height)**2
            
            d_estimated = math.sqrt(distance2**2 + height_err)
            res += abs(d_estimated - _distance)
        return res

    
    def estimate_point(self, initial_guess=[0, 0, 0]):
        """
        使用最优化算法估计待确定点的坐标
        
        :param initial_guess: 优化的初始猜测坐标，默认为 [0, 0, 0]
        :return: 优化后的三维坐标点，或者如果优化失败，返回None
        """
        self.optimization_history = []  # 清空历史记录
        bounds = [
            (self.min_lon, self.max_lon),  # 第一个变量的边界
            (self.min_lat, self.max_lat),  # 第二个变量的边界
            (initial_guess[2] - 3.2, initial_guess[2] + 1)  # 第三个变量的边界
        ]
        # result = minimize(self.ap_residuals, initial_guess, bounds=bounds,  method='SLSQP') # L-BFGS-B, TNC, or SLSQP, 
        result = least_squares(self.ap_residuals, initial_guess , args=(), bounds=([self.min_lon, self.min_lat, initial_guess[2] - 3.2], [ self.max_lon, self.max_lat, initial_guess[2] + 1]))
        if result.success:
            # print(f"result.message {result.message} result.fun {result.fun}") 
            return result
        else:
            print(result.message)
            print("优化未成功")
            return None


    def _objective(self, x):
        """
        目标函数：计算待确定点和已知点的距离与给定距离的差值的平方和
        
        :param x: 当前待优化的点的三维坐标
        :return: 距离误差平方和
        """
        total_error = 0
        for i, point in enumerate(self.known_points):
            distance_error = np.linalg.norm(x - point) - self.distances[i]
            total_error += distance_error ** 2
        
        print(f"当前坐标: {x}, 误差: {total_error}")
        self.optimization_history.append(x)  # 保存每一步优化的坐标
        return total_error


    
    def _generate_sphere_points(self, center, radius, num_points=100):
        """
        生成球体表面的点（经纬度和高度）
        :param center: 球心的经纬度（lat, lon, alt）
        :param radius: 球体的半径（以米为单位）
        :param num_points: 每个球面的点数
        :return: 生成的球体表面上的经纬度点列表
        """
        lat_center, lon_center, alt_center = center
        u = np.linspace(0, 2 * np.pi, num_points)
        v = np.linspace(0, np.pi, num_points)
        
        latitudes = []
        longitudes = []
        altitudes = []

        for elev in v:
            for azim in u:
                destination = distance(meters=radius * np.sin(elev)).destination((lon_center, lat_center), np.degrees(azim))
                lat_new = destination.latitude
                lon_new = destination.longitude
                alt_new = alt_center + radius * np.cos(elev)
                latitudes.append(lat_new)
                longitudes.append(lon_new)
                altitudes.append(alt_new)

        return np.array([latitudes, longitudes, altitudes]).T

    def visualize_spheres(self, polygons, target_ap_levels, ap_to_position, ap_level, it, result, path, num_points=100):
        """
        可视化以已知点为圆心，以距离为半径的空心球体，并展示相交部分
        """
        
        # if len(set(target_ap_levels)) == 1:
        #     self.visualize_2D(polygons, target_ap_levels, ap_to_position, ap_level, it, result, path)
            
            
        # fig = plt.figure(figsize=(20, 20))
        
        # # 创建四个子图
        # ax1 = fig.add_subplot(221, projection='3d')
        # ax2 = fig.add_subplot(222, projection='3d')
        # ax3 = fig.add_subplot(223, projection='3d')
        # ax4 = fig.add_subplot(224, projection='3d')
        
        # axes = [ax1, ax2, ax3, ax4]
        # views = [(90, -90), (20, 30), (45, 45), (60, 120)]  # 四个不同的视角
        
        # for ax, view in zip(axes, views):
        #     for target_ap_level in set(target_ap_levels):
        #         for polygon, poly_level in polygons:
        #             x, y = polygon.exterior.xy
        #             zs = np.ones_like(x) * 3.2 * (float(poly_level) - 1)
        #             if target_ap_level == int(poly_level):
        #                 ax.plot(x, y, zs)
                        
        #     ax.scatter(self.known_points[:, 0], self.known_points[:, 1], self.known_points[:, 2] - 3.2, c='blue', marker='o')
        #     ax.scatter(ap_to_position[it][0], ap_to_position[it][1], ap_level[it] * 3.2, c='red', marker='o')
        #     ax.scatter(result.x[0], result.x[1], result.x[2], c='green', marker='o')
            
        #     colors = ['r', 'g', 'b', 'y', 'c', 'm', 'orange', 'purple']
        #     sphere_data = []
        #     for i, point in enumerate(self.known_points):
        #         lat, lon, alt = point
        #         radius = self.distances[i]
        #         sphere_points = self._generate_sphere_points(point, radius, num_points)
        #         sphere_data.append((sphere_points, colors[i % len(colors)]))
            
        #     ax.grid(False)
        #     ax.view_init(elev=view[0], azim=view[1])
        #     # ax.set_axis_off()
        #     # ax.set_xlabel('Longitude')
        #     # ax.set_ylabel('Latitude')
        #     # ax.set_zlabel('Altitude')
        #     ax.set_title(f'View {views.index(view) + 1}')
        
        # plt.suptitle('3D Visualization of Spheres with Intersections from Different Angles')
        # print(f"ap_level[it]: {ap_level[it]} result.x: {result.x} target_ap_levels: {target_ap_levels}")
        # # plt.savefig(f"{path}/{it}.pdf", format='pdf')

        # plt.show()
    
        """
        可视化以已知点为圆心，以距离为半径的空心球体，并展示相交部分
        """
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        
        for target_ap_level in set(target_ap_levels):
            
            for polygon, poly_level in polygons:
                x, y = polygon.exterior.xy
                zs = np.ones_like(x) * 3.2 * (float(poly_level) - 1)
                # print(type(target_ap_level), type(poly_level)) # <class 'int'> <class 'str'>

                if target_ap_level == int(poly_level):
                    # ax.plot(x, y, zs, color='r')
                    ax.plot(x, y, zs)
                    
        ax.scatter(self.known_points[: , 0], self.known_points[: , 1 ], self.known_points[: , 2] - 3.2 , c='blue', marker='o')
        ax.scatter(ap_to_position[it][0], ap_to_position[it][1], ap_level[it] * 3.2, c='red', marker='o')
        ax.scatter(result.x[0], result.x[1], result.x[2], c='green', marker='o')
        
        
        colors = ['r', 'g', 'b', 'y', 'c', 'm', 'orange', 'purple']

        sphere_data = []
        for i, point in enumerate(self.known_points):
            lat, lon, alt = point
            radius = self.distances[i]
            sphere_points = self._generate_sphere_points(point, radius, num_points)
            sphere_data.append((sphere_points, colors[i % len(colors)]))
        

        # 绘制所有球体
        for i, (sphere_points, color) in enumerate(sphere_data):
            latitudes = sphere_points[:, 0]
            longitudes = sphere_points[:, 1]
            altitudes = sphere_points[:, 2]
            ax.plot_surface(longitudes.reshape((num_points, num_points)), 
                            latitudes.reshape((num_points, num_points)), 
                            altitudes.reshape((num_points, num_points)), 
                            color=color, alpha=0.1, label=f"Sphere {i+1}")

        # 计算并绘制相交部分
        # intersection_points = self._calculate_intersection_points(sphere_data)
        # if intersection_points.size > 0:
        #     hull = ConvexHull(intersection_points)
        #     for simplex in hull.simplices:
        #         ax.plot(intersection_points[simplex, 0], 
        #                 intersection_points[simplex, 1], 
        #                 intersection_points[simplex, 2], 'k-')

        # 设置坐标轴标签
        # ax.set_facecolor((0, 0, 0, 0))
        ax.grid(False)
        # ax.set_axis_off()
        ax.view_init(elev=15, azim=-90)  # 例如，设置仰角为20度，方位角为30度
        ax.set_xlabel('Longitude')
        ax.set_ylabel('Latitude')
        ax.set_zlabel('Altitude')
        ax.set_title('3D Visualization of Spheres with Intersections')
        print(f"ap_level[it]: {ap_level[it]} result.x: {result.x} target_ap_levels: {target_ap_levels}")
        
        plt.savefig(f"{path}/{it}.pdf", format='pdf')
        # plt.show()
        
    def _calculate_intersection_points(self, sphere_data):
        """
        计算球体相交的部分
        :param sphere_data: 包含球体点和颜色的数据
        :return: 相交部分的点
        """
        intersection_points = []
        for i in range(len(sphere_data)):
            for j in range(i + 1, len(sphere_data)):
                points_i = sphere_data[i][0]
                points_j = sphere_data[j][0]
                for point_i in points_i:
                    for point_j in points_j:
                        if np.linalg.norm(point_i - point_j) < 1e-2:  # 设定一个小阈值来判断点是否相交
                            intersection_points.append(point_i)
        return np.array(intersection_points)
    
    
    def visualize_2D(self, polygons, target_ap_levels, ap_to_position, ap_level, it, result, distances, temp_rssis ,path):
        """
        2D可视化函数
        """
        
        fig_rssi, ax_rssi = plt.subplots()

        # Plot the Polygon
        for polygon, poly_level in polygons:
            if target_ap_levels[0] == int(poly_level):
                # print(121111)poly_level
                x, y = polygon.exterior.xy
                ax_rssi.plot(x, y)
            
        ax_rssi.scatter(ap_to_position[it][0], ap_to_position[it][1], color='red', s=100, marker='o', label='True Position')
        ax_rssi.scatter(result.x[0], result.x[1], color='green', s=100, marker='o', label='Estimated Position')
        for pos, dist, ap_rssi in zip(self.known_points, distances, temp_rssis):
            
            if it in ap_to_position.keys():
                ax_rssi.scatter(pos[0], pos[1], color='blue', marker='o', label='Center')
    
                ax_rssi.text(pos[0], pos[1], f'{int(ap_rssi)}', fontsize=8, ha='right')  # annotate each dot with RSSI value
            
        ax_rssi.set_aspect('equal')
        plt.savefig(f"{path}/{it}_2D_rssi.pdf", format='pdf')             

        
        sigma = 0.66
        fig, ax = plt.subplots()
        for polygon, poly_level in polygons:
            if target_ap_levels[0] == int(poly_level):
                # print(121111)poly_level
                x, y = polygon.exterior.xy
                ax.plot(x, y)
            
        ax.scatter(self.known_points[:, 0], self.known_points[:, 1], c='blue', marker='o')
        ax.scatter(ap_to_position[it][0], ap_to_position[it][1], c='red', marker='o')
        ax.scatter(result.x[0], result.x[1], c='green', marker='o')
        
    
        for pos, dist in zip(self.known_points, distances):
        
            num_points = 100
            angles = np.linspace(0, 2 * np.pi, num_points)
            circle_points = []
            sigma_circle_points = []
            for angle in angles:
                # 计算圆周上当前点的经纬度
                destination = distance(meters=dist).destination((pos[1], pos[0]), np.degrees(angle))
                circle_points.append((destination.latitude, destination.longitude))
                
                sigma_destination = distance(meters=dist * sigma).destination((pos[1], pos[0]), np.degrees(angle))
                sigma_circle_points.append((sigma_destination.latitude, sigma_destination.longitude))
        
            # 将经纬度点转换为numpy数组
            circle_points = np.array(circle_points)
            sigma_circle_points = np.array(sigma_circle_points)
        
            if it in ap_to_position.keys():
                ax.plot(circle_points[:, 1], circle_points[:, 0], label=f'Circle with radius {dist} meters')
                # ax.text(pos[0], pos[1], f'{target_ap_list[pos][it]}', fontsize=8, ha='right')  # annotate each dot with RSSI value
                # ax.plot(sigma_circle_points[:, 1], sigma_circle_points[:, 0], linestyle='--', label=f'1-σ Circle with radius {dist * sigma} meters')

        ax.set_aspect('equal') 
        # ax.legend()
        plt.savefig(f"{path}/{it}_2D.pdf", format='pdf')
        
        # plt.show()  

        
# 使用示例
if __name__ == "__main__":
    # 已知的三维坐标点
    known_points = [
        [100.0, 30.0, 50.0],
        [101.0, 31.0, 55.0],
        [102.0, 32.0, 60.0]
    ]
    
    # 已知的距离
    distances = [10.0, 15.0, 20.0]
    
    # 创建PointEstimator实例
    estimator = PointEstimator(known_points, distances)
    
    # 估计待确定点的坐标
    initial_guess = [100.5, 30.5, 55.0]
    estimated_point = estimator.estimate_point(initial_guess=initial_guess)
    
    if estimated_point is not None:
        print("确定的三维坐标点为: ", estimated_point)
        
        # 可视化优化过程
        estimator.visualize_optimization(estimated_point, initial_guess)
