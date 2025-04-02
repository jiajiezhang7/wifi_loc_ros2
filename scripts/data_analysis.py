import json
import os
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
import seaborn as sns
from math import radians, cos, sin, asin, sqrt
import glob

# 计算地球表面两点之间的距离(哈弗辛公式，单位：米)
def haversine_distance(lat1, lon1, lat2, lon2):
    """
    计算两个经纬度坐标之间的距离(米)
    """
    # 将十进制度数转化为弧度
    lon1, lat1, lon2, lat2 = map(radians, [lon1, lat1, lon2, lat2])
    # 哈弗辛公式
    dlon = lon2 - lon1 
    dlat = lat2 - lat1 
    a = sin(dlat/2)**2 + cos(lat1) * cos(lat2) * sin(dlon/2)**2
    c = 2 * asin(sqrt(a)) 
    r = 6371000  # 地球平均半径，单位为米
    return c * r

# 加载GT数据
def load_gt_data(gt_file):
    with open(gt_file, 'r') as f:
        gt_data = json.load(f)
    return gt_data

# 加载结果数据
def load_results(results_dir):
    results = []
    # 遍历true_ap和est_ap目录
    for ap_type in ['true_ap', 'est_ap']:
        ap_dir = os.path.join(results_dir, ap_type)
        if not os.path.exists(ap_dir):
            continue
            
        # 遍历目录下所有json文件
        for result_file in glob.glob(os.path.join(ap_dir, '*.json')):
            with open(result_file, 'r') as f:
                result_list = json.load(f)
                # 确保结果是列表格式
                if isinstance(result_list, list) and len(result_list) > 0:
                    # 获取第一个结果（通常只有一个）
                    result = result_list[0]
                    # 添加AP类型信息
                    result['ap_type'] = ap_type
                    # 提取bag名称
                    filename = os.path.basename(result_file)
                    # 从文件名中提取bag名称
                    if '_true_ap_' in filename:
                        bag_name = filename.split('_true_ap_')[0]
                    elif '_est_ap_' in filename:
                        bag_name = filename.split('_est_ap_')[0]
                    else:
                        bag_name = filename.split('_localization_')[0]
                    result['bag_name'] = bag_name
                    results.append(result)
    
    return results

# 计算误差并生成数据框
def calculate_errors(results, gt_data):
    error_data = []
    
    for result in results:
        bag_name = result['bag_name']
        # 跳过没有GT数据的bag
        if bag_name not in gt_data:
            continue
            
        gt_lat, gt_lon = gt_data[bag_name]
        
        # Coarse位置误差
        if 'coarse_position' in result:
            coarse_pos = result['coarse_position']
            # 处理对象格式的位置信息
            coarse_lat = coarse_pos['latitude']
            coarse_lon = coarse_pos['longitude']
            coarse_error = haversine_distance(coarse_lat, coarse_lon, gt_lat, gt_lon)
        else:
            coarse_error = np.nan
            coarse_lat = np.nan
            coarse_lon = np.nan
            
        # Final位置误差
        if 'final_position' in result:
            final_pos = result['final_position']
            # 处理对象格式的位置信息
            final_lat = final_pos['latitude']
            final_lon = final_pos['longitude']
            final_error = haversine_distance(final_lat, final_lon, gt_lat, gt_lon)
        else:
            final_error = np.nan
            final_lat = np.nan
            final_lon = np.nan
            
        # 计算改进率
        if not np.isnan(coarse_error) and not np.isnan(final_error) and coarse_error > 0:
            improvement = (coarse_error - final_error) / coarse_error * 100
        else:
            improvement = np.nan
            
        error_data.append({
            'bag_name': bag_name,
            'ap_type': result['ap_type'],
            'coarse_error': coarse_error,
            'final_error': final_error,
            'improvement': improvement,
            'gt_lat': gt_lat,
            'gt_lon': gt_lon,
            'coarse_lat': coarse_lat,
            'coarse_lon': coarse_lon,
            'final_lat': final_lat,
            'final_lon': final_lon
        })
    
    return pd.DataFrame(error_data)

# 生成误差统计表格
def generate_error_stats(error_df):
    # 创建一个新的DataFrame来存储统计结果
    stats_data = []
    
    # 按AP类型分组统计
    for ap_type, group in error_df.groupby('ap_type'):
        stats_data.append({
            'ap_type': ap_type,
            'coarse_error_mean': group['coarse_error'].mean(),
            'coarse_error_median': group['coarse_error'].median(),
            'coarse_error_std': group['coarse_error'].std(),
            'coarse_error_min': group['coarse_error'].min(),
            'coarse_error_max': group['coarse_error'].max(),
            'final_error_mean': group['final_error'].mean(),
            'final_error_median': group['final_error'].median(),
            'final_error_std': group['final_error'].std(),
            'final_error_min': group['final_error'].min(),
            'final_error_max': group['final_error'].max(),
            'improvement_mean': group['improvement'].mean(),
            'improvement_median': group['improvement'].median(),
            'improvement_std': group['improvement'].std()
        })
    
    # 所有数据的统计
    stats_data.append({
        'ap_type': 'all',
        'coarse_error_mean': error_df['coarse_error'].mean(),
        'coarse_error_median': error_df['coarse_error'].median(),
        'coarse_error_std': error_df['coarse_error'].std(),
        'coarse_error_min': error_df['coarse_error'].min(),
        'coarse_error_max': error_df['coarse_error'].max(),
        'final_error_mean': error_df['final_error'].mean(),
        'final_error_median': error_df['final_error'].median(),
        'final_error_std': error_df['final_error'].std(),
        'final_error_min': error_df['final_error'].min(),
        'final_error_max': error_df['final_error'].max(),
        'improvement_mean': error_df['improvement'].mean(),
        'improvement_median': error_df['improvement'].median(),
        'improvement_std': error_df['improvement'].std()
    })
    
    return pd.DataFrame(stats_data).set_index('ap_type')

# 绘制误差条形图
def plot_error_bar(error_df, output_dir):
    plt.figure(figsize=(12, 8))
    
    # 计算平均误差
    avg_errors = error_df.groupby('ap_type').agg({
        'coarse_error': 'mean',
        'final_error': 'mean'
    })
    
    # 重新组织数据用于绘图
    plot_data = pd.melt(avg_errors.reset_index(), 
                        id_vars=['ap_type'],
                        value_vars=['coarse_error', 'final_error'],
                        var_name='Position Type', 
                        value_name='Error (meters)')
    
    # 使用Seaborn绘制分组条形图
    ax = sns.barplot(x='ap_type', y='Error (meters)', hue='Position Type', data=plot_data)
    
    plt.title('Average Positioning Error by AP Type')
    plt.xlabel('AP Position Type')
    plt.ylabel('Error (meters)')
    plt.grid(axis='y', linestyle='--', alpha=0.7)
    
    # 在柱状图上标注具体数值
    for i, p in enumerate(ax.patches):
        ax.annotate(f'{p.get_height():.2f}m', 
                    (p.get_x() + p.get_width() / 2., p.get_height()), 
                    ha = 'center', va = 'bottom',
                    xytext = (0, 5), textcoords = 'offset points')
    
    plt.tight_layout()
    plt.savefig(os.path.join(output_dir, 'error_bar_chart.png'), dpi=300)
    plt.close()

# 绘制误差CDF图
def plot_error_cdf(error_df, output_dir):
    plt.figure(figsize=(10, 6))
    
    # 为不同位置类型绘制CDF
    for pos_type in ['coarse_error', 'final_error']:
        # 排序误差
        sorted_errors = np.sort(error_df[pos_type].dropna().values)
        # 计算累积概率
        cdf = np.arange(1, len(sorted_errors) + 1) / len(sorted_errors)
        
        label = 'Coarse Position' if pos_type == 'coarse_error' else 'Final Position'
        plt.plot(sorted_errors, cdf, marker='.', linestyle='--', label=label)
        
    plt.grid(True, linestyle='--', alpha=0.7)
    plt.xlabel('Error (meters)')
    plt.ylabel('Cumulative Probability')
    plt.title('CDF of Positioning Errors')
    plt.legend()
    plt.tight_layout()
    plt.savefig(os.path.join(output_dir, 'error_cdf.png'), dpi=300)
    plt.close()

# 绘制改进效果散点图
def plot_improvement_scatter(error_df, output_dir):
    plt.figure(figsize=(10, 6))
    
    # 按AP类型绘制散点图
    for ap_type, color in zip(['true_ap', 'est_ap'], ['blue', 'red']):
        df_subset = error_df[error_df['ap_type'] == ap_type]
        plt.scatter(df_subset['coarse_error'], df_subset['final_error'], 
                   alpha=0.7, label=f'{ap_type}', color=color)
    
    # 添加对角线(y=x)
    max_error = max(error_df['coarse_error'].max(), error_df['final_error'].max())
    plt.plot([0, max_error], [0, max_error], 'k--', alpha=0.5, label='No Improvement')
    
    plt.grid(True, linestyle='--', alpha=0.5)
    plt.xlabel('Coarse Position Error (meters)')
    plt.ylabel('Final Position Error (meters)')
    plt.title('Positioning Improvement: Coarse vs Final Error')
    plt.legend()
    plt.axis('equal')
    plt.tight_layout()
    plt.savefig(os.path.join(output_dir, 'improvement_scatter.png'), dpi=300)
    plt.close()

# 绘制定位轨迹图
def plot_location_map(error_df, output_dir):
    plt.figure(figsize=(12, 10))
    
    # 绘制GT位置
    plt.scatter(error_df['gt_lon'], error_df['gt_lat'], 
               s=100, marker='*', c='green', label='Ground Truth')
    
    # 分别绘制不同AP类型的结果
    for ap_type, marker, color_coarse, color_final in [
        ('true_ap', 'o', 'gold', 'red'), 
        ('est_ap', 's', 'cyan', 'magenta')
    ]:
        # 筛选数据
        df_subset = error_df[error_df['ap_type'] == ap_type]
        
        # 绘制Coarse位置
        plt.scatter(df_subset['coarse_lon'], df_subset['coarse_lat'], 
                   s=50, alpha=0.7, marker=marker, c=color_coarse, edgecolor='black', 
                   label=f'Coarse Position ({ap_type})')
        
        # 绘制Final位置
        plt.scatter(df_subset['final_lon'], df_subset['final_lat'], 
                   s=50, alpha=0.7, marker=marker, c=color_final, edgecolor='black', 
                   label=f'Final Position ({ap_type})')
        
        # 连接同一测试的GT、Coarse和Final位置
        for _, row in df_subset.iterrows():
            # 使用不同线型区分不同AP类型
            linestyle = '--' if ap_type == 'true_ap' else ':'
            
            # GT到Coarse的连线
            plt.plot([row['gt_lon'], row['coarse_lon']], [row['gt_lat'], row['coarse_lat']], 
                    linestyle, color='gray', alpha=0.4)
            
            # Coarse到Final的连线
            plt.plot([row['coarse_lon'], row['final_lon']], [row['coarse_lat'], row['final_lat']], 
                    '-', color='gray', alpha=0.4)
    
    # 添加网格
    plt.grid(True, linestyle='--', alpha=0.3)
    
    # 设置更详细的标题
    plt.title('Location Map: Comparison of True AP vs Estimated AP Results', pad=15)
    
    # 设置坐标轴标签
    plt.xlabel('Longitude', labelpad=10)
    plt.ylabel('Latitude', labelpad=10)
    
    # 调整图例位置和样式
    plt.legend(bbox_to_anchor=(1.02, 1), loc='upper left', borderaxespad=0,
              fontsize='small', framealpha=0.8)
    
    # 保持纵横比例相同
    plt.axis('equal')
    
    # 调整布局，为图例留出空间
    plt.tight_layout(rect=[0, 0, 0.85, 1])
    
    # 保存图像
    plt.savefig(os.path.join(output_dir, 'location_map.png'), dpi=300, bbox_inches='tight')
    plt.close()

# 主函数
def main():
    # 路径设置
    base_dir = '/home/jay/AGLoc_ws/src/wifi_loc'
    gt_file = os.path.join(base_dir, 'gt_pose/rosbag_gt_pose.json')
    results_dir = os.path.join(base_dir, 'results_wifi_loc')
    output_dir = os.path.join(base_dir, 'analysis_results')
    
    # 创建输出目录
    os.makedirs(output_dir, exist_ok=True)
    
    # 加载数据
    gt_data = load_gt_data(gt_file)
    results = load_results(results_dir)
    
    # 计算误差
    error_df = calculate_errors(results, gt_data)
    
    # 生成统计表格
    stats = generate_error_stats(error_df)
    stats.to_csv(os.path.join(output_dir, 'error_statistics.csv'))
    
    # 生成详细的错误数据表
    error_df.to_csv(os.path.join(output_dir, 'detailed_errors.csv'), index=False)
    
    # 生成可视化图表
    plot_error_bar(error_df, output_dir)
    plot_error_cdf(error_df, output_dir)
    plot_improvement_scatter(error_df, output_dir)
    plot_location_map(error_df, output_dir)
    
    print(f"分析完成！结果保存在 {output_dir} 目录")

if __name__ == "__main__":
    main()