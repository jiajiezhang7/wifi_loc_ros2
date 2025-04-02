#!/usr/bin/env python3
import os
import sys
import glob
import time
import signal
import subprocess
import argparse
import shutil
from datetime import datetime
from tqdm import tqdm

def run_test_with_bag(bag_path, use_true_ap_positions='true', timeout=15, organize_results=True):
    """
    使用指定的bag运行测试
    
    参数:
        bag_path: rosbag的路径
        use_true_ap_positions: 是否使用真实AP位置
        timeout: 超时时间（秒）
    """
    print(f"\n{'='*80}")
    print(f"开始测试 {bag_path}")
    print(f"使用真实AP位置: {use_true_ap_positions}")
    print(f"{'='*80}\n")
    
    # 构建launch命令
    cmd = [
        "ros2", "launch", "wifi_loc", "robot_loc_with_bag.launch.py",
        f"bag_path:={bag_path}",
        f"use_true_ap_positions:={use_true_ap_positions}"
    ]
    
    # 记录开始时间
    start_time = datetime.now()
    print(f"开始时间: {start_time.strftime('%Y-%m-%d %H:%M:%S')}")
    
    try:
        # 启动进程
        process = subprocess.Popen(
            cmd, 
            stdout=subprocess.PIPE, 
            stderr=subprocess.PIPE,
            text=True,
            bufsize=1
        )
        
        # 等待进程完成或超时
        try:
            stdout, stderr = process.communicate(timeout=timeout)
            
            # 打印输出
            if stdout:
                print("标准输出:")
                print(stdout)
            
            if stderr:
                print("错误输出:")
                print(stderr)
            
            # 检查退出码
            if process.returncode != 0:
                print(f"测试失败，退出码: {process.returncode}")
            else:
                print("测试成功完成")
                
        except subprocess.TimeoutExpired:
            # 如果超时，终止进程
            print(f"测试超时（{timeout}秒），终止进程")
            process.kill()
            
            # 尝试终止所有相关进程
            try:
                # 终止ros2 bag play进程
                os.system("pkill -f 'ros2 bag play'")
                # 终止robot_loc节点
                os.system("pkill -f 'robot_loc'")
            except Exception as e:
                print(f"终止相关进程时出错: {e}")
    
    except Exception as e:
        print(f"运行测试时出错: {e}")
    
    # 记录结束时间
    end_time = datetime.now()
    duration = end_time - start_time
    print(f"结束时间: {end_time.strftime('%Y-%m-%d %H:%M:%S')}")
    print(f"测试持续时间: {duration}")
    
    # 确保所有相关进程都已终止
    try:
        os.system("pkill -f 'ros2 bag play'")
        os.system("pkill -f 'robot_loc'")
    except:
        pass
    
    # 等待一段时间，确保所有资源都已释放
    time.sleep(5)
    
    # 如果需要组织结果，则将结果移动到对应目录
    if organize_results:
        organize_test_results(bag_path, use_true_ap_positions)
    
    return True

def organize_test_results(bag_path, use_true_ap_positions):
    """
    根据AP位置类型组织测试结果
    
    参数:
        bag_path: rosbag的路径
        use_true_ap_positions: 是否使用真实AP位置
    """
    bag_name = os.path.basename(bag_path)
    ap_type = 'true_ap' if use_true_ap_positions.lower() == 'true' else 'est_ap'
    
    # 创建目标目录
    results_base_dir = '/home/jay/AGLoc_ws/results_wifi_loc'
    figs_base_dir = '/home/jay/AGLoc_ws/figs_wifi_loc'
    
    results_target_dir = os.path.join(results_base_dir, ap_type)
    figs_target_dir = os.path.join(figs_base_dir, ap_type)
    
    os.makedirs(results_target_dir, exist_ok=True)
    os.makedirs(figs_target_dir, exist_ok=True)
    
    # 移动结果文件
    try:
        # 移动JSON结果文件
        for result_file in glob.glob(f'{results_base_dir}/{bag_name}_{ap_type}_*.json'):
            file_name = os.path.basename(result_file)
            target_path = os.path.join(results_target_dir, file_name)
            shutil.move(result_file, target_path)
            print(f"已移动结果文件: {file_name} 到 {results_target_dir}")
        
        # 移动图像文件
        for fig_file in glob.glob(f'{figs_base_dir}/{bag_name}_{ap_type}_*.png'):
            file_name = os.path.basename(fig_file)
            target_path = os.path.join(figs_target_dir, file_name)
            shutil.move(fig_file, target_path)
            print(f"已移动图像文件: {file_name} 到 {figs_target_dir}")
    
    except Exception as e:
        print(f"组织结果文件时出错: {e}")

def main():
    # 解析命令行参数
    parser = argparse.ArgumentParser(description='批量运行WiFi定位测试')
    parser.add_argument('--bags_dir', type=str, default='/home/jay/AGLoc_ws/rosbag',
                        help='包含rosbag的目录')
    parser.add_argument('--bag_names', type=str, nargs='+',
                        help='要测试的特定rosbag名称列表，如果不指定则测试目录中的所有rosbag')
    parser.add_argument('--use_true_ap', type=str, choices=['true', 'false', 'both'], default='both',
                        help='是否使用真实AP位置: true, false, 或both(两者都测试)')
    parser.add_argument('--timeout', type=int, default=15,
                        help='每个测试的超时时间（秒）')
    parser.add_argument('--organize_results', action='store_true', default=True,
                        help='是否自动整理测试结果到对应目录')
    
    args = parser.parse_args()
    
    # 获取要测试的rosbag列表
    if args.bag_names:
        # 如果指定了特定的bag名称，使用它们
        bag_paths = [os.path.join(args.bags_dir, bag_name) for bag_name in args.bag_names]
    else:
        # 否则，获取目录中的所有子目录（假设每个子目录是一个rosbag）
        bag_paths = [d for d in glob.glob(os.path.join(args.bags_dir, '*')) if os.path.isdir(d)]
    
    # 确保找到了rosbag
    if not bag_paths:
        print(f"错误：在 {args.bags_dir} 中没有找到rosbag")
        return 1
    
    print(f"找到 {len(bag_paths)} 个rosbag:")
    for i, bag_path in enumerate(bag_paths, 1):
        print(f"{i}. {os.path.basename(bag_path)}")
    
    # 设置信号处理，以便可以优雅地中断测试
    def signal_handler(sig, frame):
        print("\n收到中断信号，正在清理...")
        os.system("pkill -f 'ros2 bag play'")
        os.system("pkill -f 'robot_loc'")
        sys.exit(0)
    
    signal.signal(signal.SIGINT, signal_handler)
    
    # 运行测试
    results = {}
    
    # 确定要测试的AP位置模式
    ap_modes = []
    if args.use_true_ap == 'both':
        ap_modes = ['true', 'false']
    else:
        ap_modes = [args.use_true_ap]
    
    # 使用tqdm创建总进度条
    total_tests = len(bag_paths) * len(ap_modes)
    with tqdm(total=total_tests, desc="总测试进度", unit="test") as pbar:
        # 对每种AP位置模式进行测试
        for ap_mode in ap_modes:
            print(f"\n{'='*80}")
            print(f"开始测试 AP位置模式: {'真实' if ap_mode == 'true' else '估计'}")
            print(f"{'='*80}\n")
            
            # 对每个bag进行测试
            for bag_path in bag_paths:
                bag_name = os.path.basename(bag_path)
                test_id = f"{bag_name}_{ap_mode}"
                
                print(f"\n正在测试: {bag_name} (AP模式: {'真实' if ap_mode == 'true' else '估计'})")
                success = run_test_with_bag(bag_path, ap_mode, args.timeout, args.organize_results)
                results[test_id] = success
                
                pbar.update(1)
    
    # 打印测试结果摘要
    print("\n测试结果摘要:")
    print(f"{'='*80}")
    
    # 按AP位置模式分组显示结果
    if 'true' in [mode for mode in ap_modes]:
        print("\n真实AP位置测试结果:")
        for test_id, success in results.items():
            if '_true' in test_id:
                bag_name = test_id.replace('_true', '')
                status = "完成" if success else "失败"
                print(f"{bag_name}: {status}")
    
    if 'false' in [mode for mode in ap_modes]:
        print("\n估计AP位置测试结果:")
        for test_id, success in results.items():
            if '_false' in test_id:
                bag_name = test_id.replace('_false', '')
                status = "完成" if success else "失败"
                print(f"{bag_name}: {status}")
    
    print(f"{'='*80}")
    
    return 0

if __name__ == "__main__":
    sys.exit(main())
