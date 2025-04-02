#!/usr/bin/env python3
import os
import sys
import glob
import time
import signal
import subprocess
import argparse
from datetime import datetime
from tqdm import tqdm

def run_test_with_bag(bag_path, use_true_ap_positions='true', timeout=15):
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
    
    return True

def main():
    # 解析命令行参数
    parser = argparse.ArgumentParser(description='批量运行WiFi定位测试')
    parser.add_argument('--bags_dir', type=str, default='/home/jay/AGLoc_ws/rosbag',
                        help='包含rosbag的目录')
    parser.add_argument('--bag_names', type=str, nargs='+',
                        help='要测试的特定rosbag名称列表，如果不指定则测试目录中的所有rosbag')
    parser.add_argument('--use_true_ap', type=str, choices=['true', 'false'], default='false',
                        help='是否使用真实AP位置')
    parser.add_argument('--timeout', type=int, default=15,
                        help='每个测试的超时时间（秒）')
    
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
    # 使用tqdm创建进度条
    for bag_path in tqdm(bag_paths, desc="测试进度", unit="bag"):
        bag_name = os.path.basename(bag_path)
        success = run_test_with_bag(bag_path, args.use_true_ap, args.timeout)
        results[bag_name] = success
    
    # 打印测试结果摘要
    print("\n测试结果摘要:")
    print(f"{'='*80}")
    for bag_name, success in results.items():
        status = "完成" if success else "失败"
        print(f"{bag_name}: {status}")
    print(f"{'='*80}")
    
    return 0

if __name__ == "__main__":
    sys.exit(main())
