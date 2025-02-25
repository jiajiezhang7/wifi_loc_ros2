import pickle
import sys
import os
import logging
from collections import defaultdict

# 设置日志
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger('pickle_compat')

class ROS1CompatUnpickler(pickle.Unpickler):
    """兼容ROS1序列化消息的自定义Unpickler"""
    
    def find_class(self, module, name):
        # 重定向ROS1消息类到我们的兼容层
        if module.startswith('rss.msg'):
            # 确保我们的兼容层在路径中
            utils_dir = os.path.dirname(os.path.abspath(__file__))
            if utils_dir not in sys.path:
                sys.path.insert(0, utils_dir)
                
            try:
                # 从兼容层导入
                if name in ['RssData', '_RssData']:
                    from rss.msg import RssData
                    return RssData
                elif name in ['RssDatum', '_RssDatum']:
                    from rss.msg import RssDatum
                    return RssDatum
                else:
                    logger.warning(f"未知的ROS1消息类型: {module}.{name}")
            except Exception as e:
                logger.error(f"导入兼容类时出错: {str(e)}")
                
        # 尝试标准导入
        return super().find_class(module, name)

def load_ros1_pickle(file_path):
    """加载ROS1格式的pickle文件，并返回处理后的数据"""
    try:
        with open(file_path, 'rb') as f:
            data = ROS1CompatUnpickler(f).load()
        return data
    except Exception as e:
        logger.error(f"加载ROS1 pickle文件时出错: {str(e)}")
        return None

def convert_pickle_to_json(pickle_path, json_path=None):
    """将ROS1 pickle文件转换为JSON格式"""
    import json
    
    if json_path is None:
        json_path = pickle_path.replace('.p', '.json')
    
    try:
        # 加载pickle数据
        data = load_ros1_pickle(pickle_path)
        if data is None:
            return False
            
        # 提取有用信息
        converted_data = []
        for msg in data:
            msg_data = {
                'time_start_ns': msg.time_start_ns,
                'duration_ms': msg.duration_ms,
                'mac_address': msg.mac_address,
                'freq': msg.freq,
                'rss_data': []
            }
            
            for i, rss_datum in enumerate(msg.data):
                if i < len(msg.mac_address):
                    msg_data['rss_data'].append({
                        'mac': msg.mac_address[i],
                        'rss': rss_datum.rss
                    })
                
            converted_data.append(msg_data)
        
        # 保存为JSON
        with open(json_path, 'w') as f:
            json.dump(converted_data, f)
            
        logger.info(f"成功转换文件: {pickle_path} -> {json_path}")
        return True
    except Exception as e:
        logger.error(f"转换文件失败 {pickle_path}: {str(e)}")
        return False

def extract_mac_rss_from_pickle(pickle_path):
    """从pickle文件中提取MAC地址和RSS数据的映射"""
    try:
        # 加载pickle数据
        data = load_ros1_pickle(pickle_path)
        if data is None:
            return None
            
        # 提取MAC-RSS映射
        mac_rss_map = defaultdict(list)
        
        for msg in data:
            for i, mac in enumerate(msg.mac_address):
                if i < len(msg.data):
                    mac_rss_map[mac].extend(msg.data[i].rss)
        
        return dict(mac_rss_map)
    except Exception as e:
        logger.error(f"从pickle提取数据失败 {pickle_path}: {str(e)}")
        return None 