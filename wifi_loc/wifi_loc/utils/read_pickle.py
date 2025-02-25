import pickle
from dataclasses import dataclass
from typing import List
import sys
import copyreg

# 首先需要定义相同的数据类
@dataclass
class RssDatum:
    rss: List[int]

@dataclass
class RssData:
    time_start_ns: int
    duration_ms: int
    mac_address: List[str]
    freq: List[int]
    data: List[RssDatum]

# 修改处理方式，使用__reduce__魔术方法
def _restore_rssdata(time_start_ns, duration_ms, mac_address, freq, data):
    """恢复RssData对象的辅助函数"""
    return RssData(time_start_ns, duration_ms, mac_address, freq, data)

def _restore_rssdatum(rss):
    """恢复RssDatum对象的辅助函数"""
    return RssDatum(rss)

# 创建假的旧类，用于pickle反序列化
class _MainRssData:
    def __reduce__(self):
        # 获取对象的所有属性
        return (_restore_rssdata, (self.time_start_ns, self.duration_ms, self.mac_address, self.freq, self.data))

class _MainRssDatum:
    def __reduce__(self):
        return (_restore_rssdatum, (self.rss,))

# 添加到__main__模块
if '__main__' not in sys.modules:
    sys.modules['__main__'] = type('_MainModule', (), {
        'RssData': _MainRssData,
        'RssDatum': _MainRssDatum
    })
else:
    sys.modules['__main__'].RssData = _MainRssData
    sys.modules['__main__'].RssDatum = _MainRssDatum

def read_data_from_pickle(filename):
    """从pickle文件读取RssData对象列表"""
    try:
        with open(filename, 'rb') as f:
            data_list = pickle.load(f)
        print("成功读取！")
        return data_list
    except Exception as e:
        print(f"读取pickle文件时出错: {e}")
        raise

if __name__ == '__main__':
    # 使用示例
    input_file = '/home/jay/wifi_backup/src/wifi_loc/data/6rss.p'
    data_list = read_data_from_pickle(input_file)