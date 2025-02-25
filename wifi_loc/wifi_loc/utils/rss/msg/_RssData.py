# 兼容性文件 - 模拟ROS1消息生成结构

class RssData:
    """与ROS1中rss/msg/RssData.msg兼容的类"""
    __slots__ = ['time_start_ns', 'duration_ms', 'mac_address', 'freq', 'data']
    _slot_types = ['uint64', 'uint32', 'string[]', 'int16[]', 'RssDatum[]']
    
    def __init__(self):
        self.time_start_ns = 0
        self.duration_ms = 0
        self.mac_address = []
        self.freq = []
        self.data = []
        
    def _get_types(self):
        return self._slot_types
