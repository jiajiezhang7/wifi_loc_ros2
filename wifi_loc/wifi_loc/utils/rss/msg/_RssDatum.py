# 兼容性文件 - 模拟ROS1消息生成结构

class RssDatum:
    """与ROS1中rss/msg/RssDatum.msg兼容的类"""
    __slots__ = ['rss']
    _slot_types = ['int8[]']
    
    def __init__(self):
        self.rss = []

    def _get_types(self):
        return self._slot_types
