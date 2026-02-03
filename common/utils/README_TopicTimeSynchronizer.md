# 话题时间同步器 (TopicTimeSynchronizer)

## 概述

`TopicTimeSynchronizer` 是一个基于策略1（基准时间戳同步）的多话题时间同步器，用于同步ROS2中的多个话题数据。

## 特性

- ✅ **基准时间戳同步**: 选择一个话题作为时间基准，其他话题向其对齐
- ✅ **实时数据获取**: 提供实时的同步数据接口
- ✅ **线程安全**: 使用锁机制确保多线程环境下的安全访问
- ✅ **自动缓冲区管理**: 自动管理消息缓冲区，防止内存溢出
- ✅ **详细统计信息**: 提供同步成功率、时间偏移等统计信息
- ✅ **灵活配置**: 支持自定义同步窗口、缓冲区大小等参数

## 核心概念

### 基准话题 (Reference Topic)
作为时间同步基准的话题，其他话题的消息都向其时间戳对齐。

### 同步窗口 (Sync Window)
允许的最大时间差异（毫秒），超过此窗口的消息不会被同步。

### 时间偏移 (Time Offset)
每个话题相对于基准话题的时间偏移量。

## 使用方法

### 1. 基本使用

```python
from common.utils.topic_time_synchronizer import TopicTimeSynchronizer

# 初始化同步器
synchronizer = TopicTimeSynchronizer(
    topics=["cam_high", "cam_left_wrist", "qpos"],
    reference_topic="cam_high",  # 以cam_high为基准
    sync_window_ms=50,           # 50ms同步窗口
    buffer_size=20,
    enable_statistics=True
)

# 添加消息到同步器
synchronizer.add_message("cam_high", image_data, timestamp_ns)
synchronizer.add_message("cam_left_wrist", image_data, timestamp_ns)
synchronizer.add_message("qpos", qpos_data, timestamp_ns)

# 获取同步数据
synced_data = synchronizer.get_synced_data(max_age_ms=100)
if synced_data:
    print(f"同步数据: {synced_data.data}")
    print(f"时间偏移: {synced_data.time_offsets_ns}")
```

### 2. 在ROS2节点中使用

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from common.utils.topic_time_synchronizer import TopicTimeSynchronizer

class MyNode(Node):
    def __init__(self):
        super().__init__('my_node')
        
        # 初始化同步器
        self.synchronizer = TopicTimeSynchronizer(
            topics=["cam_high", "cam_left_wrist", "qpos"],
            reference_topic="cam_high",
            sync_window_ms=50
        )
        
        # 创建订阅器
        self.create_subscription(
            Image, '/camera/camera/color/image_raw',
            lambda msg: self._image_callback(msg, "cam_high"),
            10
        )
        
        self.create_subscription(
            Float32MultiArray, '/piper/qpos',
            self._qpos_callback,
            10
        )
    
    def _image_callback(self, msg: Image, topic_name: str):
        # 提取时间戳
        timestamp_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        
        # 处理图像数据
        image_data = self._process_image(msg)
        
        # 添加到同步器
        self.synchronizer.add_message(topic_name, image_data, timestamp_ns)
    
    def _qpos_callback(self, msg: Float32MultiArray):
        # 机械臂状态没有header，使用当前时间
        timestamp_ns = int(time.time() * 1_000_000_000)
        
        # 处理状态数据
        qpos_data = np.array(msg.data, dtype=np.float32)
        
        # 添加到同步器
        self.synchronizer.add_message("qpos", qpos_data, timestamp_ns)
```

## 配置参数

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| topics | List[str] | 必填 | 需要同步的话题列表 |
| reference_topic | Optional[str] | 第一个话题 | 基准话题名称 |
| sync_window_ms | float | 50.0 | 同步时间窗口（毫秒） |
| buffer_size | int | 20 | 每个话题的缓冲区大小 |
| max_sync_queue_size | int | 10 | 同步数据队列最大大小 |
| enable_statistics | bool | True | 是否启用统计信息 |

## 数据结构

### SyncedData
```python
@dataclass
class SyncedData:
    timestamp_ns: int              # 同步时间戳
    reference_timestamp_ns: int    # 基准话题时间戳
    data: Dict[str, Any]         # 话题名 -> 数据
    time_offsets_ns: Dict[str, int]  # 话题名 -> 时间偏移
```

## 统计信息

```python
stats = synchronizer.get_statistics()
# 或
synchronizer.print_statistics()
```

统计信息包括：
- 消息接收/同步/丢弃数量
- 同步成功率
- 平均时间偏移
- 缓冲区使用率

## 集成示例

### torch_inference_node.py 集成
```python
# 在节点初始化中
sync_topics = list(args.camera_topics.keys()) + ["qpos"]
self.topic_synchronizer = TopicTimeSynchronizer(
    topics=sync_topics,
    reference_topic=list(args.camera_topics.keys())[0],
    sync_window_ms=args.sync_time_window * 1000,
    enable_statistics=True
)

# 在回调函数中
def _process_image(self, msg: Image, topic_name: str):
    image_data = self._process_image(msg)
    timestamp_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
    self.topic_synchronizer.add_message(topic_name, image_data, timestamp_ns)

# 获取同步数据
def get_latest_observation(self):
    synced_data = self.topic_synchronizer.get_synced_data(max_age_ms=100)
    if synced_data:
        return self._format_observation(synced_data)
    return None
```

## 性能考虑

1. **同步窗口大小**: 较小的窗口提供更高精度，但可能丢弃更多数据
2. **缓冲区大小**: 影响内存使用和数据延迟
3. **基准话题选择**: 建议选择频率较低或更稳定的话题作为基准
4. **时间戳精度**: 尽量使用硬件时间戳而非系统时间

## 故障排除

### 常见问题

1. **同步数据为None**
   - 检查所有话题是否都有数据
   - 确认同步窗口设置是否合适
   - 查看统计信息中的失败原因

2. **同步成功率低**
   - 增大同步窗口
   - 检查话题频率是否匹配
   - 确认时间戳准确性

3. **内存使用过高**
   - 减小缓冲区大小
   - 定期清理同步队列
   - 降低话题频率

### 调试方法

```python
# 启用详细日志
synchronizer.enable_statistics = True

# 定期打印统计信息
import threading
import time

def print_stats():
    while True:
        synchronizer.print_statistics()
        time.sleep(5)

threading.Thread(target=print_stats, daemon=True).start()
```

## 示例文件

- `sync_example.py`: 基本使用示例
- `torch_inference_node.py`: 在推理节点中的集成示例
- `s600_inference_node.py`: 在S600推理节点中的集成示例