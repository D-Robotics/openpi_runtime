# Copyright (c) 2025，D-Robotics.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

#!/usr/bin/env python3
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from builtin_interfaces.msg import Time

from collections import deque
from dataclasses import dataclass
import threading
import time
from typing import Dict, Any, Optional

# ===================== 数据结构 =====================


@dataclass
class StampedMsg:
    stamp_ns: int
    msg: Any
    topic: str


@dataclass
class SyncedData:
    stamp_ns: int
    data: Dict[str, Any]  # topic -> msg
    timestamp: Time  # ROS timestamp


# ===================== 增强版时间对齐管理器 =====================


class EnhancedTimeSyncManager:
    def __init__(self, topics_config, max_queue_size=30, slop_ns=20_000_000):
        """
        topics_config: 话题配置字典，格式为:
            {
                'topic_name': {'type': 'image', 'msg_type': Image},
                'state_topic': {'type': 'state', 'msg_type': Float32MultiArray}
            }
        max_queue_size: 每个 topic 最大缓存数
        slop_ns: 允许时间误差（ns）
        """
        self.topics_config = topics_config
        self.topics = list(topics_config.keys())
        self.slop_ns = slop_ns

        self.buffers = {topic: deque(maxlen=max_queue_size) for topic in self.topics}

        self.synced_queue = deque(maxlen=10)  # 限制同步队列大小
        self.lock = threading.Lock()
        self.running = True

    def add_msg(self, topic: str, msg):
        """添加消息到缓冲区"""
        if topic not in self.topics:
            return

        # 获取时间戳
        if hasattr(msg, "header"):
            stamp = msg.header.stamp
            stamp_ns = stamp.sec * 1_000_000_000 + stamp.nanosec
        else:
            # 如果消息没有header，使用当前时间
            stamp_ns = int(time.time() * 1_000_000_000)

        with self.lock:
            self.buffers[topic].append(StampedMsg(stamp_ns, msg, topic))
            self._try_sync()

    def _try_sync(self):
        """尝试同步所有话题的消息"""
        # 检查所有 topic 是否至少有 1 帧
        if any(len(buf) == 0 for buf in self.buffers.values()):
            return

        # 以第一个 topic 最新帧为基准
        ref_topic = self.topics[0]
        ref_msg = self.buffers[ref_topic][-1]
        ref_time = ref_msg.stamp_ns

        data = {ref_topic: ref_msg.msg}

        # 为其它 topic 找最接近的帧
        for topic, buf in self.buffers.items():
            if topic == ref_topic:
                continue

            best = min(buf, key=lambda x: abs(x.stamp_ns - ref_time))

            if abs(best.stamp_ns - ref_time) > self.slop_ns:
                return  # 时间差超过阈值，放弃同步

            data[topic] = best.msg

        # 对齐成功，清理旧帧
        for topic, buf in self.buffers.items():
            while buf and buf[0].stamp_ns <= ref_time:
                buf.popleft()

        # 创建时间戳对象
        timestamp = Time()
        timestamp.sec = ref_time // 1_000_000_000
        timestamp.nanosec = ref_time % 1_000_000_000

        self.synced_queue.append(SyncedData(ref_time, data, timestamp))

    def get_synced(self) -> Optional[SyncedData]:
        """获取同步的数据"""
        with self.lock:
            if self.synced_queue:
                return self.synced_queue.popleft()
        return None

    def stop(self):
        """停止同步管理器"""
        self.running = False


# ===================== 数据同步线程 =====================


class DataSyncThread:
    def __init__(self, topics_config, max_queue_size=30, slop_ns=20_000_000):
        self.sync_manager = EnhancedTimeSyncManager(
            topics_config, max_queue_size, slop_ns
        )
        self.thread = None
        self.running = False

    def start(self):
        """启动同步线程"""
        self.running = True
        self.thread = threading.Thread(target=self._sync_loop, daemon=True)
        self.thread.start()

    def _sync_loop(self):
        """同步循环 - 在这里可以添加额外的同步逻辑"""
        while self.running:
            time.sleep(0.001)  # 1ms 的轻量级循环

    def stop(self):
        """停止同步线程"""
        self.running = False
        self.sync_manager.stop()
        if self.thread and self.thread.is_alive():
            self.thread.join(timeout=1.0)

    def get_synced_data(self) -> Optional[SyncedData]:
        """获取同步的数据"""
        return self.sync_manager.get_synced()

    def add_msg(self, topic: str, msg):
        """添加消息到同步管理器"""
        self.sync_manager.add_msg(topic, msg)
