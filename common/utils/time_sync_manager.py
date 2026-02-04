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

from collections import deque
from dataclasses import dataclass
import threading
import os

# ===================== 数据结构 =====================

@dataclass
class StampedMsg:
    stamp_ns: int
    msg: Image
    topic: str

@dataclass
class FrameGroup:
    stamp_ns: int
    frames: dict   # topic -> Image msg

# ===================== 时间对齐管理器 =====================

class TimeSyncManager:
    def __init__(self, topics, max_queue_size=20, slop_ns=30_000_000):
        """
        topics: 需要对齐的 topic 列表（<=6）
        max_queue_size: 每个 topic 最大缓存数
        slop_ns: 允许时间误差（ns）
        """
        self.topics = topics
        self.slop_ns = slop_ns

        self.buffers = {
            topic: deque(maxlen=max_queue_size)
            for topic in topics
        }

        self.synced_queue = deque()
        self.lock = threading.Lock()

    def add_msg(self, topic: str, msg: Image):
        stamp = msg.header.stamp
        stamp_ns = stamp.sec * 1_000_000_000 + stamp.nanosec

        with self.lock:
            self.buffers[topic].append(
                StampedMsg(stamp_ns, msg, topic)
            )
            self._try_sync()

    def _try_sync(self):
        # 所有 topic 至少有 1 帧
        if any(len(buf) == 0 for buf in self.buffers.values()):
            return

        # 以第一个 topic 最新帧为基准
        ref_topic = self.topics[0]
        ref_msg = self.buffers[ref_topic][-1]
        ref_time = ref_msg.stamp_ns

        frames = {ref_topic: ref_msg.msg}

        # 为其它 topic 找最接近的帧
        for topic, buf in self.buffers.items():
            if topic == ref_topic:
                continue

            best = min(buf, key=lambda x: abs(x.stamp_ns - ref_time))

            if abs(best.stamp_ns - ref_time) > self.slop_ns:
                return

            frames[topic] = best.msg

        # 对齐成功，清理旧帧
        for topic, buf in self.buffers.items():
            while buf and buf[0].stamp_ns <= ref_time:
                buf.popleft()

        self.synced_queue.append(
            FrameGroup(ref_time, frames)
        )

    def get_synced(self):
        with self.lock:
            if self.synced_queue:
                return self.synced_queue.popleft()
        return None