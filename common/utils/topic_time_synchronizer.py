#!/usr/bin/env python3
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

"""
多话题基准时间戳同步器
使用策略1：基准时间戳同步，选择一个话题作为时间基准，其他话题向其对齐
"""

import threading
import time
from collections import deque
from dataclasses import dataclass
from typing import Dict, List, Any, Optional, Tuple


@dataclass
class SyncedData:
    """同步后的数据结构"""

    timestamp_ns: int
    reference_timestamp_ns: int
    data: Dict[str, Any]
    time_offsets_ns: Dict[str, int]  # 各话题相对于基准话题的时间偏移


class TopicTimeSynchronizer:
    """
    多话题基准时间戳同步器

    功能特性：
    1. 基准时间戳同步策略
    2. 实时数据获取
    3. 线程安全
    4. 自动缓冲区管理
    5. 详细的同步统计信息
    """

    def __init__(
        self,
        topics: List[str],
        reference_topic: Optional[str] = None,
        sync_window_ms: float = 50.0,
        buffer_size: int = 20,
        max_sync_queue_size: int = 10,
        enable_statistics: bool = True,
    ):
        """
        初始化话题同步器

        Args:
            topics: 需要同步的话题列表
            reference_topic: 基准话题，默认为第一个话题
            sync_window_ms: 同步时间窗口（毫秒）
            buffer_size: 每个话题的缓冲区大小
            max_sync_queue_size: 同步数据队列最大大小
            enable_statistics: 是否启用统计信息收集
        """
        self.topics = topics
        self.reference_topic = reference_topic or topics[0]
        self.sync_window_ns = int(sync_window_ms * 1_000_000)
        self.buffer_size = buffer_size
        self.max_sync_queue_size = max_sync_queue_size
        self.enable_statistics = enable_statistics

        # 数据缓冲区
        self.buffers = {topic: deque(maxlen=buffer_size) for topic in topics}

        # 同步数据队列
        self.synced_queue = deque(maxlen=max_sync_queue_size)

        # 线程锁
        self.lock = threading.RLock()

        # 统计信息
        self.stats = (
            {
                "messages_received": {topic: 0 for topic in topics},
                "messages_synced": {topic: 0 for topic in topics},
                "messages_dropped": {topic: 0 for topic in topics},
                "sync_attempts": 0,
                "successful_syncs": 0,
                "failed_syncs": 0,
                "average_time_offset_ms": {
                    topic: 0.0 for topic in topics if topic != self.reference_topic
                },
            }
            if enable_statistics
            else None
        )

        # 控制标志
        self._running = True

        # 验证配置
        if self.reference_topic not in topics:
            raise ValueError(
                f"Reference topic '{self.reference_topic}' not in topics list: {topics}"
            )

    def add_message(
        self, topic: str, message: Any, timestamp_ns: Optional[int] = None
    ) -> bool:
        """
        添加消息到同步器

        Args:
            topic: 话题名称
            message: 消息数据
            timestamp_ns: 时间戳（纳秒），如果为None则使用当前时间

        Returns:
            bool: 是否成功添加
        """
        if topic not in self.topics:
            return False

        if timestamp_ns is None:
            timestamp_ns = int(time.time() * 1_000_000_000)

        with self.lock:
            # 添加到缓冲区
            self.buffers[topic].append((timestamp_ns, message))

            # 更新统计
            if self.stats:
                self.stats["messages_received"][topic] += 1

            # 尝试同步
            return self._try_sync()

    def get_synced_data(
        self, max_age_ms: Optional[float] = None
    ) -> Optional[SyncedData]:
        """
        获取同步的数据

        Args:
            max_age_ms: 最大数据年龄（毫秒），如果为None则不限制

        Returns:
            SyncedData: 同步的数据，如果没有则返回None
        """
        with self.lock:
            if not self.synced_queue:
                return None

            # 获取最新的同步数据
            synced_data = self.synced_queue[-1]

            # 检查数据年龄
            if max_age_ms is not None:
                current_time_ns = int(time.time() * 1_000_000_000)
                age_ms = (current_time_ns - synced_data.timestamp_ns) / 1_000_000
                if age_ms > max_age_ms:
                    return None

            return synced_data

    def _try_sync(self) -> bool:
        """
        尝试同步数据（内部方法）

        Returns:
            bool: 是否成功同步
        """
        with self.lock:
            # 检查所有话题都有数据
            if any(len(buf) == 0 for buf in self.buffers.values()):
                return False

            # 获取基准话题的最新消息
            ref_buffer = self.buffers[self.reference_topic]
            ref_timestamp_ns, ref_message = ref_buffer[-1]

            # 为其他话题找最接近的消息
            synced_data = {self.reference_topic: ref_message}
            time_offsets = {self.reference_topic: 0}

            for topic, buffer in self.buffers.items():
                if topic == self.reference_topic:
                    continue

                # 找时间差最小的消息
                best_timestamp_ns, best_message = min(
                    buffer, key=lambda x: abs(x[0] - ref_timestamp_ns)
                )

                # 检查时间差是否在允许范围内
                time_diff_ns = abs(best_timestamp_ns - ref_timestamp_ns)
                if time_diff_ns > self.sync_window_ns:
                    return False

                synced_data[topic] = best_message
                time_offsets[topic] = best_timestamp_ns - ref_timestamp_ns

                # 更新平均时间偏移统计
                if self.stats and topic in self.stats["average_time_offset_ms"]:
                    current_avg = self.stats["average_time_offset_ms"][topic]
                    count = self.stats["messages_synced"][topic]
                    new_avg = ((current_avg * count) + (time_diff_ns / 1_000_000)) / (
                        count + 1
                    )
                    self.stats["average_time_offset_ms"][topic] = new_avg

            # 清理旧数据
            self._cleanup_old_messages(ref_timestamp_ns)

            # 创建同步数据对象
            current_time_ns = int(time.time() * 1_000_000_000)
            sync_data = SyncedData(
                timestamp_ns=current_time_ns,
                reference_timestamp_ns=ref_timestamp_ns,
                data=synced_data,
                time_offsets_ns=time_offsets,
            )

            # 添加到同步队列
            self.synced_queue.append(sync_data)

            # 更新统计
            if self.stats:
                self.stats["sync_attempts"] += 1
                self.stats["successful_syncs"] += 1
                for topic in self.topics:
                    self.stats["messages_synced"][topic] += 1

            return True

    def _cleanup_old_messages(self, reference_timestamp_ns: int):
        """清理旧消息，防止缓冲区溢出"""
        for topic, buffer in self.buffers.items():
            # 移除时间戳小于等于参考时间戳的消息
            while buffer and buffer[0][0] <= reference_timestamp_ns:
                buffer.popleft()

                # 更新丢弃统计
                if self.stats:
                    self.stats["messages_dropped"][topic] += 1

    def clear_all_buffers(self):
        """清空所有缓冲区"""
        with self.lock:
            for buffer in self.buffers.values():
                buffer.clear()
            self.synced_queue.clear()

    def get_statistics(self) -> Dict[str, Any]:
        """获取统计信息"""
        if not self.stats:
            return {}

        with self.lock:
            stats_copy = {
                "topics": self.topics.copy(),
                "reference_topic": self.reference_topic,
                "sync_window_ms": self.sync_window_ns / 1_000_000,
                "buffer_size": self.buffer_size,
                "messages_received": self.stats["messages_received"].copy(),
                "messages_synced": self.stats["messages_synced"].copy(),
                "messages_dropped": self.stats["messages_dropped"].copy(),
                "sync_attempts": self.stats["sync_attempts"],
                "successful_syncs": self.stats["successful_syncs"],
                "failed_syncs": self.stats["failed_syncs"],
                "average_time_offset_ms": self.stats["average_time_offset_ms"].copy(),
                "sync_success_rate": 0.0
                if self.stats["sync_attempts"] == 0
                else self.stats["successful_syncs"] / self.stats["sync_attempts"],
            }

            # 计算缓冲区利用率
            buffer_usage = {
                topic: len(buffer) / self.buffer_size
                for topic, buffer in self.buffers.items()
            }
            stats_copy["buffer_usage"] = buffer_usage
            stats_copy["synced_queue_size"] = len(self.synced_queue)

            return stats_copy

    def print_statistics(self):
        """打印统计信息"""
        if not self.stats:
            print("统计信息未启用")
            return

        stats = self.get_statistics()

        print("\n" + "=" * 50)
        print("话题时间同步器统计信息")
        print("=" * 50)
        print(f"话题列表: {stats['topics']}")
        print(f"基准话题: {stats['reference_topic']}")
        print(f"同步窗口: {stats['sync_window_ms']:.1f} ms")
        print(f"缓冲区大小: {stats['buffer_size']}")

        print(f"\n同步尝试次数: {stats['sync_attempts']}")
        print(f"成功同步次数: {stats['successful_syncs']}")
        print(f"失败同步次数: {stats['failed_syncs']}")
        print(f"同步成功率: {stats['sync_success_rate'] * 100:.1f}%")

        print(f"\n消息统计:")
        for topic in stats["topics"]:
            received = stats["messages_received"][topic]
            synced = stats["messages_synced"][topic]
            dropped = stats["messages_dropped"][topic]
            usage = stats["buffer_usage"][topic]

            print(f"  {topic}:")
            print(f"    接收: {received}, 同步: {synced}, 丢弃: {dropped}")
            print(f"    同步率: {synced / received * 100 if received > 0 else 0:.1f}%")
            print(f"    缓冲区使用率: {usage * 100:.1f}%")

            if topic in stats["average_time_offset_ms"]:
                offset = stats["average_time_offset_ms"][topic]
                print(f"    平均时间偏移: {offset:+.1f} ms")

        print(
            f"\n同步队列大小: {stats['synced_queue_size']}/{self.max_sync_queue_size}"
        )
        print("=" * 50)

    def stop(self):
        """停止同步器"""
        self._running = False

    def __del__(self):
        """析构函数"""
        self.stop()
