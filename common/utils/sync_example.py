#!/usr/bin/env python3
"""
话题时间同步器使用示例

这个示例展示了如何使用TopicTimeSynchronizer来同步多个ROS2话题
"""

import time
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from common.utils.topic_time_synchronizer import TopicTimeSynchronizer


class SyncExampleNode(Node):
    def __init__(self):
        super().__init__("sync_example_node")

        # 初始化话题同步器
        topics = ["cam_high", "cam_left_wrist", "qpos"]
        self.synchronizer = TopicTimeSynchronizer(
            topics=topics,
            reference_topic="cam_high",  # 以cam_high为基准
            sync_window_ms=50,  # 50ms同步窗口
            buffer_size=20,
            enable_statistics=True,
        )

        # 创建订阅器
        self.image_sub_1 = self.create_subscription(
            Image,
            "/camera/camera/color/image_raw",
            lambda msg: self._image_callback(msg, "cam_high"),
            10,
        )

        self.image_sub_2 = self.create_subscription(
            Image,
            "/camera_left/camera_left/color/image_raw",
            lambda msg: self._image_callback(msg, "cam_left_wrist"),
            10,
        )

        self.qpos_sub = self.create_subscription(
            Float32MultiArray, "/piper/qpos", self._qpos_callback, 10
        )

        # 定时器，用于获取同步数据
        self.timer = self.create_timer(0.1, self._get_synced_data)

        self.get_logger().info("同步示例节点已启动")

    def _image_callback(self, msg: Image, topic_name: str):
        """图像回调函数"""
        # 模拟图像处理
        image_data = np.random.randint(0, 255, (3, 224, 224), dtype=np.uint8)

        # 添加到同步器
        timestamp_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        self.synchronizer.add_message(topic_name, image_data, timestamp_ns)

        self.get_logger().info(f"收到 {topic_name} 图像")

    def _qpos_callback(self, msg: Float32MultiArray):
        """机械臂状态回调函数"""
        qpos_data = np.array(msg.data, dtype=np.float32)

        # 添加到同步器（使用当前时间）
        timestamp_ns = int(time.time() * 1_000_000_000)
        self.synchronizer.add_message("qpos", qpos_data, timestamp_ns)

        self.get_logger().info(f"收到机械臂状态: {qpos_data}")

    def _get_synced_data(self):
        """获取同步数据"""
        synced_data = self.synchronizer.get_synced_data(max_age_ms=100)

        if synced_data is not None:
            self.get_logger().info("获取到同步数据:")

            for topic, data in synced_data.data.items():
                if isinstance(data, np.ndarray):
                    if data.ndim == 1:  # 机械臂状态
                        self.get_logger().info(f"  {topic}: {data}")
                    else:  # 图像数据
                        self.get_logger().info(
                            f"  {topic}: shape={data.shape}, dtype={data.dtype}"
                        )

                # 打印时间偏移
                if topic in synced_data.time_offsets_ns:
                    offset_ms = synced_data.time_offsets_ns[topic] / 1_000_000
                    self.get_logger().info(f"    时间偏移: {offset_ms:+.1f}ms")

            # 打印统计信息
            stats = self.synchronizer.get_statistics()
            self.get_logger().info(
                f"同步成功率: {stats['sync_success_rate'] * 100:.1f}%"
            )

    def __del__(self):
        """析构函数"""
        if hasattr(self, "synchronizer"):
            self.synchronizer.print_statistics()


def main():
    """主函数"""
    rclpy.init()
    node = SyncExampleNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
