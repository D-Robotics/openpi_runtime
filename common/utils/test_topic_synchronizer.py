#!/usr/bin/env python3
"""
测试话题时间同步器的基本功能
"""

import sys
import os
import time
import numpy as np
from pathlib import Path

# 添加项目路径
project_root = Path(__file__).parent.parent.parent
sys.path.append(str(project_root))

from common.utils.topic_time_synchronizer import TopicTimeSynchronizer


def test_basic_functionality():
    """测试基本功能"""
    print("🧪 测试话题时间同步器基本功能...")

    # 初始化同步器
    sync = TopicTimeSynchronizer(
        topics=["cam_high", "cam_left", "qpos"],
        reference_topic="cam_high",
        sync_window_ms=50,
        enable_statistics=True,
    )

    # 模拟数据
    base_time = int(time.time() * 1_000_000_000)

    # 添加图像数据
    cam_high_data = np.random.randint(0, 255, (3, 224, 224), dtype=np.uint8)
    cam_left_data = np.random.randint(0, 255, (3, 224, 224), dtype=np.uint8)
    qpos_data = np.array([0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7], dtype=np.float32)

    # 添加数据到同步器
    sync.add_message("cam_high", cam_high_data, base_time)
    sync.add_message("cam_left", cam_left_data, base_time + 10_000_000)  # 10ms later
    sync.add_message("qpos", qpos_data, base_time + 20_000_000)  # 20ms later

    # 获取同步数据
    synced_data = sync.get_synced_data(max_age_ms=100)

    if synced_data is not None:
        print("✅ 同步成功！")
        print(f"   基准时间戳: {synced_data.reference_timestamp_ns}")
        print(f"   数据数量: {len(synced_data.data)}")
        print(f"   时间偏移: {synced_data.time_offsets_ns}")

        # 打印统计信息
        stats = sync.get_statistics()
        print(f"   同步成功率: {stats['sync_success_rate'] * 100:.1f}%")
        return True
    else:
        print("❌ 同步失败")
        return False


def test_time_window():
    """测试时间窗口限制"""
    print("\n🧪 测试时间窗口限制...")

    sync = TopicTimeSynchronizer(
        topics=["cam_high", "qpos"],
        reference_topic="cam_high",
        sync_window_ms=10,  # 很小的窗口
        enable_statistics=True,
    )

    base_time = int(time.time() * 1_000_000_000)

    # 添加超出时间窗口的数据
    cam_data = np.random.randint(0, 255, (3, 224, 224), dtype=np.uint8)
    qpos_data = np.array([0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7], dtype=np.float32)

    sync.add_message("cam_high", cam_data, base_time)
    sync.add_message("qpos", qpos_data, base_time + 50_000_000)  # 50ms later，超出窗口

    synced_data = sync.get_synced_data(max_age_ms=100)

    if synced_data is None:
        print("✅ 时间窗口限制正常工作")
        return True
    else:
        print("❌ 时间窗口限制失败")
        return False


def test_statistics():
    """测试统计信息"""
    print("\n🧪 测试统计信息...")

    sync = TopicTimeSynchronizer(
        topics=["cam1", "cam2", "qpos"],
        reference_topic="cam1",
        sync_window_ms=100,
        enable_statistics=True,
    )

    # 添加多次数据
    for i in range(5):
        base_time = int(time.time() * 1_000_000_000) + i * 100_000_000

        cam1_data = np.random.randint(0, 255, (3, 224, 224), dtype=np.uint8)
        cam2_data = np.random.randint(0, 255, (3, 224, 224), dtype=np.uint8)
        qpos_data = np.random.rand(7).astype(np.float32)

        sync.add_message("cam1", cam1_data, base_time)
        sync.add_message("cam2", cam2_data, base_time + 10_000_000)
        sync.add_message("qpos", qpos_data, base_time + 20_000_000)

        # 获取同步数据
        synced_data = sync.get_synced_data(max_age_ms=200)

        if synced_data:
            print(f"   第 {i + 1} 次同步成功")

    # 打印统计信息
    stats = sync.get_statistics()
    print(f"✅ 统计信息:")
    print(f"   总同步尝试: {stats['sync_attempts']}")
    print(f"   成功同步次数: {stats['successful_syncs']}")
    print(f"   成功率: {stats['sync_success_rate'] * 100:.1f}%")
    print(f"   平均时间偏移: {stats['average_time_offset_ms']}")

    return True


def main():
    """主测试函数"""
    print("🚀 开始测试话题时间同步器...\n")

    tests = [test_basic_functionality, test_time_window, test_statistics]

    passed = 0
    total = len(tests)

    for test in tests:
        try:
            if test():
                passed += 1
        except Exception as e:
            print(f"❌ 测试异常: {e}")

    print(f"\n📊 测试结果: {passed}/{total} 通过")

    if passed == total:
        print("🎉 所有测试通过！")
        return 0
    else:
        print("⚠️  部分测试失败")
        return 1


if __name__ == "__main__":
    sys.exit(main())
