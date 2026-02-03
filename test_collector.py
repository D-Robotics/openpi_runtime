import sys
import os
import time
import threading

# 添加路径
sys.path.append("/mnt/disk1/wang.liu/env_rdks600/test/cc_ws/tros_ws/src/openpi_runtime")

from common.utils.ros_data_collector import ROSDataCollector, ImageBuffer

class MockNode:
    def __init__(self):
        self.count = 0
        
    def spin_once(self):
        self.count += 1
        time.sleep(0.001)

def mock_callback():
    return {"timestamp": time.time()}

def test_collector():
    print("Testing ROSDataCollector...")
    node = MockNode()
    collector = ROSDataCollector(node, mock_callback)
    
    print("Starting collector...")
    collector.start()
    
    # 给一点时间让线程运行
    time.sleep(0.5)
    
    # 获取数据
    obs = collector.get_latest_observation()
    print(f"Latest observation: {obs}")
    
    if obs is None:
        print("FAIL: Observation is None")
        sys.exit(1)
        
    if "timestamp" not in obs:
        print("FAIL: Invalid observation data")
        sys.exit(1)
        
    # 验证 spin_once 是否被调用
    if node.count == 0:
        print("FAIL: spin_once not called")
        sys.exit(1)
        
    print(f"spin_once called {node.count} times")
        
    print("Stopping collector...")
    collector.stop()
    
    if collector.is_alive():
        print("FAIL: Thread failed to stop")
        sys.exit(1)
        
    print("SUCCESS: ROSDataCollector test passed.")

if __name__ == "__main__":
    test_collector()
