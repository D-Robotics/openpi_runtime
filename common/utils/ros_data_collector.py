import threading
import time
import logging
import collections
from typing import Dict, List, Optional, Callable, Any, Union

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy

logger = logging.getLogger(__name__)

class ROSDataCollector(threading.Thread):
    """
    通用ROS数据采集与时间同步类
    
    功能：
    1. 运行在独立线程中，处理ROS回调
    2. 支持动态添加需要订阅的话题
    3. 自动缓存接收到的消息
    4. 提供时间同步的数据获取接口
    """
    def __init__(self, ros_node: Node, max_buffer_size: int = 20):
        """
        Args:
            ros_node: ROS2 Node 实例
            max_buffer_size: 每个话题的最大缓存消息数
        """
        super().__init__()
        self.ros_node = ros_node
        self.max_buffer_size = max_buffer_size
        
        self.running = False
        self.lock = threading.Lock()
        self.daemon = True
        
        # 存储话题信息
        # 结构: topic_name -> {
        #   'buffer': deque([(timestamp, data), ...]), 
        #   'sub': subscriber, 
        #   'preprocess': func
        # }
        self.topics = {} 
        
        # 同步组配置
        self.sync_group = None
        self.sync_time_window = 0.1
        self.latest_synced_result = None

    def set_sync_group(self, topic_names: List[str], time_window: float = 0.1):
        """
        设置需要后台实时同步的话题组
        
        Args:
            topic_names: 话题别名列表
            time_window: 同步时间窗口
        """
        with self.lock:
            self.sync_group = topic_names
            self.sync_time_window = time_window
            logger.info(f"已设置同步组: {topic_names}, 窗口: {time_window}s")

    def add_topic(self, 
                  name: str, 
                  topic: str, 
                  msg_type: Any, 
                  preprocess_func: Optional[Callable[[Any], Any]] = None, 
                  qos_profile: Optional[QoSProfile] = None):
        """
        添加需要采集的话题
        
        Args:
            name: 话题别名（用于获取数据时的键）
            topic: ROS话题名称
            msg_type: 消息类型类
            preprocess_func: 预处理函数，接收msg返回处理后的数据。如果为None则直接存储msg
            qos_profile: QoS配置，如果为None则使用默认配置(Best Effort, Volatile)
        """
        if qos_profile is None:
             qos_profile = QoSProfile(
                reliability=QoSReliabilityPolicy.BEST_EFFORT,
                durability=QoSDurabilityPolicy.VOLATILE,
                depth=10
            )
            
        def callback(msg):
            recv_time = time.time() # 使用系统接收时间作为同步基准
            
            data = msg
            if preprocess_func:
                try:
                    data = preprocess_func(msg)
                except Exception as e:
                    logger.error(f"预处理话题 {name} 失败: {e}")
                    return
            
            with self.lock:
                if name in self.topics:
                    buf = self.topics[name]['buffer']
                    buf.append((recv_time, data))
                    if len(buf) > self.max_buffer_size:
                        buf.popleft()
                        
        sub = self.ros_node.create_subscription(
            msg_type,
            topic,
            callback,
            qos_profile
        )
        
        with self.lock:
            self.topics[name] = {
                'buffer': collections.deque(),
                'sub': sub,
                'preprocess': preprocess_func,
                'topic_name': topic
            }
        logger.info(f"已添加采集话题: {name} -> {topic}")

    def run(self):
        self.running = True
        logger.info("ROS数据采集线程已启动")
        
        while self.running:
            try:
                # 使用 timeout_sec 处理回调，避免阻塞
                rclpy.spin_once(self.ros_node, timeout_sec=0.005)
                
                # 执行后台同步
                self._sync_data_task()
                
            except Exception as e:
                logger.error(f"ROS spin 异常: {e}")
                time.sleep(0.01)

    def _sync_data_task(self):
        """后台同步任务"""
        with self.lock:
            if not self.sync_group:
                return
                
            topic_names = self.sync_group
            time_window = self.sync_time_window
            
            # 1. 检查所有话题是否存在且有数据
            for name in topic_names:
                if name not in self.topics or not self.topics[name]['buffer']:
                    return
            
            # 2. 获取每个话题的最新时间戳，并取最小值作为基准
            latest_timestamps = [self.topics[name]['buffer'][-1][0] for name in topic_names]
            ref_time = min(latest_timestamps)
            
            # 如果最新的同步结果的时间戳和当前的ref_time非常接近（或者就是它），可能不需要重复计算？
            # 但考虑到数据可能更新，这里还是简单地重新计算
            
            result = {}
            valid_sync = True
            
            # 3. 为每个话题寻找匹配数据
            for name in topic_names:
                buffer = self.topics[name]['buffer']
                best_match = None
                min_diff = float('inf')
                
                # 从后向前遍历
                for ts, data in reversed(buffer):
                    diff = abs(ts - ref_time)
                    
                    if diff <= time_window:
                        if diff < min_diff:
                            min_diff = diff
                            best_match = data
                    
                    if ts < ref_time - time_window:
                        break
                
                if best_match is not None:
                    result[name] = best_match
                else:
                    valid_sync = False
                    break
            
            if valid_sync:
                self.latest_synced_result = result

    def stop(self):
        self.running = False
        self.join()

    def get_latest_synced_data(self, topic_names: Optional[List[str]] = None, time_window: float = 0.1) -> Optional[Dict[str, Any]]:
        """
        获取指定话题的最新时间同步数据
        
        Args:
            topic_names: 需要获取的话题别名列表。如果为None，则返回后台自动同步的最新结果
            time_window: 同步时间窗口（秒），仅在 topic_names 不为 None 且不等于 sync_group 时生效
            
        Returns:
            成功则返回字典 {name: data}，失败返回 None
        """
        with self.lock:
            # 模式1: 返回后台实时同步的结果
            if topic_names is None or topic_names == self.sync_group:
                return self.latest_synced_result
            
            # 模式2: 按需计算同步结果（兼容旧逻辑）
            # 1. 检查所有话题是否存在且有数据
            for name in topic_names:
                if name not in self.topics:
                    logger.error(f"话题 {name} 未注册")
                    return None
                if not self.topics[name]['buffer']:
                    # logger.debug(f"话题 {name} 无数据")
                    return None
            
            # 2. 获取每个话题的最新时间戳，并取最小值作为基准
            latest_timestamps = [self.topics[name]['buffer'][-1][0] for name in topic_names]
            ref_time = min(latest_timestamps)
            
            result = {}
            
            # 3. 为每个话题寻找匹配数据
            for name in topic_names:
                buffer = self.topics[name]['buffer']
                best_match = None
                min_diff = float('inf')
                
                # 从后向前遍历（因为我们要找最新的，且大概率在末尾）
                for ts, data in reversed(buffer):
                    diff = abs(ts - ref_time)
                    
                    if diff <= time_window:
                        if diff < min_diff:
                            min_diff = diff
                            best_match = data
                        else:
                            # 如果差异开始变大，说明已经过了最佳匹配点（假设时间大致有序）
                            # 但由于网络抖动，时间可能不是严格单调的接收顺序，所以最好还是多看一点
                            pass
                    
                    # 优化：如果时间戳已经太旧，超出了窗口范围，停止搜索
                    if ts < ref_time - time_window:
                        break
                
                if best_match is not None:
                    result[name] = best_match
                else:
                    # 某个话题无法同步
                    return None
            
            return result
