#!/usr/bin/env python3

"""
run_pi0_s600.py - 机器人控制推理脚本

功能：
1. 订阅ROS2话题获取机械臂状态和摄像头图像数据
2. 对数据进行前处理
3. 通过Socket发送数据到推理服务器进行模型推理
4. 接收推理结果并进行后处理
5. 将处理后的动作发布到ROS2话题控制机械臂

使用方法：
python3 run_pi0_s600.py

核心流程：
1. 初始化ROS2节点和订阅器
2. 等待获取初始数据（机械臂状态和图像）
3. 加载归一化统计信息
4. 启动推理服务器
5. 主循环：
   a. 获取观测数据（机械臂状态和图像）
   b. 前处理数据
   c. 发送数据到推理服务器
   d. 接收并后处理推理结果
   e. 发布控制动作
6. 清理资源
"""

# 标准库导入
import os
import json
import dataclasses
import enum
import logging
import time
import sys
import threading
from typing import Dict, Optional, Union, List, Tuple

# 第三方库导入
import numpy as np
import cv2

# ROS2相关导入
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
import cv_bridge

# 添加项目根目录到Python路径，确保可以导入自定义模块
sys.path.append(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(os.path.join(os.path.dirname(os.path.abspath(__file__)), 'demo'))
sys.path.append(os.path.join(os.path.dirname(os.path.abspath(__file__)), 'test'))

# 导入自定义模块
import msg_pb2  # Protocol Buffers生成的消息定义，用于数据序列化/反序列化
from preprocess import preprocess  # 数据前处理函数，处理图像和状态数据
from postprocess import postprocess  # 数据后处理函数，处理推理结果
from test_postprocess_consistency import Server  # 推理服务器类，用于与推理客户端通信

# 创建logger实例，用于日志记录
logger = logging.getLogger(__name__)

class EnvMode(enum.Enum):
    """支持的模式枚举类"""
    PIPER = "piper"  # 连接真实Piper机械臂

@dataclasses.dataclass
class Args:
    """命令行参数配置"""
    num_steps: int = 50*25  # 运行总步数，默认为50*25=1250步
    env: EnvMode = EnvMode.PIPER  # 运行模式，默认为连接真实Piper机械臂
    verbose: bool = False  # 是否打印详细推理结果，默认为False
    
    # 图像话题配置 - 只配置实际存在的两个摄像头
    camera_topics: Dict[str, str] = dataclasses.field(default_factory=lambda: {
        "cam_high": "/camera/camera/color/image_raw",  # 顶部摄像头话题
        "cam_left_wrist": "/camera_left/camera_left/color/image_raw",  # 左手腕摄像头话题
    })
    
    # 服务器期望的所有摄像头（4个），包括实际存在的和需要模拟的
    expected_cameras: tuple = ("cam_high", "cam_low", "cam_left_wrist", "cam_right_wrist")
    
    # 机械臂状态话题
    qpos_topic: str = "/piper/qpos"  # 机械臂关节角度话题，包含7个关节的角度
    
    # 动作发布话题
    action_topic: str = "/aliciaD/action"  # 控制命令发布话题，发布机械臂的控制动作
    
    action_chunk_size: int = 50  # 策略返回的动作序列长度，每次推理返回50个动作
    wait_timeout: float = 10.0  # 等待数据的超时时间（秒），超过此时间未收到数据则报错
    sync_time_window: float = 0.1  # 图像同步时间窗口（秒），用于确保不同摄像头图像的时间同步
    
    # 归一化统计信息路径，用于数据预处理
    norm_stats_path: str = "norm_stats.json"
    # norm_stats_path: str = "/mnt/wang.liu/lw/project/oellm_runtime/model/put_the_yellow_mango_on_the_blue_plate_model/norm_stats.json"


    

class ImageBuffer:
    """图像缓冲区类，用于存储和处理摄像头图像
    
    功能：
    1. 为每个摄像头维护一个图像队列
    2. 存储图像及其时间戳
    3. 提供获取同步图像的方法
    4. 线程安全设计，支持多线程访问
    """
    
    def __init__(self, max_size=10):
        """初始化图像缓冲区
        
        参数:
            max_size: int - 每个摄像头的最大缓存图像数量，默认为10
        """
        self.buffer = {}  # 存储每个摄像头的图像数据，格式为 {camera_name: [image1, image2, ...]}
        self.timestamps = {}  # 存储每个图像的时间戳，格式为 {camera_name: [ts1, ts2, ...]}
        self.lock = threading.Lock()  # 线程锁，保证线程安全
        self.max_size = max_size  # 最大缓存大小
    
    def add_image(self, camera_name: str, image_data: np.ndarray, timestamp: float):
        """添加图像到缓冲区
        
        参数:
            camera_name: str - 摄像头名称
            image_data: np.ndarray - 图像数据，格式为 (3, 224, 224)
            timestamp: float - 图像时间戳，使用系统时间
        """
        with self.lock:
            # 如果该摄像头尚未在缓冲区中，初始化
            if camera_name not in self.buffer:
                self.buffer[camera_name] = []
                self.timestamps[camera_name] = []
            
            # 添加图像和时间戳
            self.buffer[camera_name].append(image_data)
            self.timestamps[camera_name].append(timestamp)
            
            # 保持缓冲区大小，超出则删除最早的图像
            if len(self.buffer[camera_name]) > self.max_size:
                self.buffer[camera_name].pop(0)
                self.timestamps[camera_name].pop(0)
    
    def get_latest_images(self, camera_names: List[str], time_window: float = 0.1) -> Dict[str, np.ndarray]:
        """获取同步的最新图像
        
        功能：
        1. 检查所有指定摄像头是否有数据
        2. 获取每个摄像头的最新图像
        3. 检查图像时间戳是否在同步窗口内
        4. 返回同步的图像字典
        
        参数:
            camera_names: List[str] - 需要获取图像的摄像头名称列表
            time_window: float - 图像同步时间窗口（秒），默认为0.1秒
            
        返回:
            Dict[str, np.ndarray] - 每个摄像头的最新图像，如果任何摄像头无数据则返回None
        """
        with self.lock:
            # 检查所有摄像头都有数据
            for cam in camera_names:
                if cam not in self.buffer or len(self.buffer[cam]) == 0:
                    return None
            
            # 获取每个摄像头的最新图像和时间戳
            latest_timestamps = {}
            latest_images = {}
            
            for cam in camera_names:
                latest_images[cam] = self.buffer[cam][-1]  # 获取最新图像
                latest_timestamps[cam] = self.timestamps[cam][-1]  # 获取对应时间戳
            
            # 检查时间同步性
            timestamps = list(latest_timestamps.values())
            max_time_diff = max(timestamps) - min(timestamps)
            
            # 如果时间差异过大，记录警告但仍然返回图像
            if max_time_diff > time_window:
                logger.warning(f"图像时间不同步: 最大差异 {max_time_diff*1000:.1f}ms > {time_window*1000:.1f}ms")
            
            return latest_images
    
    def clear(self):
        """清空缓冲区
        
        功能：
        1. 清空所有摄像头的图像数据
        2. 清空所有时间戳数据
        """
        with self.lock:
            self.buffer.clear()
            self.timestamps.clear()

class PiperROSNode(Node):
    """ROS2节点类，用于订阅图像和机械臂状态，发布动作
    
    功能：
    1. 订阅摄像头图像话题
    2. 订阅机械臂关节角度话题
    3. 处理接收到的图像和状态数据
    4. 提供获取同步图像和状态的方法
    5. 发布控制动作到机械臂
    """
    
    def __init__(self, args: Args):
        """初始化ROS2节点
        
        参数:
            args: Args - 命令行参数配置
        """
        super().__init__('piper_ros_node')
        
        # 存储参数
        self.args = args
        self.bridge = cv_bridge.CvBridge()  # 用于ROS图像消息和OpenCV图像之间的转换
        
        # 存储最新的机械臂状态数据
        self.latest_qpos = np.zeros(7, dtype=np.float32)  # 7维关节角度 [j1, j2, j3, j4, j5, j6, gripper]
        self.qpos_received = False  # 是否收到机械臂状态
        self.qpos_valid = False  # 机械臂状态是否有效
        
        # 图像缓冲区
        self.image_buffer = ImageBuffer(max_size=2)  # 最大缓存10张图像
        
        # ==================== 订阅器 ====================
        # 只订阅实际存在的两个摄像头话题
        self.camera_topics = args.camera_topics
        self.image_subscribers = {}  # 存储图像订阅器
        self.image_received = {cam: False for cam in self.camera_topics.keys()}  # 记录每个摄像头是否收到数据
        
        for cam_name, topic in self.camera_topics.items():
            # 设置合适的QoS配置（图像数据使用BEST_EFFORT，提高实时性）
            qos_profile = QoSProfile(
                reliability=QoSReliabilityPolicy.BEST_EFFORT,  # 图像数据允许丢失
                durability=QoSDurabilityPolicy.VOLATILE,  # 不存储历史数据
                depth=10  # 队列深度为10
            )
            
            # 创建图像订阅器
            self.image_subscribers[cam_name] = self.create_subscription(
                Image,
                topic,
                lambda msg, cam=cam_name: self._image_callback(msg, cam),  # 使用lambda函数传递摄像头名称
                qos_profile
            )
            logger.info(f"已订阅摄像头话题 {cam_name}: {topic}")
        
        # 订阅机械臂状态话题
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,  # 机械臂状态需要可靠传输
            durability=QoSDurabilityPolicy.VOLATILE,  # 不存储历史数据
            depth=10  # 队列深度为10
        )
        
        self.qpos_subscriber = self.create_subscription(
            Float32MultiArray,
            args.qpos_topic,
            self._qpos_callback,
            qos_profile
        )
        logger.info(f"已订阅机械臂状态话题: {args.qpos_topic}")
        
        # ==================== 发布器 ====================
        # 发布动作到aliciaD
        try:
            self.action_publisher = self.create_publisher(
                Float32MultiArray,  # 动作消息类型
                args.action_topic,  # 动作发布话题
                60  # 队列深度
            )
            logger.info(f"已创建动作发布话题: {args.action_topic}")
            self.publisher_created = True
        except Exception as e:
            logger.error(f"创建动作发布话题失败: {e}")
            self.publisher_created = False
        
        # 消息计数器
        self.image_count = {cam: 0 for cam in self.camera_topics.keys()}  # 每个摄像头的消息计数
        self.qpos_count = 0  # 机械臂状态消息计数
        
        # 启动时间
        self.start_time = time.time()
    
    def _image_callback(self, msg: Image, cam_name: str):
        """图像回调函数，处理接收到的图像消息
        
        功能：
        1. 接收ROS2图像消息
        2. 转换为OpenCV格式
        3. 调整图像大小为224x224
        4. 转换为CHW格式（通道在前）
        5. 添加到图像缓冲区
        6. 更新计数和状态
        
        【数据采集点】：实时采集摄像头图像数据，每次收到图像消息时更新
        【更新频率】：取决于摄像头发布频率，通常为30Hz
        
        参数:
            msg: Image - ROS2图像消息
            cam_name: str - 摄像头名称
        """
        try:
            current_time = time.time()
            
            # 将ROS图像消息转换为OpenCV格式（RGB8）
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
            
            # 调整图像大小为224x224，模型期望的输入尺寸
            cv_image = cv2.resize(cv_image, (224, 224))
            
            # 转换为CHW格式（通道在前），模型期望的输入格式
            image_chw = cv_image.transpose(2, 0, 1).astype(np.uint8)  # 形状变为 (3, 224, 224)
            
            # 添加到图像缓冲区
            self.image_buffer.add_image(cam_name, image_chw, current_time)
            
            # 更新计数
            self.image_count[cam_name] += 1
            
            # 标记该摄像头已收到数据
            self.image_received[cam_name] = True
            
            # 每30帧打印一次信息
            if self.image_count[cam_name] % 30 == 0:
                logger.info(f"收到 {cam_name} 图像 (总数: {self.image_count[cam_name]})")
                
        except Exception as e:
            logger.error(f"处理 {cam_name} 图像回调失败: {e}")
            raise
    
    def _qpos_callback(self, msg: Float32MultiArray):
        """机械臂状态回调函数，处理接收到的关节角度消息
        
        功能：
        1. 接收机械臂关节角度消息
        2. 检查数据维度是否足够
        3. 提取前7维关节角度数据
        4. 更新状态和计数
        5. 定期打印信息
        
        【数据采集点】：实时采集机械臂关节角度数据，每次收到状态消息时更新
        【更新频率】：取决于机械臂控制器发布频率，通常为50Hz
        【数据内容】：7维关节角度 [j1, j2, j3, j4, j5, j6, gripper]
        
        参数:
            msg: Float32MultiArray - 机械臂关节角度消息
        """
        try:
            # 检查数据维度是否足够（至少需要7个关节角度）
            if len(msg.data) >= 7:
                # 提取前7维关节角度数据 [j1, j2, j3, j4, j5, j6, gripper]
                self.latest_qpos = np.array(msg.data[:7], dtype=np.float32)
                self.qpos_received = True  # 标记已收到数据
                self.qpos_valid = True  # 标记数据有效
                self.qpos_count += 1  # 更新计数
                
                # 每50次打印一次信息
                if self.qpos_count % 50 == 0:
                    logger.info(f"收到机械臂状态 (总数: {self.qpos_count}): {self.latest_qpos}")
            else:
                logger.error(f"qpos数据维度不足: {len(msg.data)} < 7")
                self.qpos_valid = False  # 标记数据无效
                raise ValueError(f"qpos数据维度不足: {len(msg.data)} < 7")
                
        except Exception as e:
            logger.error(f"处理qpos回调失败: {e}")
            self.qpos_valid = False  # 发生异常时标记数据无效
            raise
    
    def get_synced_images(self) -> Dict[str, np.ndarray]:
        """获取图像数据并构建服务器期望的4摄像头格式
        
        功能：
        1. 获取实际存在的摄像头的同步图像
        2. 检查图像数据是否有效
        3. 为缺失的摄像头创建黑图
        4. 构建完整的4摄像头图像字典
        
        返回:
            Dict[str, np.ndarray] - 包含所有4个摄像头图像的数据字典，格式为 {camera_name: image_data}
        """
        # 获取实际存在的摄像头的图像
        actual_cameras = list(self.camera_topics.keys())  # 获取配置中实际存在的摄像头
        latest_images = self.image_buffer.get_latest_images(actual_cameras, self.args.sync_time_window)
        
        if latest_images is None:
            raise RuntimeError("无法获取实际摄像头的图像数据")
        
        images = {}
        
        # 处理实际存在的摄像头
        for cam_key in actual_cameras:
            if cam_key in latest_images:
                images[cam_key] = latest_images[cam_key]
            else:
                raise RuntimeError(f"必须的摄像头 {cam_key} 图像数据缺失")
        
        # 为缺失的摄像头创建黑图（服务器期望4个摄像头，实际只有2个）
        for cam_key in self.args.expected_cameras:
            if cam_key not in images:
                logger.info(f"为缺失的摄像头 {cam_key} 创建黑图")
                images[cam_key] = np.zeros((3, 224, 224), dtype=np.uint8)  # 创建黑图
        
        return images
    
    def get_qpos(self) -> np.ndarray:
        """获取机械臂状态
        
        功能：
        1. 检查机械臂状态是否有效
        2. 返回最新的关节角度数据副本
        
        返回:
            np.ndarray - 7维关节角度数据 [j1, j2, j3, j4, j5, j6, gripper]
        """
        if not self.qpos_valid:
            raise RuntimeError("机械臂状态无效或未收到")
        return self.latest_qpos.copy()  # 返回副本，避免外部修改
    
    def publish_action(self, action: np.ndarray):
        """发布动作到aliciaD
        
        功能：
        1. 检查发布器是否创建成功
        2. 检查动作维度是否足够
        3. 提取前7维动作数据
        4. 创建ROS2消息
        5. 发布消息
        6. 打印详细信息（如果启用了verbose模式）
        
        参数:
            action: np.ndarray - 动作数据，包含至少7维 [j1, j2, j3, j4, j5, j6, gripper]
        """
        if not self.publisher_created:
            raise RuntimeError("动作发布话题未创建")
        
        try:
            # 检查动作维度是否足够（至少需要7个维度）
            if len(action) < 7:
                raise ValueError(f"动作维度不足: {len(action)} < 7")
            
            # 提取前7维（机械臂只需要前7维）
            action_7d = action[:7]  # [j1, j2, j3, j4, j5, j6, gripper]
            
            # 创建ROS2消息
            msg = Float32MultiArray()
            msg.data = action_7d.tolist()  # 转换为列表格式
            
            # 发布消息
            self.action_publisher.publish(msg)
            
            # 打印发布的动作话题信息
            # logger.info(f"发布动作到话题 {self.args.action_topic}: {action_7d}")
                
        except Exception as e:
            logger.error(f"发布动作失败: {e}")
            raise
    
    def spin_once(self):
        """处理一次ROS2回调，非阻塞
        
        功能：
        1. 调用rclpy.spin_once处理一次回调
        2. 设置超时时间为0.001秒，确保非阻塞
        """
        rclpy.spin_once(self, timeout_sec=0.001)  # 非阻塞调用，最多等待1ms
    
    def wait_for_arm_movement(self, target_action: np.ndarray, timeout: float = 2.0, tolerance: float = 0.01):
        """等待机械臂运动到位
        
        功能：
        1. 记录目标动作的关节角度
        2. 循环检查当前关节角度与目标角度的误差
        3. 如果误差小于阈值，或者超过最大等待时间，则返回
        
        参数:
            target_action: np.ndarray - 目标动作的关节角度
            timeout: float - 最大等待时间（秒），默认为2秒
            tolerance: float - 位置误差阈值（弧度），默认为0.01
            
        返回:
            bool - 是否在超时前运动到位
        """
        start_time = time.time()
        target_joints = target_action[:6]  # 只关注前6个关节角度
        
        logger.info(f"等待机械臂运动到位...")
        logger.info(f"  目标关节角度: {target_joints}")
        
        while time.time() - start_time < timeout:
            # 处理ROS2回调，获取最新的关节角度
            self.spin_once()
            
            # 获取当前关节角度
            current_joints = self.latest_qpos[:6]  # 只关注前6个关节角度
            
            # 计算位置误差
            position_error = np.abs(current_joints - target_joints)
            max_error = np.max(position_error)
            
            # logger.info(f"  当前关节角度: {current_joints}")
            # logger.info(f"  位置误差: {position_error}")
            # logger.info(f"  最大误差: {max_error:.4f}, 阈值: {tolerance}")
            
            # 如果误差小于阈值，认为运动到位
            if max_error < tolerance:
                logger.info(f"  ✓ 机械臂运动到位，最大误差: {max_error:.4f} < {tolerance}")
                return True
            
            # 小延时，避免CPU占用过高
            time.sleep(0.05)  # 50ms
        
        # 超时
        logger.warning(f"  ⚠ 机械臂运动超时 ({timeout}秒)，最大误差: {max_error:.4f} > {tolerance}")
        return False
    
    def wait_for_data(self, timeout: float) -> bool:
        """等待所有数据就绪
        
        功能：
        1. 循环等待直到所有数据就绪或超时
        2. 定期处理ROS2回调
        3. 检查所有必须的摄像头是否都收到了数据
        4. 检查机械臂状态是否收到且有效
        5. 检查发布器是否创建成功
        6. 定期打印等待进度
        
        参数:
            timeout: float - 超时时间（秒）
            
        返回:
            bool - 是否在超时前获取到所有数据
        """
        start_time = time.time()
        
        while time.time() - start_time < timeout:
            # 处理一次ROS2回调
            self.spin_once()
            
            # 检查所有必须的摄像头是否都收到了数据
            all_images_received = all(self.image_received.values())
            
            # 检查机械臂状态是否收到且有效
            qpos_ready = self.qpos_received and self.qpos_valid
            
            # 检查发布器是否创建成功
            publisher_ready = self.publisher_created
            
            # 如果所有数据都就绪，返回True
            if all_images_received and qpos_ready and publisher_ready:
                logger.info("所有数据都已就绪！")
                logger.info(f"摄像头接收情况: {self.image_received}")
                logger.info(f"机械臂状态已接收: {self.qpos_received}")
                logger.info(f"动作发布器已创建: {self.publisher_created}")
                return True
            
            # 每2秒打印一次等待进度
            elapsed = time.time() - start_time
            if elapsed % 2.0 < 0.1:  # 每2秒打印一次
                logger.info(f"等待数据中... 已等待 {elapsed:.1f}秒")
                logger.info(f"摄像头接收情况: {self.image_received}")
                logger.info(f"机械臂状态已接收: {self.qpos_received} (有效: {self.qpos_valid})")
                logger.info(f"动作发布器已创建: {self.publisher_created}")
            
            # 小延时，避免CPU占用过高
            time.sleep(0.01)  # 10ms延时
        
        # 超时，返回False
        logger.error(f"等待数据超时 ({timeout}秒)")
        logger.error(f"摄像头接收情况: {self.image_received}")
        logger.error(f"机械臂状态已接收: {self.qpos_received} (有效: {self.qpos_valid})")
        logger.error(f"动作发布器已创建: {self.publisher_created}")
        return False

def _get_piper_observation(ros_node: PiperROSNode) -> dict:
    """获取Piper观测数据
    
    功能：
    1. 获取机械臂状态
    2. 构建14维状态向量（左臂7维 + 右臂7维填充0）
    3. 获取同步的图像数据
    4. 检查图像数据完整性和形状
    5. 返回完整的观测数据字典
    
    【数据整合点】：整合最新的机械臂状态和同步的图像数据
    【调用时机】：当动作队列为空时调用，准备新的推理数据
    【数据同步】：确保不同摄像头图像的时间戳在同步窗口内
    
    参数:
        ros_node: PiperROSNode - ROS2节点实例
        
    返回:
        dict - 包含状态、图像和提示词的观测数据
            "state": 14维状态向量 [左臂7维, 右臂7维(0填充)]
            "images": 4个摄像头的图像数据 {camera_name: image_data}
            "prompt": 任务提示词
    """
    # 获取机械臂状态
    try:
        left_arm_state = ros_node.get_qpos()  # 获取左臂7维关节角度
    except Exception as e:
        raise RuntimeError(f"获取机械臂状态失败: {e}")
    
    # 构建14维状态向量（左臂7维 + 右臂7维填充0）
    full_state = np.concatenate([left_arm_state, np.zeros(7)])  # 右臂填充0
    
    # 打印状态数据
    logger.info(f"发送到服务器的状态数据:")
    logger.info(f"  关节角度: {left_arm_state[:6]}")  # 前6个关节角度
    logger.info(f"  夹爪位置: {left_arm_state[6]:.3f}")  # 第7个是夹爪位置
    logger.info(f"  完整状态向量(14维): {full_state}")

    # 获取图像
    try:
        images = ros_node.get_synced_images()  # 获取4个摄像头的同步图像
    except Exception as e:
        raise RuntimeError(f"获取图像数据失败: {e}")
    
    # 检查是否包含所有需要的摄像头
    required_cameras = ros_node.args.expected_cameras
    missing_cameras = [cam for cam in required_cameras if cam not in images]
    
    if missing_cameras:
        raise RuntimeError(f"缺少以下摄像头的图像数据: {missing_cameras}")
    
    # 检查图像形状
    for cam_name, img in images.items():
        if img.shape != (3, 224, 224):  # 检查是否为 (3, 224, 224) 格式
            raise RuntimeError(f"摄像头 {cam_name} 图像形状错误: {img.shape}，期望 (3, 224, 224)")
    
    logger.info(f"成功获取所有 {len(images)} 个摄像头图像")
    
    # 返回观测数据字典
    return {
        "state": full_state.astype(np.float32),  # 转换为float32
        "images": images,  # 4个摄像头的图像数据
        "prompt": "put the box",  # TODO: 支持动态指令输入
    }

def main(args: Args) -> None:
    """主函数
    
    功能：
    1. 初始化ROS2
    2. 创建ROS2节点
    3. 等待初始数据就绪
    4. 加载归一化统计信息
    5. 启动推理服务器
    6. 主循环：
       a. 获取观测数据
       b. 前处理数据
       c. 执行推理
       d. 后处理结果
       e. 执行动作序列
    7. 处理异常和清理资源
    
    参数:
        args: Args - 命令行参数配置
    """
    
    # 初始化ROS2
    logger.info("初始化ROS2...")
    rclpy.init()
    
    try:
        # 创建ROS2节点
        ros_node = PiperROSNode(args)
    except Exception as e:
        logger.error(f"创建ROS节点失败: {e}")
        sys.exit(1)
    
    # 等待初始数据
    logger.info(f"等待初始数据 (超时: {args.wait_timeout}秒)...")
    
    if not ros_node.wait_for_data(args.wait_timeout):
        logger.error("无法获取到所有必要的数据，程序退出")
        try:
            ros_node.destroy_node()
        except:
            pass
        rclpy.shutdown()
        sys.exit(1)
    
    logger.info("所有数据准备就绪，开始连接服务器...")
    
    # 全局配置变量
    adapt_to_pi = False  # 是否将数据转换为Pi内部运行时使用的空间格式
    # adapt_to_pi = True  # 如需使用Pi内部空间格式，取消注释此行
    
    # 加载归一化统计信息
    norm_stats = None
    if os.path.exists(args.norm_stats_path):
        with open(args.norm_stats_path, 'r') as f:
            norm_stats = json.load(f)
        logger.info(f"✓ 加载归一化统计信息: {args.norm_stats_path}")
    else:
        logger.error(f"错误: {args.norm_stats_path} not found")
        raise FileNotFoundError(f"归一化统计信息文件不存在: {args.norm_stats_path}")
    
    # 初始化Server类并启动服务器（端口8888）
    server = Server(8888)
    server.start()
    
    # 主循环
    logger.info(f"开始运行 {args.num_steps} 步...")
    
    # 初始化一阶低通滤波器相关变量
    previous_filtered_action = None
    low_pass_alpha = 0.2  # 滤波系数，0-1之间，越接近1滤波效果越强
    logger.info(f"启用一阶低通滤波，滤波系数: {low_pass_alpha}")
    
    try:
        for step in range(args.num_steps):
            step_start = time.time()
            logger.info(f"\n=== 开始循环步骤 {step + 1}/{args.num_steps} ===")
            
            try:
                # 步骤0: 第一次手动确认 - 是否开始采集数据
                # logger.info(f"\n[步骤0] 手动确认...")
                # logger.info("请确认是否开始采集数据？")
                # confirm_start = input("输入 'y' 开始采集，输入其他键取消: ")
                # if confirm_start.lower() != 'y':
                #     logger.info("用户取消采集，跳过本次循环")
                #     continue
                logger.info("自动开始采集数据...")
                
                # 步骤1: 收集数据
                logger.info(f"\n[步骤1] 收集数据...")
                
                # 确保获取最新的数据
                data_collection_start = time.time()
                for _ in range(50):
                    ros_node.spin_once()
                    time.sleep(0.01)
                
                # 获取观测数据
                observation_start = time.time()
                observation = _get_piper_observation(ros_node)
                observation_time = time.time() - observation_start
                data_collection_time = time.time() - data_collection_start
                logger.info(f"✓ 数据收集完成，总耗时: {data_collection_time*1000:.1f}ms")
                logger.info(f"  其中观测数据获取耗时: {observation_time*1000:.1f}ms")
                
                # 步骤2: 前处理
                logger.info(f"\n[步骤2] 前处理...")
                preprocess_start = time.time()
                
                # 转换图像格式
                processed_images = {}
                for cam_name, img in observation["images"].items():
                    if img.shape[0] == 3:  # CHW格式
                        img_hwc = np.transpose(img, (1, 2, 0))
                    else:  # HWC格式
                        img_hwc = img
                    img_bgr = cv2.cvtColor(img_hwc, cv2.COLOR_RGB2BGR)
                    processed_images[cam_name] = img_bgr
                
                # 进行前处理
                preprocessed = preprocess(
                    images=processed_images,
                    state=observation["state"],
                    prompt=observation["prompt"],
                    adapt_to_pi=adapt_to_pi,
                    norm_stats=norm_stats["norm_stats"] if norm_stats else None
                )
                preprocess_time = time.time() - preprocess_start
                logger.info(f"✓ 前处理完成，耗时: {preprocess_time*1000:.1f}ms")
                
                # 步骤2.5: 显示采集的数据内容
                logger.info(f"\n[步骤2.5] 显示采集的数据...")
                logger.info(f"采集到的状态数据:")
                logger.info(f"  关节角度: {observation['state'][:6]}")
                logger.info(f"  夹爪位置: {observation['state'][6]:.3f}")
                logger.info(f"  完整状态向量: {observation['state']}")
                logger.info(f"采集到的图像数据:")
                for cam_name, img in observation['images'].items():
                    logger.info(f"  {cam_name}: 形状={img.shape}, 数据类型={img.dtype}")
                logger.info(f"任务提示词: {observation['prompt']}")
                
                # 步骤3: 推理
                logger.info(f"\n[步骤3] 推理...")
                
                infer_result = None
                infer_time = 0.0
                
                if hasattr(server, 'sock_fd') and server.sock_fd:
                    # 准备发送数据
                    pre_images = {
                        "1": preprocessed["image"]["base_0_rgb"],
                        "2": preprocessed["image"]["left_wrist_0_rgb"],
                        "3": preprocessed["image"]["right_wrist_0_rgb"]
                    }
                    
                    obs = {
                        "images": pre_images,
                        "prompt": preprocessed["tokenized_prompt"].astype(np.int32),
                        "state": preprocessed["state"].astype(np.float32)
                    }
                    
                    dtypes = {
                        "images": msg_pb2.Tensor.FLOAT32,
                        "prompt": msg_pb2.Tensor.INT32,
                        "state": msg_pb2.Tensor.FLOAT32
                    }
                    
                    # 打印state数据
                    logger.info(f"发送到服务器的state数据: {obs['state']}")
                    
                    # 步骤3.5: 第二次手动确认 - 是否发送数据
                    # logger.info("\n[步骤3.5] 手动确认...")
                    # logger.info("请确认是否发送推理数据？")
                    # confirm_send = input("输入 'y' 发送数据，输入其他键重新采集: ")
                    # if confirm_send.lower() != 'y':
                    #     logger.info("用户取消发送，重新采集数据")
                    #     # 跳过后续步骤，重新开始循环
                    #     continue
                    logger.info("自动发送推理数据...")
                    
                    # 发送数据并接收结果
                    inference_start = time.time()

                    
                    server.send(obs, dtypes)
                    result = server.receive()


                    
                    if result and "raw_outputs" in result:
                        infer_result = result["raw_outputs"]
                        infer_time = (time.time() - inference_start) * 1000
                        logger.info(f"✓ 推理完成，耗时: {infer_time:.2f} ms")
                        logger.info(f"✓ 推理结果形状: {infer_result.shape}")
                    else:
                        logger.error(f"✗ 未获取到有效推理结果")
                        continue
                else:
                    logger.error(f"✗ 未检测到推理客户端连接")
                    continue
                
                # 步骤4: 后处理
                logger.info(f"\n[步骤4] 后处理...")
                postprocess_start = time.time()
                
                # 处理批次维度
                if infer_result.ndim == 3 and infer_result.shape[0] == 1:
                    infer_result = infer_result.squeeze(0)
                elif infer_result.ndim > 2:
                    infer_result = infer_result.reshape(-1, 32)
                
                # 执行后处理
                # 获取当前state数据用于Delta动作还原
                current_state = observation["state"]
                logger.info(f"用于后处理的state数据: {current_state}")
                
                processed_actions = postprocess(
                    actions=infer_result,
                    adapt_to_pi=adapt_to_pi,
                    save_result=False,
                    norm_stats=norm_stats,
                    state=current_state,
                    skip_delta_restore=False  # 使用Delta动作还原
                )
                
                # 验证动作格式
                if not isinstance(processed_actions, np.ndarray):
                    raise TypeError(f"动作类型错误: {type(processed_actions)}")
                
                # 保存推理结果为.npy文件
                # save_path = f"infer_result_{step+1}.npy"
                # np.save(save_path, processed_actions)
                postprocess_time = time.time() - postprocess_start
                logger.info(f"✓ 后处理完成，耗时: {postprocess_time*1000:.1f}ms")
                logger.info(f"  获取动作序列: 形状 {processed_actions.shape}, 步数 {len(processed_actions)}")
                # logger.info(f"  推理结果已保存到: {save_path}")
                
                # 步骤5: 执行所有动作
                logger.info(f"\n[步骤5] 执行动作序列...")
                execution_start = time.time()
                
                for i, action in enumerate(processed_actions):
                    logger.info(f"  执行动作 {i + 1}/{len(processed_actions)}")
                    try:
                        # 计算当前时间步的滤波系数
                        # 随着时间步接近末尾，逐渐增加滤波系数，确保最后能够到达目标位置
                        steps_total = len(processed_actions)
                        steps_remaining = steps_total - i
                        
                        if steps_remaining <= 1:
                            # 最后一个时间步，完全使用原始动作，确保准确到达目标位置
                            current_alpha = 1.0
                            filtered_action = action
                            logger.info("  最后一个时间步，使用原始动作确保到达目标位置")
                        elif steps_remaining <= 5:
                            # 接近末尾的5个时间步，逐渐增加滤波系数
                            # 从0.4线性增加到1.0
                            current_alpha = 0.4 + (1.0 - 0.4) * ((5 - steps_remaining) / 4)
                            if previous_filtered_action is None:
                                filtered_action = action
                            else:
                                filtered_action = current_alpha * action + (1 - current_alpha) * previous_filtered_action
                            logger.info(f"  接近末尾，使用动态滤波系数: {current_alpha:.2f}")
                        else:
                            # 其他时间步，使用默认滤波系数
                            current_alpha = low_pass_alpha
                            if previous_filtered_action is None:
                                filtered_action = action
                            else:
                                filtered_action = current_alpha * action + (1 - current_alpha) * previous_filtered_action
                        
                        # 更新上一次的滤波动作
                        previous_filtered_action = filtered_action
                        
                        logger.info(f"  原始动作: {action}")
                        logger.info(f"  滤波后动作: {filtered_action}")
                        
                        # 注释掉发送动作的代码
                        ros_node.publish_action(filtered_action)
                        # logger.info(f"  动作已准备好，但未发送: {filtered_action}")
                    except Exception as e:
                        logger.error(f"发布动作失败: {e}")
                    
                    # 小延时，确保动作按顺序执行
                    time.sleep(0.01)
                
                execution_time = time.time() - execution_start
                logger.info(f"✓ 动作序列执行完成，共 {len(processed_actions)} 步，耗时: {execution_time*1000:.1f}ms")
                
                # 步骤6: 等待动作发送完毕
                # logger.info(f"\n[步骤6] 等待动作发送完毕...")
                # time.sleep(0.5)
                logger.info(f"✓ 动作发送完毕")
                
                # 步骤7: 延时1秒
                # logger.info(f"\n[步骤7] 延时1秒...")
                # time.sleep(1.0)
                # logger.info(f"✓ 延时完成")
                
                # 处理ROS2回调
                ros_node.spin_once()
                
                step_time = time.time() - step_start
                logger.info(f"\n=== 循环步骤 {step + 1} 完成，耗时: {step_time:.2f}s ===")
                
                # 打印时间汇总
                logger.info(f"\n=== 循环步骤 {step + 1} 时间汇总 ===")
                logger.info(f"总耗时: {step_time*1000:.1f}ms")
                logger.info(f"  1. 数据收集: {data_collection_time*1000:.1f}ms ({data_collection_time/step_time*100:.1f}%)")
                logger.info(f"  2. 前处理: {preprocess_time*1000:.1f}ms ({preprocess_time/step_time*100:.1f}%)")
                logger.info(f"  3. 推理: {infer_time:.2f}ms ({infer_time/1000/step_time*100:.1f}%)")
                logger.info(f"  4. 后处理: {postprocess_time*1000:.1f}ms ({postprocess_time/step_time*100:.1f}%)")
                logger.info(f"  5. 执行: {execution_time*1000:.1f}ms ({execution_time/step_time*100:.1f}%)")
                
            except Exception as e:
                logger.error(f"循环步骤执行失败: {e}", exc_info=True)
                # 继续运行，下次循环再尝试
                continue
    
    except KeyboardInterrupt:
        logger.info("用户中断程序")
    except Exception as e:
        logger.error(f"主循环异常: {e}", exc_info=True)
    finally:
        # 清理资源
        logger.info("清理资源...")
        try:
            # 关闭服务器连接
            server.close()
            logger.info("  ✓ 服务器连接已关闭")
            
            # 打印消息统计信息
            counts = ros_node.image_count
            logger.info(f"摄像头消息统计: {counts}, qpos消息统计: {ros_node.qpos_count}")
            
            # 销毁ROS2节点
            ros_node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
            logger.info("ROS2节点已销毁")
        except Exception as e:
            logger.error(f"清理资源失败: {e}")
    
    logger.info("运行完成")

if __name__ == "__main__":
    """程序入口点"""
    # 配置日志
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s - %(levelname)s - %(message)s'
    )
    
    # 导入os模块（虽然在文件开头已经导入，但为了代码完整性再次导入）
    import os
    
    # 解析命令行参数
    args = Args()
    
    # 运行主函数
    logger.info("开始运行run_pi0_s600.py...")
    main(args)
