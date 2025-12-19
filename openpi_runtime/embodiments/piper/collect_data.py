#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import h5py
import sys
import termios
import tty
import select
import threading
import time
from pathlib import Path
from datetime import datetime
import cv2

# 导入您的模块
from piper_sdk import C_PiperInterface_V2


class DataCollector:
    """简化版数据收集器：去掉时间同步管理器，增强错误处理"""
    
    def __init__(self):
        # 配置参数
        self.record_freq = 30.0  # 记录频率(Hz)
        self.min_frame_interval = 1.0 / self.record_freq
        self.time_sync_tolerance = 0.05  # 时间同步容差：50ms
        
        # 初始化ROS
        rclpy.init()
        self.node = Node('data_collection_node')
        self.logger = self.node.get_logger()
        
        # 初始化机械臂
        self.logger.info("连接机械臂...")
        self.piper = C_PiperInterface_V2("can0")
        self.piper.ConnectPort()
        # if not self.piper.ConnectPort():
        #     self.logger.error("机械臂连接失败！")
        #     sys.exit(1)
        
        # HDF5配置
        self.hdf5_config = {
            'image_width': 640,
            'image_height': 480,
            'action_dim': 14,
            'qpos_dim': 14,
            'left_arm_dim': 6,
            'right_arm_dim': 6,
        }
        
        # 数据缓冲区
        self.data_buffer: Dict[str, List] = {
            'cam_high': [],      # 全局摄像头图像
            'cam_left_wrist': [], # 左腕摄像头图像
            'action': [],        # 动作数据
            'timestamps': []     # 时间戳
        }
        
        # 最新图像缓存 (手动同步用)
        self.latest_images = {
            'cam_high': None,    # (image, timestamp)
            'cam_left_wrist': None
        }
        
        # 状态控制
        self.is_recording = False
        self.last_frame_time = 0.0
        self.buffer_lock = threading.Lock()
        self.episode_count = 0
        
        # 初始化图像订阅
        self.setup_image_subscriptions()
        
        # 启动键盘监听线程
        self.logger.info("启动键盘监听...")
        self.logger.info('按 "s" 开始记录，按 "d" 停止记录，按 "q" 退出程序')
        self.keyboard_thread = threading.Thread(target=self.keyboard_listener, daemon=True)
        self.keyboard_thread.start()
        
        # 设置终端为非阻塞输入
        self.setup_terminal()
        
        # 创建处理定时器
        self.timer = self.node.create_timer(0.01, self.process_frame)
        
    def setup_terminal(self):
        """设置终端为非阻塞输入模式"""
        self.fd = sys.stdin.fileno()
        self.old_settings = termios.tcgetattr(self.fd)
        tty.setcbreak(self.fd)
    
    def restore_terminal(self):
        """恢复终端设置"""
        termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old_settings)
        
    def setup_image_subscriptions(self):
        """设置图像订阅"""
        self.bridge = CvBridge()
        
        # 声明参数
        self.node.declare_parameter('camera_topic_name', '/camera/camera/color/image_raw')
        self.node.declare_parameter('camera_left_topic_name', '/camera_left/camera_left/color/image_raw')
        
        # 获取参数
        camera_topic = self.node.get_parameter('camera_topic_name').value
        camera_left_topic = self.node.get_parameter('camera_left_topic_name').value
        
        # 创建订阅
        self.node.create_subscription(
            Image,
            camera_topic,
            self.cam_high_callback,
            qos_profile_sensor_data
        )
        self.logger.info(f"已订阅: {camera_topic} -> cam_high")
        
        self.node.create_subscription(
            Image,
            camera_left_topic,
            self.cam_left_wrist_callback,
            qos_profile_sensor_data
        )
        self.logger.info(f"已订阅: {camera_left_topic} -> cam_left_wrist")
        
    def cam_high_callback(self, msg):
        """全局摄像头回调"""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg)
            timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            self.latest_images['cam_high'] = (cv_image, timestamp)
        except Exception as e:
            self.logger.error(f"cam_high回调失败: {e}")
            
    def cam_left_wrist_callback(self, msg):
        """左腕摄像头回调"""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg)
            timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            self.latest_images['cam_left_wrist'] = (cv_image, timestamp)
        except Exception as e:
            self.logger.error(f"cam_left_wrist回调失败: {e}")
            
    def process_frame(self):
        """处理帧数据（手动同步）"""
        if not self.is_recording:
            return
            
        # 检查帧率
        current_time = time.time()
        if current_time - self.last_frame_time < self.min_frame_interval:
            return
        
        # 检查两个摄像头是否都有最新数据
        if (self.latest_images['cam_high'] is None or 
            self.latest_images['cam_left_wrist'] is None):
            return
        
        # 检查时间同步
        ts_high = self.latest_images['cam_high'][1]
        ts_left = self.latest_images['cam_left_wrist'][1]
        time_diff = abs(ts_high - ts_left)
        
        if time_diff > self.time_sync_tolerance:
            # 时间不同步，丢弃旧的数据
            if ts_high < ts_left:
                self.latest_images['cam_high'] = None
            else:
                self.latest_images['cam_left_wrist'] = None
            return
        
        # 数据有效，准备记录
        self.last_frame_time = current_time
        
        # 读取机械臂状态
        try:
            joint_angles = self.read_arm_state()
        except Exception as e:
            self.logger.error(f"读取机械臂状态失败: {e}")
            return
        
        # 保存到缓冲区
        with self.buffer_lock:
            self.data_buffer['cam_high'].append(self.latest_images['cam_high'][0].copy())
            self.data_buffer['cam_left_wrist'].append(self.latest_images['cam_left_wrist'][0].copy())
            self.data_buffer['action'].append(joint_angles)
            self.data_buffer['timestamps'].append(ts_high)
            
        frame_idx = len(self.data_buffer['action'])
        self.logger.info(f"记录帧 {frame_idx}: 关节={joint_angles[:7]}, 时间差={time_diff*1000:.1f}ms")
        
    def read_arm_state(self) -> np.ndarray:
        """读取机械臂关节角度"""
        state = np.zeros(self.hdf5_config['action_dim'], dtype=np.float32)
        
        # 获取数据
        left_arm_msg = self.piper.GetArmJointMsgs()
        left_gripper_msg = self.piper.GetArmGripperMsgs()
        
        # 精确提取6个关节角度 (单位: 0.001度 → 度)
        state[0] = float(left_arm_msg.joint_state.joint_1) / 1000.0
        state[1] = float(left_arm_msg.joint_state.joint_2) / 1000.0
        state[2] = float(left_arm_msg.joint_state.joint_3) / 1000.0
        state[3] = float(left_arm_msg.joint_state.joint_4) / 1000.0
        state[4] = float(left_arm_msg.joint_state.joint_5) / 1000.0
        state[5] = float(left_arm_msg.joint_state.joint_6) / 1000.0
        
        # 精确提取左夹爪角度 (单位: 0.001mm → mm)
        state[12] = float(left_gripper_msg.gripper_state.grippers_angle) / 1000.0
        
        return state
        
    def keyboard_listener(self):
        """键盘监听线程"""
        while True:
            if select.select([sys.stdin], [], [], 0.1)[0]:
                key = sys.stdin.read(1).lower()
                if key == 's':
                    self.start_recording()
                elif key == 'd':
                    self.stop_recording()
                elif key == 'q':
                    self.logger.info("收到退出指令...")
                    self.stop_recording()
                    rclpy.shutdown()
                    break
                    
    def start_recording(self):
        """开始记录数据"""
        with self.buffer_lock:
            if self.is_recording:
                self.logger.warn("已在记录中！")
                return
                
            # 清空缓冲区
            self.data_buffer = {
                'cam_high': [],
                'cam_left_wrist': [],
                'action': [],
                'timestamps': []
            }
            self.latest_images = {'cam_high': None, 'cam_left_wrist': None}
            self.last_frame_time = 0.0
            self.is_recording = True
            
        self.episode_count += 1
        self.logger.warn(f"\n{'='*50}")
        self.logger.warn(f"开始记录第 {self.episode_count} 段数据...")
        self.logger.warn(f"按 'd' 停止记录")
        self.logger.warn(f"{'='*50}\n")
        
    def stop_recording(self):
        """停止记录并保存"""
        with self.buffer_lock:
            if not self.is_recording:
                return
                
            self.is_recording = False
            
        num_frames = len(self.data_buffer['action'])
        
        if num_frames == 0:
            self.logger.warn("没有记录到任何数据！")
            return
            
        self.logger.warn(f"\n{'='*50}")
        self.logger.warn(f"停止记录，共 {num_frames} 帧")
        self.logger.warn("正在保存HDF5文件...")
        
        try:
            # 保存HDF5文件
            output_path = Path(f'datasets/episode_{self.episode_count}_{datetime.now().strftime("%Y%m%d_%H%M%S")}.hdf5')
            self.save_recording_to_hdf5(output_path)
            self.logger.warn(f"文件已保存: {output_path}")
        except Exception as e:
            self.logger.error(f"保存失败: {e}")
            import traceback
            traceback.print_exc()
        finally:
            self.latest_images = {'cam_high': None, 'cam_left_wrist': None}
            
        self.logger.warn(f"{'='*50}\n")
        
    def get_black_image_bytes(self) -> bytes:
        """获取黑色图像的字节串（作为占位符）"""
        black_img = np.zeros((self.hdf5_config['image_height'],
                             self.hdf5_config['image_width'], 3), dtype=np.uint8)
        success, encoded = cv2.imencode('.jpg', black_img, 
                                       [cv2.IMWRITE_JPEG_QUALITY, 90])
        if not success:
            raise ValueError("黑色图像编码失败")
        return encoded.tobytes()
        
    def encode_cv_image_to_bytes(self, image: np.ndarray) -> bytes:
        """将OpenCV图像编码为JPEG字节串（增强版）"""
        if image is None:
            self.logger.error("接收到空图像！")
            raise ValueError("空图像无法编码")
        
        # 确保图像是uint8类型
        if image.dtype != np.uint8:
            if image.max() <= 1.0:
                image = (image * 255).astype(np.uint8)
            else:
                image = image.astype(np.uint8)
        
        # 确保是3通道BGR
        if len(image.shape) == 2:
            image = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
        elif len(image.shape) == 3 and image.shape[2] == 4:
            image = cv2.cvtColor(image, cv2.COLOR_BGRA2BGR)

        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        
        # 调整尺寸
        if (image.shape[1] != self.hdf5_config['image_width'] or 
            image.shape[0] != self.hdf5_config['image_height']):
            image = cv2.resize(image, 
                             (self.hdf5_config['image_width'], 
                              self.hdf5_config['image_height']))
        
        # 编码为JPEG
        success, encoded_buffer = cv2.imencode('.jpg', image, 
                                              [cv2.IMWRITE_JPEG_QUALITY, 90])
        if not success:
            raise ValueError("图像编码失败")
            
        return encoded_buffer.tobytes()
        
    def save_recording_to_hdf5(self, output_path: Path):
        """将内存数据保存为HDF5文件（修复版）"""
        num_timesteps = len(self.data_buffer['action'])
        if num_timesteps == 0:
            self.logger.error("无数据可保存！")
            return
            
        self.logger.info(f"准备保存 {num_timesteps} 帧数据...")
        
        with h5py.File(output_path, 'w') as f:
            # 1. 保存action数据
            action_data = np.array(self.data_buffer['action'], dtype=np.float32)
            f.create_dataset("action", data=action_data)
            
            # 2. 创建observations组
            obs_group = f.create_group("observations")
            images_group = obs_group.create_group("images")
            
            # 3. 编码并保存cam_high（带错误检查）
            self.logger.info("编码cam_high图像...")
            encoded_high = []
            for i, img in enumerate(self.data_buffer['cam_high']):
                try:
                    if img is None or img.size == 0:
                        self.logger.warn(f"第{i}帧cam_high图像为空，使用占位符")
                        encoded_high.append(self.get_black_image_bytes())
                    else:
                        encoded = self.encode_cv_image_to_bytes(img)
                        if len(encoded) > 0:
                            encoded_high.append(encoded)
                        else:
                            self.logger.warn(f"第{i}帧cam_high编码为空，使用占位符")
                            encoded_high.append(self.get_black_image_bytes())
                except Exception as e:
                    self.logger.warn(f"第{i}帧cam_high编码失败: {e}，使用占位符")
                    encoded_high.append(self.get_black_image_bytes())
            
            # 验证所有元素都是有效的bytes
            for i, data in enumerate(encoded_high):
                if not isinstance(data, bytes) or len(data) == 0:
                    self.logger.error(f"第{i}帧cam_high编码结果无效: {type(data)}")
                    raise ValueError(f"无效编码结果类型: {type(data)}")
            
            # 安全创建numpy数组（修复核心问题）
            max_len_high = max(len(img) for img in encoded_high)
            self.logger.info(f"cam_high: {len(encoded_high)} 帧, 最大字节长度: {max_len_high}")
            
            try:
                # 修复：直接转换bytes列表为numpy数组，不使用np.void()
                cam_high_array = np.array(encoded_high, dtype=f"S{max_len_high}")
                images_group.create_dataset("cam_high", data=cam_high_array)
                self.logger.info("cam_high数据集创建成功")
            except Exception as e:
                self.logger.error(f"创建cam_high数据集失败: {e}")
                raise
            
            # 4. 编码并保存cam_left_wrist（带错误检查）
            self.logger.info("编码cam_left_wrist图像...")
            encoded_left = []
            for i, img in enumerate(self.data_buffer['cam_left_wrist']):
                try:
                    if img is None or img.size == 0:
                        self.logger.warn(f"第{i}帧cam_left_wrist图像为空，使用占位符")
                        encoded_left.append(self.get_black_image_bytes())
                    else:
                        encoded = self.encode_cv_image_to_bytes(img)
                        if len(encoded) > 0:
                            encoded_left.append(encoded)
                        else:
                            self.logger.warn(f"第{i}帧cam_left_wrist编码为空，使用占位符")
                            encoded_left.append(self.get_black_image_bytes())
                except Exception as e:
                    self.logger.warn(f"第{i}帧cam_left_wrist编码失败: {e}，使用占位符")
                    encoded_left.append(self.get_black_image_bytes())
            
            # 验证
            for i, data in enumerate(encoded_left):
                if not isinstance(data, bytes) or len(data) == 0:
                    self.logger.error(f"第{i}帧cam_left_wrist编码结果无效: {type(data)}")
                    raise ValueError(f"无效编码结果类型: {type(data)}")
            
            max_len_left = max(len(img) for img in encoded_left)
            self.logger.info(f"cam_left_wrist: {len(encoded_left)} 帧, 最大字节长度: {max_len_left}")
            
            try:
                # 修复：直接转换bytes列表为numpy数组，不使用np.void()
                cam_left_array = np.array(encoded_left, dtype=f"S{max_len_left}")
                images_group.create_dataset("cam_left_wrist", data=cam_left_array)
                self.logger.info("cam_left_wrist数据集创建成功")
            except Exception as e:
                self.logger.error(f"创建cam_left_wrist数据集失败: {e}")
                raise
            
            # 5. 创建占位cam_right_wrist
            black_bytes = self.get_black_image_bytes()
            cam_right_array = np.array([black_bytes] * num_timesteps, dtype=f"S{len(black_bytes)}")
            images_group.create_dataset("cam_right_wrist", data=cam_right_array)
            self.logger.info("cam_right_wrist占位数据集创建成功")
            
            # 6. 保存其他占位数据
            obs_group.create_dataset("qpos", 
                data=np.zeros((num_timesteps, self.hdf5_config['qpos_dim']), dtype=np.float32))
            obs_group.create_dataset("left_arm_dim", 
                data=np.full(num_timesteps, self.hdf5_config['left_arm_dim'], dtype=np.int64))
            obs_group.create_dataset("right_arm_dim", 
                data=np.full(num_timesteps, self.hdf5_config['right_arm_dim'], dtype=np.int64))
            
            # 7. 添加元数据
            f.attrs['creation_time'] = datetime.now().isoformat()
            f.attrs['description'] = 'DexArt格式数据，右臂数据为占位符'
            f.attrs['num_timesteps'] = num_timesteps
        
        self.logger.info(f"HDF5文件保存成功: {num_timesteps} 帧")
        
    def run(self):
        """主运行循环"""
        try:
            self.logger.warn("系统运行中...")
            rclpy.spin(self.node)
        except KeyboardInterrupt:
            self.logger.info("接收到中断信号")
        finally:
            self.cleanup()
            
    def cleanup(self):
        """清理资源"""
        self.logger.info("正在清理资源...")
        self.stop_recording()
        self.restore_terminal()
        # self.piper.ClosePort()
        self.node.destroy_node()
        rclpy.shutdown()
        self.logger.info("程序已退出")


def main():
    """主函数"""
    try:
        collector = DataCollector()
        collector.run()
    except Exception as e:
        print(f"程序异常退出: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)


if __name__ == '__main__':
    main()