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
from collections import deque
from dataclasses import dataclass
from typing import Dict, List

# 导入机械臂SDK
from piper_sdk import C_PiperInterface_V2
import alicia_d_sdk  # 添加示教臂SDK

# ===================== 时间同步管理器 =====================

@dataclass
class StampedMsg:
    stamp_ns: int
    msg: Image
    topic: str

@dataclass
class FrameGroup:
    stamp_ns: int
    frames: dict   # topic -> Image msg

class TimeSyncManager:
    def __init__(self, topics, max_queue_size=20, slop_ns=30_000_000):
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
            self.buffers[topic].append(StampedMsg(stamp_ns, msg, topic))
            self._try_sync()

    def _try_sync(self):
        if any(len(buf) == 0 for buf in self.buffers.values()):
            return

        ref_topic = self.topics[0]
        ref_msg = self.buffers[ref_topic][-1]
        ref_time = ref_msg.stamp_ns
        frames = {ref_topic: ref_msg.msg}

        for topic, buf in self.buffers.items():
            if topic == ref_topic:
                continue

            best = min(buf, key=lambda x: abs(x.stamp_ns - ref_time))
            if abs(best.stamp_ns - ref_time) > self.slop_ns:
                return

            frames[topic] = best.msg

        # 清理旧帧
        for topic, buf in self.buffers.items():
            while buf and buf[0].stamp_ns <= ref_time:
                buf.popleft()

        self.synced_queue.append(FrameGroup(ref_time, frames))

    def get_synced(self):
        with self.lock:
            if self.synced_queue:
                return self.synced_queue.popleft()
        return None

# ===================== 数据收集器（增强版） =====================

class DataCollector:
    """增强版数据收集器：集成时间同步管理器与机械臂示教同步功能"""
    
    def __init__(self, sync_mode=False, leader_port="/dev/ttyUSB1"):
        self.record_freq = 30.0
        self.min_frame_interval = 1.0 / self.record_freq
        self.time_sync_tolerance = 0.05
        
        rclpy.init()
        self.node = Node('data_collection_node')
        self.logger = self.node.get_logger()
        
        # 同步模式标志
        self.sync_mode = sync_mode
        self.leader_port = leader_port
        
        # 初始化机械臂
        self.logger.info("连接机械臂...")
        self.piper = C_PiperInterface_V2("can0")
        self.piper.ConnectPort()
        
        # 如果是同步模式，连接示教臂
        if self.sync_mode:
            self.logger.info("示教同步模式已启用，连接示教臂...")
            self.leader_robot = alicia_d_sdk.create_robot(
                port=leader_port,
                baudrate=1000000,
                robot_version="v5_6",
                robot_type="leader",
                gripper_type="50mm"
            )
            if not self.leader_robot.connect():
                self.logger.error("示教臂连接失败！")
                sys.exit(1)
            self.logger.info("示教臂连接成功")
        
        # 使能操作臂
        self._enable_follower_arm()
        
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
            'cam_high': [],
            'cam_left_wrist': [],
            'action': [],
            'timestamps': []
        }
        
        # 状态控制
        self.is_recording = False
        self.last_frame_time = 0.0
        self.buffer_lock = threading.Lock()
        self.episode_count = 0
        
        # 设置图像订阅
        self.setup_image_subscriptions()
        
        # 启动键盘监听
        self.logger.info("启动键盘监听...")
        if self.sync_mode:
            self.logger.info('按 "s" 开始记录+同步，按 "d" 停止，按 "q" 退出')
        else:
            self.logger.info('按 "s" 开始记录，按 "d" 停止记录，按 "q" 退出程序')
        self.keyboard_thread = threading.Thread(target=self.keyboard_listener, daemon=True)
        self.keyboard_thread.start()
        
        # 设置终端
        self.setup_terminal()
        
        # 创建处理定时器
        self.timer = self.node.create_timer(0.01, self.process_frame)
        
    def setup_terminal(self):
        self.fd = sys.stdin.fileno()
        self.old_settings = termios.tcgetattr(self.fd)
        tty.setcbreak(self.fd)
    
    def restore_terminal(self):
        termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old_settings)
        
    def _enable_follower_arm(self):
        """使能操作臂（follower）"""
        enable_flag = False
        timeout = 5
        start_time = time.time()
        
        while not enable_flag:
            elapsed_time = time.time() - start_time
            print("--------------------")
            enable_flag = self.piper.GetArmLowSpdInfoMsgs().motor_1.foc_status.driver_enable_status and \
                self.piper.GetArmLowSpdInfoMsgs().motor_2.foc_status.driver_enable_status and \
                self.piper.GetArmLowSpdInfoMsgs().motor_3.foc_status.driver_enable_status and \
                self.piper.GetArmLowSpdInfoMsgs().motor_4.foc_status.driver_enable_status and \
                self.piper.GetArmLowSpdInfoMsgs().motor_5.foc_status.driver_enable_status and \
                self.piper.GetArmLowSpdInfoMsgs().motor_6.foc_status.driver_enable_status
            print("操作臂使能状态:", enable_flag)
            self.piper.EnableArm(7)
            self.piper.GripperCtrl(0, 1000, 0x01, 0)
            
            if elapsed_time > timeout:
                print("使能超时，退出程序")
                sys.exit(1)
            time.sleep(0.1)
        
        self.logger.info("操作臂使能成功")
        
    def joint_projection(self, joint_position):
        """
        将示教臂关节角度映射到操作臂的关节限制范围内
        支持Alicia Duo到Piper的映射
        """
        motor_dir = [1, -1, -1, -1, -1, -1]
        
        # Alicia Duo限位（弧度）
        alicia_limits = [
            [-3.14, 3.14],    # J1
            [0, 3.14],        # J2
            [-3.14, 0],       # J3 
            [-3.14, 3.14],    # J4
            [-1.57, 1.57],    # J5
            [-3.14, 3.14]     # J6
        ]
        
        # Piper限位（弧度）
        piper_limits = [
            [-2.68, 2.68],    # J1
            [0, 3.40],        # J2
            [-3.05, 0],       # J3
            [-1.85, 1.85],    # J4
            [-1.31, 1.31],    # J5
            [-1.75, 1.75]     # J6
        ]
        
        projected_joints = []
        
        for i in range(6):
            alicia_min, alicia_max = alicia_limits[i]
            piper_min, piper_max = piper_limits[i]
            joint_val = joint_position[i] * motor_dir[i]
            
            # 限制在Alicia限位内
            joint_val = max(alicia_min, min(alicia_max, joint_val))
            
            # 归一化到[0,1]
            alicia_range = alicia_max - alicia_min
            if alicia_range > 0:
                normalized = (joint_val - alicia_min) / alicia_range
            else:
                normalized = 0
            
            # 映射到Piper范围
            piper_range = piper_max - piper_min
            projected_val = piper_min + normalized * piper_range
            projected_joints.append(round(projected_val, 3))
        
        # 添加夹爪占位
        projected_joints.append(0.0)
        return projected_joints
        
    def set_follower_joint_value(self, position, gripper):
        """设置操作臂关节值"""
        factor = 57295.7795  # 1000*180/3.1415926
        
        joint_0 = round(position[0] * factor)
        joint_1 = round(position[1] * factor)    
        joint_2 = round(position[2] * factor)
        joint_3 = round(position[3] * factor)
        joint_4 = round(position[4] * factor)
        joint_5 = round(position[5] * factor)
        joint_6 = int(gripper * 1000)
        
        self.piper.MotionCtrl_2(0x01, 0x01, 100, 0x00)
        self.piper.JointCtrl(joint_0, joint_1, joint_2, joint_3, joint_4, joint_5)
        self.piper.GripperCtrl(joint_6, 3000, 0x01, 0)
        
    def setup_image_subscriptions(self):
        """设置图像订阅并集成时间同步"""
        self.bridge = CvBridge()
        
        self.node.declare_parameter('camera_topic_name', '/camera/camera/color/image_raw')
        self.node.declare_parameter('camera_left_topic_name', '/camera_left/camera_left/color/image_raw')
        
        camera_topic = self.node.get_parameter('camera_topic_name').value
        camera_left_topic = self.node.get_parameter('camera_left_topic_name').value
        
        self.image_topic_names = [camera_topic, camera_left_topic]
        
        # 初始化时间同步管理器
        self.sync_mgr = TimeSyncManager(
            topics=self.image_topic_names,
            max_queue_size=30,
            slop_ns=int(self.time_sync_tolerance * 1_000_000_000)
        )
        
        # 为每个 topic 创建订阅
        for topic_name in self.image_topic_names:
            self.node.create_subscription(
                Image,
                topic_name,
                lambda msg, t=topic_name: self.sync_mgr.add_msg(t, msg),
                qos_profile_sensor_data
            )
            self.logger.info(f"已订阅: {topic_name} (通过时间同步管理器)")
        
    def process_frame(self):
        """处理帧数据（集成同步控制）"""
        current_time = time.time()
        
        # 同步模式：读取示教臂状态并控制操作臂
        if self.sync_mode and self.is_recording:
            try:
                # 读取示教臂状态
                leader_joints = self.leader_robot.get_joints()
                leader_gripper = self.leader_robot.get_gripper()
                
                # 映射到操作臂关节空间
                follower_joints = self.joint_projection(leader_joints)
                
                # 控制操作臂
                self.set_follower_joint_value(follower_joints, leader_gripper)
                
            except Exception as e:
                self.logger.error(f"同步控制失败: {e}")
        
        # 数据收集逻辑
        if not self.is_recording:
            return
            
        if current_time - self.last_frame_time < self.min_frame_interval:
            return
        
        # 从同步管理器获取对齐的帧组
        group = self.sync_mgr.get_synced()
        if group is None:
            return
        
        ts_ref = group.stamp_ns / 1_000_000_000.0
        self.last_frame_time = current_time
        
        # 转换图像消息
        try:
            imgs = {}
            for topic, img_msg in group.frames.items():
                cv_img = self.bridge.imgmsg_to_cv2(img_msg)
                imgs[topic] = cv_img
                
            img_high = imgs[self.image_topic_names[0]]
            img_left = imgs[self.image_topic_names[1]]
            
        except Exception as e:
            self.logger.error(f"图像转换失败: {e}")
            return
        
        # 读取操作臂状态用于数据记录
        try:
            joint_angles = self.read_arm_state()
        except Exception as e:
            self.logger.error(f"读取机械臂状态失败: {e}")
            return
        
        # 保存到缓冲区
        with self.buffer_lock:
            self.data_buffer['cam_high'].append(img_high.copy())
            self.data_buffer['cam_left_wrist'].append(img_left.copy())
            self.data_buffer['action'].append(joint_angles)
            self.data_buffer['timestamps'].append(ts_ref)
            
        frame_idx = len(self.data_buffer['action'])
        self.logger.info(f"记录帧 {frame_idx}: 关节={joint_angles[:7]}, ts={ts_ref:.3f}")
        
    def read_arm_state(self) -> np.ndarray:
        """读取操作臂关节角度"""
        state = np.zeros(self.hdf5_config['action_dim'], dtype=np.float32)
        left_arm_msg = self.piper.GetArmJointMsgs()
        left_gripper_msg = self.piper.GetArmGripperMsgs()
        
        # 从SDK对象中提取数据（单位：0.001度/0.001mm）
        state[0] = float(left_arm_msg.joint_state.joint_1) / 1000.0
        state[1] = float(left_arm_msg.joint_state.joint_2) / 1000.0
        state[2] = float(left_arm_msg.joint_state.joint_3) / 1000.0
        state[3] = float(left_arm_msg.joint_state.joint_4) / 1000.0
        state[4] = float(left_arm_msg.joint_state.joint_5) / 1000.0
        state[5] = float(left_arm_msg.joint_state.joint_6) / 1000.0
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
                
            self.data_buffer = {
                'cam_high': [],
                'cam_left_wrist': [],
                'action': [],
                'timestamps': []
            }
            self.last_frame_time = 0.0
            self.is_recording = True
            
        self.episode_count += 1
        self.logger.warn(f"\n{'='*50}")
        if self.sync_mode:
            self.logger.warn(f"开始同步记录第 {self.episode_count} 段数据...")
            self.logger.warn("示教臂运动将实时控制操作臂")
        else:
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
            output_path = Path(f'datasets/episode_{self.episode_count}_{datetime.now().strftime("%Y%m%d_%H%M%S")}.hdf5')
            self.save_recording_to_hdf5(output_path)
            self.logger.warn(f"文件已保存: {output_path}")
        except Exception as e:
            self.logger.error(f"保存失败: {e}")
            import traceback
            traceback.print_exc()
            
        self.logger.warn(f"{'='*50}\n")
        
    def get_black_image_bytes(self) -> bytes:
        black_img = np.zeros((self.hdf5_config['image_height'],
                             self.hdf5_config['image_width'], 3), dtype=np.uint8)
        success, encoded = cv2.imencode('.jpg', black_img, 
                                       [cv2.IMWRITE_JPEG_QUALITY, 90])
        if not success:
            raise ValueError("黑色图像编码失败")
        return encoded.tobytes()
        
    def encode_cv_image_to_bytes(self, image: np.ndarray) -> bytes:
        if image is None or image.size == 0:
            self.logger.error("接收到空图像！")
            raise ValueError("空图像无法编码")
        
        if image.dtype != np.uint8:
            image = (image * 255).astype(np.uint8) if image.max() <= 1.0 else image.astype(np.uint8)
        
        if len(image.shape) == 2:
            image = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
        elif len(image.shape) == 3 and image.shape[2] == 4:
            image = cv2.cvtColor(image, cv2.COLOR_BGRA2BGR)

        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        
        if (image.shape[1] != self.hdf5_config['image_width'] or 
            image.shape[0] != self.hdf5_config['image_height']):
            image = cv2.resize(image, 
                             (self.hdf5_config['image_width'], 
                              self.hdf5_config['image_height']))
        
        success, encoded_buffer = cv2.imencode('.jpg', image, 
                                              [cv2.IMWRITE_JPEG_QUALITY, 90])
        if not success:
            raise ValueError("图像编码失败")
        return encoded_buffer.tobytes()
        
    def save_recording_to_hdf5(self, output_path: Path):
        """将内存数据保存为HDF5文件"""
        num_timesteps = len(self.data_buffer['action'])
        if num_timesteps == 0:
            self.logger.error("无数据可保存！")
            return
            
        self.logger.info(f"准备保存 {num_timesteps} 帧数据...")
        
        with h5py.File(output_path, 'w') as f:
            # 保存action数据
            action_data = np.array(self.data_buffer['action'], dtype=np.float32)
            f.create_dataset("action", data=action_data)
            
            # 创建observations组
            obs_group = f.create_group("observations")
            images_group = obs_group.create_group("images")
            
            # 编码并保存cam_high
            self.logger.info("编码cam_high图像...")
            encoded_high = []
            for i, img in enumerate(self.data_buffer['cam_high']):
                try:
                    if img is None or img.size == 0:
                        self.logger.warn(f"第{i}帧cam_high图像为空，使用占位符")
                        encoded_high.append(self.get_black_image_bytes())
                    else:
                        encoded = self.encode_cv_image_to_bytes(img)
                        encoded_high.append(encoded if len(encoded) > 0 else self.get_black_image_bytes())
                except Exception as e:
                    self.logger.warn(f"第{i}帧cam_high编码失败: {e}，使用占位符")
                    encoded_high.append(self.get_black_image_bytes())
            
            max_len_high = max(len(img) for img in encoded_high)
            cam_high_array = np.array(encoded_high, dtype=f"S{max_len_high}")
            images_group.create_dataset("cam_high", data=cam_high_array)
            
            # 编码并保存cam_left_wrist
            self.logger.info("编码cam_left_wrist图像...")
            encoded_left = []
            for i, img in enumerate(self.data_buffer['cam_left_wrist']):
                try:
                    if img is None or img.size == 0:
                        self.logger.warn(f"第{i}帧cam_left_wrist图像为空，使用占位符")
                        encoded_left.append(self.get_black_image_bytes())
                    else:
                        encoded = self.encode_cv_image_to_bytes(img)
                        encoded_left.append(encoded if len(encoded) > 0 else self.get_black_image_bytes())
                except Exception as e:
                    self.logger.warn(f"第{i}帧cam_left_wrist编码失败: {e}，使用占位符")
                    encoded_left.append(self.get_black_image_bytes())
            
            max_len_left = max(len(img) for img in encoded_left)
            cam_left_array = np.array(encoded_left, dtype=f"S{max_len_left}")
            images_group.create_dataset("cam_left_wrist", data=cam_left_array)
            
            # 创建占位cam_right_wrist
            black_bytes = self.get_black_image_bytes()
            cam_right_array = np.array([black_bytes] * num_timesteps, dtype=f"S{len(black_bytes)}")
            images_group.create_dataset("cam_right_wrist", data=cam_right_array)
            
            # 保存其他占位数据
            obs_group.create_dataset("qpos", 
                data=np.zeros((num_timesteps, self.hdf5_config['qpos_dim']), dtype=np.float32))
            obs_group.create_dataset("left_arm_dim", 
                data=np.full(num_timesteps, self.hdf5_config['left_arm_dim'], dtype=np.int64))
            obs_group.create_dataset("right_arm_dim", 
                data=np.full(num_timesteps, self.hdf5_config['right_arm_dim'], dtype=np.int64))
            
            # 添加元数据
            f.attrs['creation_time'] = datetime.now().isoformat()
            f.attrs['description'] = 'DexArt格式数据，右臂数据为占位符'
            f.attrs['num_timesteps'] = num_timesteps
            f.attrs['sync_mode'] = self.sync_mode  # 记录是否启用同步
        
        self.logger.info(f"HDF5文件保存成功: {num_timesteps} 帧")
        
    def run(self):
        """主运行循环"""
        try:
            mode_str = "示教同步模式" if self.sync_mode else "数据采集模式"
            self.logger.warn(f"{mode_str}运行中...")
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
        self.node.destroy_node()
        if self.sync_mode:
            self.leader_robot.disconnect()
        self.logger.info("程序已退出")


def main():
    """主函数"""
    try:
        # 添加命令行参数
        import argparse
        parser = argparse.ArgumentParser(description="数据收集与示教同步工具")
        parser.add_argument('--sync', action='store_true', 
                           help="启用示教同步模式")
        parser.add_argument('--leader_port', type=str, default="/dev/ttyUSB0",
                           help="示教臂串口端口 (默认: /dev/ttyUSB0)")
        args = parser.parse_args()
        
        collector = DataCollector(sync_mode=args.sync, leader_port=args.leader_port)
        collector.run()
    except Exception as e:
        print(f"程序异常退出: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)


if __name__ == '__main__':
    main()