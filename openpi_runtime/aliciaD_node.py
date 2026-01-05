#!/usr/bin/env python3
"""
aliciaD_node.py - 发布AliciaD机械臂状态
发布话题: /aliciaD/action (Float32MultiArray) - [关节弧度1-6, 夹爪状态(0/1)]
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import alicia_d_sdk
import time
import threading

def joint_projection(joint_position):
    """
    Project Alicia Duo joint positions to Piper joint limits
    
    Alicia duo joints limit: 
    J1: ± 180°      [-3.14, 3.14]
    J2: 0° ~ 180°   [0, 3.14]
    J3 -180° ~ 0°   [-3.14, 0]
    J4: ± 180°      [-3.14, 3.14]
    J5: [-90°, 90°] [-1.57, 1.57]
    J6: ± 180°      [-3.14, 3.14]

    Piper joints limit:
    J1: ±154°       [-2.68, 2.68]
    J2 :0°~195°     [0, 3.40]
    J3: -175°~0°    [-3.05, 0]
    J4: -106°~106°  [-1.85, 1.85]
    J5: -75°~75°    [-1.31, 1.31]
    J6: ±100°       [-1.75, 1.75]
    """
    motor_dir = [1, -1, -1, -1, -1, -1]
    # Define Alicia Duo limits (radians)
    alicia_limits = [
        [-3.14, 3.14],    # J1
        [0, 3.14],        # J2
        [-3.14, 0],       # J3 
        [-3.14, 3.14],    # J4
        [-1.57, 1.57],    # J5
        [-3.14, 3.14]     # J6
    ]
    
    # Define Piper limits (radians)
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
        joint_val = joint_position[i] * motor_dir[i]  # Apply motor direction
        
        # Clamp to Alicia limits first
        joint_val = max(alicia_min, min(alicia_max, joint_val))
        
        # Normalize to [0, 1] based on Alicia range
        alicia_range = alicia_max - alicia_min
        normalized = (joint_val - alicia_min) / alicia_range
        
        # Map to Piper range
        piper_range = piper_max - piper_min
        projected_val = piper_min + normalized * piper_range
        projected_joints.append(round(projected_val, 2))
    return projected_joints


class AliciaDNode(Node):
    def __init__(self):
        super().__init__('aliciaD_node')
        
        # 创建发布者
        self.publisher_ = self.create_publisher(
            Float32MultiArray, 
            '/aliciaD/action', 
            3
        )
        
        # 发布频率 (Hz)
        self.publish_rate = 50  
        self.timer = self.create_timer(1.0/self.publish_rate, self.publish_status)
        
        # 机器人状态
        self.current_joints = [0.0] * 6  # 6个关节
        self.current_gripper = 0.0
        
        # 机器人实例
        self.robot = None
        
        # 初始化机器人连接
        self.init_robot()
        
        self.get_logger().info('AliciaD节点已启动，发布频率: {}Hz'.format(self.publish_rate))
    
    def init_robot(self):
        """初始化机器人连接"""
        try:
            self.robot = alicia_d_sdk.create_robot(
                port="/dev/ttyUSB0",
                baudrate=1000000,
                robot_version="v5_6",
                robot_type="follower",
                gripper_type="50mm"
            )
            
            if self.robot.connect():
                self.get_logger().info('AliciaD机器人连接成功')
            else:
                self.get_logger().error('AliciaD机器人连接失败')
                self.robot = None
                
        except Exception as e:
            self.get_logger().error('初始化机器人失败: {}'.format(str(e)))
            self.robot = None
    
    def read_robot_status(self):
        """读取机器人状态"""
        if self.robot is None:
            # 尝试重新连接
            self.init_robot()
            return None, None
        
        try:
            joints = self.robot.get_joints()  # 6个关节的弧度值
            gripper_raw = self.robot.get_gripper()  # 夹爪原始值

            piper_joint_pos = joint_projection(joints)
            
            # 夹爪二值化
            gripper = 1.0 if gripper_raw >= 95 else 0.0
            
            return piper_joint_pos, gripper
            
        except Exception as e:
            self.get_logger().warn('读取机器人状态失败: {}'.format(str(e)))
            self.robot = None
            return None, None
    
    def publish_status(self):
        """发布机器人状态"""
        joints, gripper = self.read_robot_status()
        
        if joints is None or gripper is None:
            # 使用上次的有效值
            joints = self.current_joints
            gripper = self.current_gripper
        else:
            # 更新当前状态
            self.current_joints = joints
            self.current_gripper = gripper
        
        # 创建消息
        msg = Float32MultiArray()
        # 数据格式: [j1, j2, j3, j4, j5, j6, gripper]
        msg.data = list(joints) + [gripper]
        
        # 发布消息
        self.publisher_.publish(msg)
        
        # 可选：调试日志
        self.get_logger().debug('发布: {}'.format([round(x, 3) for x in msg.data]))
    
    def destroy_node(self):
        """节点销毁时清理资源"""
        if self.robot is not None:
            try:
                self.robot.disconnect()
                self.get_logger().info('机器人已断开连接')
            except:
                pass
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    
    node = AliciaDNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()