#!/usr/bin/env python3
"""
piper_node.py - 控制Piper机械臂
订阅话题: /aliciaD/action (Float32MultiArray) - AliciaD状态
发布话题: /piper/qpos (Float32MultiArray) - Piper状态
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from piper_sdk import C_PiperInterface_V2
import numpy as np
from math import pi
import time

def enable_fun(piper:C_PiperInterface_V2):
    '''
    使能机械臂并检测使能状态,尝试5s,如果使能超时则退出程序
    '''
    enable_flag = False
    # 设置超时时间（秒）
    timeout = 5
    # 记录进入循环前的时间
    start_time = time.time()
    elapsed_time_flag = False
    while not (enable_flag):
        elapsed_time = time.time() - start_time
        print("--------------------")
        enable_flag = piper.GetArmLowSpdInfoMsgs().motor_1.foc_status.driver_enable_status and \
            piper.GetArmLowSpdInfoMsgs().motor_2.foc_status.driver_enable_status and \
            piper.GetArmLowSpdInfoMsgs().motor_3.foc_status.driver_enable_status and \
            piper.GetArmLowSpdInfoMsgs().motor_4.foc_status.driver_enable_status and \
            piper.GetArmLowSpdInfoMsgs().motor_5.foc_status.driver_enable_status and \
            piper.GetArmLowSpdInfoMsgs().motor_6.foc_status.driver_enable_status
        print("使能状态:",enable_flag)
        piper.EnableArm(7)
        piper.GripperCtrl(0,1000,0x01, 0)
        print("--------------------")
        # 检查是否超过超时时间
        if elapsed_time > timeout:
            print("超时....")
            elapsed_time_flag = True
            enable_flag = True
            break
        time.sleep(1)
        pass
    if(elapsed_time_flag):
        print("程序自动使能超时,退出程序")
        exit(0)

class PiperNode(Node):
    def __init__(self):
        super().__init__('piper_node')
        
        # 订阅AliciaD状态
        self.subscription = self.create_subscription(
            Float32MultiArray,
            '/aliciaD/action',
            self.aliciaD_callback,
            3
        )
        
        # 发布Piper状态
        self.publisher_ = self.create_publisher(
            Float32MultiArray,
            '/piper/qpos',
            3
        )
        
        # 发布频率
        self.publish_rate = 50  # 100Hz
        self.timer = self.create_timer(1.0/self.publish_rate, self.publish_qpos)
        
        # 当前状态
        self.current_qpos = [0.0] * 7  # 6关节 + 1夹爪
        self.target_qpos = [0.0] * 7
        self.last_aliciaD_msg = None
        
        # 初始化Piper机械臂
        self.init_piper()
        
        self.get_logger().info('Piper节点已启动')
    
    def init_piper(self):
        """初始化Piper机械臂"""
        try:
            self.arm = C_PiperInterface_V2(can_name="can0")
            self.arm.ConnectPort()
            self.arm.EnableArm(motor_num=7)
            enable_fun(piper=self.arm)
            self.arm.GripperCtrl(0,1000,0x01, 0)
            # self.arm.MotionCtrl_2(0x01, 0x01, 100, 0x00)
            
            self.get_logger().info('Piper机械臂连接成功')
            self.arm_connected = True
            
        except Exception as e:
            self.get_logger().error('Piper机械臂连接失败: {}'.format(str(e)))
            self.arm_connected = False
    
    def aliciaD_callback(self, msg):
        """AliciaD状态回调函数"""
        self.last_aliciaD_msg = msg
        
        if not self.arm_connected:
            self.get_logger().warn('Piper未连接，忽略控制指令')
            return
        
        if len(msg.data) >= 7:
            # 提取关节角度和夹爪状态
            # self.get_logger().info(f"rx data: {msg.data}")
            alicia_joints = list(msg.data[:6])
            alicia_gripper = msg.data[6]
            
            self.target_qpos[:6] = alicia_joints
            
            # self.target_qpos[6] = alicia_gripper
            self.target_qpos[6] = 74000 if alicia_gripper == 1 else 50
            
            # self.get_logger().info(f"target_qpos: {self.target_qpos}")

            # 控制Piper
            self.control_piper(self.target_qpos)
    
    def control_piper(self, qpos):
        """控制Piper机械臂到目标位置"""
        if not self.arm_connected:
            return
        
        try:
            # 关节角度转换：弧度 -> 0.001度
            # 转换系数: 1 rad = 180/pi * 1000 * 0.001度
            f = 57295.7795  # 180/pi * 1000

            self.arm.MotionCtrl_2(0x01, 0x01, 100, 0x00)
            
            joint_targets = [round(angle * f) for angle in qpos[:6]]
            
            # 关节控制
            self.arm.JointCtrl(*joint_targets)
            
            # 夹爪控制 (0-1000映射到0-1000)
            gripper_value = int(qpos[6])

            self.arm.GripperCtrl(gripper_value, 3000, 0x01, 0)
            time.sleep(0.005)
            
        except Exception as e:
            self.get_logger().error('控制Piper失败: {}'.format(str(e)))
            self.arm_connected = False
    
    def read_piper_status(self):
        """读取Piper机械臂当前状态"""
        if not self.arm_connected:
            return None
        
        try:
            # 读取关节状态
            joints_msg = self.arm.GetArmJointMsgs()
            joints = joints_msg.joint_state
            
            # 转换: 0.001度 -> 弧度
            joint_pos = [
                joints.joint_1 * 0.001 * pi / 180.0,
                joints.joint_2 * 0.001 * pi / 180.0,
                joints.joint_3 * 0.001 * pi / 180.0,
                joints.joint_4 * 0.001 * pi / 180.0,
                joints.joint_5 * 0.001 * pi / 180.0,
                joints.joint_6 * 0.001 * pi / 180.0
            ]
            
            # 读取夹爪状态
            gripper_msg = self.arm.GetArmGripperMsgs()
            gripper_raw = gripper_msg.gripper_state.grippers_angle
            
            # 夹爪二值化 (阈值可根据实际情况调整)
            GRIPPER_OPEN_THRESHOLD = 64000
            gripper_raw = 1.0 if gripper_raw > GRIPPER_OPEN_THRESHOLD else 0.0
            gripper_pos = gripper_raw
            
            return joint_pos + [gripper_pos]
            
        except Exception as e:
            self.get_logger().warn('读取Piper状态失败: {}'.format(str(e)))
            self.arm_connected = False
            return None
    
    def publish_qpos(self):
        """发布Piper的qpos状态"""
        qpos = self.read_piper_status()
        
        if qpos is None:
            # 使用上次的有效值
            qpos = self.current_qpos
        else:
            # 更新当前状态
            self.current_qpos = qpos
        
        # 创建并发布消息
        msg = Float32MultiArray()
        msg.data = qpos
        self.publisher_.publish(msg)
        
        # 可选：调试日志
        # self.get_logger().debug('Piper状态: {}'.format([round(x, 3) for x in qpos]))
    
    def destroy_node(self):
        """节点销毁时清理资源"""
        if self.arm_connected:
            try:
                # self.arm.DisableArm(motor_num=7)
                # self.arm.DisconnectPort()
                self.get_logger().info('Piper机械臂已断开连接')
            except:
                pass
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    
    node = PiperNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()