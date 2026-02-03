#!/usr/bin/env python3
"""
piper_node.py - 控制Piper机械臂
订阅话题: /aliciaD/action (Float32MultiArray) - AliciaD状态
发布话题: /piper/qpos (Float32MultiArray) - Piper状态

运行方式：
1. 直接运行（支持传统命令行参数）：
   python3 piper_node.py --publish_rate 100 --gripper_open_value 80000

2. ROS2运行（支持ROS2参数格式）：
   ros2 run package_name piper_node --ros-args -p publish_rate:=100.0 -p gripper_open_value:=80000

3. 通过launch文件运行

可配置参数:
  - subscribe_topic: 订阅的动作话题名 (默认: /aliciaD/action)
  - publish_topic: 发布的状态话题名 (默认: /piper/qpos)
  - publish_rate: 发布状态的频率(Hz) (默认: 50)
  - gripper_open_value: 控制夹爪打开时的控制量 (默认: 74000)
  - gripper_close_value: 控制夹爪关闭时的控制量 (默认: 50)
  - gripper_threshold: 控制夹爪开关判定阈值 (默认: 0.5)
  - gripper_state_threshold: 判断夹爪开关状态的阈值 (默认: 64000)
  - gripper_torque: 控制夹爪开关的力矩 (默认: 3000)
  - can_name: CAN接口名称 (默认: can0)
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from piper_sdk import C_PiperInterface_V2
from math import pi
import time
import sys
import argparse

def enable_fun(piper: C_PiperInterface_V2, gripper_torque=3000):
    '''
    使能机械臂并检测使能状态,尝试5s,如果使能超时则退出程序
    '''
    enable_flag = False
    # 设置超时时间（秒）
    timeout = 5
    # 记录进入循环前的时间
    start_time = time.time()
    elapsed_time_flag = False
    
    while not enable_flag:
        elapsed_time = time.time() - start_time
        
        # 检查所有电机的使能状态
        low_msg = piper.GetArmLowSpdInfoMsgs()
        enable_flag = all([
            low_msg.motor_1.foc_status.driver_enable_status,
            low_msg.motor_2.foc_status.driver_enable_status,
            low_msg.motor_3.foc_status.driver_enable_status,
            low_msg.motor_4.foc_status.driver_enable_status,
            low_msg.motor_5.foc_status.driver_enable_status,
            low_msg.motor_6.foc_status.driver_enable_status
        ])
        
        print("--------------------")
        print(f"使能状态: {enable_flag}")
        
        # 使能机械臂
        piper.EnableArm(7)
        
        # 控制夹爪到初始位置
        piper.GripperCtrl(0, gripper_torque, 0x01, 0)
        print("--------------------")
        
        # 检查是否超过超时时间
        if elapsed_time > timeout:
            print("使能超时....")
            elapsed_time_flag = True
            enable_flag = True
            break
            
        time.sleep(1)
    
    if elapsed_time_flag:
        print("程序自动使能超时,退出程序")
        return 1
    
    return 0

class PiperNode(Node):
    def __init__(self, **kwargs):
        super().__init__('piper_node')
        
        # 1. 声明所有参数
        self.declare_parameter('subscribe_topic', kwargs.get('subscribe_topic', '/aliciaD/action'))
        self.declare_parameter('publish_topic', kwargs.get('publish_topic', '/piper/qpos'))
        self.declare_parameter('publish_rate', kwargs.get('publish_rate', 50.0))
        self.declare_parameter('gripper_open_value', kwargs.get('gripper_open_value', 74000))
        self.declare_parameter('gripper_close_value', kwargs.get('gripper_close_value', 50))
        self.declare_parameter('gripper_threshold', kwargs.get('gripper_threshold', 0.5))
        self.declare_parameter('gripper_state_threshold', kwargs.get('gripper_state_threshold', 64000))
        self.declare_parameter('gripper_torque', kwargs.get('gripper_torque', 3000))
        self.declare_parameter('can_name', kwargs.get('can_name', 'can0'))
        
        # 2. 获取参数值
        self.subscribe_topic = self.get_parameter('subscribe_topic').get_parameter_value().string_value
        self.publish_topic = self.get_parameter('publish_topic').get_parameter_value().string_value
        self.publish_rate = self.get_parameter('publish_rate').get_parameter_value().double_value
        self.gripper_open_value = self.get_parameter('gripper_open_value').get_parameter_value().integer_value
        self.gripper_close_value = self.get_parameter('gripper_close_value').get_parameter_value().integer_value
        self.gripper_threshold = self.get_parameter('gripper_threshold').get_parameter_value().double_value
        self.gripper_state_threshold = self.get_parameter('gripper_state_threshold').get_parameter_value().integer_value
        self.gripper_torque = self.get_parameter('gripper_torque').get_parameter_value().integer_value
        self.can_name = self.get_parameter('can_name').get_parameter_value().string_value
        
        # 3. 打印参数信息
        self.get_logger().info(
            f"""
================ Node Parameters ================
subscribe_topic         : {self.subscribe_topic}
publish_topic          : {self.publish_topic}
publish_rate           : {self.publish_rate} Hz
gripper_open_value     : {self.gripper_open_value}
gripper_close_value    : {self.gripper_close_value}
gripper_threshold      : {self.gripper_threshold}
gripper_state_threshold: {self.gripper_state_threshold}
gripper_torque         : {self.gripper_torque}
can_name               : {self.can_name}
================================================
            """
        )
        
        # 4. 订阅AliciaD状态
        self.subscription = self.create_subscription(
            Float32MultiArray,
            self.subscribe_topic,
            self.aliciaD_callback,
            3
        )
        
        # 5. 发布Piper状态
        self.publisher_ = self.create_publisher(
            Float32MultiArray,
            self.publish_topic,
            3
        )
        
        # 6. 发布频率
        if self.publish_rate <= 0:
            self.get_logger().warn(f'无效的发布频率: {self.publish_rate}, 使用默认值50Hz')
            self.publish_rate = 50.0
            
        self.timer = self.create_timer(1.0 / self.publish_rate, self.publish_qpos)
        
        # 7. 当前状态
        self.current_qpos = [0.0] * 7  # 6关节 + 1夹爪
        self.target_qpos = [0.0] * 7
        self.last_aliciaD_msg = None
        
        # 8. 初始化Piper机械臂
        self.init_piper()
        
        self.get_logger().info('Piper节点已启动')
    
    def init_piper(self):
        """初始化Piper机械臂"""
        try:
            self.arm = C_PiperInterface_V2(can_name=self.can_name)
            self.arm.ConnectPort()
            
            # 使用参数化的夹爪力矩值
            ret = enable_fun(piper=self.arm, gripper_torque=self.gripper_torque)
            if ret != 0:
                self.get_logger().error('Piper机械臂使能失败')
                self.arm_connected = False
                return
            
            self.get_logger().info('Piper机械臂连接成功')
            self.arm_connected = True
            
        except Exception as e:
            self.get_logger().error(f'Piper机械臂连接失败: {str(e)}')
            self.arm_connected = False
    
    def aliciaD_callback(self, msg):
        """AliciaD状态回调函数"""
        self.last_aliciaD_msg = msg
        
        if not self.arm_connected:
            self.get_logger().warn('Piper未连接，忽略控制指令')
            return
        
        if len(msg.data) >= 7:
            # 提取关节角度和夹爪状态
            alicia_joints = list(msg.data[:6])
            alicia_gripper = msg.data[6]
            
            self.target_qpos[:6] = alicia_joints
            
            # 使用配置的阈值和控制值
            if alicia_gripper >= self.gripper_threshold:
                self.target_qpos[6] = float(self.gripper_open_value)
                self.get_logger().debug(f'夹爪打开: {self.gripper_open_value}')
            else:
                self.target_qpos[6] = float(self.gripper_close_value)
                self.get_logger().debug(f'夹爪关闭: {self.gripper_close_value}')
            
            # 控制Piper
            self.control_piper(self.target_qpos)
        else:
            self.get_logger().warn(f'接收到的消息数据长度不足: {len(msg.data)} < 7')
    
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
            
            # 夹爪控制，使用配置的力矩值
            gripper_value = int(qpos[6])
            self.arm.GripperCtrl(gripper_value, self.gripper_torque, 0x01, 0)
            time.sleep(0.005)
            
        except Exception as e:
            self.get_logger().error(f'控制Piper失败: {str(e)}')
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
            
            # 使用配置的阈值判断夹爪状态
            gripper_state = 1.0 if gripper_raw > self.gripper_state_threshold else 0.0
            
            return joint_pos + [gripper_state]
            
        except Exception as e:
            self.get_logger().warn(f'读取Piper状态失败: {str(e)}')
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
        
        # 调试信息
        self.get_logger().debug(f'发布状态: {qpos}')
    
    def destroy_node(self):
        """节点销毁时清理资源"""
        if hasattr(self, 'arm_connected') and self.arm_connected:
            try:
                self.get_logger().info('正在断开Piper机械臂连接...')
                # 关闭机械臂（如果SDK提供了关闭方法）
                self.get_logger().info('Piper机械臂已断开连接')
            except Exception as e:
                self.get_logger().warn(f'断开连接时发生错误: {str(e)}')
        super().destroy_node()

def parse_arguments():
    """解析命令行参数"""
    parser = argparse.ArgumentParser(description='Piper机械臂控制节点')
    
    parser.add_argument('--subscribe_topic', type=str, default='/aliciaD/action',
                       help='订阅的动作话题名 (默认: /aliciaD/action)')
    parser.add_argument('--publish_topic', type=str, default='/piper/qpos',
                       help='发布的状态话题名 (默认: /piper/qpos)')
    parser.add_argument('--publish_rate', type=float, default=50.0,
                       help='发布状态的频率(Hz) (默认: 50)')
    parser.add_argument('--gripper_open_value', type=int, default=74000,
                       help='控制夹爪打开时的控制量 (默认: 74000)')
    parser.add_argument('--gripper_close_value', type=int, default=50,
                       help='控制夹爪关闭时的控制量 (默认: 50)')
    parser.add_argument('--gripper_threshold', type=float, default=0.5,
                       help='控制夹爪开关判定阈值 (默认: 0.5)')
    parser.add_argument('--gripper_state_threshold', type=int, default=64000,
                       help='判断夹爪开关状态的阈值 (默认: 64000)')
    parser.add_argument('--gripper_torque', type=int, default=3000,
                       help='控制夹爪开关的力矩 (默认: 3000)')
    parser.add_argument('--can_name', type=str, default='can0',
                       help='CAN接口名称 (默认: can0)')
    
    return parser.parse_args()

def main(args=None):
    """主函数，支持两种参数传递方式"""
    
    # 检查是否使用ROS2参数格式
    is_ros_args = False
    for arg in sys.argv:
        if '--ros-args' in arg or '-p' in arg:
            is_ros_args = True
            break
    
    if is_ros_args:
        # 方式1：使用ROS2参数系统
        rclpy.init(args=args)
        node = PiperNode()  # ROS2会处理参数
    else:
        # 方式2：使用传统命令行参数
        cli_args = parse_arguments()
        
        # 准备ROS2参数
        ros_args = ['--ros-args']
        for param_name, param_value in vars(cli_args).items():
            if isinstance(param_value, str):
                ros_args.extend(['-p', f'{param_name}:={param_value}'])
            else:
                ros_args.extend(['-p', f'{param_name}:={str(param_value)}'])
        
        # 初始化ROS2并传递参数
        rclpy.init(args=ros_args)
        
        # 创建节点并传递参数
        node_params = {
            'subscribe_topic': cli_args.subscribe_topic,
            'publish_topic': cli_args.publish_topic,
            'publish_rate': cli_args.publish_rate,
            'gripper_open_value': cli_args.gripper_open_value,
            'gripper_close_value': cli_args.gripper_close_value,
            'gripper_threshold': cli_args.gripper_threshold,
            'gripper_state_threshold': cli_args.gripper_state_threshold,
            'gripper_torque': cli_args.gripper_torque,
            'can_name': cli_args.can_name
        }
        node = PiperNode(**node_params)
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()