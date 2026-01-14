#!/usr/bin/env python3
"""
aliciaD_node.py - 发布AliciaD机械臂状态
发布话题: /aliciaD/action (Float32MultiArray) - [关节弧度1-6, 夹爪状态(0/1)]

运行方式：
1. 直接运行（支持传统命令行参数）：
   python3 aliciaD_node.py --publish_rate 100 --port /dev/ttyACM0

2. ROS2运行（支持ROS2参数格式）：
   ros2 run package_name aliciaD_node --ros-args -p publish_rate:=100.0 -p port:=/dev/ttyACM0

3. 通过launch文件运行
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import alicia_d_sdk
import sys
import argparse

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
    def __init__(self, **kwargs):
        super().__init__('aliciaD_node')
        
        # 1. 声明所有参数
        self.declare_parameter('publish_topic', kwargs.get('publish_topic', '/aliciaD/action'))
        self.declare_parameter('publish_rate', kwargs.get('publish_rate', 50.0))
        self.declare_parameter('port', kwargs.get('port', '/dev/ttyUSB0'))
        self.declare_parameter('baudrate', kwargs.get('baudrate', 1000000))
        self.declare_parameter('gripper_threshold', kwargs.get('gripper_threshold', 70))
        self.declare_parameter('robot_version', kwargs.get('robot_version', 'v5_6'))
        self.declare_parameter('robot_type', kwargs.get('robot_type', 'follower'))
        self.declare_parameter('gripper_type', kwargs.get('gripper_type', '50mm'))
        
        # 2. 获取参数值
        self.publish_topic = self.get_parameter('publish_topic').get_parameter_value().string_value
        self.publish_rate = self.get_parameter('publish_rate').get_parameter_value().double_value
        self.port = self.get_parameter('port').get_parameter_value().string_value
        self.baudrate = self.get_parameter('baudrate').get_parameter_value().integer_value
        self.gripper_threshold = self.get_parameter('gripper_threshold').get_parameter_value().integer_value
        self.robot_version = self.get_parameter('robot_version').get_parameter_value().string_value
        self.robot_type = self.get_parameter('robot_type').get_parameter_value().string_value
        self.gripper_type = self.get_parameter('gripper_type').get_parameter_value().string_value
        
        # 3. 打印参数信息
        self.get_logger().info(
            f"""
================ Node Parameters ================
publish_topic      : {self.publish_topic}
publish_rate       : {self.publish_rate} Hz
port               : {self.port}
baudrate           : {self.baudrate}
gripper_threshold  : {self.gripper_threshold}
robot_version      : {self.robot_version}
robot_type         : {self.robot_type}
gripper_type       : {self.gripper_type}
================================================
            """
        )
        
        # 4. 创建发布者
        self.publisher_ = self.create_publisher(
            Float32MultiArray, 
            self.publish_topic, 
            3
        )
        
        # 5. 发布频率验证和设置
        if self.publish_rate <= 0:
            self.get_logger().warn(f'无效的发布频率: {self.publish_rate}, 使用默认值50Hz')
            self.publish_rate = 50.0
            
        self.timer = self.create_timer(1.0/self.publish_rate, self.publish_status)
        
        # 6. 机器人状态
        self.current_joints = [0.0] * 6  # 6个关节
        self.current_gripper = 0.0
        
        # 7. 机器人实例
        self.robot = None
        
        # 8. 初始化机器人连接
        self.init_robot()
        
        self.get_logger().info(f'AliciaD节点已启动，发布频率: {self.publish_rate}Hz')
    
    def init_robot(self):
        """初始化机器人连接"""
        try:
            self.get_logger().info(f'正在连接机器人: port={self.port}, baudrate={self.baudrate}')
            
            self.robot = alicia_d_sdk.create_robot(
                port=self.port,
                baudrate=self.baudrate,
                robot_version=self.robot_version,
                robot_type=self.robot_type,
                gripper_type=self.gripper_type
            )
            
            if self.robot.connect():
                self.get_logger().info('AliciaD机器人连接成功')
                
                # 获取初始状态以验证连接
                try:
                    joints = self.robot.get_joints()
                    gripper_raw = self.robot.get_gripper()
                    self.get_logger().info(f'初始状态 - 关节: {joints}, 夹爪: {gripper_raw}')
                except Exception as e:
                    self.get_logger().warn(f'读取初始状态时出错: {str(e)}')
                    
            else:
                self.get_logger().error('AliciaD机器人连接失败')
                self.robot = None
                
        except Exception as e:
            self.get_logger().error(f'初始化机器人失败: {str(e)}')
            self.robot = None
    
    def read_robot_status(self):
        """读取机器人状态"""
        if self.robot is None:
            # 尝试重新连接
            self.get_logger().warn('机器人未连接，尝试重新连接...')
            self.init_robot()
            return None, None
        
        try:
            joints = self.robot.get_joints()  # 6个关节的弧度值
            gripper_raw = self.robot.get_gripper()  # 夹爪原始值

            # 投影到Piper关节范围
            piper_joint_pos = joint_projection(joints)
            
            # 使用配置的阈值进行夹爪二值化
            gripper = 1.0 if gripper_raw >= self.gripper_threshold else 0.0
            
            return piper_joint_pos, gripper
            
        except Exception as e:
            self.get_logger().warn(f'读取机器人状态失败: {str(e)}')
            self.robot = None
            return None, None
    
    def publish_status(self):
        """发布机器人状态"""
        joints, gripper = self.read_robot_status()
        
        if joints is None or gripper is None:
            # 使用上次的有效值
            joints = self.current_joints
            gripper = self.current_gripper
            self.get_logger().debug('使用缓存的状态数据')
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
        
        # 定期显示状态（每10次发布显示一次）
        if hasattr(self, '_publish_count'):
            self._publish_count += 1
        else:
            self._publish_count = 0
            
        if self._publish_count % 10 == 0:
            self.get_logger().debug(f'发布状态: 关节={joints}, 夹爪={gripper}')
    
    def destroy_node(self):
        """节点销毁时清理资源"""
        if self.robot is not None:
            try:
                self.get_logger().info('正在断开机器人连接...')
                self.robot.disconnect()
                self.get_logger().info('机器人已断开连接')
            except Exception as e:
                self.get_logger().warn(f'断开连接时发生错误: {str(e)}')
        super().destroy_node()

def parse_arguments():
    """解析命令行参数"""
    parser = argparse.ArgumentParser(description='AliciaD机械臂节点')
    
    parser.add_argument('--publish_topic', type=str, default='/aliciaD/action',
                       help='发布的状态话题名 (默认: /aliciaD/action)')
    parser.add_argument('--publish_rate', type=float, default=50.0,
                       help='状态发布频率(Hz) (默认: 50)')
    parser.add_argument('--port', type=str, default='/dev/ttyUSB0',
                       help='串口号 (默认: /dev/ttyUSB0)')
    parser.add_argument('--baudrate', type=int, default=1000000,
                       help='波特率 (默认: 1000000)')
    parser.add_argument('--gripper_threshold', type=int, default=70,
                       help='夹爪二值化阈值 (默认: 70)')
    
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
        node = AliciaDNode()  # ROS2会处理参数
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
            'publish_topic': cli_args.publish_topic,
            'publish_rate': cli_args.publish_rate,
            'port': cli_args.port,
            'baudrate': cli_args.baudrate,
            'gripper_threshold': cli_args.gripper_threshold
        }
        node = AliciaDNode(**node_params)
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()