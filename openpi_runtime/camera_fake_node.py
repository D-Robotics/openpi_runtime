#!/usr/bin/env python3
"""
camera_fake_node.py - 发布模拟相机图像
发布话题: /camera_fake/right_wrist (sensor_msgs/msg/Image) - 纯灰色图像

运行方式：
1. 直接运行（支持传统命令行参数）：
   python3 camera_fake_node.py --width 640 --height 480 --publish_rate 30

2. ROS2运行（支持ROS2参数格式）：
   ros2 run openpi_runtime camera_fake_node --ros-args -p width:=640 -p height:=480 -p publish_rate:=30.0
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import numpy as np
from cv_bridge import CvBridge

class CameraFakeNode(Node):
    def __init__(self, **kwargs):
        super().__init__('camera_fake_node')
        
        # 1. 声明所有参数
        self.declare_parameter('width', kwargs.get('width', 640))
        self.declare_parameter('height', kwargs.get('height', 480))
        self.declare_parameter('publish_rate', kwargs.get('publish_rate', 30.0))
        self.declare_parameter('topic_name', kwargs.get('topic_name', '/camera_fake/right_wrist'))
        
        # 2. 获取参数值
        self.width = self.get_parameter('width').get_parameter_value().integer_value
        self.height = self.get_parameter('height').get_parameter_value().integer_value
        self.publish_rate = self.get_parameter('publish_rate').get_parameter_value().double_value
        self.topic_name = self.get_parameter('topic_name').get_parameter_value().string_value
        
        # 3. 打印参数信息
        self.get_logger().info(
            f"""
================ Node Parameters ================
width              : {self.width}
height             : {self.height}
publish_rate       : {self.publish_rate} Hz
topic_name         : {self.topic_name}
================================================
            """
        )
        
        # 4. 创建发布者
        self.publisher_ = self.create_publisher(
            Image, 
            self.topic_name, 
            10
        )
        
        # 5. 发布频率验证和设置
        if self.publish_rate <= 0:
            self.get_logger().warn(f'无效的发布频率: {self.publish_rate}, 使用默认值30Hz')
            self.publish_rate = 30.0
            
        self.timer = self.create_timer(1.0/self.publish_rate, self.publish_image)
        
        # 6. 初始化CVBridge
        self.bridge = CvBridge()
        
        self.get_logger().info(f'相机模拟节点已启动，发布频率: {self.publish_rate}Hz')
    
    def publish_image(self):
        """发布纯灰色图像"""
        # 创建纯灰色图像 (0-255, 灰色使用128)
        gray_value = 128
        image = np.full((self.height, self.width), gray_value, dtype=np.uint8)
        
        # 转换为ROS图像消息
        msg = self.bridge.cv2_to_imgmsg(image, encoding='mono8')
        
        # 设置时间戳
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'right_wrist_camera'
        
        # 发布消息
        self.publisher_.publish(msg)

import sys
import argparse

def parse_arguments():
    """解析命令行参数"""
    parser = argparse.ArgumentParser(description='相机模拟节点')
    
    parser.add_argument('--width', type=int, default=640,
                       help='图像宽度 (默认: 640)')
    parser.add_argument('--height', type=int, default=480,
                       help='图像高度 (默认: 480)')
    parser.add_argument('--publish_rate', type=float, default=30.0,
                       help='发布频率(Hz) (默认: 30)')
    parser.add_argument('--topic_name', type=str, default='/camera_fake/right_wrist',
                       help='发布话题名 (默认: /camera_fake/right_wrist)')
    
    return parser.parse_args()

def main(args=None):
    """主函数，支持两种参数传递方式"""
    
    # 检查是否使用ROS2参数格式
    is_ros_args = False
    for arg in sys.argv:
        if '--ros-args' == arg or arg == '-p':
            is_ros_args = True
            break
    
    if is_ros_args:
        # 方式1：使用ROS2参数系统
        rclpy.init(args=args)
        node = CameraFakeNode()  # ROS2会处理参数
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
            'width': cli_args.width,
            'height': cli_args.height,
            'publish_rate': cli_args.publish_rate,
            'topic_name': cli_args.topic_name
        }
        node = CameraFakeNode(**node_params)
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
