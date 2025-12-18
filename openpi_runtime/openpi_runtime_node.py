# Copyright (c) 2025，D-Robotics.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

#!/usr/bin/env python3

import cv2
import cv_bridge
import numpy as np
import queue

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, JointState

import openpi_runtime.msg_pb2 as msg_pb2
from openpi_runtime.time_sync_manager import TimeSyncManager
from openpi_runtime.image_process import ImageProcessor
from openpi_runtime.model_client import Client

class OpenpiRuntimeNode(Node):
    def __init__(self):
        super().__init__('openpi_runtime_node')
        self.get_logger().warn('Openpi Runtime Node has been started.')

        # 1. common setting
        self.declare_parameter('user_prompt', "beat block hammer")
        self.declare_parameter('max_limit_num', 50)
        self.declare_parameter('state_sub_topic_name', "/joint_states")
        self.declare_parameter('camera_topic_name', '/camera/camera/color/image_raw')
        self.declare_parameter('camera_left_topic_name', '/camera_left/camera_left/color/image_raw')
        self.declare_parameter('camera_right_topic_name', '/camera_right/camera_right/color/image_raw')
        self.declare_parameter('embodiment', None)  # 本体类型，默认值为 None

        self.user_prompt = self.get_parameter('user_prompt').get_parameter_value().string_value
        self.max_limit_num = self.get_parameter('max_limit_num').get_parameter_value().integer_value
        self.state_sub_topic_name = self.get_parameter('state_sub_topic_name').get_parameter_value().string_value
        self.camera_topic_name = self.get_parameter('camera_topic_name').get_parameter_value().string_value
        self.camera_left_topic_name = self.get_parameter('camera_left_topic_name').get_parameter_value().string_value
        self.camera_right_topic_name = self.get_parameter('camera_right_topic_name').get_parameter_value().string_value
        self.embodiment = self.get_parameter('embodiment').get_parameter_value().string_value

        self.get_logger().warn(
            f"""
================ Node Parameters ================
user_prompt             : {self.user_prompt}
max_limit_num           : {self.max_limit_num}
state_sub_topic_name    : {self.state_sub_topic_name}
camera_topic_name       : {self.camera_topic_name}
camera_left_topic_name  : {self.camera_left_topic_name}
camera_right_topic_name : {self.camera_right_topic_name}
================================================
        """
        )

        # 2. arm status
        self.state_queue = queue.Queue(maxsize=100)
        self.state_sub = self.create_subscription(
            JointState, 
            self.state_sub_topic_name, 
            lambda msg: self.state_queue.put(msg),
            10
        )

        # init embodiment
        if self.embodiment == "" or self.embodiment is None:
            pass
        elif self.embodiment == "piper":
            from piper_sdk import C_PiperInterface_V2
            # from openpi_runtime.embodiments.piper.piper import piper_enable

            self.left_arm = C_PiperInterface_V2("can_piper")
            self.left_arm.ConnectPort()
            '''
            使能机械臂并检测使能状态,尝试5秒,超时则退出
            '''
            timeout = 5
            start_time = time.time()

            while True:
                low_msg = self.left_arm.GetArmLowSpdInfoMsgs()
                enabled = all([
                    low_msg.motor_1.foc_status.driver_enable_status,
                    low_msg.motor_2.foc_status.driver_enable_status,
                    low_msg.motor_3.foc_status.driver_enable_status,
                    low_msg.motor_4.foc_status.driver_enable_status,
                    low_msg.motor_5.foc_status.driver_enable_status,
                    low_msg.motor_6.foc_status.driver_enable_status
                ])
                print("--------------------")
                print("使能状态:", enabled)
                if enabled:
                    return 0
                    break
                self.left_arm.EnableArm(7)
                self.left_arm.GripperCtrl(0, 1000, 0x01, 0)
                if time.time() - start_time > timeout:
                    print("使能超时，退出程序")
                    return 1
                    sys.exit(1)
                time.sleep(1)





        # 3. image tensor setting
        self.bridge = cv_bridge.CvBridge()
        self.processor = ImageProcessor(target_size=(224, 224))
        self.render_type = False

        self.image_topic_names = [
            self.camera_topic_name,
            self.camera_left_topic_name,
            # self.camera_right_topic_name,
            # '/camera/camera/depth/image_rect_raw',
            # '/camera_left/camera_left/depth/image_rect_raw',
            # '/camera_right/camera_right/depth/image_rect_raw'
        ]
        self.sync_mgr = TimeSyncManager(
            topics=self.image_topic_names,
            max_queue_size=30,
            slop_ns=20_000_000   # 20 ms
        )

        for image_topic_name in self.image_topic_names:
            self.create_subscription(
                Image,
                image_topic_name,
                lambda msg, t=image_topic_name: self.sync_mgr.add_msg(t, msg),
                qos_profile_sensor_data
            )
            self.get_logger().info(f"Subscribed to {image_topic_name}")

        # 4. openpi model runtime params
        self.client = Client(port=8888)
        self.client.connect()

        self.openpi_running_status = True
        self.success_status = False
        self.current_num = 0

        # 5. main program in timer
        self.timer = self.create_timer(0.01, self.process_synced)

    def process_synced(self):

        group = self.sync_mgr.get_synced()

        if self.user_prompt is None:
            return

        if group is None:
            return

        if not self.openpi_running_status:
            return

        # 1. pre process images ======
        self.get_logger().info(
            f"[SYNC] ts={group.stamp_ns} "
            f"topics={list(group.frames.keys())}"
        )
        imgs = []
        campare_stamp = None
        for idx, (topic, img_msg) in enumerate(group.frames.items()):
            if idx == 0:
                campare_stamp = img_msg.header.stamp
            self.get_logger().info(
                f"  {topic}: encoding={img_msg.encoding}, width={img_msg.width}, height={img_msg.height}, timestamp={img_msg.header.stamp.sec}_{img_msg.header.stamp.nanosec}"
            )

            img = self.bridge.imgmsg_to_cv2(img_msg)            
            if self.render_type:
                stamp = img_msg.header.stamp
                filename = f"{str(idx)}_{stamp.sec}_{stamp.nanosec}.png"
                cv2.imwrite(filename, img)

            model_input = self.processor(img)
            imgs.append(model_input)

        # 2. get arms' states
        state = self.get_arm_state(campare_stamp)
        if state is None:
            self.get_logger().error("Get Arm State Failed! Please check!")
            return

        # 3. openpi model infer using socket client
        obs = {
                "images": {
                    "1": imgs[0], 
                    "2": imgs[1],
                    "3": imgs[1]},  # TODO
                "prompt": np.array((self.user_prompt)),
                "state": state
            }
        dtypes = {
                "images": msg_pb2.Tensor.UINT8,
                "prompt":msg_pb2.Tensor.STRING,
                "state": msg_pb2.Tensor.FLOAT64
            }

        self.client.send(obs, dtypes)
        result = self.client.receive()
        
        for prompt in result["prompt"][0]:
            ret_left = self.arm_control(prompt[0: 7], "left")
            ret_right = self.arm_control(prompt[7: 14], "right")
            if ret_left != 0 or ret_right != 0:
                self.openpi_running_status = False
                self.get_logger().error("Arm Opperation Failed! Please check!")
                return

        if self.check_success():
            self.openpi_running_status = False
            self.get_logger().warn("Task Succssed!")
            return
        
        if self.max_limit_num == self.current_num:
            self.openpi_running_status = False
            self.get_logger().error(f"The task has reached the maximum number {self.max_limit_num} of attempts. Task Failed!")
            return
        
        self.current_num += 1
        self.get_logger().warn(f"Openpi runtime count: {self.current_num}")

    def check_success(self):
        # TODO need fill the catched logic
        # if True:
        #     self.get_logger().warn("Task Succssed!")
        #     return True
        return False

    def get_arm_state(self, campare_stamp):
        state = np.zeros((1, 14), dtype=np.float64)
        if self.embodiment == "" or self.embodiment == None:
            if self.state_queue.empty():
                return None
            state = self.state_queue.get().position
            state = np.array(state).reshape(1, 14)
            
        elif self.embodiment == "piper":
            left_arm_msg = self.left_arm.GetArmJointMsgs()
            left_gripper_msg = self.left_arm.GetArmGripperMsgs()
            state[0, 0] = float(left_arm_msg.joint_state.joint_1) / 1000.0
            state[0, 1] = float(left_arm_msg.joint_state.joint_2) / 1000.0
            state[0, 2] = float(left_arm_msg.joint_state.joint_3) / 1000.0
            state[0, 3] = float(left_arm_msg.joint_state.joint_4) / 1000.0
            state[0, 4] = float(left_arm_msg.joint_state.joint_5) / 1000.0
            state[0, 5] = float(left_arm_msg.joint_state.joint_6) / 1000.0
            state[0, 6] = float(left_gripper_msg.gripper_state.grippers_angle) / 1000.0

        return state

    def arm_control(self, prompt, arm_type="left"):
        if self.embodiment == "" or self.embodiment == None:
            pass
        elif self.embodiment == "piper": 
            from openpi_runtime.piper.piper import pi0_control_piper
            if arm_type == "left":
                pi0_control_piper(self.left_arm, prompt)
            elif arm_type == "right":
                pass
        return 0

# ===================== main =====================

def main(args=None):
    rclpy.init(args=args)
    node = OpenpiRuntimeNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
