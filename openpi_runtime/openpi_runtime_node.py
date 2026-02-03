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
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, JointState

from std_msgs.msg import Float32MultiArray

import openpi_runtime.msg_pb2 as msg_pb2
from openpi_runtime.time_sync_manager import TimeSyncManager
from openpi_runtime.enhanced_time_sync_manager import (
    EnhancedTimeSyncManager,
    DataSyncThread,
)
from openpi_runtime.image_process import ImageProcessor
from openpi_runtime.model_client import Client


class OpenpiRuntimeNode(Node):
    def __init__(self):
        super().__init__("openpi_runtime_node")
        self.get_logger().warn("Openpi Runtime Node has been started.")

        # 在程序内指定归一化统计信息
        self.norm_stats = {
            "actions": {
                "mean": [
                    0.0001243453152710572,
                    0.012514984235167503,
                    0.01984572969377041,
                    0.0013279641279950738,
                    -0.006489368621259928,
                    0.0007160089444369078,
                    0.49960339069366455,
                    -0.0007041643257252872,
                    0.017729410901665688,
                    0.018347295001149178,
                    0.001461873296648264,
                    -0.005420160014182329,
                    0.00036438318784348667,
                    0.4558684229850769,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                ],
                "std": [
                    0.11045020073652267,
                    0.17611068487167358,
                    0.15441367030143738,
                    0.1229512095451355,
                    0.17716124653816223,
                    0.16023924946784973,
                    0.4079183042049408,
                    0.12189175188541412,
                    0.1991070955991745,
                    0.1783520132303238,
                    0.15127407014369965,
                    0.19766294956207275,
                    0.19989325106143951,
                    0.3901549279689789,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                ],
            },
            "state": {
                "mean": [
                    0.09093472361564636,
                    0.4708079695701599,
                    -0.7088257670402527,
                    0.09846086800098419,
                    0.4463886022567749,
                    -0.23006314039230347,
                    0.4967166483402252,
                    -0.06535510718822479,
                    0.27723944187164307,
                    -0.5581566095352173,
                    -0.07980682700872421,
                    0.5131563544273376,
                    0.23896420001983643,
                    0.46351152658462524,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                ],
                "std": [
                    0.2981235384941101,
                    0.5875609517097473,
                    0.5399824976921082,
                    0.2838532328605652,
                    0.4769848585128784,
                    0.4568907916545868,
                    0.3795894682407379,
                    0.3157930374145508,
                    0.5589326620101929,
                    0.5273948311805725,
                    0.3175123333930969,
                    0.4825332760810852,
                    0.5743520855903625,
                    0.3592356741428375,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                ],
            },
        }

        # 1. common setting
        # self.declare_parameter('user_prompt', "uncap the pen")
        self.declare_parameter("user_prompt", "put the bux")
        self.declare_parameter("max_limit_num", 50)
        self.declare_parameter("state_sub_topic_name", "/piper/qpos")
        self.declare_parameter("camera_topic_name", "/camera/camera/color/image_raw")
        self.declare_parameter(
            "camera_left_topic_name", "/camera_left/camera_left/color/image_raw"
        )
        self.declare_parameter(
            "camera_right_topic_name", "/camera_right/camera_right/color/image_raw"
        )
        self.declare_parameter("embodiment", None)  # 本体类型，默认值为 None
        self.declare_parameter("action_pub_topic", "/aliciaD/action")
        self.declare_parameter("action_pub_frequency", 30.0)  # Hz

        self.user_prompt = (
            self.get_parameter("user_prompt").get_parameter_value().string_value
        )
        self.max_limit_num = (
            self.get_parameter("max_limit_num").get_parameter_value().integer_value
        )
        self.state_sub_topic_name = (
            self.get_parameter("state_sub_topic_name")
            .get_parameter_value()
            .string_value
        )
        self.camera_topic_name = (
            self.get_parameter("camera_topic_name").get_parameter_value().string_value
        )
        self.camera_left_topic_name = (
            self.get_parameter("camera_left_topic_name")
            .get_parameter_value()
            .string_value
        )
        self.camera_right_topic_name = (
            self.get_parameter("camera_right_topic_name")
            .get_parameter_value()
            .string_value
        )
        self.embodiment = (
            self.get_parameter("embodiment").get_parameter_value().string_value
        )
        self.action_pub_topic = (
            self.get_parameter("action_pub_topic").get_parameter_value().string_value
        )

        self.get_logger().warn(
            f"""
================ Node Parameters ================
user_prompt             : {self.user_prompt}
max_limit_num           : {self.max_limit_num}
state_sub_topic_name    : {self.state_sub_topic_name}
action_pub_topic        : {self.action_pub_topic}
camera_topic_name       : {self.camera_topic_name}
camera_left_topic_name  : {self.camera_left_topic_name}
camera_right_topic_name : {self.camera_right_topic_name}
================================================
        """
        )

        # 2. arm status - 使用增强版时间同步管理器
        self.bridge = cv_bridge.CvBridge()
        self.processor = ImageProcessor(target_size=(224, 224), adapt_to_pi=True)
        self.render_type = False

        # 配置需要同步的话题
        self.image_topic_names = [
            self.camera_topic_name,
            self.camera_left_topic_name,
            # self.camera_right_topic_name,
        ]

        self.topics_config = {
            self.camera_topic_name: {"type": "image", "msg_type": Image},
            self.camera_left_topic_name: {"type": "image", "msg_type": Image},
            self.state_sub_topic_name: {"type": "state", "msg_type": Float32MultiArray},
        }

        # 创建数据同步线程
        self.data_sync_thread = DataSyncThread(
            topics_config=self.topics_config,
            max_queue_size=30,
            slop_ns=20_000_000,  # 20 ms
        )
        self.data_sync_thread.start()

        # 订阅图像话题
        for image_topic_name in self.image_topic_names:
            self.create_subscription(
                Image,
                image_topic_name,
                lambda msg, t=image_topic_name: self.data_sync_thread.add_msg(t, msg),
                qos_profile_sensor_data,
            )
            self.get_logger().info(f"Subscribed to {image_topic_name}")

        # 订阅机械臂状态话题
        self.state_sub = self.create_subscription(
            Float32MultiArray,
            self.state_sub_topic_name,
            lambda msg: self.data_sync_thread.add_msg(self.state_sub_topic_name, msg),
            3,
        )
        self.get_logger().info(f"Subscribed to {self.state_sub_topic_name}")

        # arm control
        self.action_publisher = self.create_publisher(
            Float32MultiArray, self.action_pub_topic, 3
        )

        # 4. openpi model runtime params
        self.client = Client(port=8888)
        self.client.connect()

        self.openpi_running_status = True
        self.success_status = False
        self.current_num = 0

        # 5. main program in timer
        self.timer = self.create_timer(0.01, self.process_synced)

    def __del__(self):
        """析构函数，确保清理资源"""
        try:
            if hasattr(self, "data_sync_thread"):
                self.data_sync_thread.stop()
        except:
            pass

    def process_synced(self):
        """处理同步的数据"""
        if self.user_prompt is None:
            return

        if not self.openpi_running_status:
            return

        # 获取同步的数据
        synced_data = self.data_sync_thread.get_synced_data()
        if synced_data is None:
            return

        # 1. 处理图像数据
        self.get_logger().info(
            f"[SYNC] ts={synced_data.stamp_ns} topics={list(synced_data.data.keys())}"
        )

        imgs = []
        timestamp = synced_data.timestamp

        for idx, topic in enumerate(self.image_topic_names):
            if topic not in synced_data.data:
                self.get_logger().error(f"Missing {topic} in synced data")
                return

            img_msg = synced_data.data[topic]
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

        # 2. 处理机械臂状态数据
        if self.state_sub_topic_name not in synced_data.data:
            self.get_logger().error(
                f"Missing {self.state_sub_topic_name} in synced data"
            )
            return

        state_msg = synced_data.data[self.state_sub_topic_name]
        state = self.process_arm_state(state_msg)
        if state is None:
            self.get_logger().error("Process Arm State Failed! Please check!")
            return

            img = self.bridge.imgmsg_to_cv2(img_msg)
            if self.render_type:
                stamp = img_msg.header.stamp
                filename = f"{str(idx)}_{stamp.sec}_{stamp.nanosec}.png"
                cv2.imwrite(filename, img)

            model_input = self.processor(img)
            imgs.append(model_input)

        # 3. openpi model infer using socket client
        # 创建黑色占位图像用于右臂摄像头
        black_img = np.zeros_like(imgs[1])

        # 前处理：将自然语言指令转换为pi内部运行时使用的指令格式
        preprocessed_prompt = self._preprocess_prompt(
            self.user_prompt, adapt_to_pi=True
        )

        obs = {
            "images": {
                "1": imgs[0],
                "2": imgs[1],
                "3": black_img,
            },  # 右臂使用黑色占位图像
            "prompt": np.array((preprocessed_prompt)),
            "state": state,
        }
        dtypes = {
            "images": msg_pb2.Tensor.UINT8,
            "prompt": msg_pb2.Tensor.STRING,
            "state": msg_pb2.Tensor.FLOAT64,
        }

        self.client.send(obs, dtypes)
        result = self.client.receive()

        if result is None:
            self.get_logger().error("未收到推理结果，请检查推理端连接！")
            return

        # 后处理：将32维动作数据转换为14维并应用反归一化等处理
        processed_prompt = self.postprocess_actions(
            result["prompt"][0], adapt_to_pi=True
        )

        for prompt in processed_prompt:
            ret_left = self.arm_control(prompt[0:7], "left")
            ret_right = self.arm_control(prompt[7:14], "right")
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
            self.get_logger().error(
                f"The task has reached the maximum number {self.max_limit_num} of attempts. Task Failed!"
            )
            return

        self.current_num += 1
        self.get_logger().warn(f"Openpi runtime count: {self.current_num}")

    def check_success(self):
        # TODO need fill the catched logic
        # if True:
        #     self.get_logger().warn("Task Succssed!")
        #     return True
        return False

    def process_arm_state(self, state_msg):
        """处理机械臂状态消息，只处理7维Float32MultiArray消息并扩展为14维"""
        try:
            # 检查消息类型是否为Float32MultiArray
            if not hasattr(state_msg, "data"):
                self.get_logger().error(
                    f"消息类型错误: 期望Float32MultiArray，实际{type(state_msg)}"
                )
                return None

            # 获取数据维度
            data_len = len(state_msg.data)

            # 只处理7维数据
            if data_len != 7:
                self.get_logger().error(
                    f"状态数据维度错误: 期望7维(/piper/qpos)，实际{data_len}维。数据: {state_msg.data}"
                )
                return None

            # 7维数据，假设是左臂数据，右臂补0
            state = np.zeros((1, 14), dtype=np.float64)
            state[0, 0:7] = state_msg.data[:7]  # 左臂7维
            state[0, 7:14] = 0.0  # 右臂补0

            # 前处理：将标准Aloha关节状态转换为pi内部运行时使用的关节状态
            preprocessed_state = self._preprocess_joint_state(
                state[0], adapt_to_pi=True
            )
            state[0] = preprocessed_state

            return state

        except Exception as e:
            # 其他异常处理
            self.get_logger().error(f"处理机械臂状态时发生异常: {e}", exc_info=True)
            return None

    def get_arm_state(self, campare_stamp):
        """获取机械臂状态（保留原函数以兼容旧代码）"""
        # 这个函数现在主要用于兼容，实际使用process_arm_state
        return None

        try:
            # 从队列获取最新状态消息
            state_msg = self.state_queue.get_nowait()

            # 检查消息类型是否为Float32MultiArray
            if not hasattr(state_msg, "data"):
                self.get_logger().error(
                    f"消息类型错误: 期望Float32MultiArray，实际{type(state_msg)}"
                )
                return None

            # 获取数据维度
            data_len = len(state_msg.data)

            # 只处理7维数据
            if data_len != 7:
                self.get_logger().error(
                    f"状态数据维度错误: 期望7维(/piper/qpos)，实际{data_len}维。数据: {state_msg.data}"
                )
                return None

            # 7维数据，假设是左臂数据，右臂补0
            state = np.zeros((1, 14), dtype=np.float64)
            state[0, 0:7] = state_msg.data[:7]  # 左臂7维
            state[0, 7:14] = 0.0  # 右臂补0

            # 前处理：将标准Aloha关节状态转换为pi内部运行时使用的关节状态
            preprocessed_state = self._preprocess_joint_state(
                state[0], adapt_to_pi=True
            )
            state[0] = preprocessed_state

            return state

        except queue.Empty:
            # 队列为空时返回None
            self.get_logger().debug("状态队列已空")
            return None
        except Exception as e:
            # 其他异常处理
            self.get_logger().error(f"获取机械臂状态时发生异常: {e}", exc_info=True)
            return None

    def arm_control(self, prompt, arm_type="left"):
        if arm_type == "left":
            self.get_logger().info(f"left arm")
            self.get_logger().info(f"arm_control接收到prompt: {prompt}")
            # self.get_logger().info(f"prompt类型: {type(prompt)}")
            # if hasattr(prompt, 'shape'):
            #     self.get_logger().info(f"prompt形状: {prompt.shape}")
            # else:
            #     self.get_logger().info(f"prompt长度: {len(prompt)}")

            left_action = np.array(prompt)
            # 创建Float32MultiArray消息
            action_msg = Float32MultiArray()
            action_msg.data = [float(x) for x in left_action]

            # 发布动作
            self.action_publisher.publish(action_msg)
            time.sleep(0.02)

        elif arm_type == "right":
            pass
            # self.get_logger().info(f"right arm")

        # if self.embodiment == "" or self.embodiment == None:
        #     pass
        # elif self.embodiment == "piper":
        #     from openpi_runtime.piper.piper import pi0_control_piper
        #     if arm_type == "left":
        #         pi0_control_piper(self.left_arm, prompt)
        #     elif arm_type == "right":
        #         pass
        return 0

    def _joint_flip_mask(self):
        """Used to convert between aloha and pi joint angles."""
        return np.array([1, -1, -1, 1, 1, 1, 1, 1, -1, -1, 1, 1, 1, 1])

    def _normalize(self, x, min_val, max_val):
        return (x - min_val) / (max_val - min_val)

    def _unnormalize(self, x, min_val, max_val):
        return x * (max_val - min_val) + min_val

    def _gripper_from_angular(self, value):
        # Convert from the gripper position used by pi0 to the gripper position that is used by Aloha.
        value = value + 0.5476
        return self._normalize(value, min_val=-0.6213, max_val=1.4910)

    def _gripper_to_angular(self, gripper_value):
        """
        将标准Aloha夹爪值转换为pi内部运行时使用的角度值
        与process_adapt_to_pi_true.md文档中第153-166行完全一致
        """
        # 将0-1范围转换为-1到1范围
        normalized_value = 2 * gripper_value - 1
        # 使用反正切函数转换为角度值
        return np.arctan2(normalized_value, 1)  # 返回弧度值

    def _preprocess_joint_state(self, joint_state, adapt_to_pi=True):
        """
        将标准Aloha关节状态转换为pi内部运行时使用的关节状态（前处理）
        与process_adapt_to_pi_true.md文档中第192-211行完全一致
        """
        if adapt_to_pi:
            # 1. 应用关节翻转
            flipped_joints = joint_state * self._joint_flip_mask()

            # 2. 转换夹爪值
            # 左侧夹爪（索引6）
            flipped_joints[6] = self._gripper_to_angular(joint_state[6])
            # 右侧夹爪（索引13）
            flipped_joints[13] = self._gripper_to_angular(joint_state[13])

            return flipped_joints
        else:
            return joint_state

    def _preprocess_prompt(self, prompt, adapt_to_pi=True):
        """
        将自然语言指令转换为pi内部运行时使用的指令格式（前处理）
        与process_adapt_to_pi_true.md文档中第248-263行完全一致
        """
        if adapt_to_pi:
            # 1. 指令标准化
            # 将指令转换为pi内部运行时支持的标准化格式
            standardized_prompt = prompt.strip()

            # 2. 指令增强
            # 为指令添加额外的上下文信息以提高模型的理解准确率
            # 包括环境信息、物体属性和任务约束
            enhanced_prompt = standardized_prompt

            # 3. 指令优先级调整
            # 根据pi内部运行时的要求调整指令的优先级
            # 这里简单实现，实际项目中可能需要更复杂的优先级调整逻辑
            priority = 5  # 默认中等优先级

            return enhanced_prompt
        else:
            return prompt

    def _encode_actions(self, actions, adapt_to_pi=True):
        """与process_adapt_to_pi_true.md文档中第869-877行完全一致"""
        if adapt_to_pi:
            # 先进行夹爪角度转换的逆操作
            actions[:, [6, 13]] = self._gripper_from_angular(actions[:, [6, 13]])
            # 再执行关节翻转的逆操作
            actions = self._joint_flip_mask() * actions
        return actions

    def postprocess_actions(self, actions, adapt_to_pi=True):
        """执行完整后处理步骤（与openpi-main项目中的处理流程一致）"""
        # 确保actions是二维数组（形状为[N, 32]）
        actions = np.asarray(actions)
        if actions.ndim == 1:
            actions = actions.reshape(1, -1)

        # 步骤1: 只保留前14个维度（与AlohaOutputs.__call__第100行一致）
        actions = actions[:, :14]

        # 步骤2: 反归一化（将动作从归一化空间转换回原始空间）
        # 检查norm_stats的结构，正确的路径是norm_stats.actions.mean和norm_stats.actions.std
        if (
            "actions" in self.norm_stats
            and "mean" in self.norm_stats["actions"]
            and "std" in self.norm_stats["actions"]
        ):
            action_mean = np.array(self.norm_stats["actions"]["mean"])[
                :14
            ]  # 只使用前14个维度
            action_std = np.array(self.norm_stats["actions"]["std"])[
                :14
            ]  # 只使用前14个维度
            # 确保action_mean和action_std的形状与动作维度匹配
            if action_mean.ndim == 0:
                action_mean = np.full(actions.shape[1], action_mean)
            if action_std.ndim == 0:
                action_std = np.full(actions.shape[1], action_std)
            # 反归一化
            actions = actions * action_std + action_mean

        # 步骤3: 应用关节翻转和爪手转换（与AlohaOutputs.__call__第101行和_encode_actions函数一致）
        actions = self._encode_actions(actions, adapt_to_pi=adapt_to_pi)

        # 步骤4: 只保留左臂的7个关节角度（索引0-6）
        actions = actions[:, :7]

        return actions


# ===================== main =====================


def main(args=None):
    rclpy.init(args=args)
    node = OpenpiRuntimeNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # 清理资源
        if hasattr(node, "data_sync_thread"):
            node.data_sync_thread.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
