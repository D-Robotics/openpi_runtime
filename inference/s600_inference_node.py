#!/usr/bin/env python3

from typing import Dict, Optional, Any
import os
import json
import dataclasses
import enum
import logging
import time
import sys

import numpy as np
import cv2
import tyro

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
import cv_bridge

from common.utils.ros_data_collector import ROSDataCollector
from common.utils.topic_time_synchronizer import TopicTimeSynchronizer
from common.msg import msg_pb2
from common.pi0_process.preprocess import preprocess
from common.pi0_process.postprocess import postprocess
from common.ServerUtils import Server

logger = logging.getLogger(__name__)


class EnvMode(enum.Enum):
    PIPER = "piper"


@dataclasses.dataclass
class Args:
    num_steps: int = 1250
    env: EnvMode = EnvMode.PIPER
    verbose: bool = False

    camera_topics: Dict[str, str] = dataclasses.field(
        default_factory=lambda: {
            "cam_high": "/camera/camera/color/image_raw",
            "cam_left_wrist": "/camera_left/camera_left/color/image_raw",
        }
    )

    expected_cameras: tuple = ("cam_high", "cam_low", "cam_left_wrist", "cam_right_wrist")

    qpos_topic: str = "/piper/qpos"
    action_topic: str = "/aliciaD/action"
    action_chunk_size: int = 50
    wait_timeout: float = 10.0
    sync_time_window: float = 0.1
    norm_stats_path: str = ""


class PiperROSNode(Node):
    def __init__(self, args: Args):
        super().__init__("piper_ros_node")
        self.args = args

        self.declare_parameter("norm_stats_path", args.norm_stats_path)
        self.args.norm_stats_path = self.get_parameter("norm_stats_path").value or args.norm_stats_path

        self.declare_parameter("num_steps", args.num_steps)
        self.args.num_steps = self.get_parameter("num_steps").value

        self.declare_parameter("verbose", args.verbose)
        self.args.verbose = self.get_parameter("verbose").value

        self.declare_parameter("action_topic", args.action_topic)
        self.args.action_topic = self.get_parameter("action_topic").value

        self.declare_parameter("qpos_topic", args.qpos_topic)
        self.args.qpos_topic = self.get_parameter("qpos_topic").value

        self.declare_parameter("wait_timeout", args.wait_timeout)
        self.args.wait_timeout = self.get_parameter("wait_timeout").value

        self.bridge = cv_bridge.CvBridge()
        self.collector = ROSDataCollector(self)

        sync_topics = list(args.camera_topics.keys()) + ["qpos"]
        self.topic_synchronizer = TopicTimeSynchronizer(
            topics=sync_topics,
            reference_topic=list(args.camera_topics.keys())[0],
            sync_window_ms=args.sync_time_window * 1000,
            buffer_size=20,
            max_sync_queue_size=10,
        )

        self.camera_topics = args.camera_topics
        self.image_received = {cam: False for cam in self.camera_topics.keys()}

        for cam_name, topic in self.camera_topics.items():
            qos = QoSProfile(
                reliability=QoSReliabilityPolicy.BEST_EFFORT,
                durability=QoSDurabilityPolicy.VOLATILE,
                depth=10,
            )
            self.collector.add_topic(cam_name, topic, Image, lambda msg, name=cam_name: self._process_image(msg, name), qos)
            logger.info(f"订阅摄像头: {cam_name} -> {topic}")

        qos = QoSProfile(reliability=QoSReliabilityPolicy.RELIABLE, durability=QoSDurabilityPolicy.VOLATILE, depth=10)
        self.collector.add_topic("qpos", args.qpos_topic, Float32MultiArray, self._process_qpos, qos)
        logger.info(f"订阅qpos: {args.qpos_topic}")

        self.collector.set_sync_group(sync_topics, args.sync_time_window)
        self.collector.start()

        self.action_publisher = self.create_publisher(Float32MultiArray, args.action_topic, 60)
        logger.info(f"创建动作发布: {args.action_topic}")
        self.publisher_created = True

        self.image_count = {cam: 0 for cam in self.camera_topics.keys()}
        self.qpos_count = 0
        self.qpos_received = False
        self.qpos_valid = False
        self.latest_qpos = None
        self.start_time = time.time()

    def _process_image(self, msg: Image, cam_name: str = None):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="rgb8")
            cv_image = cv2.resize(cv_image, (224, 224))
            image_chw = cv_image.transpose(2, 0, 1).astype(np.uint8)

            if cam_name:
                timestamp_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
                self.topic_synchronizer.add_message(cam_name, image_chw, timestamp_ns)
                self.image_received[cam_name] = True

            return image_chw
        except Exception as e:
            logger.error(f"处理图像失败: {e}")
            raise

    def _process_qpos(self, msg: Float32MultiArray):
        try:
            if len(msg.data) >= 7:
                data = np.array(msg.data[:7], dtype=np.float32)
                timestamp_ns = int(time.time() * 1_000_000_000)
                self.topic_synchronizer.add_message("qpos", data, timestamp_ns)
                self.latest_qpos = data
                self.qpos_received = True
                self.qpos_valid = True
                return data
            else:
                logger.error(f"qpos数据维度不足: {len(msg.data)} < 7")
                self.qpos_valid = False
                raise ValueError(f"qpos数据维度不足: {len(msg.data)} < 7")
        except Exception as e:
            logger.error(f"处理qpos失败: {e}")
            self.qpos_valid = False
            raise

    def get_latest_observation(self) -> Optional[Dict[str, Any]]:
        synced_data = self.topic_synchronizer.get_synced_data(max_age_ms=100)
        if synced_data is None:
            return None

        images = {}
        for cam_name in self.camera_topics.keys():
            if cam_name not in synced_data.data:
                return None
            images[cam_name] = synced_data.data[cam_name]
            self.image_received[cam_name] = True
            self.image_count[cam_name] += 1

        qpos = synced_data.data.get("qpos")
        if qpos is None:
            return None

        self.qpos_count += 1

        for cam_key in self.args.expected_cameras:
            if cam_key not in images:
                images[cam_key] = np.zeros((3, 224, 224), dtype=np.uint8)

        full_state = np.concatenate([qpos, np.zeros(7)])
        return {"state": full_state.astype(np.float32), "images": images, "prompt": "put the box"}

    def publish_action(self, action: np.ndarray):
        if not self.publisher_created:
            raise RuntimeError("动作发布话题未创建")

        try:
            if len(action) < 7:
                raise ValueError(f"动作维度不足: {len(action)} < 7")

            action_7d = action[:7]
            msg = Float32MultiArray()
            msg.data = action_7d.tolist()
            self.action_publisher.publish(msg)
        except Exception as e:
            logger.error(f"发布动作失败: {e}")
            raise

    def wait_for_data(self, timeout: float) -> bool:
        start_time = time.time()
        while time.time() - start_time < timeout:
            all_images_received = all(self.image_received.values())
            qpos_ready = self.qpos_received and self.qpos_valid

            if all_images_received and qpos_ready and self.publisher_created:
                logger.info("所有数据已就绪")
                return True

            elapsed = time.time() - start_time
            if elapsed % 2.0 < 0.1:
                logger.info(f"等待数据中... {elapsed:.1f}秒")

            time.sleep(0.01)

        logger.error(f"等待数据超时 ({timeout}秒)")
        return False


def main(args: Args = None) -> None:
    logger.info("初始化ROS2...")
    if not rclpy.ok():
        rclpy.init()

    if args is None:
        tyro_args = []
        for arg in sys.argv[1:]:
            if arg == '--ros-args':
                break
            tyro_args.append(arg)
        args = tyro.cli(Args, args=tyro_args)

    try:
        ros_node = PiperROSNode(args)
    except Exception as e:
        logger.error(f"创建ROS节点失败: {e}")
        sys.exit(1)

    if not ros_node.wait_for_data(args.wait_timeout):
        logger.error("无法获取数据，程序退出")
        try:
            ros_node.destroy_node()
        except:
            pass
        rclpy.shutdown()
        sys.exit(1)

    if not os.path.exists(args.norm_stats_path):
        logger.error(f"归一化统计文件不存在: {args.norm_stats_path}")
        sys.exit(1)

    with open(args.norm_stats_path, "r") as f:
        norm_stats = json.load(f)
    logger.info(f"加载归一化统计: {args.norm_stats_path}")

    server = Server(8888)
    server.start()
    print(f"开始运行 {args.num_steps} 步...")

    previous_filtered_action = None

    print("预热阶段：等待话题数据同步稳定...")
    for i in range(10):
        time.sleep(0.1)
        obs = ros_node.get_latest_observation()
        if obs is not None:
            print(f"  预热 {i+1}/10: 获取到观测数据 OK")
        else:
            print(f"  预热 {i+1}/10: 未获取到观测数据")
    print("预热完成，开始主循环")

    for step in range(args.num_steps):
        previous_filtered_action = None
        step_start = time.time()
        print(f"\n=== 步骤 {step + 1}/{args.num_steps} ===")

        data_collection_start = time.time()
        observation = ros_node.get_latest_observation()
        data_collection_time = time.time() - data_collection_start
        
        if observation is None:
            print(f"  未获取到观测数据 (耗时: {data_collection_time*1000:.1f}ms)")
            time.sleep(0.01)
            continue

        try:
            preprocess_start = time.time()
            
            processed_images = {}
            for cam_name, img in observation["images"].items():
                if img.shape[0] == 3:
                    img_hwc = np.transpose(img, (1, 2, 0))
                else:
                    img_hwc = img
                img_bgr = cv2.cvtColor(img_hwc, cv2.COLOR_RGB2BGR)
                processed_images[cam_name] = img_bgr
            
            preprocessed = preprocess(
                images=processed_images,
                state=observation["state"],
                prompt=observation["prompt"],
                adapt_to_pi=False,
                norm_stats=norm_stats["norm_stats"] if norm_stats else None
            )
            preprocess_time = time.time() - preprocess_start

            pre_images = {
                "1": preprocessed["image"]["base_0_rgb"],
                "2": preprocessed["image"]["left_wrist_0_rgb"],
                "3": preprocessed["image"]["right_wrist_0_rgb"]
            }

            obs = {
                "images": pre_images,
                "prompt": preprocessed["tokenized_prompt"].astype(np.int32),
                "state": preprocessed["state"].astype(np.float32),
            }

            dtypes = {
                "images": msg_pb2.Tensor.FLOAT32,
                "prompt": msg_pb2.Tensor.INT32,
                "state": msg_pb2.Tensor.FLOAT32,
            }

            if not hasattr(server, 'sock_fd') or server.sock_fd is None:
                print("服务器未连接")
                continue

            inference_start = time.time()
            server.send(obs, dtypes)

            result = server.receive()
            inference_time = time.time() - inference_start
            
            if result is None or "raw_outputs" not in result:
                print(f"未收到有效推理结果 (推理耗时: {inference_time*1000:.1f}ms)")
                continue

            action_result = result["raw_outputs"]
            if action_result.ndim == 3 and action_result.shape[0] == 1:
                action_result = action_result.squeeze(0)
            elif action_result.ndim > 2:
                action_result = action_result.reshape(-1, 32)

            postprocess_start = time.time()
            processed_actions = postprocess(
                actions=action_result,
                adapt_to_pi=False,
                save_result=False,
                norm_stats=norm_stats,
                state=observation["state"],
                skip_delta_restore=False
            )
            if not isinstance(processed_actions, np.ndarray):
                raise TypeError(f"动作类型错误: {type(processed_actions)}")
            postprocess_time = time.time() - postprocess_start

            execution_start = time.time()
            
            current_state = observation["state"][:7]
            previous_joints = current_state[:6].copy()
            previous_gripper = current_state[6]
            
            for i, action in enumerate(processed_actions):
                try:
                    action_joints = action[:6]
                    action_gripper = action[6]
                    is_first_action = (i == 0)
                    is_last_action = (i == len(processed_actions) - 1)
                    
                    if is_first_action:
                        # 第一个动作：从当前状态到 action[0] 插值10次
                        transition_steps = 10
                        for t in range(transition_steps):
                            ratio = (t + 1) / transition_steps
                            interpolated_joints = (1 - ratio) * current_state[:6] + ratio * action_joints
                            ros_node.publish_action(np.concatenate([interpolated_joints, [action_gripper]]))
                            time.sleep(0.01)
                    elif is_last_action:
                        # 最后一个动作：从 action[48] 到 action[49] 插值10次
                        transition_steps = 10
                        for t in range(transition_steps):
                            ratio = (t + 1) / transition_steps
                            final_joints = (1 - ratio) * previous_joints + ratio * action_joints
                            ros_node.publish_action(np.concatenate([final_joints, [action_gripper]]))
                            time.sleep(0.01)
                    else:
                        # 中间的动作：使用一阶低通滤波
                        alpha = 0.15
                        filtered_joints = alpha * action_joints + (1 - alpha) * previous_joints
                        ros_node.publish_action(np.concatenate([filtered_joints, [action_gripper]]))
                        time.sleep(0.01)
                    
                    previous_joints = action_joints.copy()
                    previous_gripper = action_gripper
                except Exception as e:
                    print(f"发布动作失败: {e}")
            
            execution_time = time.time() - execution_start
            step_time = time.time() - step_start
            print(f"  数据采集: {data_collection_time*1000:.1f}ms | 前处理: {preprocess_time*1000:.1f}ms | 推理: {inference_time*1000:.1f}ms | 后处理: {postprocess_time*1000:.1f}ms | 执行: {execution_time*1000:.1f}ms | 总耗时: {step_time*1000:.1f}ms")

        except Exception as e:
            print(f"循环步骤执行失败: {e}")
            time.sleep(0.01)
            continue

    print("运行完成，清理资源...")
    server.stop()
    ros_node.destroy_node()
    rclpy.shutdown()
    print("程序退出")


if __name__ == "__main__":
    main()
