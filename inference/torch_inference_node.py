import dataclasses
import enum
import logging
import time
import sys
import threading
from typing import Dict, Optional, Union, List, Tuple, Any

import numpy as np
from common.utils import websocket_client_policy as _websocket_client_policy
from common.utils.ros_data_collector import ROSDataCollector
from common.utils.topic_time_synchronizer import TopicTimeSynchronizer
import tyro


# ==================== 导入ROS2相关 ====================
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
import cv2
import cv_bridge
# ====================================================

logger = logging.getLogger(__name__)


class EnvMode(enum.Enum):
    """支持的模式"""

    PIPER = "piper"  # 连接真实Piper机械臂


@dataclasses.dataclass
class Args:
    """命令行参数"""

    host: str = "120.48.157.2"
    port: int | None = 55536
    api_key: str | None = None
    num_steps: int = 50 * 50
    env: EnvMode = EnvMode.PIPER
    verbose: bool = False  # 是否打印详细推理结果

    # 图像话题配置 - 只配置实际存在的两个摄像头
    camera_topics: Dict[str, str] = dataclasses.field(
        default_factory=lambda: {
            "cam_high": "/camera/camera/color/image_raw",
            "cam_left_wrist": "/camera_left/camera_left/color/image_raw",
        }
    )

    # 服务器期望的所有摄像头（4个）
    expected_cameras: tuple = (
        "cam_high",
        "cam_low",
        "cam_left_wrist",
        "cam_right_wrist",
    )

    # 机械臂状态话题
    qpos_topic: str = "/piper/qpos"

    # 动作发布话题
    action_topic: str = "/aliciaD/action"

    action_chunk_size: int = 50  # 策略返回的动作序列长度
    wait_timeout: float = 10.0  # 等待数据的超时时间
    sync_time_window: float = 0.1  # 图像同步时间窗口（秒）


# ==================== ROS2节点 ====================
class PiperROSNode(Node):
    """ROS2节点，用于订阅图像和qpos，发布action"""

    def __init__(self, args: Args):
        super().__init__("piper_ros_node")

        # 存储参数
        self.args = args
        self.bridge = cv_bridge.CvBridge()

        # 初始化数据采集器（保持原有的用于基本的数据收集）
        self.collector = ROSDataCollector(self)

        # 初始化新的时间同步器
        sync_topics = list(args.camera_topics.keys()) + ["qpos"]
        self.topic_synchronizer = TopicTimeSynchronizer(
            topics=sync_topics,
            reference_topic=list(args.camera_topics.keys())[
                0
            ],  # 使用第一个摄像头作为基准
            sync_window_ms=args.sync_time_window * 1000,  # 转换为毫秒
            buffer_size=20,
            max_sync_queue_size=10,
            enable_statistics=True,
        )

        # ==================== 订阅器 ====================
        # 只订阅实际存在的两个摄像头话题
        self.camera_topics = args.camera_topics
        self.image_received = {cam: False for cam in self.camera_topics.keys()}

        for cam_name, topic in self.camera_topics.items():
            # 设置合适的QoS配置
            qos_profile = QoSProfile(
                reliability=QoSReliabilityPolicy.BEST_EFFORT,
                durability=QoSDurabilityPolicy.VOLATILE,
                depth=10,
            )

            # 使用 collector 添加话题（需要lambda传递cam_name）
            self.collector.add_topic(
                cam_name,
                topic,
                Image,
                lambda msg, name=cam_name: self._process_image(msg, name),
                qos_profile,
            )
            logger.info(f"已订阅摄像头话题 {cam_name}: {topic}")

        # 订阅机械臂状态话题
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=10,
        )

        self.collector.add_topic(
            "qpos", args.qpos_topic, Float32MultiArray, self._process_qpos, qos_profile
        )
        logger.info(f"已订阅机械臂状态话题: {args.qpos_topic}")

        # 设置同步组
        sync_topics = list(self.camera_topics.keys()) + ["qpos"]
        self.collector.set_sync_group(sync_topics, args.sync_time_window)

        # 启动数据采集线程
        self.collector.start()

        # ==================== 发布器 ====================
        # 发布动作到aliciaD
        try:
            self.action_publisher = self.create_publisher(
                Float32MultiArray, args.action_topic, 10
            )
            logger.info(f"已创建动作发布话题: {args.action_topic}")
            self.publisher_created = True
        except Exception as e:
            logger.error(f"创建动作发布话题失败: {e}")
            self.publisher_created = False

        # 消息计数器
        self.image_count = {cam: 0 for cam in self.camera_topics.keys()}
        self.qpos_count = 0

        # 启动时间
        self.start_time = time.time()

    def _process_image(self, msg: Image, topic_name: str):
        """图像处理回调函数"""
        try:
            # 转换图像
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="rgb8")

            # 调整大小
            cv_image = cv2.resize(cv_image, (224, 224))

            # 转换为CHW格式
            image_chw = cv_image.transpose(2, 0, 1).astype(np.uint8)

            # 添加到时间同步器
            timestamp_ns = (
                msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
            )
            self.topic_synchronizer.add_message(topic_name, image_chw, timestamp_ns)

            return image_chw

        except Exception as e:
            logger.error(f"处理图像回调失败: {e}")
            raise

    def _process_qpos(self, msg: Float32MultiArray):
        """机械臂状态处理函数"""
        try:
            if len(msg.data) >= 7:
                qpos_data = np.array(msg.data[:7], dtype=np.float32)

                # 添加到时间同步器（使用模拟时间戳，因为Float32MultiArray没有header）
                timestamp_ns = int(time.time() * 1_000_000_000)
                self.topic_synchronizer.add_message("qpos", qpos_data, timestamp_ns)

                return qpos_data
            else:
                logger.error(f"qpos数据维度不足: {len(msg.data)} < 7")
                raise ValueError(f"qpos数据维度不足: {len(msg.data)} < 7")

        except Exception as e:
            logger.error(f"处理qpos回调失败: {e}")
            raise

        """获取最新的时间同步观测数据"""
        # 使用新的时间同步器获取数据
        synced_data = self.topic_synchronizer.get_synced_data(max_age_ms=100)

        if synced_data is None:
            return None

        # 组装数据
        images = {}
        for cam_name in self.camera_topics.keys():
            if cam_name in synced_data.data:
                images[cam_name] = synced_data.data[cam_name]
                # 更新接收状态
                self.image_received[cam_name] = True
                self.image_count[cam_name] += 1
            else:
                return None

        qpos = synced_data.data.get("qpos")
        if qpos is None:
            return None

        self.qpos_count += 1

        # 为缺失的摄像头创建黑图
        for cam_key in self.args.expected_cameras:
            if cam_key not in images:
                # logger.debug(f"为缺失的摄像头 {cam_key} 创建黑图")
                images[cam_key] = np.zeros((3, 224, 224), dtype=np.uint8)

        full_state = np.concatenate([qpos, np.zeros(7)])

        return {
            "state": full_state.astype(np.float32),
            "images": images,
            "prompt": "put the box",
        }

    def publish_action(self, action: np.ndarray):
        """发布动作到aliciaD"""
        if not self.publisher_created:
            raise RuntimeError("动作发布话题未创建")

        try:
            if len(action) < 7:
                raise ValueError(f"动作维度不足: {len(action)} < 7")

            # 提取前7维
            action_7d = action[:7]

            # 创建消息
            msg = Float32MultiArray()
            msg.data = action_7d.tolist()

            # 发布
            self.action_publisher.publish(msg)

            if self.args.verbose:
                logger.info(f"发布动作: {action_7d}")

        except Exception as e:
            logger.error(f"发布动作失败: {e}")
            raise

    def spin_once(self):
        """处理一次ROS2回调"""
        rclpy.spin_once(self, timeout_sec=0.001)

    def wait_for_data(self, timeout: float) -> bool:
        """等待所有数据就绪"""
        start_time = time.time()

        while time.time() - start_time < timeout:
            # 尝试获取一次数据
            obs = self.get_latest_observation()
            if obs is not None:
                logger.info("所有数据都已就绪！")
                return True

            # 显示进度
            elapsed = time.time() - start_time
            if elapsed % 2.0 < 0.1:  # 每2秒打印一次
                logger.info(f"等待数据中... 已等待 {elapsed:.1f}秒")

            time.sleep(0.01)

        logger.error(f"等待数据超时 ({timeout}秒)")
        return False


def _get_piper_observation(ros_node: PiperROSNode) -> dict:
    """获取Piper观测数据"""
    # 获取机械臂状态
    try:
        left_arm_state = ros_node.get_qpos()
    except Exception as e:
        raise RuntimeError(f"获取机械臂状态失败: {e}")

    # 构建14维状态向量（左臂7维 + 右臂7维填充0）
    full_state = np.concatenate([left_arm_state, np.zeros(7)])

    # 打印状态数据
    logger.info(f"发送到服务器的状态数据:")
    logger.info(f"  关节角度: {left_arm_state[:6]}")
    logger.info(f"  夹爪位置: {left_arm_state[6]:.3f}")
    logger.info(f"  完整状态向量(14维): {full_state}")

    # 获取图像
    try:
        images = ros_node.get_synced_images()
    except Exception as e:
        raise RuntimeError(f"获取图像数据失败: {e}")

    # 检查是否包含所有需要的摄像头
    required_cameras = ros_node.args.expected_cameras
    missing_cameras = [cam for cam in required_cameras if cam not in images]

    if missing_cameras:
        raise RuntimeError(f"缺少以下摄像头的图像数据: {missing_cameras}")

    # 检查图像形状
    for cam_name, img in images.items():
        if img.shape != (3, 224, 224):
            raise RuntimeError(
                f"摄像头 {cam_name} 图像形状错误: {img.shape}，期望 (3, 224, 224)"
            )

    logger.info(f"成功获取所有 {len(images)} 个摄像头图像")

    return {
        "state": full_state.astype(np.float32),
        "images": images,
        "prompt": "put the box",  # TODO: 支持动态指令输入
    }


def main(args: Args) -> None:
    """主函数"""

    # 初始化ROS2
    logger.info("初始化ROS2...")
    rclpy.init()

    try:
        ros_node = PiperROSNode(args)
    except Exception as e:
        logger.error(f"创建ROS节点失败: {e}")
        sys.exit(1)

    # 等待初始数据
    logger.info(f"等待初始数据 (超时: {args.wait_timeout}秒)...")

    if not ros_node.wait_for_data(args.wait_timeout):
        logger.error("无法获取到所有必要的数据，程序退出")
        try:
            ros_node.destroy_node()
        except:
            pass
        rclpy.shutdown()
        sys.exit(1)

    # 初始化并启动数据采集线程
    # data_collector = ROSDataCollector(ros_node, lambda: _get_piper_observation(ros_node))
    # data_collector.start()

    logger.info("所有数据准备就绪，开始连接服务器...")

    # 创建策略客户端
    try:
        policy = _websocket_client_policy.WebsocketClientPolicy(
            host=args.host,
            port=args.port,
            api_key=args.api_key,
        )
        logger.info(f"服务器元数据: {policy.get_server_metadata()}")
    except Exception as e:
        logger.error(f"连接服务器失败: {e}")
        ros_node.collector.stop()
        ros_node.destroy_node()
        rclpy.shutdown()
        sys.exit(1)

    # 预热模型
    logger.info("预热模型...")
    for i in range(2):
        try:
            # 等待数据
            obs = None
            for _ in range(50):  # 尝试5秒
                obs = ros_node.get_latest_observation()
                if obs is not None:
                    break
                time.sleep(0.1)

            if obs is None:
                raise RuntimeError("无法获取观测数据")

            action = policy.infer(obs)
            if args.verbose:
                logger.info(f"预热步骤 {i + 1}: 动作形状 {action['actions'].shape}")
        except Exception as e:
            logger.error(f"预热失败: {e}", exc_info=True)
            ros_node.collector.stop()
            ros_node.destroy_node()
            rclpy.shutdown()
            sys.exit(1)

    # 主循环
    logger.info(f"开始运行 {args.num_steps} 步...")

    # 动作序列相关变量
    action_queue = []  # 存储从策略获取的动作序列
    current_action_step = 0  # 当前执行到动作序列的第几步

    try:
        for step in range(args.num_steps):
            step_start = time.time()

            # 如果动作队列为空，获取新的动作序列
            if len(action_queue) == 0:
                try:
                    # 获取观测
                    obs_start = time.time()
                    observation = ros_node.get_latest_observation()

                    if observation is None:
                        logger.warning("未获取到观测数据，跳过本次推理")
                        time.sleep(0.01)
                        continue

                    obs_time = time.time() - obs_start

                    # 打印状态数据
                    full_state = observation["state"]
                    images = observation["images"]
                    logger.info(f"发送到服务器的状态数据:")
                    logger.info(f"  关节角度: {full_state[:6]}")
                    logger.info(f"  夹爪位置: {full_state[6]:.3f}")
                    logger.info(f"  完整状态向量(14维): {full_state}")
                    logger.info(f"成功获取所有 {len(images)} 个摄像头图像")

                    # 策略推理
                    infer_start = time.time()
                    action_result = policy.infer(observation)
                    infer_time = time.time() - infer_start

                    # 提取动作序列（shape: [num_actions, action_dim]）
                    action_queue = action_result["actions"]

                    # 验证动作格式
                    if not isinstance(action_queue, np.ndarray):
                        raise TypeError(f"动作类型错误: {type(action_queue)}")

                    logger.info(
                        f"获取动作序列: 形状 {action_queue.shape}, 步数 {len(action_queue)}"
                    )

                    # 打印第一个和最后一个动作的详细信息
                    if len(action_queue) > 0:
                        first_action = action_queue[0]
                        last_action = action_queue[-1]
                        logger.info(f"  第一个动作(14维): {first_action}")
                        logger.info(f"    关节角度: {first_action[:6]}")
                        logger.info(f"    夹爪位置: {first_action[6]:.3f}")
                        logger.info(f"  最后一个动作(14维): {last_action}")
                        logger.info(f"    关节角度: {last_action[:6]}")
                        logger.info(f"    夹爪位置: {last_action[6]:.3f}")

                    current_action_step = 0  # 重置动作步数

                    if args.verbose:
                        logger.info(
                            f"观测耗时: {obs_time * 1000:.1f}ms, 推理耗时: {infer_time * 1000:.1f}ms"
                        )

                except Exception as e:
                    logger.error(f"获取新动作序列失败: {e}", exc_info=True)
                    # 继续运行，下次循环再尝试
                    time.sleep(0.1)
                    continue

            # 从动作队列中提取当前要执行的动作
            try:
                if current_action_step >= len(action_queue):
                    logger.warning(
                        f"动作步数 {current_action_step} 超出序列长度 {len(action_queue)}"
                    )
                    action_queue = []  # 清空队列，重新获取
                    current_action_step = 0
                    continue

                # 提取当前动作（shape: [14,]）
                current_action = action_queue[current_action_step]

                # 确保是当前时间步的动作
                if current_action.ndim != 1 or len(current_action) != 14:
                    raise ValueError(
                        f"动作维度错误: {current_action.shape}, 期望 (14,)"
                    )

                # 打印详细信息
                if args.verbose or step % 10 == 0:
                    logger.info(
                        f"\n步骤 {step + 1}:"
                        f"\n  动作序列步 {current_action_step + 1}/{len(action_queue)}"
                        f"\n  动作前7维: {current_action[:7]}"
                    )

                # 发布控制命令（只使用前7维）
                try:
                    ros_node.publish_action(current_action)
                except Exception as e:
                    logger.error(f"发布动作失败: {e}")
                    # 继续运行，下次再尝试

                # 移动到动作序列的下一步
                current_action_step += 1

                # 如果已经执行完所有动作，清空队列（下一步将获取新序列）
                if current_action_step >= len(action_queue):
                    logger.info(f"动作序列执行完成，共 {len(action_queue)} 步")
                    action_queue = []
                    current_action_step = 0

                # DataCollector 已经在后台运行 spin_once，这里不需要调用了

                # 添加小延时，避免过快循环
                time.sleep(0.01)  # 10ms

            except Exception as e:
                logger.error(f"执行动作失败: {e}", exc_info=True)
                # 清空队列，避免持续错误
                action_queue = []
                current_action_step = 0
                # 继续运行
                continue

    except KeyboardInterrupt:
        logger.info("用户中断程序")
    except Exception as e:
        logger.error(f"主循环异常: {e}", exc_info=True)
    finally:
        logger.info("清理资源...")
        try:
            ros_node.collector.stop()
        except:
            pass
        try:
            if hasattr(ros_node, "topic_synchronizer"):
                ros_node.topic_synchronizer.stop()
        except:
            pass
        if ros_node:
            ros_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    logger.info("运行完成")


if __name__ == "__main__":
    logging.basicConfig(
        level=logging.INFO, format="%(asctime)s - %(levelname)s - %(message)s"
    )
    main(tyro.cli(Args))
