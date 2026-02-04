[English](./README.md) | 简体中文

# OpenPI Runtime S600 推理节点

## 概述

OpenPI Runtime 是基于 [Pi0](https://github.com/Physical-Intelligence/openpi) 量化部署的视觉语言动作模型（VLA）推理运行时。本项目实现了 S600 机械臂的端到端控制流程，通过接收相机图像和机械臂状态，结合用户指令生成动作序列并控制机械臂执行。

该系统采用客户端-服务端架构：S600 推理节点作为客户端负责数据采集、预处理和动作执行；Pi0 推理模型作为服务端负责动作预测。

## 系统架构

```
┌─────────────────────────────────────────────────────────────────┐
│                      S600 推理节点系统架构                        │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│  ┌──────────────┐    ┌──────────────┐    ┌──────────────┐      │
│  │  头部相机     │    │  左腕相机     │    │  右腕相机     │      │
│  │  /camera/    │    │ /camera_left/│    │   (黑图)     │      │
│  │  camera/     │    │ camera_left/ │    │              │      │
│  │  color/      │    │ color/       │    │              │      │
│  │  image_raw   │    │ image_raw    │    │              │      │
│  └──────┬───────┘    └──────┬───────┘    └──────┬───────┘      │
│         │                   │                   │               │
│         └───────────────────┼───────────────────┘               │
│                             ▼                                   │
│              ┌──────────────────────────────┐                  │
│              │     话题同步管理器             │                  │
│              │  TopicTimeSynchronizer       │                  │
│              │   时间窗口: 100ms             │                  │
│              └──────────────┬───────────────┘                  │
│                             ▼                                   │
│         ┌───────────────────────────────┐                       │
│         │        S600 推理节点            │                       │
│         │   s600_inference_node.py      │                       │
│         ├───────────────────────────────┤                       │
│         │  1. 数据采集 (0.1ms)           │                       │
│         │  2. 前处理 (3.2ms)            │                       │
│         │  3. 推理 (192.5ms)            │                       │
│         │  4. 后处理 (0.1ms)            │                       │
│         │  5. 动作插值与滤波执行           │                       │
│         └───────────────┬───────────────┘                       │
│                         ▼                                       │
│              ┌─────────────────────┐                            │
│              │   Pi0 推理服务器      │                            │
│              │   (OE-LLM 工具包)    │                            │
│              └──────────┬──────────┘                            │
│                         ▼                                       │
│              ┌─────────────────────┐                            │
│              │    piper_node        │                            │
│              │  机械臂控制节点       │                            │
│              └─────────────────────┘                            │
│                                                                 │
└─────────────────────────────────────────────────────────────────┘
```

## 核心特性

- **多相机同步采集**：支持头部相机、左腕相机图像同步采集，右腕相机使用黑图占位
- **话题时间同步**：基于 ROS2 话题时间戳同步，确保数据一致性
- **端到端低延迟**：数据采集→前处理→推理→后处理→执行，全链路耗时监控
- **动作平滑处理**：首个动作从当前状态插值过渡，最后一个动作平滑收敛，中间动作一阶低通滤波
- **关节与夹爪分离控制**：6个关节使用滤波插值，夹爪状态直接发布

## 技术规格

### 输入数据格式

| 数据类型 | 形状 | 数据类型 | 说明 |
|---------|------|---------|------|
| 图像数据 (3路) | [3, 224, 224] | uint8 | 头部相机、左腕相机、右腕相机（黑图） |
| 状态数据 | [14] | float32 | 7个关节位置 + 7个目标关节位置 |
| 提示词 | - | string | 用户指令，如 "put the yellow mango on the blue plate" |

### 输出动作格式

| 数据类型 | 形状 | 数据类型 | 说明 |
|---------|------|---------|------|
| 动作序列 | [50, 14] | float32 | 50个时间步的动作，每步包含左臂的6个关节 + 夹爪，和右臂的6个关节 + 夹爪 |

### 性能指标

| 阶段 | 平均耗时 | 说明 |
|-----|---------|------|
| 数据采集 | 0.1ms | ROS2 话题同步与数据获取 |
| 前处理 | 3.2ms | 图像格式转换、归一化、tokenization |
| 推理 | 192.5ms | Pi0 模型推理（服务端） |
| 后处理 | 0.1ms | 动作解码、Delta 还原 |
| 动作执行 | 700.5ms | 50个动作 + 插值，总计约70个时间步 |
| 单步循环 | 896.4ms | 包含所有阶段的完整流程 |

## 开发环境

| 项目 | 版本/规格 |
|-----|----------|
| 编程语言 | Python 3.12 |
| 操作系统 | Ubuntu 24.04 |
| ROS2 版本 | Jazzy |
| 机械臂平台 | S600 |
| 编译工具 | colcon |
| 推理框架 | Pi0 (OE-LLM) |

## 依赖项

### Python 依赖

```
numpy>=1.24.0
opencv-python>=4.8.0
tyro>=0.7.0
cv-bridge
```

### ROS2 包依赖

- realsense2_camera package: Publishes image messages in realsense for the example D457.
- websocket package: Renders image messages.

## 模型获取

HBM 量化模型可从 Hugging Face 获取：https://huggingface.co/D-Robotics/openpi

| 版本 | 文件夹 | 模型 | 提示词 |
|-----|-------|------|--------|
| v0.1.0 | put_the_box | pi0_base, HBM | put the box |
| v0.2.0 | pi0_put_the_yellow_mango_on_the_blue_plate | pi0_base, HBM | put the yellow mango on the blue plate |

## 参数说明

| 参数名 | 类型 | 默认值 | 说明 |
|-------|------|-------|------|
| `num_steps` | int | 1250 | 最大控制步数 |
| `norm_stats_path` | string | "" | 归一化统计文件路径 |
| `action_topic` | string | "/aliciaD/action" | 动作发布话题 |
| `qpos_topic` | string | "/piper/qpos" | 状态订阅话题 |
| `wait_timeout` | float | 10.0 | 数据等待超时时间（秒） |
| `sync_time_window` | float | 0.1 | 话题同步时间窗口（秒） |

| Parameter Name | Explanation | Mandatory | Default Value | Remarks |
|---------------|-------------|-----------|---------------|---------|
| user_prompt | Task name | No | beat block hammer | |
| max_limit_num | the max time the robot can try | No | 50 | |
| state_sub_topic_name | Subscribe to robotic arm state | No | /joint_states | |
| camera_topic_name | Brain side camera topic name for subscribing image msg | No | /camera/camera/color/image_raw | |
| camera_left_topic_name | Left-arm Camera on topic name for subscribing image msg | No | /camera_left/camera_left/color/image_raw | |

## 编译

### S600 Ubuntu系统上编译

1. 编译环境确认

   - 板端已安装 S600 Ubuntu 系统
   - 当前编译终端已设置 TogetherROS 环境变量：`source PATH/setup.bash`（PATH 为 TogetherROS 的安装路径）
   - 已安装 ROS2 编译工具 colcon（若未安装，执行 `pip install -U colcon-common-extensions`）
   - 已编译 dnn node package

2. 编译

```bash
colcon build --packages-select openpi_runtime
```

### docker交叉编译 S600版本

1. 编译环境确认

   在 docker 中编译，且 docker 中已安装好 TogetherROS。docker 安装、交叉编译说明、TogetherROS 编译和部署说明详见机器人开发平台 robot_dev_config repo 中的 README.md。

2. 编译

   默认打开 shared mem 通信方式。

```bash
bash robot_dev_config/build.sh -p S600 -s openpi_runtime
```

## 运行说明

### 1. 启动相机节点

**头部相机（D457）**

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=40
ros2 launch realsense2_camera rs_launch.py \
  serial_no:='_234322302783' \
  camera_namespace:=camera \
  camera_name:=camera \
  rgb_camera.color_profile:=640x480x50
```

**左腕相机（D457）**

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=40
ros2 launch realsense2_camera rs_launch.py \
  serial_no:='_234322306420' \
  camera_namespace:=camera_left \
  camera_name:=camera_left \
  rgb_camera.color_profile:=640x480x50
```

### 2. 启动 piper 节点

```bash
export COLCON_CURRENT_PREFIX=./install
source /opt/ros/jazzy/setup.bash
source ./install/setup.bash
export ROS_DOMAIN_ID=40

python3 install/lib/openpi_runtime/piper_node \
  --subscribe_topic /aliciaD/action \
  --publish_topic /piper/qpos \
  --publish_rate 50.0 \
  --gripper_open_value 74000 \
  --gripper_close_value 50 \
  --gripper_threshold 0.5 \
  --gripper_state_threshold 64000 \
  --gripper_torque 3500 \
  --can_name can0
```

### 3. 启动 S600 推理节点

**命令行启动**
norm_stats_path要与你的模型文件匹配，注意修改路径。
```bash
export COLCON_CURRENT_PREFIX=./install
source /opt/ros/jazzy/setup.bash
source ./install/setup.bash
export ROS_DOMAIN_ID=40

python3 install/lib/openpi_runtime/s600_inference_node \
  --ros-args \
  -p norm_stats_path:=norm_stats.json \
  -p action_topic:=/aliciaD/action \
  -p qpos_topic:=/piper/qpos \
  -p num_steps:=1250
```

## 动作执行算法

### 动作平滑策略

本节点采用三段式动作平滑策略，确保动作序列既流畅又能准确到达目标状态：

```
当前状态 ──→ 插值10次 ──→ action[0] ──→ 一阶滤波 ──→ action[1] ──→ ... ──→ action[48] ──→ 插值10次 ──→ action[49]
     │                            │              │                                     │
     ▼                            ▼              ▼                                     ▼
  初始位置                   首个动作        中间48个动作                         最后一个动作
                          (平滑过渡)      (alpha=0.15)                         (平滑收敛)
```

### 算法细节

1. **第一段插值**（首个动作）

   从当前状态到 action[0] 线性插值 10 次，避免从静止状态直接跳变到目标动作。

2. **中间段滤波**（action[1] 到 action[48]）

   使用一阶低通滤波器，滤波器系数：alpha = 0.15

   ```
   filtered = alpha × action + (1 - alpha) × previous_filtered
   ```

3. **第二段插值**（最后一个动作）

   从 action[48] 到 action[49] 线性插值 10 次，确保动作平滑收敛到最终目标。

4. **夹爪处理**

   - 第 7 维（夹爪）不参与滤波插值
   - 直接使用推理输出的夹爪状态（0 或 1）

### 执行流程

| 阶段 | 动作数 | 说明 |
|-----|-------|------|
| 第一段插值 | 10 | 当前状态 → action[0] |
| 中间段滤波 | 48 | action[1] → action[48] |
| 第二段插值 | 10 | action[48] → action[49] |
| **总计** | **68** | 约 1.36 秒（每步 20ms） |

## 运行结果

以下视频演示了 S600 机械臂执行 Pi0 模型的过程：

[![S600 Pi0 Demo](./resource/s600_pi0_cover.jpg)](./resource/s600_pi0.mp4)

**视频说明**：演示了 S600 机械臂接收用户指令 "put the yellow mango on the blue plate" 后，通过 Pi0 模型推理生成动作序列，并平滑执行的全过程。可以看到机械臂从初始位置平稳抓取黄色芒果并放置到蓝色餐盘上。

**查看视频**：点击上方图片或 [下载视频](./resource/s600_pi0.mp4) 查看完整演示。

## 文件结构

```
openpi_runtime/
├── inference/
│   └── s600_inference_node.py    # S600 推理主节点
├── robot/
│   └── piper_node.py              # Piper 机械臂控制节点
├── common/
│   ├── utils/
│   │   ├── ros_data_collector.py
│   │   └── topic_time_synchronizer.py
│   ├── pi0_process/
│   │   ├── preprocess.py          # 推理前处理
│   │   └── postprocess.py         # 推理后处理
│   ├── ServerUtils/
│   │   └── server.py              # TCP 通信服务端
│   └── msg/
│       └── msg.proto              # 消息协议定义
├── launch/
│   ├── run_pi0_s600.launch        # 启动文件
│   └── readme.md                  # 启动说明
├── resource/
│   └── s600_pi0.mp4               # 运行演示视频
├── scripts/
│   └── data_collector/            # 数据采集工具与说明文档
└── README_cn.md                   # 本文档
```
> **数据采集**：如需采集训练数据集，请参考 [数据采集文档](./scripts/data_collector/README.md)。

## 故障排除

### 常见问题

**1. 无法获取观测数据**

- 检查相机节点是否正常运行
- 确认 ROS_DOMAIN_ID 设置一致
- 验证话题名称是否正确

**2. 推理连接失败**

- 确认 Pi0 推理服务已启动
- 检查端口 8888 是否被占用
- 验证 norm_stats.json 文件路径正确

**3. 动作执行不连续**

- 检查网络延迟
- 确认 CAN 通信正常
- 调整 `sync_time_window` 参数

## 参考资料

- [openpi](https://github.com/Physical-Intelligence/openpi)
- [ROS2 官方文档](https://docs.ros.org/)
- [Realsense SDK](https://github.com/IntelRealSense/realsense-ros)
- [aliciaD](https://docs.sparklingrobo.com/docs/alicia-d-series/leader/doc_00_intro)
- [piper](https://github.com/agilexrobotics/piper_sdk)


