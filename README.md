
English| [简体中文](./README_cn.md)

Getting Started with Openpi Runtime Package
=======


# Feature Introduction

The Openpi Runtime package is an VLA (Vision Language Action Model) example of quantized deployment based on [Pi0](https://github.com/Physical-Intelligence/openpi), which is the basic model in openpi. The input datas consist of three parts:

- "images"：{
    "0": [1, 3, 224, 224], dtype=uint8,
    "1": [1, 3, 224, 224], dtype=uint8,
    "2": [1, 3, 224, 224], dtype=uint8
}

- "prompt": dtype=string, for example "catch the bottle."

- "state": [1, 14], dtype=float64, the stare of the two robotics arms.

The image data comes from at least three subscribed image messages. Additionally, we provide the pi0 runtime server on D-robotics OE-LLM. By the infer of pi0 server, we can get the actions to control the robotics arms directly again and again util we finilly finish the task.

# Development Environment

- Programming Language: Python3
- Development Platform: RDK S600
- System Version: Ubuntu 24.04
- Compilation Toolchain: Linaro GCC 13.3.0

# Compilation

- S600 Version: Supports compilation on the S600 Ubuntu system and cross-compilation using Docker on a PC.

It also supports controlling the dependencies and functionality of the compiled pkg through compilation options.

## Dependency Libraries

- OpenCV: 3.4.5

ROS Packages:

- cv_bridge
- sensor_msgs

## Compilation Options

1. Compilation Environment Verification

- The S600 Ubuntu system is installed on the board.
- The current compilation terminal has set up the TogetherROS environment variable: `source PATH/setup.bash`. Where PATH is the installation path of TogetherROS.
- The ROS2 compilation tool colcon is installed. If the installed ROS does not include the compilation tool colcon, it needs to be installed manually. Installation command for colcon: `pip install -U colcon-common-extensions`.

2. Compilation

- Compilation command: `colcon build --packages-select openpi_runtime`

## Docker Cross-Compilation for S600 Version

1. Compilation Environment Verification

- Compilation within docker, and TogetherROS has been installed in the docker environment. For instructions on docker installation, cross-compilation, TogetherROS compilation, and deployment, please refer to the README.md in the robot development platform's robot_dev_config repo.
- The dnn node package has been compiled.
- The hbm_img_msgs package has been compiled (see Dependency section for compilation methods).

2. Compilation

- Compilation command:

  ```shell
  # RDK S600
  bash robot_dev_config/build.sh -p S600 -s openpi_runtime
  ```

## Notes


# Instructions

## Dependencies

- realsense2_camera package: Publishes image messages in realsense for the example D457.
- websocket package: Renders image messages.

## Parameters

| Parameter Name      | Explanation                            | Mandatory            | Default Value       | Remarks                                                                 |
| ------------------- | -------------------------------------- | -------------------- | ------------------- | ----------------------------------------------------------------------- |
| user_prompt           | Task name   | No                   | beat block hammer                   |                                                                         |
| max_limit_num               | the max time the robot can try                       | No                   | 50     |                                                                         |
| state_sub_topic_name   | Subscribe to robotic arm state | No  | /joint_states                   |                                                                         |
| camera_topic_name | Brain side camera topic name for subscribing image msg | No                   | /camera/camera/color/image_raw | |
| camera_left_topic_name | Left-arm Camera on topic name for subscribing image msg| No                   | /camera_left/camera_left/color/image_raw | |

## Running

## Running Pi0 on RDK S600 Ubuntu System

Running method 1, use the executable file to start:

```shell
export COLCON_CURRENT_PREFIX=./install
source /opt/ros/jazzy/setup.bash
source ./install/setup.bash

ros2 run openpi_runtime openpi_runtime_node --ros-args -p max_limit_num:=50 --log-level warn
```

Running method 2, use the python file to start:

```shell
export COLCON_CURRENT_PREFIX=./install
source /opt/ros/jazzy/setup.bash
source ./install/setup.bash

python3 install/lib/openpi_runtime/openpi_runtime_node
```

Running method 3, using a launch file:

```shell
export COLCON_CURRENT_PREFIX=./install
source /opt/ros/jazzy/setup.bash
source ./install/setup.bash

# Start the launch file, publish rgb8 images from D457.
ros2 launch openpi_runtime runtime.launch.py
```

## Running Others on RDK S600 Ubuntu System

- Start the pi0 server in oe_llm(comming soon).

```shell
bash run_pi0.sh
```

- Publish the robotic arm state in empty data.

```shell
ros2 topic pub /joint_states sensor_msgs/msg/JointState "{name: [], position: [-0.1300568,0.50693603,0.54202605,-0.2851898,-0.03326998,-0.12191405,0.56831681,-0.0183007,-0.04226554,-0.02073833,0.03956214,-0.00720679,-0.02525674,0.46239254], velocity: [], effort: []}" -r 1
```

- Pulish the D457 sensor using 'GMSL' mode.

```shell
source /opt/tros/jazzy/setup.bash
ros2 launch realsense2_camera rs_launch.py serial_no:='_234322306420' camera_namespace:=camera camera_name:=camera rgb_camera.color_profile:=1280x720x30
```

```shell
source /opt/tros/jazzy/setup.bash
ros2 launch realsense2_camera rs_launch.py serial_no:='_241122306184' camera_namespace:=camera_left camera_name:=camera_left rgb_camera.color_profile:=1280x720x30
```

# Results Analysis

## RDK S600 Pi0 Server Result

Command executed: `bash run_pi0.sh"`

```shell
[UCP]: log level = 3
[UCP]: UCP version = 3.12.3
[VP]: log level = 3
[DNN]: log level = 3
[HPL]: log level = 3
[UCPT]: log level = 6
Stage: 6
config_path: pi0_config.json
siglip_hbm_path: /root/zixi01.chen/pi0_hbm_1204/pi0_siglip_ptq.hbm
paligemma_hbm_path: /root/zixi01.chen/pi0_hbm_1204/pi0_gemma_llm_ptq.hbm
action_hbm_path: /root/zixi01.chen/pi0_hbm_1204/pi0_gemma_expert_ptq.hbm
token_config_path: ../../configs/Pi0_config/
norm_stats_path: ../../configs/Pi0_config/norm_stats.json
server_socket: 127.0.0.1:8888
[I][968167][12-13][20:58:15:690][xlm_impl.cc:39][pi0][XlmImpl] max_batch_num is: 1
siglip_backends: [2 3 4]
paligemma_backends: [2 3 4 5]
action_backends: [2 3 4 5]
Stage: 6
[BPU][[BPU_MONITOR]][281467506159296][INFO]BPULib verison(2, 2, 15)[f21ee84]!
[DNN]: 3.12.3_(4.5.4 HBRT)
[I][968167][12-13][20:58:21:345][model_manager.cc:213][pi0][mod_mgr] Load hbm file '/root/zixi01.chen/pi0_hbm_1204/pi0_siglip_ptq.hbm' success.
[I][968167][12-13][20:58:21:345][model_manager.cc:235][pi0][mod_mgr] model_count_ is: 1
[I][968167][12-13][20:58:21:345][model_manager.cc:258][pi0][mod_mgr] Load dnn model success. hbm_name is: pi0_siglip, model_name is: siglip
[I][968167][12-13][20:58:21:345][model_manager.cc:235][pi0][mod_mgr] model_count_ is: 1
[I][968167][12-13][20:58:21:345][model_manager.cc:235][pi0][mod_mgr] model_count_ is: 1
[I][968167][12-13][20:58:23:060][model_manager.cc:213][pi0][mod_mgr] Load hbm file '/root/zixi01.chen/pi0_hbm_1204/pi0_gemma_llm_ptq.hbm' success.
[I][968167][12-13][20:58:23:060][model_manager.cc:235][pi0][mod_mgr] model_count_ is: 1
[I][968167][12-13][20:58:23:060][model_manager.cc:258][pi0][mod_mgr] Load dnn model success. hbm_name is: pi0_paligemma, model_name is: gemma
[I][968167][12-13][20:58:23:279][model_manager.cc:213][pi0][mod_mgr] Load hbm file '/root/zixi01.chen/pi0_hbm_1204/pi0_gemma_expert_ptq.hbm' success.
[I][968167][12-13][20:58:23:279][model_manager.cc:235][pi0][mod_mgr] model_count_ is: 1
[I][968167][12-13][20:58:23:279][model_manager.cc:258][pi0][mod_mgr] Load dnn model success. hbm_name is: pi0_action, model_name is: gemma_expert
xlm init success
开始连接：127.0.0.1:8888
正在努力连接中，已请求 13 秒...连接成功：127.0.0.1:8888
接收成功，长度：451847字节
===== 解析 Header 信息 =====
序列号: 0
时间戳: 1765630719.155770063
重置仿真：0
===== 解析 Body 信息 =====
接收到 3 个图像张量：
类型=1，维度=1 3 224 224
类型=1，维度=1 3 224 224
类型=1，维度=1 3 224 224
接收到 1 个语言张量：
类型=2，维度=
接收到 1 个状态张量：
类型=0，维度=1 14
===== 开启 Pi0 推理 =====
Preprocess time: 1.447ms.
Siglip infer time: 21.463ms.
Paligemma infer time: 53.858ms.
Action infer time: 69.756ms.
Postprocess time: 0.083ms.
Pi0 Total time: 146.964ms.
===========================
发送成功，长度：5651字节
```

## RDK S600 Runtime(Client) Result:

Command executed: `ros2 run openpi_runtime openpi_runtime_node --ros-args -p max_limit_num:=50 --log-level warn"`

```bash
[WARN] [1765633774.309818394] [openpi_runtime_node]: Openpi Runtime Node has been started.
[WARN] [1765633774.311431634] [openpi_runtime_node]:
================ Node Parameters ================
user_prompt             : beat block hammer
max_limit_num           : 10
state_sub_topic_name    : /joint_states
camera_topic_name       : /camera/camera/color/image_raw
camera_left_topic_name  : /camera_left/camera_left/color/image_raw
camera_right_topic_name : /camera_right/camera_right/color/image_raw
================================================

服务器启动成功，等待客户端连接...（端口：8888）
客户端已连接：IP=127.0.0.1, 端口=50938

```

# data collection
##  data collection pipeline
首先启动piper_node和aliciaD_node，确认piper机械臂能够被aliciaD示教，然后使用rosbag的保存命令保存话题信息。

piper、aliciaD节点发布话题 -> rosbag -> .hdf5


## start camera node
### camera node
view doc

### piper_node

```bash
export ROS_DOMAIN_ID=40
export COLCON_CURRENT_PREFIX=./install
source /opt/ros/jazzy/setup.bash
source ./install/setup.bash

python3 install/lib/openpi_runtime/piper_node
```


### aliciaD_node
```bash
export COLCON_CURRENT_PREFIX=./install
source /opt/ros/jazzy/setup.bash
source ./install/setup.bash
export ROS_DOMAIN_ID=40

python3 install/lib/openpi_runtime/aliciaD_node

```

## record task
task:
```
"/aliciaD/action"
"/piper/qpos" 
"/camera/camera/color/image_raw"
"/camera_left/camera_left/color/image_raw"
```


Do not save to a network mounted folder, as disk read and write speeds cannot keep up with the save speed, resulting in serious packet loss

record：
```bash
#!/bin/bash

BAG_NAME="rosbag2_$(date +%Y%m%d_%H%M%S)"
SOURCE_DIR="/mnt/wang.liu/mnt/datasets"                 

ros2 bag record \
  --output "$BAG_NAME" \
  --storage mcap \
  --max-cache-size 1000000000 \
  --max-bag-size 104857600 \
  --compression-mode file \
  --compression-format zstd \
  --topics \
    /aliciaD/action \
    /piper/qpos \
    /camera/camera/color/image_raw \
    /camera_left/camera_left/color/image_raw

```

## replay
Close aliciaD_node and replay the/aliciaD/action topic. Piper will repeat recorded actions
```bash
ros2 bag play rosbag2_20260101_154003/ --topics /aliciaD/action
```


## convert
```bash
python bag_2_hdf5.py
python rename_hdf5.py
```

# RDKs600 infer hbm model,full pipline
```
source /opt/ros/jazzy/setup.bash
python run_pi0_s600.py
```