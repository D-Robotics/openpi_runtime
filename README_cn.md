[English](./README.md) | 简体中文

Getting Started with Openpi Runtime
=======


# 功能介绍

Openpi package 是基于 [Pi0](https://github.com/Physical-Intelligence/openpi) 量化部署的 VLA(视觉语言动作大模型) 使用示例。算法输入依赖几个部分:

- "images"：{
    "0": [1, 3, 224, 224], dtype=uint8,
    "1": [1, 3, 224, 224], dtype=uint8,
    "2": [1, 3, 224, 224], dtype=uint8
}

- "prompt": dtype=string, for example "catch the bottle."

- "state": [1, 14], dtype=float64, the stare of the two robotics arms.

图像数据来源于订阅到的image msg, 示例中采用3个D457发布图片。此外, 本功能包需要配套地瓜机器人大模型工具包(OE-LLM) 使用, 具体是为了启动 Pi0 推理模型, 作为一个Server节点使用。推理完成后, 最终会获得机械臂控制的关节信息, 并且可以直接作用在机械臂控制上, 直到该任务完成或者达到用户设置的最大控制次数。本示例展示了一个控制流程示例, 实际应用中, 需要配合具体机械臂接入使用。

# 开发环境

- 编程语言: Python3
- 开发平台: S600
- 系统版本：Ubuntu 24.04
- 编译工具链: Linux GCC 13.3.0

# 编译

- S600版本：支持在S600 Ubuntu系统上编译和在PC上使用docker交叉编译两种方式。

同时支持通过编译选项控制编译pkg的依赖和pkg的功能。

## 依赖库

- opencv:3.4.5

ros package：

- cv_bridge
- sensor_msgs

## 编译选项

## S600 Ubuntu系统上编译

1、编译环境确认

- 板端已安装S600 Ubuntu系统。
- 当前编译终端已设置TogetherROS环境变量：`source PATH/setup.bash`。其中PATH为TogetherROS的安装路径。
- 已安装ROS2编译工具colcon。安装的ROS不包含编译工具colcon, 需要手动安装colcon。colcon安装命令：`pip install -U colcon-common-extensions`
- 已编译dnn node package

2、编译

- 编译命令：`colcon build --packages-select openpi_runtime`

## docker交叉编译 S600版本

1、编译环境确认

- 在docker中编译, 并且docker中已经安装好TogetherROS。docker安装、交叉编译说明、TogetherROS编译和部署说明详见机器人开发平台robot_dev_config repo中的README.md。

2、编译

- 编译命令：

  ```shell
  # RDK S600
  bash robot_dev_config/build.sh -p S600 -s openpi_runtime
  ```

- 编译选项中默认打开了shared mem通信方式。

## 注意事项


# 使用介绍

## 依赖

- mipi_cam package：发布图片msg
- usb_cam package：发布图片msg
- websocket package：渲染图片和ai感知msg

## 参数

| 参数名             | 解释                                  | 是否必须             | 默认值              | 备注                                                                    |
| ------------------ | ------------------------------------- | -------------------- | ------------------- | ----------------------------------------------------------------------- |
| user_prompt           | 用户定义的任命名   | No                   | beat block hammer                   |                                                                         |
| max_limit_num               | VLA模型最大尝试次数 try                       | No                   | 50     |                                                                         |
| state_sub_topic_name   | 订阅机械臂状态的话题 | No  | /joint_states                   |                                                                         |
| camera_topic_name | 头部相机发布的话题名 | No                   | /camera/camera/color/image_raw | |
| camera_left_topic_name | 左臂相机发布的话题名 | No                   | /camera_left/camera_left/color/image_raw | |


## 运行

- 编译成功后, 将生成的install路径拷贝到地平线RDK上（如果是在RDK上编译, 忽略拷贝步骤）, 并执行如下命令运行。

## 在RDK S600 Ubuntu系统上运行Pi0相关

运行方式1, 使用可执行文件启动：
```shell
export COLCON_CURRENT_PREFIX=./install
source /opt/ros/jazzy/setup.bash
source ./install/setup.bash

ros2 run openpi_runtime openpi_runtime_node --ros-args -p max_limit_num:=50 --log-level warn
```

运行模式2, 使用python脚本启动

```shell
export COLCON_CURRENT_PREFIX=./install
source /opt/ros/jazzy/setup.bash
source ./install/setup.bash

python3 install/lib/openpi_runtime/openpi_runtime_node
```

运行方式3, 使用launch文件启动：
```shell
export COLCON_CURRENT_PREFIX=./install
source /opt/ros/jazzy/setup.bash
source ./install/setup.bash

# 启动launch文件, 使用D457 sensor发布rgb8格式图片
ros2 launch openpi_runtime runtime.launch.py
```

## 在RDK S600 Ubuntu系统上运行其他程序

- 启动 Pi0 推理服务节点, 通过 OE-LLM 获取。

```shell
bash run_pi0.sh
```

- 发布一个空的机械臂状态信息

```shell
ros2 topic pub /joint_states sensor_msgs/msg/JointState "{name: [], position: [-0.1300568,0.50693603,0.54202605,-0.2851898,-0.03326998,-0.12191405,0.56831681,-0.0183007,-0.04226554,-0.02073833,0.03956214,-0.00720679,-0.02525674,0.46239254], velocity: [], effort: []}" -r 1
```

- 发布 D457 话题消息

```shell
source /opt/tros/jazzy/setup.bash
ros2 launch realsense2_camera rs_launch.py serial_no:='_234322306420' camera_namespace:=camera camera_name:=camera rgb_camera.color_profile:=1280x720x30
```

```shell
source /opt/tros/jazzy/setup.bash
ros2 launch realsense2_camera rs_launch.py serial_no:='_241122306184' camera_namespace:=camera_left camera_name:=camera_left rgb_camera.color_profile:=1280x720x30
```

# 结果分析

## S600 Pi0 server端 结果展示

运行命令：`bash run_pi0.sh"`

```bash
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

## S600 Pi0 client端 结果展示

运行命令: `ros2 run openpi_runtime openpi_runtime_node --ros-args -p max_limit_num:=50 --log-level warn"`

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


# 数采
首先启动piper_node和aliciaD_node，确认piper机械臂能够被aliciaD示教，然后使用rosbag的保存命令保存话题信息。

数采链路为：
相机、piper、aliciaD节点发布话题 -> rosbag -> .hdf5

详细的数采操作：
[数采](./openpi_runtime/data_collection/data_collection.md)


# x86推理，RDKs600执行
在x86端启动推理服务，在RDK端运行以下命令：
```bash
python run_piper_x86.py --host 120.48.157.2 --port 55536
```