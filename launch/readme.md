# 按顺序执行一下节点
## 相机
相机部分需要启动多个节点
``` shell
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=40
ros2 launch realsense2_camera rs_launch.py serial_no:='_234322302783' camera_namespace:=camera camera_name:=camera rgb_camera.color_profile:=640x480x50
```

``` shell
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=40
ros2 launch realsense2_camera rs_launch.py serial_no:='_234322306420' camera_namespace:=camera_left camera_name:=camera_left rgb_camera.color_profile:=640x480x50
```

``` shell
source /opt/tros/jazzy/setup.bash
export ROS_DOMAIN_ID=40
ros2 launch hobot_codec hobot_codec.launch.py codec_in_mode:=ros codec_in_format:=rgb8 codec_out_mode:=ros codec_out_format:=jpeg codec_sub_topic:=/camera/camera/color/image_raw codec_pub_topic:=/image_jpeg
```

``` shell
source /opt/tros/jazzy/setup.bash
export ROS_DOMAIN_ID=40
ros2 launch websocket websocket.launch.py websocket_image_topic:=/image_jpeg websocket_only_show_image:=true websocket_channel:=0
```
``` shell
source /opt/tros/jazzy/setup.bash
export ROS_DOMAIN_ID=40
ros2 launch hobot_codec hobot_codec.launch.py codec_in_mode:=ros codec_in_format:=rgb8 codec_out_mode:=ros codec_out_format:=jpeg codec_sub_topic:=/camera_left/camera_left/color/image_raw codec_pub_topic:=/image_left_jpeg
```

``` shell
source /opt/tros/jazzy/setup.bash
export ROS_DOMAIN_ID=40
ros2 launch websocket websocket.launch.py websocket_image_topic:=/image_left_jpeg websocket_only_show_image:=true websocket_channel:=1
```


## piper
``` shell
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

## s600
单独启动节点的示例命令：
``` shell
export COLCON_CURRENT_PREFIX=./install
source /opt/ros/jazzy/setup.bash
source ./install/setup.bash
export ROS_DOMAIN_ID=40

python3 install/lib/openpi_runtime/s600_inference_node \
    --ros-args \
    -p norm_stats_path:=/mnt/wang.liu/mount/tros_ws/src/openpi_runtime/openpi_runtime/norm_stats.json \
    -p action_topic:=/aliciaD/action \
    -p qpos_topic:=/piper/qpos \
    -p num_steps:=1250
```
