# 数据集采集

## 数采硬件
两个相机、一个piper、一个alicia-D

## 运行相关节点

### 相机
```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=40
ros2 launch realsense2_camera rs_launch.py serial_no:='_234322302783' camera_namespace:=camera camera_name:=camera rgb_camera.color_profile:=640x480x50
```

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=40
ros2 launch realsense2_camera rs_launch.py serial_no:='_234322306420' camera_namespace:=camera_left camera_name:=camera_left rgb_camera.color_profile:=640x480x50
```


### fake_cameta
```bash
export COLCON_CURRENT_PREFIX=./install
source /opt/ros/jazzy/setup.bash
source ./install/setup.bash
export ROS_DOMAIN_ID=40
python3 install/lib/openpi_runtime/camera_fake_node --width 640 --height 480 --publish_rate 50
```




### piper
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




### alicia-D
```bash
export COLCON_CURRENT_PREFIX=./install
source /opt/ros/jazzy/setup.bash
source ./install/setup.bash
export ROS_DOMAIN_ID=40

python3 install/lib/openpi_runtime/aliciaD_node \
  --ros-args \
  -p publish_topic:=/aliciaD/action \
  -p publish_rate:=50.0 \
  -p port:=/dev/ttyUSB0 \
  -p baudrate:=1000000 \
  -p gripper_threshold:=70 \
  -p use_filter:=True \
  -p filter_alpha:=0.1

```

## 录制数据集
录制以下话题：
```
"/aliciaD/action"
"/piper/qpos" 
"/camera/camera/color/image_raw"
"/camera_left/camera_left/color/image_raw"
```


在 record_task.sh 里修改保存的路径，不要保存到网络挂载的文件夹，磁盘读写速度跟不上保存速度，会有严重的丢包.


```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=40
./record_task.sh
```


## 检查数据集
关闭 alicia-D 节点，然后使用 ros2 bag play 回放录制好的数据集
```bash
ros2 bag play rosbag2_20260101_154003/ --topics /aliciaD/action
```

## 整理数据集

### 转换数据集
```bash
python bag_2_hdf5.py -i /path/to/input -o /path/to/output
```


### 重命名数据集
```bash
python rename_hdf5.py /path/to/output --start 0
```

