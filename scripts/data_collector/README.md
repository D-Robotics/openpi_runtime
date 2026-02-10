# Dataset Collection

## Hardware Configuration

- 2x D457 RGB cameras (head camera + left wrist camera)
- 1x Piper robotic arm (execution arm)
- 1x Alicia-D teleoperation arm (input device)

## Running Required Nodes

### Camera Nodes

**Head Camera**

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=40
ros2 launch realsense2_camera rs_launch.py \
  serial_no:='_234322302783' \
  camera_namespace:=camera \
  camera_name:=camera \
  rgb_camera.color_profile:=640x480x50
```

**Left Wrist Camera**

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=40
ros2 launch realsense2_camera rs_launch.py \
  serial_no:='_234322306420' \
  camera_namespace:=camera_left \
  camera_name:=camera_left \
  rgb_camera.color_profile:=640x480x50
```

### Virtual Camera Node

This node publishes dummy images when real cameras are not available (for testing purposes):

```bash
export COLCON_CURRENT_PREFIX=./install
source /opt/ros/jazzy/setup.bash
source ./install/setup.bash
export ROS_DOMAIN_ID=40
python3 install/lib/openpi_runtime/camera_fake_node \
  --width 640 \
  --height 480 \
  --publish_rate 50
```

### Piper Node

Controls the execution robotic arm. The node subscribes to action commands from the inference node and publishes joint states:

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

### Alicia-D Node

The teleoperation input device node. It reads joint positions from the Alicia-D arm and publishes them as action commands:

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

## Recording Dataset

### Topics to Record

```
"/aliciaD/action"
"/piper/qpos"
"/camera/camera/color/image_raw"
"/camera_left/camera_left/color/image_raw"
```

### Important Notes

Modify the save path in `record_task.sh` before starting the recording. **Do not** save to network-mounted directories. Network disk I/O speed cannot keep up with the real-time recording requirements, which will cause severe data loss.

### Start Recording

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=40
./record_task.sh
```

## Dataset Validation

After stopping the Alicia-D node, use `ros2 bag play` to replay and validate the recorded dataset:

```bash
ros2 bag play rosbag2_20260101_154003/ --topics /aliciaD/action
```

## Dataset Post-processing

### Format Conversion

Convert ROS bag format to HDF5 for training:

```bash
python bag_2_hdf5.py -i rosbag/ -o hdf5/
```

### File Renaming

Rename the converted HDF5 files with sequential indices:

```bash
python rename_hdf5.py ./hdf5 --start 0
```
