#!/bin/bash

# 使用rosbag记录话题

# 配置路径
BAG_NAME="rosbag/rosbag2_$(date +%Y%m%d_%H%M%S)"

# 执行录制（移除max-bag-size参数）
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
    /camera_left/camera_left/color/image_raw \
    /camera_fake/right_wrist