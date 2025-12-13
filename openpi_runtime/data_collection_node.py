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

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

from openpi_runtime.time_sync_manager import TimeSyncManager

class DataCollectionNode(Node):
    def __init__(self):
        super().__init__('data_collection_node')
        self.get_logger().warn('Data Collection Node has been started.')

        # 1. image tensor setting
        self.bridge = cv_bridge.CvBridge()

        # Declare parameters
        self.declare_parameter('render_type', False)
        self.declare_parameter('camera_topic_name', '/camera/camera/color/image_raw')
        self.declare_parameter('camera_left_topic_name', '/camera_left/camera_left/color/image_raw')
        self.declare_parameter('camera_right_topic_name', '/camera_right/camera_right/color/image_raw')

        # Get parameter values
        self.render_type = self.get_parameter('render_type').get_parameter_value().bool_value
        self.camera_topic_name = self.get_parameter('camera_topic_name').get_parameter_value().string_value
        self.camera_left_topic_name = self.get_parameter('camera_left_topic_name').get_parameter_value().string_value
        self.camera_right_topic_name = self.get_parameter('camera_right_topic_name').get_parameter_value().string_value

        self.image_topic_names = [
            self.camera_topic_name,
            self.camera_left_topic_name,
            self.camera_right_topic_name
        ]
        self.sync_mgr = TimeSyncManager(
            topics=self.image_topic_names,
            max_queue_size=30,
            slop_ns=20_000_000   # 20 ms
        )

        for image_topic_name in self.image_topic_names:
            self.create_subscription(
                Image,
                image_topic_name,
                lambda msg, t=image_topic_name: self.sync_mgr.add_msg(t, msg),
                qos_profile_sensor_data
            )
            self.get_logger().info(f"Subscribed to {image_topic_name}")

        # 2. main program in timer
        self.timer = self.create_timer(0.01, self.process_synced)

    def process_synced(self):

        group = self.sync_mgr.get_synced()

        if group is None:
            return

        # 1. pre process images ======
        self.get_logger().info(
            f"[SYNC] ts={group.stamp_ns} "
            f"topics={list(group.frames.keys())}"
        )
        imgs = []
        for idx, (topic, img_msg) in enumerate(group.frames.items()):
            self.get_logger().info(
                f"  {topic}: encoding={img_msg.encoding}, width={img_msg.width}, height={img_msg.height}, timestamp={img_msg.header.stamp.sec}_{img_msg.header.stamp.nanosec}"
            )

            img = self.bridge.imgmsg_to_cv2(img_msg)
            imgs.append(img)
            ret = self.process_images(imgs)
            if self.render_type:
                stamp = img_msg.header.stamp
                filename = f"{str(idx)}_{stamp.sec}_{stamp.nanosec}.png"
                cv2.imwrite(filename, img)

    def process_images(self, imgs):
        # TODO 

        return 0

# ===================== main =====================

def main(args=None):
    rclpy.init(args=args)
    node = DataCollectionNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
