<?xml version="1.0"?>
<launch>
    <executable cmd="export COLCON_CURRENT_PREFIX=./install &amp;&amp; source /opt/ros/jazzy/setup.bash &amp;&amp; source ./install/setup.bash &amp;&amp; export ROS_DOMAIN_ID=40 &amp;&amp; python3 install/lib/openpi_runtime/piper_node --subscribe_topic /aliciaD/action --publish_topic /piper/qpos --publish_rate 50.0 --gripper_open_value 74000 --gripper_close_value 50 --gripper_threshold 0.5 --gripper_state_threshold 64000 --gripper_torque 3500 --can_name can0" launch-prefix="" name="piper_node" pkg="openpi_runtime" respawn="false" />

    <executable cmd="export COLCON_CURRENT_PREFIX=./install &amp;&amp; source /opt/ros/jazzy/setup.bash &amp;&amp; source ./install/setup.bash &amp;&amp; export ROS_DOMAIN_ID=40 &amp;&amp; python3 install/lib/openpi_runtime/s600_inference_node --ros-args -p norm_stats_path:=/mnt/wang.liu/mount/tros_ws/src/openpi_runtime/openpi_runtime/norm_stats.json -p action_topic:=/aliciaD/action -p qpos_topic:=/piper/qpos -p num_steps:=1250" launch-prefix="" name="s600_inference_node" pkg="openpi_runtime" respawn="false" />

</launch>
