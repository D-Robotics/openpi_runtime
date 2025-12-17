# 运行方式1, 使用可执行文件启动
# export COLCON_CURRENT_PREFIX=./install
# source /opt/ros/jazzy/setup.bash
# source ./install/setup.bash

# ros2 run openpi_runtime openpi_runtime_node --ros-args -p max_limit_num:=50 --log-level warn


# 运行模式2, 使用python脚本启动
export COLCON_CURRENT_PREFIX=./install
source /opt/ros/jazzy/setup.bash
source ./install/setup.bash
export ROS_DOMAIN_ID=50
python3 install/lib/openpi_runtime/openpi_runtime_node --ros-args -p max_limit_num:=50 --log-level warn
# python3 install/lib/openpi_runtime/openpi_runtime_node


# 运行方式3, 使用launch文件启动
# export COLCON_CURRENT_PREFIX=./install
# source /opt/ros/jazzy/setup.bash
# source ./install/setup.bash

# # 启动launch文件, 使用D457 sensor发布rgb8格式图片
# ros2 launch openpi_runtime runtime.launch.py



# source /opt/ros/jazzy/setup.bash
# source install/setup.bash
# export ROS_DOMAIN_ID=50
# ros2 run openpi_runtime openpi_runtime_node --ros-args -p max_limit_num:=50 --log-level warn