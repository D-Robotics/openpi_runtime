#!/usr/bin/env python3
"""
ROS话题发布器：实时发布Piper机械臂的qpos和qvel数据
发布频率可配置（默认50Hz）
"""
import rospy
from std_msgs.msg import Float32MultiArray
from piper_sdk import C_PiperInterface_V2
import numpy as np
from math import pi

# ==================== 配置参数 ====================
CAN_PORT = "can0"
PUBLISH_FREQ = 30  # 发布频率(Hz)，可修改为30/50/100等
GRIPPER_OPEN_THRESHOLD = 65000  # 夹爪打开阈值(0.001mm)
GRIPPER_MAX_SPEED = 10.0  # 夹爪最大速度mm/s，用于归一化

# ==================== ROS初始化 ====================
rospy.init_node('piper_arm_publisher', anonymous=True)
qpos_pub = rospy.Publisher('left_arm/observations/qpos', Float32MultiArray, queue_size=10)
qvel_pub = rospy.Publisher('left_arm/observations/qvel', Float32MultiArray, queue_size=10)
rate = rospy.Rate(PUBLISH_FREQ)  # 使用配置的频率

# ==================== 机械臂初始化 ====================
arm = C_PiperInterface_V2(can_name=CAN_PORT)
arm.ConnectPort()
arm.EnableArm(motor_num=7)

# ==================== 速度计算缓存 ====================
gripper_last_pos = None
last_time = None

print(f"开始实时发布数据 (频率: {PUBLISH_FREQ}Hz, 按 Ctrl+C 退出)...")
print("话题: left_arm/observations/qpos & qvel")

try:
    while not rospy.is_shutdown():
        # ========== qpos采集与发布 ==========
        joints = arm.GetArmJointMsgs().joint_state
        joint_pos = np.array([
            joints.joint_1, joints.joint_2, joints.joint_3,
            joints.joint_4, joints.joint_5, joints.joint_6
        ], dtype=float) * 0.001 * pi / 180.0
        
        gripper_raw = arm.GetArmGripperMsgs().gripper_state.grippers_angle
        gripper_pos = 1.0 if gripper_raw > GRIPPER_OPEN_THRESHOLD else 0.0
        
        qpos = np.concatenate([joint_pos, [gripper_pos]])
        qpos_pub.publish(Float32MultiArray(data=qpos.tolist()))

        # ========== qvel采集与发布 ==========
        spd_info = arm.GetArmHighSpdInfoMsgs()
        joint_vel = np.array([
            spd_info.motor_1.motor_speed,
            spd_info.motor_2.motor_speed,
            spd_info.motor_3.motor_speed,
            spd_info.motor_4.motor_speed,
            spd_info.motor_5.motor_speed,
            spd_info.motor_6.motor_speed
        ], dtype=float) * 0.001  # rad/s

        gripper_raw_data = arm.GetArmGripperMsgs()
        current_time = gripper_raw_data.time_stamp
        current_pos = gripper_raw_data.gripper_state.grippers_angle
        
        if gripper_last_pos is not None and last_time is not None:
            dt = current_time - last_time
            if dt > 0.001:
                gripper_vel = (current_pos - gripper_last_pos) / (dt * 1000)  # mm/s
                gripper_vel_norm = np.clip(gripper_vel / GRIPPER_MAX_SPEED, -1.0, 1.0)
            else:
                gripper_vel_norm = 0.0
        else:
            gripper_vel_norm = 0.0
        
        gripper_last_pos = current_pos
        last_time = current_time
        
        qvel = np.concatenate([joint_vel, [gripper_vel_norm]])
        qvel_pub.publish(Float32MultiArray(data=qvel.tolist()))

        rate.sleep()  # 按配置频率休眠

except KeyboardInterrupt:
    print("\n退出中...")

finally:
    arm.DisableArm(motor_num=7)
    arm.DisconnectPort()
    rospy.signal_shutdown("手动退出")