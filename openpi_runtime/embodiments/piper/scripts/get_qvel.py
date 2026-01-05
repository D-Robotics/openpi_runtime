#!/usr/bin/env python3
"""
实时获取Piper机械臂qvel数据
qvel格式: [arm_qvel(6), gripper_velocity(1)]
"""
from piper_sdk import C_PiperInterface_V2
import numpy as np
import time

# 配置
CAN_PORT = "can0"

# 初始化
arm = C_PiperInterface_V2(can_name=CAN_PORT)
arm.ConnectPort()
arm.EnableArm(motor_num=7)

# 夹爪速度计算缓存
gripper_last_pos = None
last_time = None

print("开始实时采集qvel数据 (按 Ctrl+C 退出)...")
print("格式: [j1, j2, j3, j4, j5, j6, gripper_vel]\n")

try:
    while True:
        # 1. 获取关节速度 (6个)
        # 单位: 0.001rad/s → 转换为 rad/s
        spd_info = arm.GetArmHighSpdInfoMsgs()
        joint_vel = np.array([
            spd_info.motor_1.motor_speed,
            spd_info.motor_2.motor_speed,
            spd_info.motor_3.motor_speed,
            spd_info.motor_4.motor_speed,
            spd_info.motor_5.motor_speed,
            spd_info.motor_6.motor_speed
        ], dtype=float) * 0.001
        
        # 2. 获取夹爪当前位置并计算速度
        gripper_raw = arm.GetArmGripperMsgs()
        current_time = gripper_raw.time_stamp
        current_pos = gripper_raw.gripper_state.grippers_angle  # 单位: 0.001mm
        
        if gripper_last_pos is not None and last_time is not None:
            dt = current_time - last_time
            if dt > 0.001:  # 避免除零
                # 计算速度: mm/s，然后归一化 (假设最大速度 10mm/s)
                gripper_vel = (current_pos - gripper_last_pos) / (dt * 1000)
                gripper_vel_norm = np.clip(gripper_vel / 10.0, -1.0, 1.0)
            else:
                gripper_vel_norm = 0.0
        else:
            gripper_vel_norm = 0.0
        
        # 更新缓存
        gripper_last_pos = current_pos
        last_time = current_time
        
        # 3. 构建qvel向量
        qvel = np.concatenate([joint_vel, [gripper_vel_norm]])
        
        # 4. 实时显示
        print(f"\r{qvel}", end="", flush=True)
        
        # 控制采集频率 (约100Hz)
        time.sleep(0.02)

except KeyboardInterrupt:
    print("\n\n采集结束!")

finally:
    # 清理
    arm.DisableArm(motor_num=7)
    arm.DisconnectPort()