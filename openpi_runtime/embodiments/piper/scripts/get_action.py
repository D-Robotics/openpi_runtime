#!/usr/bin/env python3
"""
简单的Piper机械臂Action数据采集脚本
Action格式: [arm_qpos(6), gripper_position(1)]
"""
from piper_sdk import C_PiperInterface_V2
import numpy as np
from math import pi

# ==================== 配置参数 ====================
CAN_PORT = "can0"  # 根据实际情况修改
GRIPPER_MAX_MM = 100.0  # 夹爪最大行程：70(小型)或100(大型)，需根据实际硬件修改
GRIPPER_THRESHOLD = 65000

# ==================== 初始化 ====================
print("1. 连接机械臂...")
arm = C_PiperInterface_V2(can_name=CAN_PORT)
arm.ConnectPort()
arm.EnableArm(motor_num=7)  # 启用6个关节 + 1个夹爪

# ==================== 采集数据 ====================
print("\n2. 采集原始数据...")

# 获取6个关节角度
# 原始数据: 6个整数值，单位是0.001度
# 示例: 90000 表示 90.000度
joint_raw = arm.GetArmJointMsgs().joint_state
joints_raw_array = np.array([
    joint_raw.joint_1,  # int, 0.001度
    joint_raw.joint_2,  # int, 0.001度
    joint_raw.joint_3,  # int, 0.001度
    joint_raw.joint_4,  # int, 0.001度
    joint_raw.joint_5,  # int, 0.001度
    joint_raw.joint_6   # int, 0.001度
], dtype=np.int32)
print(f"原始关节数据: {joints_raw_array}")
print(f"单位: 0.001度 (示例: {joints_raw_array[0]} → {joints_raw_array[0]*0.001:.3f}度)")

# 获取夹爪位置
# 原始数据: 整数值，单位是0.001mm
# 示例: 50000 表示 50.000mm
gripper_raw = arm.GetArmGripperMsgs().gripper_state.grippers_angle  # int, 0.001mm
print(f"\n原始夹爪数据: {gripper_raw}")
print(f"单位: 0.001mm (示例: {gripper_raw} → {gripper_raw*0.001:.3f}mm)")

# ==================== 数据转换 ====================
print("\n3. 转换数据...")

# 关节位置: 0.001度 → 弧度
# 转换公式: 弧度 = 值 × 0.001 × π / 180
joint_positions = joints_raw_array.astype(np.float32) * 0.001 * pi / 180.0
print(f"关节位置(弧度): {joint_positions}")
print(f"值范围: [{joint_positions.min():.3f}, {joint_positions.max():.3f}] rad")

# 夹爪位置: 0.001mm → 归一化[0,1]
# 转换公式: 归一化值 = 当前值 / (最大行程 × 1000)
# 0表示完全闭合，1表示完全张开
if gripper_raw >= GRIPPER_THRESHOLD:
    gripper_position = 1
else:
    gripper_position = 0
print(f"夹爪位置(归一化): {gripper_position:.3f}")
print(f"值范围: [0.0, 1.0] (0=闭合, 1=张开)")

# ==================== 构建Action ====================
print("\n4. 构建Action向量...")
action = np.concatenate([joint_positions, [gripper_position]])
print(f"Action: {action}")
print(f"Shape: {action.shape}, Dtype: {action.dtype}")
print(f"内容: [joint1, joint2, joint3, joint4, joint5, joint6, gripper]")

# ==================== 清理 ====================
arm.DisableArm(motor_num=7)
arm.DisconnectPort()
print("\n5. 采集完成!")