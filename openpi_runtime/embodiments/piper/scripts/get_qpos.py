#!/usr/bin/env python3
"""
实时获取Piper机械臂qpos数据
qpos格式: [arm_qpos(6), gripper_position(1)]
按Ctrl+C退出
"""
from piper_sdk import C_PiperInterface_V2
import numpy as np
from math import pi
import time

# ==================== 配置 ====================
CAN_PORT = "can0"
GRIPPER_OPEN_THRESHOLD = 65000  # 夹爪打开判定阈值(0.001mm)

# ==================== 初始化 ====================
arm = C_PiperInterface_V2(can_name=CAN_PORT)
arm.ConnectPort()
arm.EnableArm(motor_num=7)

print("开始实时采集qpos数据 (按 Ctrl+C 退出)...\n")
print("格式: [j1, j2, j3, j4, j5, j6, gripper(0/1)]")

try:
    while True:
        # 采集关节数据
        joints = arm.GetArmJointMsgs().joint_state
        
        # 转换: 关节角度(0.001度) → 弧度
        joint_pos = np.array([
            joints.joint_1, joints.joint_2, joints.joint_3,
            joints.joint_4, joints.joint_5, joints.joint_6
        ], dtype=float) * 0.001 * pi / 180.0
        
        # 采集夹爪数据
        gripper_raw = arm.GetArmGripperMsgs().gripper_state.grippers_angle
        
        # 转换: 夹爪位置 → 二值化[0,1]
        gripper_pos = 1.0 if gripper_raw > GRIPPER_OPEN_THRESHOLD else 0.0
        
        # 构建qpos
        qpos = np.concatenate([joint_pos, [gripper_pos]])
        
        # 实时显示
        print(f"\r{qpos}", end="", flush=True)
        
        # 控制采集频率
        time.sleep(0.02)  # 约100Hz

except KeyboardInterrupt:
    print("\n\n采集结束!")

finally:
    # ==================== 清理 ====================
    arm.DisableArm(motor_num=7)
    arm.DisconnectPort()