import time
import sys
from piper_sdk import C_PiperInterface_V2
import re

# 使能机械臂
def piper_enable(piper: C_PiperInterface_V2):
    '''
    使能机械臂并检测使能状态,尝试5秒,超时则退出
    '''
    timeout = 5
    start_time = time.time()

    while True:
        low_msg = piper.GetArmLowSpdInfoMsgs()
        enabled = all([
            low_msg.motor_1.foc_status.driver_enable_status,
            low_msg.motor_2.foc_status.driver_enable_status,
            low_msg.motor_3.foc_status.driver_enable_status,
            low_msg.motor_4.foc_status.driver_enable_status,
            low_msg.motor_5.foc_status.driver_enable_status,
            low_msg.motor_6.foc_status.driver_enable_status
        ])
        print("--------------------")
        print("使能状态:", enabled)
        if enabled:
            return 0
            break
        piper.EnableArm(7)
        piper.GripperCtrl(0, 1000, 0x01, 0)
        if time.time() - start_time > timeout:
            print("使能超时，退出程序")
            return 1
            sys.exit(1)
        time.sleep(1)

# 控制机械臂和夹爪
def pi0_control_piper(piper: C_PiperInterface_V2, joint_and_gripper: list):
    factor = 57324.840764  # 弧度 → 编码器单位 (1000 * 180 / π)
    gripper_max = 75 # 夹爪最大行程
    j0 = round(joint_and_gripper[0] * factor)
    j1 = round(joint_and_gripper[1] * factor)
    j2 = round(joint_and_gripper[2] * factor)
    j3 = round(joint_and_gripper[3] * factor)
    j4 = round(joint_and_gripper[4] * factor)
    j5 = round(joint_and_gripper[5] * factor)
    gripper = int(joint_and_gripper[6] * gripper_max * 1000)

    piper.MotionCtrl_2(0x01, 0x01, 100, 0x00)  # 模式设置
    piper.JointCtrl(j0, j1, j2, j3, j4, j5)
    piper.GripperCtrl(abs(gripper), 1000, 0x01, 0)

    time.sleep(0.02) # 控制执行帧率，最大50hz，也就是0.02秒

# 机械臂运动到初始位置
def piper_go_zero(piper: C_PiperInterface_V2):
    position = [0,0,0,0,0,0,0]
    pi0_control_piper(piper, position)
# lw end


