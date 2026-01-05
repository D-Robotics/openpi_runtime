#!/usr/bin/env python3
# -*-coding:utf8-*-
"""
Piper机械臂控制器（关节模式专用版）
仅支持关节空间控制，参考原始代码的简洁模式
"""

import time
import threading
from typing import Optional, List, Union
import numpy as np
# from piper_sdk import *
from piper_sdk import C_PiperInterface_V2


class PiperConnectionError(Exception):
    """机械臂连接异常"""
    pass

class PiperEnableError(Exception):
    """机械臂使能异常"""
    pass

class PiperController:
    """
    Piper机械臂控制器（关节模式专用）
    专注于关节空间控制，接口简洁，符合原始使用习惯
    
    Args:
        can_channel: CAN通道名称（如"can0"）
        arm_id: 机械臂标识符（用于多臂场景）
    """
    
    def __init__(self, can_channel: str, arm_id: str = "default") -> None:
        """
        初始化机械臂控制器
        
        Args:
            can_channel: CAN通道名称，如"can0"、"can_piper"
            arm_id: 机械臂ID，如"left"、"right"
        """
        self.arm_id = arm_id
        self.can_channel = can_channel
        self._piper: Optional[C_PiperInterface_V2] = None
        self._connected: bool = False
        self._enabled: bool = False
        
        # 实例级别的线程锁
        self._lock: threading.Lock = threading.Lock()
        
        # 关节转换系数：弧度 → SDK单位(0.001度)
        # 1000 * 180 / π = 57295.7795 (使用代码中的精确值)
        self.JOINT_FACTOR = 57295.7795
        
        # 夹爪转换系数：mm → SDK单位(0.001mm)
        self.GRIPPER_FACTOR = 1000 * 1000
        
        print(f"[PiperController:{self.arm_id}] 初始化完成，CAN通道: {can_channel}")
    
    # ==================== 连接管理接口 ====================
    
    def connect(self, timeout: int = 10) -> None:
        """
        连接机械臂
        
        Args:
            timeout: 连接超时时间（秒）
            
        Raises:
            PiperConnectionError: 连接失败时抛出
        """
        with self._lock:
            if self._connected:
                print(f"[PiperController:{self.arm_id}] 机械臂已连接")
                return
                
            try:
                print(f"[PiperController:{self.arm_id}] 正在连接...")
                self._piper = C_PiperInterface_V2(self.can_channel)
                self._piper.ConnectPort()
                
                # 验证连接
                start_time = time.time()
                while time.time() - start_time < timeout:
                    if self._piper.GetArmStatus() is not None:
                        self._connected = True
                        print(f"[PiperController:{self.arm_id}] 连接成功")
                        return
                    time.sleep(0.1)
                
                raise PiperConnectionError("连接超时")
                
            except Exception as e:
                raise PiperConnectionError(f"[PiperController:{self.arm_id}] 连接失败: {e}")
    
    def disconnect(self) -> None:
        """断开机械臂连接"""
        with self._lock:
            if self._connected:
                try:
                    self.disable()
                except:
                    pass
                self._connected = False
                print(f"[PiperController:{self.arm_id}] 机械臂已断开")
    
    def is_connected(self) -> bool:
        """检查是否已连接"""
        return self._connected
    
    # ==================== 核心功能接口 ====================
    
    def reset(self) -> None:
        """
        重置机械臂
        恢复到初始状态并清除错误
        """
        self._check_connection()
        print(f"[PiperController:{self.arm_id}] 重置机械臂...")
        
        with self._lock:
            self._piper.MotionCtrl_1(0x02, 0, 0)  # 完整复位
            self._piper.MotionCtrl_2(0, 0, 0, 0x00)  # 位置速度模式
        
        time.sleep(0.5)
        self._enabled = False
    
    def enable(self, timeout: int = 10) -> None:
        """
        使能机械臂（使用简化模式）
        
        Args:
            timeout: 使能超时时间（秒）
            
        Raises:
            PiperEnableError: 使能失败时抛出
        """
        self._check_connection()
        
        if self._enabled:
            print(f"[PiperController:{self.arm_id}] 机械臂已使能")
            return
        
        print(f"[PiperController:{self.arm_id}] 正在使能...")
        start_time = time.time()
        
        while time.time() - start_time < timeout:
            try:
                with self._lock:
                    if self._piper.EnablePiper():  # 使用简化使能函数
                        self._enabled = True
                        # 初始化夹爪
                        self._piper.GripperCtrl(0, 1000, 0x01, 0)
                        print(f"[PiperController:{self.arm_id}] 使能成功")
                        return
            except Exception as e:
                print(f"[PiperController:{self.arm_id}] 使能失败，重试中: {e}")
            
            time.sleep(0.01)
        
        raise PiperEnableError(f"[PiperController:{self.arm_id}] 使能超时")
    
    def disable(self) -> None:
        """失能机械臂"""
        self._check_connection()
        if self._enabled:
            with self._lock:
                self._piper.DisableArm(7)
            self._enabled = False
            print(f"[PiperController:{self.arm_id}] 机械臂已失能")
    
    def is_enabled(self) -> bool:
        """检查是否已使能"""
        return self._enabled
    
    # ==================== 状态获取接口 ====================
    
    def get_state(self) -> Optional[np.ndarray]:
        """
        获取机械臂当前状态
        
        Returns:
            8维状态数组或None（如果获取失败）:
            [joint_0, joint_1, joint_2, joint_3, joint_4, joint_5,  # 关节角度(度)
             gripper_angle]                                           # 夹爪开度(mm)
        """
        self._check_connection()
        
        # 获取关节数据
        arm_msg = self._piper.GetArmJointMsgs()
        gripper_msg = self._piper.GetArmGripperMsgs()
        
        if arm_msg is None or gripper_msg is None:
            print(f"[PiperController:{self.arm_id}] 警告：无法获取机械臂状态")
            return None
        
        state = np.zeros(8, dtype=np.float64)
        
        # 提取6个关节角度（0.001度 → 度）
        state[0] = float(arm_msg.joint_state.joint_1) / 1000.0
        state[1] = float(arm_msg.joint_state.joint_2) / 1000.0
        state[2] = float(arm_msg.joint_state.joint_3) / 1000.0
        state[3] = float(arm_msg.joint_state.joint_4) / 1000.0
        state[4] = float(arm_msg.joint_state.joint_5) / 1000.0
        state[5] = float(arm_msg.joint_state.joint_6) / 1000.0
        
        # 提取夹爪角度（0.001mm → mm）
        state[6] = float(gripper_msg.gripper_state.grippers_angle) / 1000.0
        
        return state
    
    def get_joint_angles(self) -> Optional[np.ndarray]:
        """
        获取关节角度
        
        Returns:
            6维关节角度数组（单位：度）或None
        """
        state = self.get_state()
        return state[:6] if state is not None else None
    
    def get_gripper_angle(self) -> Optional[float]:
        """
        获取夹爪开度
        
        Returns:
            夹爪开度（单位：mm）或None
        """
        state = self.get_state()
        return state[6] if state is not None else None
    
    # ==================== 运动控制接口 ====================
    
    def move_joints(self, joint_angles: Union[List[float], np.ndarray], 
                   gripper_angle: Optional[float] = None,
                   speed: int = 100) -> None:
        """
        关节空间运动控制（核心接口）
        
        Args:
            joint_angles: 6个关节角度（支持弧度或度）
            gripper_angle: 夹爪开度（mm），可选
            speed: 运动速度（0-100）
        """
        self._check_connection()
        self._check_enabled()
        
        # 确保是6个关节
        if len(joint_angles) != 6:
            raise ValueError("必须提供6个关节角度")
        
        # 转换关节角度到SDK单位
        joints = [self._angle_to_sdk(a) for a in joint_angles]
        
        # 夹爪控制（可选）
        gripper_cmd = 0
        if gripper_angle is not None:
            gripper_cmd = int(gripper_angle * self.GRIPPER_FACTOR)
        
        # 发送控制指令（参考原始代码的调用顺序）
        with self._lock:
            self._piper.MotionCtrl_2(0x01, 0x01, speed, 0x00)  # 设置关节模式
            self._piper.JointCtrl(*joints)                      # 控制6个关节
            self._piper.GripperCtrl(abs(gripper_cmd), 1000, 0x01, 0)  # 控制夹爪
        
        # 打印状态（参考原始代码）
        print(f"[PiperController:{self.arm_id}] 状态: {self._piper.GetArmStatus()}")
    
    # ==================== 夹爪控制接口 ====================
    
    def set_gripper(self, angle: float, speed: int = 1000, force: int = 0x01) -> None:
        """
        单独控制夹爪
        
        Args:
            angle: 夹爪开度（mm）
            speed: 运动速度
            force: 力控模式
        """
        self._check_connection()
        self._check_enabled()
        
        # 转换到SDK单位
        angle_sdk = int(angle * self.GRIPPER_FACTOR)
        
        with self._lock:
            self._piper.GripperCtrl(abs(angle_sdk), speed, force, 0)
        
        print(f"[PiperController:{self.arm_id}] 夹爪开度: {angle}mm")
    
    # ==================== 内部工具方法 ====================
    
    def _check_connection(self) -> None:
        """检查连接状态"""
        if not self._connected or self._piper is None:
            raise PiperConnectionError(f"[PiperController:{self.arm_id}] 机械臂未连接，请先调用connect()")
    
    def _check_enabled(self) -> None:
        """检查使能状态"""
        if not self._enabled:
            raise PiperEnableError(f"[PiperController:{self.arm_id}] 机械臂未使能，请先调用enable()")
    
    def _angle_to_sdk(self, angle: float) -> int:
        """
        角度转换到SDK单位
        
        Args:
            angle: 输入角度（度或弧度）
            
        Returns:
            SDK单位（0.001度）
        """
        # 如果角度值较小（<2π），假设输入的是弧度，转换为度
        if abs(angle) < np.pi * 2:
            angle = np.degrees(angle)
        
        return int(angle * self.JOINT_FACTOR)
    
    # ==================== 上下文管理器 ====================
    
    def __enter__(self) -> 'PiperController':
        """上下文管理器入口"""
        self.connect()
        self.enable()
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb) -> None:
        """上下文管理器出口"""
        self.disable()
        self.disconnect()


# ==================== 使用示例 ====================

def demo_joint_control():
    """
    关节控制示例（参考原始代码的使用方式）
    """
    print("=" * 60)
    print("关节控制示例")
    print("=" * 60)
    
    factor = 57295.7795
    
    # 创建机械臂实例
    arm = PiperController("can0", arm_id="main_arm")
    
    try:

        # arm.reset()
        # arm.reset()
        time.sleep(1)
        print("connect arm")
        # 连接和使能
        arm.connect()
        # arm.reset()
        # arm.reset()


        arm.enable()
        
        position = [0, 0, 0, 0, 0, 0, 0]  # 前6个是关节，第7个是夹爪
        count = 0
        
        # 参考原始代码的循环控制模式
        while count < 900:  # 运行4.5秒
            count += 1
            
            if count == 0:
                print("1-----------")
                position = [0, 0, 0, 0, 0, 0, 0]
            elif count == 300:
                print("2-----------")
                # 使用弧度制输入，与SDK单位转换
                # position = [0.2, 0.2, -0.2, 0.3, -0.2, 0.5, 0.08]
            elif count == 600:
                print("1-----------")
                position = [0, 0, 0, 0, 0, 0, 0]
                count = 0
            
            # 控制机械臂（自动处理单位转换）
            arm.move_joints(
                joint_angles=position[:6],      # 6个关节（弧度）
                gripper_angle=position[6],      # 夹爪开度（mm）
                speed=100
            )
            
            # 打印当前状态
            print(f"目标位置: {position}")
            print(f"当前状态: {arm.get_state()}")
            
            time.sleep(0.005)
    
    except KeyboardInterrupt:
        print("\n用户中断")
    
    finally:
        # 清理资源
        # arm.reset()
        # arm.disconnect()
        print("演示完成\n")


def demo_multi_arm():
    """多臂协同控制示例"""
    print("=" * 60)
    print("双臂协同控制示例")
    print("=" * 60)
    
    # 创建两个机械臂实例
    left_arm = PiperController("can0", arm_id="left_arm")
    right_arm = PiperController("can1", arm_id="right_arm")
    
    try:
        # 同时连接和使能
        left_arm.connect()
        right_arm.connect()
        left_arm.enable()
        right_arm.enable()
        
        # 双臂镜像运动
        left_arm.move_joints([0.1, 0.1, -0.1, 0, 0, 0])
        right_arm.move_joints([-0.1, -0.1, 0.1, 0, 0, 0])
        
        time.sleep(1)
        
        # 同时读取状态
        print(f"左臂状态: {left_arm.get_joint_angles()}")
        print(f"右臂状态: {right_arm.get_joint_angles()}")
        
        # 重置双臂
        left_arm.reset()
        right_arm.reset()
        
    finally:
        left_arm.disconnect()
        right_arm.disconnect()
        print("双臂演示完成\n")


def main():
    """主函数"""
    try:
        # 演示关节控制（参考原始代码）
        demo_joint_control()
        
        # 根据硬件情况启用多臂演示
        # demo_multi_arm()
        
    except Exception as e:
        print(f"\n[错误] {e}")
        return 1
    
    return 0


if __name__ == "__main__":
    exit(main())