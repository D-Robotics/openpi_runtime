#!/usr/bin/env python3
"""
最简化的ROS2话题录制脚本
"""

import os
import sys
import signal
import subprocess
import time
import glob
import threading

class SimpleRecorder:
    def __init__(self):
        self.save_folder = "save_folder"
        if not os.path.exists(self.save_folder):
            os.makedirs(self.save_folder)
        
        self.topics = [
            '/aliciaD/action',
            '/piper/qpos',
            '/camera/camera/color/image_raw',
            '/camera_left/camera_left/color/image_raw'
        ]
        
        self.is_recording = False
        self.process = None
        self.should_exit = False
        
        # 获取下一个序号
        self.next_num = self._get_next_number()
        
        # 启动键盘监听线程
        self.keyboard_thread = threading.Thread(target=self._keyboard_listener, daemon=True)
        self.keyboard_thread.start()
    
    def _get_next_number(self):
        """获取下一个可用的bag文件序号"""
        existing_files = glob.glob(os.path.join(self.save_folder, "rosbag_*"))
        numbers = []
        for file in existing_files:
            try:
                base = os.path.basename(file).split('.')[0]
                if base.startswith('rosbag_'):
                    num = int(base.split('_')[1])
                    numbers.append(num)
            except:
                pass
        return max(numbers) + 1 if numbers else 1
    
    def _keyboard_listener(self):
        """键盘监听线程"""
        import termios
        import tty
        
        # 保存原始终端设置
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        
        try:
            # 设置为原始模式
            tty.setraw(fd)
            
            print("\n控制命令: s-开始录制, d-停止录制, q-退出")
            
            while not self.should_exit:
                # 读取单个字符
                char = sys.stdin.read(1).lower()
                
                if char == 'q':
                    print("\n收到退出指令")
                    self.should_exit = True
                    break
                elif char == 's' and not self.is_recording:
                    self.start()
                elif char == 'd' and self.is_recording:
                    self.stop()
                
                time.sleep(0.01)
                
        finally:
            # 恢复终端设置
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    
    def start(self):
        """开始录制"""
        if self.is_recording:
            print("已经在录制中")
            return
        
        output_path = os.path.join(self.save_folder, f"rosbag_{self.next_num}")
        cmd = ['ros2', 'bag', 'record', '-o', output_path] + self.topics
        
        print(f"\n开始录制: rosbag_{self.next_num}")
        print("按 d 停止录制")
        
        try:
            # 启动录制进程
            self.process = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True
            )
            self.is_recording = True
            
            # 启动输出读取线程
            threading.Thread(target=self._read_output, daemon=True).start()
            
        except Exception as e:
            print(f"启动录制失败: {e}")
            self.process = None
    
    def _read_output(self):
        """读取进程输出"""
        if self.process:
            try:
                # 读取错误输出
                for line in self.process.stderr:
                    if line.strip():
                        print(f"录制: {line.strip()}")
            except:
                pass
    
    def stop(self):
        """停止录制"""
        if not self.is_recording or not self.process:
            return
        
        print("\n停止录制...")
        
        try:
            # 发送中断信号
            self.process.send_signal(signal.SIGINT)
            
            # 等待进程结束
            try:
                self.process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                print("强制终止录制进程")
                self.process.terminate()
                self.process.wait()
            
            self.is_recording = False
            self.process = None
            
            # 检查文件
            bag_files = glob.glob(os.path.join(self.save_folder, f"rosbag_{self.next_num}*"))
            if bag_files:
                total_size = sum(os.path.getsize(f) for f in bag_files)
                print(f"录制完成: rosbag_{self.next_num} ({total_size/1024/1024:.2f} MB)")
            else:
                print("录制完成，但未找到保存的文件")
            
            # 更新序号
            self.next_num += 1
            print(f"下一个文件: rosbag_{self.next_num}")
            
        except Exception as e:
            print(f"停止录制失败: {e}")
            self.is_recording = False
            self.process = None
    
    def cleanup(self):
        """清理资源"""
        if self.is_recording:
            self.stop()
        self.should_exit = True

def main():
    print("=" * 60)
    print("ROS2 话题录制器")
    print("=" * 60)
    
    # 显示保存路径
    save_folder = "save_folder"
    save_path = os.path.abspath(save_folder)
    print(f"保存文件夹: {save_path}")
    
    # 显示话题
    topics = [
        '/aliciaD/action',
        '/piper/qpos',
        '/camera/camera/color/image_raw',
        '/camera_left/camera_left/color/image_raw'
    ]
    print("录制话题:")
    for topic in topics:
        print(f"  {topic}")
    print("-" * 60)
    
    # 显示已有文件
    if os.path.exists(save_folder):
        existing_files = glob.glob(os.path.join(save_folder, "rosbag_*"))
        if existing_files:
            print("已有文件:")
            for file in sorted(existing_files):
                try:
                    size = os.path.getsize(file)
                    name = os.path.basename(file)
                    print(f"  {name} - {size/1024/1024:.2f} MB")
                except:
                    pass
            print("-" * 60)
    
    # 创建录制器
    recorder = SimpleRecorder()
    
    try:
        # 主循环
        while not recorder.should_exit:
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("\n收到键盘中断")
    finally:
        recorder.cleanup()
        print("程序已退出")

if __name__ == '__main__':
    main()