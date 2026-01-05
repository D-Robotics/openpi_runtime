#!/usr/bin/env python3
"""
Rosbag2 转 HDF5 转换脚本（最终修复版）
修复HDF5变长数据集写入问题
"""

import os
import sys
import numpy as np
import h5py
import cv2
import tempfile
import shutil
import subprocess
import yaml
from typing import Dict, List, Tuple, Optional, Any
from dataclasses import dataclass
from pathlib import Path

# ROS2相关导入
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import Float32MultiArray

@dataclass
class DataPoint:
    """数据点容器"""
    timestamp: float
    data: Any

class Rosbag2Hdf5Converter:
    """Rosbag2转HDF5转换器"""
    
    def __init__(self, input_dir: str, output_dir: str, target_fps: int = 50):
        self.input_dir = Path(input_dir)
        self.output_dir = Path(output_dir)
        self.target_fps = target_fps
        self.target_dt = 1.0 / target_fps
        
        # 话题映射配置
        self.topic_config = {
            '/aliciaD/action': {
                'type': 'Float32MultiArray',
                'hdf5_path': '/action',
                'target_dim': 14,
                'pad_value': 0.0
            },
            '/piper/qpos': {
                'type': 'Float32MultiArray',
                'hdf5_path': '/observations/qpos',
                'target_dim': 14,
                'pad_value': 0.0
            },
            '/camera/camera/color/image_raw': {
                'type': 'Image',
                'hdf5_path': '/observations/images/cam_high',
                'encoding': 'jpeg'
            },
            '/camera_left/camera_left/color/image_raw': {
                'type': 'Image',
                'hdf5_path': '/observations/images/cam_left_wrist',
                'encoding': 'jpeg'
            }
        }
        
        # 创建输出目录
        self.output_dir.mkdir(parents=True, exist_ok=True)
        
        # 临时解压目录
        self.temp_base_dir = Path('/tmp/rosbag_unzip_final')
        self.temp_base_dir.mkdir(parents=True, exist_ok=True)
        
    def find_bag_files(self) -> List[Path]:
        """查找所有包含mcap.zstd文件的目录"""
        bag_dirs = []
        
        # 查找包含mcap.zstd的目录
        for zstd_file in self.input_dir.rglob('*.mcap.zstd'):
            bag_dir = zstd_file.parent
            if bag_dir not in bag_dirs:
                bag_dirs.append(bag_dir)
        
        if not bag_dirs:
            print(f"警告: 在 {self.input_dir} 中未找到rosbag2文件")
        
        return sorted(bag_dirs)
    
    def decompress_zstd(self, bag_dir: Path) -> Optional[Path]:
        """解压zstd文件到临时目录并精确修复metadata"""
        zstd_files = list(bag_dir.glob('*.mcap.zstd'))
        original_metadata = bag_dir / 'metadata.yaml'
        
        if not zstd_files:
            print(f"  警告: 未找到zstd文件")
            return None
        
        if not original_metadata.exists():
            print(f"  错误: 未找到metadata.yaml")
            return None
        
        # 创建临时目录
        temp_dir = self.temp_base_dir / bag_dir.name
        temp_dir.mkdir(parents=True, exist_ok=True)
        
        print(f"  解压并修复: {bag_dir.name}")
        
        # 解压所有zstd文件
        mcap_files = []
        for zstd_file in zstd_files:
            mcap_file = temp_dir / zstd_file.stem  # 移除.zstd后缀
            mcap_files.append(mcap_file)
            
            if mcap_file.exists():
                print(f"    已存在: {mcap_file.name}")
                continue
                
            try:
                # 使用zstd命令解压
                cmd = ['zstd', '-d', '-c', str(zstd_file)]
                with open(mcap_file, 'wb') as f:
                    subprocess.run(cmd, stdout=f, check=True, stderr=subprocess.PIPE)
                print(f"    解压: {zstd_file.name} → {mcap_file.name}")
            except subprocess.CalledProcessError as e:
                print(f"    解压失败 {zstd_file.name}: {e.stderr.decode()}")
                continue
            except FileNotFoundError:
                print(f"    错误: 未找到zstd命令，请安装zstd工具")
                return None
        
        # 复制原始metadata到临时目录
        temp_metadata = temp_dir / 'metadata.yaml'
        shutil.copy2(original_metadata, temp_metadata)
        
        # 精确修复metadata的4个关键字段
        self._fix_metadata_exactly(temp_metadata, mcap_files)
        
        return temp_dir
    
    def _fix_metadata_exactly(self, metadata_path: Path, mcap_files: List[Path]):
        """精确修改metadata.yaml的4个字段，保持其他内容完全不变"""
        print(f"    精确修复metadata: {metadata_path.name}")
        
        # 读取原始metadata
        with open(metadata_path) as f:
            content = f.read()
        
        # 备份原始文件
        backup_path = metadata_path.with_suffix('.yaml.original')
        with open(backup_path, 'w') as f:
            f.write(content)
        print(f"    原始metadata已备份: {backup_path.name}")
        
        # 解析YAML
        try:
            data = yaml.safe_load(content)
            info = data['rosbag2_bagfile_information']
        except Exception as e:
            print(f"    解析metadata失败: {e}")
            return
        
        # 1. 修改compression_format和compression_mode
        info['compression_format'] = ""
        info['compression_mode'] = ""
        print(f"    ✓ 修复compression_format和compression_mode")
        
        # 2. 修改relative_file_paths
        original_paths = info.get('relative_file_paths', [])
        fixed_paths = [path.replace('.mcap.zstd', '.mcap') for path in original_paths]
        info['relative_file_paths'] = fixed_paths
        print(f"    ✓ 修复relative_file_paths ({len(fixed_paths)}个文件)")
        
        # 3. 修改files列表中的path字段
        if 'files' in info:
            for file_info in info['files']:
                if 'path' in file_info:
                    file_info['path'] = file_info['path'].replace('.mcap.zstd', '.mcap')
            print(f"    ✓ 修复files列表中的path字段")
        
        # 写回文件（使用safe_dump避免Python标签）
        with open(metadata_path, 'w') as f:
            yaml.safe_dump(data, f, default_flow_style=False, sort_keys=False)
        
        print(f"    ✓ metadata修复完成")
        
        # 验证修复后的文件
        try:
            with open(metadata_path) as f:
                test_data = yaml.safe_load(f)
            print(f"    ✓ 修复后的metadata验证通过")
        except Exception as e:
            print(f"    ✗ 修复后的metadata验证失败: {e}")
    
    def read_bag(self, bag_dir: Path) -> Dict[str, List[DataPoint]]:
        """读取rosbag2文件"""
        print(f"  读取bag: {bag_dir.name}")
        
        # 解压并修复metadata
        temp_dir = self.decompress_zstd(bag_dir)
        if temp_dir is None:
            print(f"    错误: 无法解压bag文件")
            return {}
        
        try:
            # 使用临时目录读取
            return self._read_bag_from_dir(temp_dir)
        except Exception as e:
            print(f"    读取失败: {e}")
            return {}
    
    def _read_bag_from_dir(self, bag_dir: Path) -> Dict[str, List[DataPoint]]:
        """从目录读取rosbag2"""
        # 检查必要的文件
        metadata_path = bag_dir / 'metadata.yaml'
        if not metadata_path.exists():
            print(f"    错误: 缺少metadata.yaml")
            return {}
        
        # 验证metadata格式
        try:
            with open(metadata_path) as f:
                test_content = f.read()
            print(f"    metadata文件已加载（{len(test_content.splitlines())}行）")
        except Exception as e:
            print(f"    metadata读取失败: {e}")
            return {}
        
        mcap_files = list(bag_dir.glob('*.mcap'))
        if not mcap_files:
            print(f"    错误: 未找到mcap文件")
            return {}
        
        print(f"    发现 {len(mcap_files)} 个mcap文件")
        
        # 创建reader
        storage_options = rosbag2_py.StorageOptions(
            uri=str(bag_dir),
            storage_id='mcap'
        )
        converter_options = rosbag2_py.ConverterOptions(
            input_serialization_format='cdr',
            output_serialization_format='cdr'
        )
        
        try:
            reader = rosbag2_py.SequentialReader()
            reader.open(storage_options, converter_options)
            print(f"    ✓ bag文件打开成功")
        except Exception as e:
            print(f"    ✗ 打开bag文件失败: {e}")
            return {}
        
        # 获取话题信息
        topic_types = reader.get_all_topics_and_types()
        type_map = {topic.name: topic.type for topic in topic_types}
        print(f"    发现话题: {list(type_map.keys())}")
        
        # 初始化数据存储
        data = {topic: [] for topic in self.topic_config.keys()}
        message_counts = {topic: 0 for topic in self.topic_config.keys()}
        
        # 读取消息
        frame_count = 0
        while reader.has_next():
            try:
                topic, data_msg, timestamp = reader.read_next()
                if topic in data:
                    msg_type = get_message(type_map[topic])
                    msg = deserialize_message(data_msg, msg_type)
                    
                    # 提取数据
                    if isinstance(msg, Float32MultiArray):
                        value = np.array(msg.data, dtype=np.float32)
                    elif isinstance(msg, Image) or hasattr(msg, 'data'):
                        value = self._process_image(msg)
                    else:
                        continue
                    
                    # 转换为秒
                    ts_sec = timestamp * 1e-9
                    data[topic].append(DataPoint(ts_sec, value))
                    message_counts[topic] += 1
                    frame_count += 1
                    
            except Exception as e:
                print(f"    读取消息失败: {e}")
                continue
        
        # 打印统计信息
        for topic, count in message_counts.items():
            if count > 0:
                print(f"    {topic}: {count} 条消息")
        
        if frame_count == 0:
            print(f"    警告: 未读取到任何有效消息")
        
        return data
    
    def _process_image(self, msg: Image) -> np.ndarray:
        """处理图像消息"""
        try:
            # 处理CompressedImage
            if hasattr(msg, 'format') and 'compressed' in getattr(msg, 'format', ''):
                img = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
                if img is None:
                    img = np.zeros((480, 640, 3), dtype=np.uint8)
                img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
                return img
            
            # 处理普通Image消息
            if msg.encoding == 'rgb8':
                img = np.frombuffer(msg.data, dtype=np.uint8).reshape(
                    msg.height, msg.width, 3
                )
                img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
            elif msg.encoding == 'bgr8':
                img = np.frombuffer(msg.data, dtype=np.uint8).reshape(
                    msg.height, msg.width, 3
                )
                img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
            else:
                print(f"    警告: 不支持的图像编码 {msg.encoding}，使用黑色图像")
                img = np.zeros((480, 640, 3), dtype=np.uint8)
            
            return img
        except Exception as e:
            print(f"    图像处理失败: {e}，使用黑色图像")
            return np.zeros((480, 640, 3), dtype=np.uint8)
    
    def synchronize_data(self, data: Dict[str, List[DataPoint]]) -> List[Dict[str, Any]]:
        """数据同步到50Hz"""
        print("  开始数据同步...")
        
        if not data or all(len(v) == 0 for v in data.values()):
            print("    警告: 没有数据可处理")
            return []
        
        # 获取时间范围
        all_timestamps = []
        for points in data.values():
            all_timestamps.extend([p.timestamp for p in points])
        
        if not all_timestamps:
            return []
        
        t_start = min(all_timestamps)
        t_end = max(all_timestamps)
        
        # 生成目标时间戳
        target_times = np.arange(
            np.ceil(t_start / self.target_dt) * self.target_dt,
            t_end,
            self.target_dt
        )
        
        print(f"    时间范围: {t_start:.3f} - {t_end:.3f} 秒")
        print(f"    目标帧数: {len(target_times)} 帧 @ {self.target_fps}Hz")
        
        # 为每个话题创建插值函数
        interpolated_data = {}
        for topic, points in data.items():
            if not points:
                continue
            
            ts = np.array([p.timestamp for p in points])
            values = [p.data for p in points]
            
            if isinstance(values[0], np.ndarray):
                if values[0].ndim == 1:
                    interpolated = self._interpolate_arrays(ts, values, target_times)
                else:
                    interpolated = self._interpolate_images(ts, values, target_times)
            else:
                interpolated = None
            
            interpolated_data[topic] = interpolated
        
        # 组合数据
        synchronized = []
        for i, t in enumerate(target_times):
            frame = {'timestamp': t}
            for topic, config in self.topic_config.items():
                if topic in interpolated_data and interpolated_data[topic] is not None:
                    frame[topic] = interpolated_data[topic][i]
                else:
                    frame[topic] = None
            synchronized.append(frame)
        
        print(f"    同步完成: {len(synchronized)} 帧")
        return synchronized
    
    def _interpolate_arrays(self, ts: np.ndarray, values: List[np.ndarray], 
                           target_ts: np.ndarray) -> List[np.ndarray]:
        """插值一维数组数据"""
        values_array = np.array(values)
        interpolated = np.zeros((len(target_ts), values_array.shape[1]))
        for i in range(values_array.shape[1]):
            interpolated[:, i] = np.interp(target_ts, ts, values_array[:, i])
        return [interpolated[i] for i in range(len(target_ts))]
    
    def _interpolate_images(self, ts: np.ndarray, images: List[np.ndarray], 
                           target_ts: np.ndarray) -> List[np.ndarray]:
        """图像数据插值（最近邻）"""
        interpolated = []
        for t in target_ts:
            idx = np.argmin(np.abs(ts - t))
            interpolated.append(images[idx].copy())
        return interpolated
    
    def create_hdf5(self, synchronized_data: List[Dict[str, Any]], 
                   output_path: Path):
        """创建HDF5文件"""
        print(f"  创建HDF5: {output_path.name}")
        
        if not synchronized_data:
            print("    警告: 没有数据可写入")
            return False
        
        try:
            with h5py.File(output_path, 'w') as f:
                num_frames = len(synchronized_data)
                
                # 创建数据集
                for topic, config in self.topic_config.items():
                    if config['type'] == 'Float32MultiArray':
                        dset = f.create_dataset(
                            config['hdf5_path'],
                            shape=(num_frames, config['target_dim']),
                            dtype=np.float32,
                            compression='gzip',
                            compression_opts=4
                        )
                        
                        # 填充数据
                        for i, frame in enumerate(synchronized_data):
                            if frame[topic] is not None:
                                data = frame[topic]
                                if len(data) < config['target_dim']:
                                    padded = np.pad(
                                        data, 
                                        (0, config['target_dim'] - len(data)),
                                        mode='constant',
                                        constant_values=config['pad_value']
                                    )
                                else:
                                    padded = data[:config['target_dim']]
                                dset[i] = padded
                    
                    elif config['type'] == 'Image':
                        first_img = None
                        for frame in synchronized_data:
                            if frame[topic] is not None:
                                first_img = frame[topic]
                                break
                        
                        if first_img is not None:
                            height, width, channels = first_img.shape
                            
                            dset = f.create_dataset(
                                config['hdf5_path'],
                                shape=(num_frames,),
                                dtype=h5py.special_dtype(vlen=np.uint8)
                            )
                            
                            # 填充图像数据（JPEG压缩）
                            for i, frame in enumerate(synchronized_data):
                                if frame[topic] is not None:
                                    img = frame[topic]
                                    _, encoded = cv2.imencode('.jpg', img)
                                    # FIX: 将bytes转换为numpy数组
                                    dset[i] = np.frombuffer(encoded.tobytes(), dtype=np.uint8)
                                else:
                                    black_img = np.zeros((height, width, channels), dtype=np.uint8)
                                    _, encoded = cv2.imencode('.jpg', black_img)
                                    dset[i] = np.frombuffer(encoded.tobytes(), dtype=np.uint8)
                
                # 特殊处理：cam_right_wrist（黑色图像）
                self._add_black_camera(synchronized_data, f, 
                                     '/camera_left/camera_left/color/image_raw', 
                                     '/observations/images/cam_right_wrist')
                # 添加cam_low（黑色图像）
                self._add_black_camera(synchronized_data, f, 
                                    '/camera_left/camera_left/color/image_raw', 
                                    '/observations/images/cam_low')                
                # 保存时间戳
                timestamps = np.array([f['timestamp'] for f in synchronized_data], dtype=np.float64)
                f.create_dataset('/timestamp', data=timestamps)
                
                # 添加元数据
                f.attrs['fps'] = self.target_fps
                f.attrs['num_frames'] = num_frames
                f.attrs['description'] = 'Rosbag2 to HDF5 conversion'
                
            print(f"    写入完成: {num_frames} 帧")
            return True
            
        except Exception as e:
            print(f"    HDF5写入失败: {e}")
            import traceback
            traceback.print_exc()
            return False
    
    def _add_black_camera(self, synchronized_data: List[Dict[str, Any]], 
                         hdf5_file: h5py.File, src_topic: str, target_path: str):
        """添加黑色相机数据集"""
        first_img = None
        for frame in synchronized_data:
            if frame[src_topic] is not None:
                first_img = frame[src_topic]
                break
        
        if first_img is None:
            height, width, channels = 480, 640, 3
        else:
            height, width, channels = first_img.shape
        
        dset = hdf5_file.create_dataset(
            target_path,
            shape=(len(synchronized_data),),
            dtype=h5py.special_dtype(vlen=np.uint8)
        )
        
        black_img = np.zeros((height, width, channels), dtype=np.uint8)
        _, encoded = cv2.imencode('.jpg', black_img)
        # FIX: 将bytes转换为numpy数组
        black_jpg = np.frombuffer(encoded.tobytes(), dtype=np.uint8)
        
        for i in range(len(synchronized_data)):
            dset[i] = black_jpg
        
        print(f"    创建黑色相机数据集: {target_path}")
    
    def convert_single_bag(self, bag_dir: Path) -> bool:
        """转换单个bag目录"""
        print(f"\n{'='*60}")
        print(f"开始转换: {bag_dir.name}")
        
        try:
            # 读取数据（自动解压并修复metadata）
            data = self.read_bag(bag_dir)
            if not data or all(len(v) == 0 for v in data.values()):
                print(f"  ✗ 错误: 无法读取数据或数据为空")
                return False
            
            # 检查必需话题
            has_data = {topic: len(points) > 0 for topic, points in data.items()}
            print(f"  数据检查: {has_data}")
            
            if not any(has_data.values()):
                print(f"  ✗ 错误: 所有话题都为空")
                return False
            
            # 数据同步
            synchronized = self.synchronize_data(data)
            if not synchronized:
                print(f"  ✗ 错误: 数据同步失败")
                return False
            
            # 创建输出文件名
            output_name = f"{bag_dir.name}.hdf5"
            output_path = self.output_dir / output_name
            
            # 写入HDF5
            success = self.create_hdf5(synchronized, output_path)
            
            if success:
                print(f"  ✓ 转换成功: {output_path}")
            else:
                print(f"  ✗ 转换失败: {bag_dir.name}")
            
            return success
            
        except Exception as e:
            print(f"  ✗ 转换异常: {e}")
            import traceback
            traceback.print_exc()
            return False
    
    def convert_all(self):
        """批量转换所有bag文件"""
        print("="*60)
        print("Rosbag2 转 HDF5 批量转换（精确修复metadata）")
        print(f"输入目录: {self.input_dir}")
        print(f"输出目录: {self.output_dir}")
        print(f"目标频率: {self.target_fps}Hz")
        print("="*60)
        
        bag_dirs = self.find_bag_files()
        if not bag_dirs:
            print(f"✗ 错误: 在 {self.input_dir} 中未找到rosbag2文件")
            return
        
        print(f"发现 {len(bag_dirs)} 个bag目录:")
        for bag_dir in bag_dirs:
            print(f"  - {bag_dir}")
        
        print("\n开始批量转换...")
        results = []
        for bag_dir in bag_dirs:
            success = self.convert_single_bag(bag_dir)
            results.append((bag_dir, success))
        
        # 打印总结
        print("\n" + "="*60)
        print("转换总结:")
        success_count = sum(1 for _, s in results if success)
        for bag_dir, success in results:
            status = "✓ 成功" if success else "✗ 失败"
            print(f"  {status}: {bag_dir.name}")
        
        print(f"\n总计: {success_count}/{len(results)} 成功")
        
        # 打印临时目录位置
        print(f"\n临时解压文件位于: {self.temp_base_dir}")
        print(f"可以手动删除以释放空间: rm -rf {self.temp_base_dir}")


def check_dependencies():
    """检查依赖"""
    print("检查依赖...")
    
    # 检查zstd命令
    try:
        subprocess.run(['zstd', '--version'], capture_output=True, check=True)
        print("  ✓ zstd 命令可用")
    except (subprocess.CalledProcessError, FileNotFoundError):
        print("  ✗ zstd 命令未找到，请先安装: sudo apt install zstd")
        return False
    
    # 检查Python包
    try:
        import rosbag2_py
        print("  ✓ rosbag2_py 可用")
    except ImportError:
        print("  ✗ rosbag2_py 未安装，请运行: pip install rosbag2-py")
        return False
    
    # 检查PyYAML
    try:
        import yaml
        print("  ✓ PyYAML 可用")
    except ImportError:
        print("  ✗ PyYAML 未安装，请运行: pip install PyYAML")
        return False
    
    return True


def main():
    """主函数"""
    # 检查依赖
    if not check_dependencies():
        sys.exit(1)
    
    # 输入输出目录（根据你的实际路径修改）
    INPUT_DIR = '/mnt/wang.liu/mnt/datasets'  # 修改为你的实际输入目录
    OUTPUT_DIR = '/mnt/wang.liu/mnt/datasets_hdf5'  # 修改为你的实际输出目录
    
    # 检查输入目录
    if not Path(INPUT_DIR).exists():
        print(f"✗ 错误: 输入目录不存在: {INPUT_DIR}")
        sys.exit(1)
    
    # 创建转换器
    converter = Rosbag2Hdf5Converter(INPUT_DIR, OUTPUT_DIR, target_fps=50)
    
    # 批量转换
    converter.convert_all()


if __name__ == '__main__':
    main()