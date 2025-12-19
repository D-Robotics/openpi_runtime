import h5py
import numpy as np
import json
from pathlib import Path
import argparse
from typing import Dict, List, Tuple, Optional, Any
import io
from PIL import Image
import cv2
import logging
from datetime import datetime
import warnings

# 设置日志
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

class DexArtDataExtractor:
    """
    DexArt HDF5数据提取器
    提取和转换episode_1.hdf5格式的所有数据
    """
    
    def __init__(self, hdf5_path: str, output_dir: str = "extracted_data"):
        """
        初始化提取器
        
        参数:
            hdf5_path: HDF5文件路径
            output_dir: 输出目录
        """
        self.hdf5_path = Path(hdf5_path)
        self.output_dir = Path(output_dir)
        self.h5file = None
        
        # 创建输出目录
        self.output_dir.mkdir(parents=True, exist_ok=True)
        
        # 子目录结构
        self.subdirs = {
            "images": self.output_dir / "images",
            "numpy": self.output_dir / "numpy",
            "videos": self.output_dir / "videos",
            "json": self.output_dir / "json"
        }
        
        for subdir in self.subdirs.values():
            subdir.mkdir(exist_ok=True)
    
    def open_hdf5(self) -> None:
        """打开HDF5文件"""
        if not self.hdf5_path.exists():
            raise FileNotFoundError(f"HDF5文件不存在: {self.hdf5_path}")
        
        try:
            self.h5file = h5py.File(self.hdf5_path, 'r')
            logger.info(f"成功打开HDF5文件: {self.hdf5_path}")
        except Exception as e:
            raise IOError(f"无法打开HDF5文件: {e}")
    
    def close_hdf5(self) -> None:
        """关闭HDF5文件"""
        if self.h5file:
            self.h5file.close()
            logger.info("HDF5文件已关闭")
    
    def extract_action_data(self) -> Dict[str, Any]:
        """提取action数据"""
        logger.info("开始提取action数据...")
        
        if "action" not in self.h5file:
            raise ValueError("未找到action数据集")
        
        dataset = self.h5file["action"]
        action_data = dataset[:]  # 形状: (143, 14)
        
        # 保存为numpy文件
        np.save(self.subdirs["numpy"] / "action.npy", action_data)
        
        # 保存为CSV（便于查看）
        np.savetxt(self.subdirs["numpy"] / "action.csv", action_data, delimiter=",", fmt="%.6f")
        
        # 提取统计信息
        stats = {
            "shape": action_data.shape,
            "dtype": str(action_data.dtype),
            "range": {
                "min": float(np.min(action_data)),
                "max": float(np.max(action_data)),
                "mean": float(np.mean(action_data)),
                "std": float(np.std(action_data))
            },
            "dimension_names": [
                "left_arm_joint_0", "left_arm_joint_1", "left_arm_joint_2",
                "left_arm_joint_3", "left_arm_joint_4", "left_arm_joint_5",
                "right_arm_joint_0", "right_arm_joint_1", "right_arm_joint_2",
                "right_arm_joint_3", "right_arm_joint_4", "right_arm_joint_5",
                "left_gripper", "right_gripper"
            ]
        }
        
        logger.info(f"action数据提取完成: {action_data.shape}")
        return {
            "data": action_data,
            "stats": stats,
            "file_paths": [
                str(self.subdirs["numpy"] / "action.npy"),
                str(self.subdirs["numpy"] / "action.csv")
            ]
        }
    
    def extract_qpos_data(self) -> Dict[str, Any]:
        """提取qpos（关节位置）数据"""
        logger.info("开始提取qpos数据...")
        
        if "observations/qpos" not in self.h5file:
            raise ValueError("未找到observations/qpos数据集")
        
        dataset = self.h5file["observations/qpos"]
        qpos_data = dataset[:]  # 形状: (143, 14)
        
        # 分离各部分数据
        left_arm_data = qpos_data[:, :6]    # 左臂关节位置
        right_arm_data = qpos_data[:, 6:12]  # 右臂关节位置
        left_gripper_data = qpos_data[:, 12:13]  # 左夹爪位置
        right_gripper_data = qpos_data[:, 13:]   # 右夹爪位置
        
        # 保存所有数据
        np.save(self.subdirs["numpy"] / "qpos.npy", qpos_data)
        np.save(self.subdirs["numpy"] / "qpos_left_arm.npy", left_arm_data)
        np.save(self.subdirs["numpy"] / "qpos_right_arm.npy", right_arm_data)
        np.save(self.subdirs["numpy"] / "qpos_left_gripper.npy", left_gripper_data)
        np.save(self.subdirs["numpy"] / "qpos_right_gripper.npy", right_gripper_data)
        
        # 保存为CSV
        np.savetxt(self.subdirs["numpy"] / "qpos.csv", qpos_data, delimiter=",", fmt="%.6f")
        np.savetxt(self.subdirs["numpy"] / "qpos_left_arm.csv", left_arm_data, delimiter=",", fmt="%.6f")
        np.savetxt(self.subdirs["numpy"] / "qpos_left_gripper.csv", left_gripper_data, delimiter=",", fmt="%.6f")
        
        # 提取统计信息
        stats = {
            "full_data": {
                "shape": qpos_data.shape,
                "dtype": str(qpos_data.dtype)
            },
            "left_arm": {
                "shape": left_arm_data.shape,
                "range": {
                    "min": float(np.min(left_arm_data)),
                    "max": float(np.max(left_arm_data)),
                    "mean": float(np.mean(left_arm_data)),
                    "std": float(np.std(left_arm_data))
                },
                "joint_names": [
                    "left_shoulder_pan_joint", "left_shoulder_lift_joint",
                    "left_elbow_joint", "left_wrist_1_joint",
                    "left_wrist_2_joint", "left_wrist_3_joint"
                ]
            },
            "left_gripper": {
                "shape": left_gripper_data.shape,
                "range": {
                    "min": float(np.min(left_gripper_data)),
                    "max": float(np.max(left_gripper_data)),
                    "mean": float(np.mean(left_gripper_data)),
                    "std": float(np.std(left_gripper_data))
                }
            }
        }
        
        logger.info(f"qpos数据提取完成: {qpos_data.shape}")
        logger.info(f"左臂关节范围: [{stats['left_arm']['range']['min']:.3f}, {stats['left_arm']['range']['max']:.3f}]")
        
        return {
            "data": {
                "full": qpos_data,
                "left_arm": left_arm_data,
                "right_arm": right_arm_data,
                "left_gripper": left_gripper_data,
                "right_gripper": right_gripper_data
            },
            "stats": stats
        }
    
    def decode_image_bytes(self, byte_data: bytes) -> np.ndarray:
        """
        解码图像字节数据
        
        参数:
            byte_data: 图像字节数据
            
        返回:
            解码后的图像数组 (RGB格式)
        """
        try:
            # 使用PIL解码
            image = Image.open(io.BytesIO(byte_data))
            
            # 转换为RGB（如果必要）
            if image.mode != 'RGB':
                image = image.convert('RGB')
            
            img_array = np.array(image)
            return img_array
        except Exception as e:
            try:
                # 如果PIL失败，尝试OpenCV
                img_array = cv2.imdecode(np.frombuffer(byte_data, np.uint8), cv2.IMREAD_COLOR)
                if img_array is not None:
                    # OpenCV默认是BGR，转换为RGB
                    img_array = cv2.cvtColor(img_array, cv2.COLOR_BGR2RGB)
                    return img_array
            except Exception as e2:
                raise ValueError(f"图像解码失败: PIL错误={e}, OpenCV错误={e2}")
    
    def extract_camera_images(self, camera_name: str) -> Dict[str, Any]:
        """
        提取特定摄像头的图像数据
        
        参数:
            camera_name: 摄像头名称 ('cam_high', 'cam_left_wrist', 'cam_right_wrist')
            
        返回:
            提取结果字典
        """
        logger.info(f"开始提取{camera_name}图像数据...")
        
        dataset_path = f"observations/images/{camera_name}"
        if dataset_path not in self.h5file:
            raise ValueError(f"未找到数据集: {dataset_path}")
        
        dataset = self.h5file[dataset_path]
        
        # 创建摄像头专属目录
        camera_dir = self.subdirs["images"] / camera_name
        camera_dir.mkdir(exist_ok=True)
        
        # 解码和保存所有图像
        all_images = []
        image_stats = {
            "total_frames": dataset.shape[0],
            "successful_decodes": 0,
            "failed_decodes": 0,
            "shapes": [],
            "file_paths": []
        }
        
        for i in range(dataset.shape[0]):
            try:
                # 获取字节数据
                byte_data = dataset[i]
                if not isinstance(byte_data, bytes):
                    byte_data = byte_data.tobytes()
                
                # 解码图像
                img_array = self.decode_image_bytes(byte_data)
                
                # 保存为PNG
                img = Image.fromarray(img_array)
                image_path = camera_dir / f"frame_{i:04d}.png"
                img.save(image_path)
                
                all_images.append(img_array)
                image_stats["successful_decodes"] += 1
                image_stats["shapes"].append(img_array.shape)
                image_stats["file_paths"].append(str(image_path))
                
                if (i + 1) % 20 == 0:
                    logger.info(f"  {camera_name}: 已解码 {i+1}/{dataset.shape[0]} 帧")
                    
            except Exception as e:
                logger.warning(f"  {camera_name} 帧 {i} 解码失败: {e}")
                image_stats["failed_decodes"] += 1
                all_images.append(None)  # 保留位置
        
        # 保存为numpy数组（仅成功的帧）
        successful_images = [img for img in all_images if img is not None]
        if successful_images:
            images_array = np.stack(successful_images)
            np.save(self.subdirs["numpy"] / f"{camera_name}_images.npy", images_array)
            image_stats["numpy_file"] = str(self.subdirs["numpy"] / f"{camera_name}_images.npy")
            image_stats["numpy_shape"] = images_array.shape
        
        # 创建视频文件
        if successful_images:
            self.create_video(successful_images, camera_name, fps=30)
            image_stats["video_file"] = str(self.subdirs["videos"] / f"{camera_name}.mp4")
        
        # 统计信息
        if image_stats["successful_decodes"] > 0:
            first_shape = image_stats["shapes"][0]
            image_stats["resolution"] = f"{first_shape[1]}x{first_shape[0]}"
            image_stats["channels"] = first_shape[2] if len(first_shape) > 2 else 1
            image_stats["is_rgb"] = (image_stats["channels"] == 3)
        
        logger.info(f"{camera_name}图像提取完成: {image_stats['successful_decodes']}/{image_stats['total_frames']} 帧成功")
        
        return {
            "camera_name": camera_name,
            "all_images": all_images,  # 包含None的列表
            "successful_images": successful_images,
            "stats": image_stats
        }
    
    def create_video(self, images: List[np.ndarray], camera_name: str, fps: int = 30) -> None:
        """
        从图像列表创建视频
        
        参数:
            images: 图像数组列表
            camera_name: 摄像头名称
            fps: 帧率
        """
        if not images:
            logger.warning(f"没有图像数据，无法创建{camera_name}视频")
            return
        
        # 获取视频尺寸
        height, width = images[0].shape[:2]
        
        # 创建视频写入器
        video_path = self.subdirs["videos"] / f"{camera_name}.mp4"
        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        out = cv2.VideoWriter(str(video_path), fourcc, fps, (width, height))
        
        try:
            for i, img in enumerate(images):
                # 转换为BGR（OpenCV格式）
                if len(img.shape) == 3 and img.shape[2] == 3:
                    img_bgr = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
                else:
                    # 如果是灰度图，转换为3通道
                    if len(img.shape) == 2:
                        img = cv2.cvtColor(img, cv2.COLOR_GRAY2RGB)
                    img_bgr = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
                
                out.write(img_bgr)
                
                if (i + 1) % 20 == 0:
                    logger.info(f"  正在写入视频: {i+1}/{len(images)} 帧")
            
            out.release()
            logger.info(f"视频创建完成: {video_path}")
            
        except Exception as e:
            logger.error(f"创建视频失败: {e}")
            if out.isOpened():
                out.release()
    
    def extract_arm_dim_data(self) -> Dict[str, Any]:
        """提取左右臂维度数据"""
        logger.info("开始提取臂维度数据...")
        
        arm_data = {}
        
        for arm_name in ["left_arm_dim", "right_arm_dim"]:
            dataset_path = f"observations/{arm_name}"
            if dataset_path in self.h5file:
                dataset = self.h5file[dataset_path]
                data = dataset[:]
                
                # 保存数据
                np.save(self.subdirs["numpy"] / f"{arm_name}.npy", data)
                np.savetxt(self.subdirs["numpy"] / f"{arm_name}.csv", data, delimiter=",", fmt="%d")
                
                arm_data[arm_name] = {
                    "data": data,
                    "shape": data.shape,
                    "dtype": str(data.dtype),
                    "unique_values": np.unique(data).tolist(),
                    "file_paths": [
                        str(self.subdirs["numpy"] / f"{arm_name}.npy"),
                        str(self.subdirs["numpy"] / f"{arm_name}.csv")
                    ]
                }
                
                logger.info(f"{arm_name}: 形状={data.shape}, 唯一值={np.unique(data)}")
            else:
                logger.warning(f"未找到数据集: {dataset_path}")
        
        return arm_data
    
    def create_data_alignment_report(self, extracted_data: Dict[str, Any]) -> Dict[str, Any]:
        """创建数据对齐报告"""
        logger.info("创建数据对齐报告...")
        
        report = {
            "extraction_time": datetime.now().isoformat(),
            "source_file": str(self.hdf5_path),
            "output_directory": str(self.output_dir),
            "data_alignment": {},
            "statistics": {}
        }
        
        # 检查时间步对齐
        timesteps = {}
        
        # action数据
        if "action" in extracted_data:
            action_data = extracted_data["action"]["data"]
            timesteps["action"] = action_data.shape[0]
            report["statistics"]["action"] = extracted_data["action"]["stats"]
        
        # qpos数据
        if "qpos" in extracted_data:
            qpos_data = extracted_data["qpos"]["data"]["full"]
            timesteps["qpos"] = qpos_data.shape[0]
            report["statistics"]["qpos"] = extracted_data["qpos"]["stats"]
        
        # 图像数据
        for cam_name in ["cam_high", "cam_left_wrist", "cam_right_wrist"]:
            if cam_name in extracted_data:
                cam_data = extracted_data[cam_name]
                timesteps[cam_name] = cam_data["stats"]["total_frames"]
                report["statistics"][cam_name] = cam_data["stats"]
        
        # 检查是否所有时间序列长度一致
        if timesteps:
            all_timesteps = list(timesteps.values())
            is_aligned = all(t == all_timesteps[0] for t in all_timesteps)
            
            report["data_alignment"] = {
                "is_aligned": is_aligned,
                "timesteps": timesteps,
                "expected_timesteps": 143
            }
            
            if is_aligned:
                logger.info(f"✅ 所有数据时间步对齐: {all_timesteps[0]} 步")
            else:
                logger.warning(f"❌ 数据时间步未对齐: {timesteps}")
        
        # 保存报告
        report_path = self.subdirs["json"] / "extraction_report.json"
        with open(report_path, 'w', encoding='utf-8') as f:
            json.dump(report, f, indent=2, ensure_ascii=False)
        
        logger.info(f"数据对齐报告保存到: {report_path}")
        
        return report
    
    def create_readme_file(self, extraction_report: Dict[str, Any]) -> None:
        """创建README文件说明数据结构"""
        readme_path = self.output_dir / "README.md"
        
        readme_content = f"""# DexArt 数据提取结果

# ## 基本信息
# - **源文件**: {extraction_report['source_file']}
# - **提取时间**: {extraction_report['extraction_time']}
# - **输出目录**: {extraction_report['output_directory']}

# ## 目录结构

## 注意事项
1. 所有时间序列数据都已对齐（143个时间步）
2. 图像已从字节串解码为RGB格式
3. 数据范围已在报告中记录
4. 如果图像解码失败，对应位置为None

---
*本文件由DexArt数据提取器自动生成*
"""
        
        with open(readme_path, 'w', encoding='utf-8') as f:
            f.write(readme_content)
        
        logger.info(f"README文件创建完成: {readme_path}")
    
    def extract_all_data(self) -> Dict[str, Any]:
        """提取所有数据"""
        logger.info("=" * 60)
        logger.info("开始提取DexArt HDF5文件所有数据")
        logger.info("=" * 60)
        
        try:
            self.open_hdf5()
            
            extracted_data = {}
            
            # 1. 提取action数据
            extracted_data["action"] = self.extract_action_data()
            
            # 2. 提取qpos数据
            extracted_data["qpos"] = self.extract_qpos_data()
            
            # 3. 提取摄像头图像数据
            for camera_name in ["cam_high", "cam_left_wrist", "cam_right_wrist"]:
                try:
                    extracted_data[camera_name] = self.extract_camera_images(camera_name)
                except Exception as e:
                    logger.error(f"提取{camera_name}失败: {e}")
                    extracted_data[camera_name] = {"error": str(e)}
            
            # 4. 提取臂维度数据
            extracted_data["arm_dim"] = self.extract_arm_dim_data()
            
            # 5. 创建数据对齐报告
            extraction_report = self.create_data_alignment_report(extracted_data)
            
            # 6. 创建README文件
            self.create_readme_file(extraction_report)
            
            # 7. 打印总结
            self.print_extraction_summary(extracted_data, extraction_report)
            
            return {
                "extracted_data": extracted_data,
                "report": extraction_report,
                "output_dir": str(self.output_dir)
            }
            
        except Exception as e:
            logger.error(f"数据提取失败: {e}")
            raise
        finally:
            self.close_hdf5()
    
    def print_extraction_summary(self, extracted_data: Dict[str, Any], report: Dict[str, Any]) -> None:
        """打印提取总结"""
        logger.info("=" * 60)
        logger.info("🎉 数据提取完成!")
        logger.info("=" * 60)
        
        print(f"\n📁 输出目录: {self.output_dir}")
        print(f"📊 总文件数: 约 {len(list(self.output_dir.rglob('*')))} 个文件")
        
        if "action" in extracted_data:
            action_data = extracted_data["action"]["data"]
            print(f"\n🤖 动作数据:")
            print(f"  • 形状: {action_data.shape}")
            print(f"  • 文件: {self.subdirs['numpy'] / 'action.npy'}")
        
        if "qpos" in extracted_data:
            qpos_data = extracted_data["qpos"]["data"]["full"]
            left_arm_data = extracted_data["qpos"]["data"]["left_arm"]
            print(f"\n🦾 关节位置数据:")
            print(f"  • 完整qpos: {qpos_data.shape}")
            print(f"  • 左臂关节: {left_arm_data.shape}")
            print(f"  • 左夹爪: {extracted_data['qpos']['data']['left_gripper'].shape}")
        
        for cam_name in ["cam_high", "cam_left_wrist", "cam_right_wrist"]:
            if cam_name in extracted_data and "stats" in extracted_data[cam_name]:
                stats = extracted_data[cam_name]["stats"]
                print(f"\n📸 {cam_name}图像:")
                print(f"  • 总帧数: {stats['total_frames']}")
                print(f"  • 成功解码: {stats['successful_decodes']}")
                print(f"  • 分辨率: {stats.get('resolution', 'N/A')}")
                print(f"  • 图像目录: {self.subdirs['images'] / cam_name}")
        
        print(f"\n📄 报告文件:")
        print(f"  • extraction_report.json: {self.subdirs['json'] / 'extraction_report.json'}")
        print(f"  • README.md: {self.output_dir / 'README.md'}")
        
        if report.get("data_alignment", {}).get("is_aligned", False):
            print(f"\n✅ 数据对齐: 所有 {report['data_alignment']['timesteps'].get('action', 'N/A')} 个时间步已对齐")
        else:
            print(f"\n⚠️  数据未完全对齐")


def main():
    """主函数"""
    parser = argparse.ArgumentParser(description='提取DexArt HDF5文件所有数据')
    parser.add_argument('input', type=str, help='输入HDF5文件路径')
    parser.add_argument('-o', '--output', type=str, default='extracted_data', 
                       help='输出目录 (默认: extracted_data)')
    parser.add_argument('-v', '--verbose', action='store_true', help='显示详细信息')
    
    args = parser.parse_args()
    
    # 设置日志级别
    if args.verbose:
        logging.getLogger().setLevel(logging.DEBUG)
    
    try:
        # 创建提取器
        extractor = DexArtDataExtractor(args.input, args.output)
        
        # 执行提取
        result = extractor.extract_all_data()
        
        print(f"\n🎉 提取完成! 数据已保存到: {result['output_dir']}")
        
    except Exception as e:
        print(f"\n❌ 提取失败: {e}")
        import traceback
        traceback.print_exc()


if __name__ == "__main__":
    main()