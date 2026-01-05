import os
import re
import sys
import argparse
from pathlib import Path
from typing import List, Optional, Tuple
from datetime import datetime


class HDF5BatchRenamer:
    """HDF5文件批量重命名工具"""
    
    def __init__(self, verbose: bool = True, dry_run: bool = False):
        """
        初始化重命名器
        
        Args:
            verbose: 是否显示详细信息
            dry_run: 是否模拟运行（不实际重命名）
        """
        self.verbose = verbose
        self.dry_run = dry_run
        self.pattern = re.compile(r"episode_(\d+)\.hdf5")
        
    def validate_path(self, path: str) -> Path:
        """验证路径是否存在"""
        path_obj = Path(path)
        if not path_obj.exists():
            raise ValueError(f"路径不存在: {path}")
        if not path_obj.is_dir():
            raise ValueError(f"不是文件夹: {path}")
        return path_obj
    
    def get_sorted_files(self, folder: Path, extension: str = ".hdf5") -> List[Path]:
        """获取排序后的文件列表"""
        files = list(folder.glob(f"*{extension}"))
        
        # 按文件创建时间排序
        files.sort(key=lambda x: x.stat().st_ctime)
        
        if self.verbose:
            print(f"找到 {len(files)} 个{extension}文件")
            
        return files
    
    def find_available_indices(self, folder: Path, 
                               start: int = 0, 
                               count: int = None) -> List[int]:
        """查找可用的索引编号"""
        existing_indices = set()
        
        for file in folder.iterdir():
            if file.is_file() and file.suffix == ".hdf5":
                match = self.pattern.match(file.name)
                if match:
                    existing_indices.add(int(match.group(1)))
        
        # 生成连续索引
        max_index = max(existing_indices) if existing_indices else 0
        if count is None:
            # 生成足够多的索引
            available = [i for i in range(start, max_index + count + 1) 
                        if i not in existing_indices]
        else:
            # 生成指定数量的索引
            available = []
            i = start
            while len(available) < count:
                if i not in existing_indices:
                    available.append(i)
                i += 1
                
        return available
    
    def rename_single_file(self, old_path: Path, new_path: Path) -> bool:
        """重命名单个文件"""
        if self.dry_run:
            print(f"[模拟] 重命名: {old_path.name} -> {new_path.name}")
            return True
            
        try:
            old_path.rename(new_path)
            if self.verbose:
                print(f"✓ 重命名: {old_path.name} -> {new_path.name}")
            return True
        except Exception as e:
            print(f"✗ 重命名失败 {old_path.name}: {e}")
            return False
    
    def batch_rename(self, 
                    source_folder: str,
                    start_index: int = 0,
                    prefix: str = "episode_",
                    suffix: str = ".hdf5",
                    padding: int = 0) -> Tuple[int, int]:
        """
        批量重命名文件
        
        Args:
            source_folder: 源文件夹路径
            start_index: 起始索引
            prefix: 文件名前缀
            suffix: 文件名后缀
            padding: 数字填充位数（0表示不填充）
            
        Returns:
            成功和失败的数量
        """
        folder = self.validate_path(source_folder)
        files = self.get_sorted_files(folder)
        
        if not files:
            print("没有找到需要重命名的文件")
            return 0, 0
        
        success = 0
        failed = 0
        
        print(f"开始处理文件夹: {folder}")
        print(f"文件数量: {len(files)}")
        print(f"起始索引: {start_index}")
        print("-" * 60)
        
        for i, file in enumerate(files):
            # 生成新文件名
            index = start_index + i
            if padding > 0:
                index_str = f"{index:0{padding}d}"
            else:
                index_str = str(index)
                
            new_name = f"{prefix}{index_str}{suffix}"
            new_path = file.parent / new_name
            
            # 检查是否与旧文件名相同
            if file.name == new_name:
                if self.verbose:
                    print(f"○ 跳过: {file.name} (名称已正确)")
                continue
                
            # 检查目标文件是否已存在
            if new_path.exists():
                print(f"✗ 跳过: {file.name} -> {new_name} (目标文件已存在)")
                failed += 1
                continue
            
            # 执行重命名
            if self.rename_single_file(file, new_path):
                success += 1
            else:
                failed += 1
        
        print("-" * 60)
        print(f"完成! 成功: {success}, 失败: {failed}")
        
        return success, failed
    
    def cleanup_empty_names(self, folder: str) -> int:
        """清理空文件名（如.hdf5文件）"""
        folder_path = self.validate_path(folder)
        cleaned = 0
        
        for file in folder_path.iterdir():
            if file.is_file() and file.name == ".hdf5":
                timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                new_name = f"unnamed_{timestamp}.hdf5"
                new_path = file.parent / new_name
                
                if self.rename_single_file(file, new_path):
                    cleaned += 1
                    
        if cleaned > 0:
            print(f"清理了 {cleaned} 个空文件名")
            
        return cleaned


def parse_arguments():
    """解析命令行参数"""
    parser = argparse.ArgumentParser(
        description="批量重命名HDF5文件工具",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
示例:
  %(prog)s D:\\project\\data
  %(prog)s D:\\project\\data --start 0 --dry-run
  %(prog)s D:\\project\\data --prefix "data_" --padding 4
        """
    )
    
    parser.add_argument(
        "folder",
        help="要处理的文件夹路径"
    )
    
    parser.add_argument(
        "--start", "-s",
        type=int,
        default=0,
        help="起始索引 (默认: 0)"
    )
    
    parser.add_argument(
        "--prefix", "-p",
        default="episode_",
        help="文件名前缀 (默认: episode_)"
    )
    
    parser.add_argument(
        "--padding", "-w",
        type=int,
        default=0,
        help="数字填充位数，0表示不填充 (默认: 0)"
    )
    
    parser.add_argument(
        "--dry-run", "-n",
        action="store_true",
        help="模拟运行，不实际重命名文件"
    )
    
    parser.add_argument(
        "--verbose", "-v",
        action="store_true",
        help="显示详细信息"
    )
    
    parser.add_argument(
        "--cleanup",
        action="store_true",
        help="清理空文件名"
    )
    
    return parser.parse_args()


def main():
    """主函数"""
    args = parse_arguments()
    
    try:
        # 创建重命名器
        renamer = HDF5BatchRenamer(
            verbose=args.verbose,
            dry_run=args.dry_run
        )
        
        if args.cleanup:
            # 清理空文件名
            renamer.cleanup_empty_names(args.folder)
        
        # 执行批量重命名
        success, failed = renamer.batch_rename(
            source_folder=args.folder,
            start_index=args.start,
            prefix=args.prefix,
            padding=args.padding
        )
        
        if failed > 0:
            sys.exit(1)
            
    except KeyboardInterrupt:
        print("\n操作被用户中断")
        sys.exit(130)
    except Exception as e:
        print(f"错误: {e}")
        sys.exit(1)


if __name__ == "__main__":
    # 简单用法（如果不需要命令行参数）
    renamer = HDF5BatchRenamer()
    renamer.batch_rename(
        r"D:\project\pi0\openpi\datasets\put_orange\datasets_hdf5",
        start_index=0
    )
    
    # 使用命令行参数
    # main()