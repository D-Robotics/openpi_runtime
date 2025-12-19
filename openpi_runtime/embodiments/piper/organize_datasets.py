import os
import shutil
import glob

# =============== 请在这里修改你的路径（按实际填写） ===============
source_hdf5_dir = "./raw_data"  # ✅ 你的 .hdf5 文件所在文件夹（例如：/Users/you/raw_data）
instructions_file = "./instructions.json"  # ✅ 你的 instructions.json 文件路径
# ===============================================================

output_base = "datasets"  # 输出根目录（默认是 datasets，不用改）

# 检查路径是否存在
if not os.path.isdir(source_hdf5_dir):
    raise FileNotFoundError(f"❌ 错误：源文件夹不存在！请检查路径：{source_hdf5_dir}")
if not os.path.isfile(instructions_file):
    raise FileNotFoundError(f"❌ 错误：instructions.json 不存在！请检查路径：{instructions_file}")

# 创建输出目录（如果不存在）
os.makedirs(output_base, exist_ok=True)

# 获取所有 .hdf5 文件（按文件名排序，确保顺序一致）
hdf5_files = glob.glob(os.path.join(source_hdf5_dir, "*.hdf5"))
hdf5_files.sort()  # 排序后，文件按顺序处理（如 file1.hdf5, file2.hdf5 会变成 episode_0, episode_1）

if not hdf5_files:
    print("❌ 没有找到 .hdf5 文件！请检查源文件夹。")
    exit(0)

# 处理每个文件
for idx, file_path in enumerate(hdf5_files):
    # 生成新文件名：episode_0.hdf5, episode_1.hdf5...
    new_filename = f"episode_{idx}.hdf5"
    # 创建对应的 episode 文件夹（如 episode_0）
    episode_dir = os.path.join(output_base, f"episode_{idx}")
    os.makedirs(episode_dir, exist_ok=True)
    
    # 移动文件到新位置
    new_file_path = os.path.join(episode_dir, new_filename)
    shutil.move(file_path, new_file_path)
    
    # 复制 instructions.json 到该文件夹（每个 episode 都有）
    shutil.copy(instructions_file, os.path.join(episode_dir, "instructions.json"))

print(f"✅ 整理完成！共处理 {len(hdf5_files)} 个文件，输出到：{os.path.abspath(output_base)}")
print("📁 结构示例：")
print(f"datasets/")
print(f"└── episode_{idx}/")
print(f"    ├── episode_{idx}.hdf5")
print(f"    └── instructions.json")