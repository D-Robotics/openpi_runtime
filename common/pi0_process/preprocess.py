import os
import numpy as np
import cv2
import json
from einops import rearrange
from transformers import AutoTokenizer

# 加载tokenizer
try:
    tokenizer = AutoTokenizer.from_pretrained(
        os.path.dirname(os.path.abspath(__file__)),
        trust_remote_code=True
    )
except Exception as e:
    # 如果加载失败，尝试使用默认的bert-base-uncased，避免程序直接崩溃
    print(f"Warning: Failed to load local tokenizer: {e}")
    print("Trying to load 'bert-base-uncased' from huggingface...")
    tokenizer = AutoTokenizer.from_pretrained("bert-base-uncased")

def _joint_flip_mask() -> np.ndarray:
    return np.array([1, -1, -1, 1, 1, 1, 1, 1, -1, -1, 1, 1, 1, 1])

def _normalize(x, min_val, max_val):
    return (x - min_val) / (max_val - min_val)

def _unnormalize(x, min_val, max_val):
    return x * (max_val - min_val) + min_val

def _gripper_to_angular(value):
    value = _unnormalize(value, min_val=0.01844, max_val=0.05800)
    
    def linear_to_radian(linear_position, arm_length, horn_radius):
        value = (horn_radius**2 + linear_position**2 - arm_length**2) / (2 * horn_radius * linear_position)
        return np.arcsin(np.clip(value, -1.0, 1.0))
    
    value = linear_to_radian(value, arm_length=0.036, horn_radius=0.022)
    return _normalize(value, min_val=0.5476, max_val=1.6296)

def load_norm_stats(norm_stats_path):
    """加载归一化统计信息"""
    with open(norm_stats_path, 'r') as f:
        data = json.load(f)
    return data["norm_stats"]

def normalize_data(x, stats):
    """使用提供的统计信息归一化数据"""
    mean = np.array(stats["mean"])
    std = np.array(stats["std"])
    return (x - mean) / (std + 1e-6)

def _decode_state(state: np.ndarray, *, adapt_to_pi: bool = False) -> np.ndarray:
    if adapt_to_pi:
        state = _joint_flip_mask() * state
        state[[6, 13]] = _gripper_to_angular(state[[6, 13]])
    return state

def preprocess(images, state=None, prompt="do something", adapt_to_pi=True, save_result=False, output_dir=None, norm_stats=None):
    """前处理函数
    
    参数:
        images: dict[str, np.ndarray] or list[np.ndarray] - 摄像头图片数据
            字典格式: 键为摄像头名称，值为OpenCV格式图片
                摄像头名称必须是: cam_high, cam_low, cam_left_wrist, cam_right_wrist
            列表格式: 按顺序提供图片，支持3张或4张图片
                4张图片顺序: [上, 左, 右, 下] 对应 [cam_high, cam_left_wrist, cam_right_wrist, cam_low]
                3张图片顺序: [上, 左, 右] 对应 [cam_high, cam_left_wrist, cam_right_wrist]
            OpenCV格式图片: np.ndarray，形状为 (H, W, C)，颜色空间为 BGR，数据类型为 uint8
        state: np.ndarray - 14维状态数据 (可选)
        prompt: str - 任务提示词 (可选)
        adapt_to_pi: bool - 是否将数据转换为Pi内部运行时使用的空间格式
        save_result: bool - 是否保存处理结果
        output_dir: str - 保存结果的目录路径 (仅当save_result=True时需要)
        norm_stats: dict - 归一化统计信息 (可选)
            如果提供，将对状态数据进行归一化处理
    
    返回:
        processed_data: dict - 处理后的数据
    """
    # 转换列表格式为字典格式
    if isinstance(images, list):
        img_count = len(images)
        if img_count not in [3, 4]:
            raise ValueError(f"Expected 3 or 4 images in list format, got {img_count}")
        
        # 按照顺序分配摄像头名称
        if img_count == 4:
            # 上、左、右、下
            cam_order = ["cam_high", "cam_left_wrist", "cam_right_wrist", "cam_low"]
        else:  # 3张图片
            # 上、左、右
            cam_order = ["cam_high", "cam_left_wrist", "cam_right_wrist"]
        
        images_dict = {}
        for i, img in enumerate(images):
            images_dict[cam_order[i]] = img
        
        images = images_dict
    
    # 验证图片数据格式
    expected_cams = ["cam_high", "cam_low", "cam_left_wrist", "cam_right_wrist"]
    for cam_name, img in images.items():
        if cam_name not in expected_cams:
            raise ValueError(f"Unexpected camera name: {cam_name}, expected one of {expected_cams}")
        
        if not isinstance(img, np.ndarray):
            raise TypeError(f"Image for {cam_name} must be numpy array, got {type(img)}")
            
        if len(img.shape) != 3 or img.shape[2] != 3:
            raise ValueError(f"Image for {cam_name} must be (H, W, C) BGR format, got shape {img.shape}")
            
        if img.dtype != np.uint8:
            raise TypeError(f"Image for {cam_name} must be uint8 dtype, got {img.dtype}")
    
    # 验证状态数据
    if state is not None:
        if not isinstance(state, np.ndarray):
            raise TypeError(f"State must be numpy array, got {type(state)}")
        if state.shape != (14,):
            raise ValueError(f"State must be 14-dimensional, got shape {state.shape}")
    
    processed_data = {}
    
    # 处理图片
    processed_images = {}
    for cam_name, img in images.items():
        # 转换颜色空间 BGR -> RGB
        img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        # 调整大小
        img_resized = cv2.resize(img_rgb, (224, 224))
        # 将像素值从 0-255 缩放到 0-1 范围
        img_normalized = img_resized.astype(np.float32) / 255.0
        # 转换为 CHW 格式
        img_chw = rearrange(img_normalized, "h w c -> c h w")
        processed_images[cam_name] = img_chw
    
    # 相机映射
    mapped_images = {}
    mapped_image_masks = {}
    
    # 基本图像处理 (cam_high -> base_0_rgb)
    if "cam_high" in processed_images:
        mapped_images["base_0_rgb"] = processed_images["cam_high"]
        mapped_image_masks["base_0_rgb"] = np.ones((3, 224, 224), dtype=np.float32)
    else:
        raise ValueError("Missing required image: cam_high")
    
    # 处理手腕图像
    wrist_images = {
        "left_wrist_0_rgb": "cam_left_wrist",
        "right_wrist_0_rgb": "cam_right_wrist"
    }
    
    for dest, source in wrist_images.items():
        if source in processed_images:
            mapped_images[dest] = processed_images[source]
            mapped_image_masks[dest] = np.ones((3, 224, 224), dtype=np.float32)
        else:
            # 如果缺少手腕图像，使用黑色图像填充
            mapped_images[dest] = np.zeros_like(mapped_images["base_0_rgb"])
            mapped_image_masks[dest] = np.zeros((3, 224, 224), dtype=np.float32)
    
    # 处理低视角图像 (cam_low -> base_1_rgb)
    if "cam_low" in processed_images:
        mapped_images["base_1_rgb"] = processed_images["cam_low"]
        mapped_image_masks["base_1_rgb"] = np.ones((3, 224, 224), dtype=np.float32)
    
    processed_data["image"] = mapped_images
    processed_data["image_mask"] = mapped_image_masks
    
    # 处理状态数据
    if state is not None:
        decoded_state = _decode_state(state, adapt_to_pi=adapt_to_pi)
    else:
        decoded_state = np.zeros(14, dtype=np.float32)
    
    # 将状态从14维填充到32维，确保使用float32类型
    state_padded = np.zeros(32, dtype=np.float32)
    state_padded[:14] = decoded_state.astype(np.float32)
    
    # 对状态数据进行归一化
    if norm_stats is not None:
        # 处理不同的norm_stats结构
        if "norm_stats" in norm_stats and "state" in norm_stats["norm_stats"]:
            processed_data["state"] = normalize_data(state_padded, norm_stats["norm_stats"]["state"]).astype(np.float32)
        elif "state" in norm_stats:
            processed_data["state"] = normalize_data(state_padded, norm_stats["state"]).astype(np.float32)
        else:
            processed_data["state"] = state_padded.astype(np.float32)
    else:
        processed_data["state"] = state_padded.astype(np.float32)
    
    # 对图像进行归一化
    if norm_stats is not None:
        # 使用简单的线性映射将像素值从0-1范围直接映射到-1到1范围
        for cam_name, img in processed_data["image"].items():
            # 线性映射: (img - 0.5) * 2
            img_normalized = (img - 0.5) * 2
            processed_data["image"][cam_name] = img_normalized
    
    # 添加提示词
    processed_data["prompt"] = prompt
    
    # 添加tokenized_prompt
    if prompt is not None:
        # 保存原始padding方向
        original_padding_side = tokenizer.padding_side
        
        # 设置为右padding
        tokenizer.padding_side = "right"
        
        # 将字符串提示词转换为token_ids
        tokenization = tokenizer(
            prompt,
            return_tensors="np",
            truncation=True,
            max_length=48,
            padding="max_length"
        )
        
        # 恢复原始padding方向
        tokenizer.padding_side = original_padding_side
        
        token_ids = tokenization.input_ids[0].astype(np.int64)
        processed_data["tokenized_prompt"] = token_ids
        
        # 添加tokenized_prompt_mask
        attention_mask = tokenization.attention_mask[0].astype(np.int64)
        processed_data["tokenized_prompt_mask"] = attention_mask
    
    # 保存处理结果
    if save_result:
        if output_dir is None:
            raise ValueError("output_dir must be provided when save_result=True")
        
        os.makedirs(output_dir, exist_ok=True)
        output_path = os.path.join(output_dir, "preprocessed_data.npz")
        
        # 准备保存的数据
        save_data = {}
        
        # 添加状态和token数据
        if "state" in processed_data:
            save_data["state"] = processed_data["state"]
        if "tokenized_prompt" in processed_data:
            save_data["tokenized_prompt"] = processed_data["tokenized_prompt"]
        if "tokenized_prompt_mask" in processed_data:
            save_data["tokenized_prompt_mask"] = processed_data["tokenized_prompt_mask"]
        
        # 添加图像数据
        if "image" in processed_data:
            for name, img in processed_data["image"].items():
                save_data[f"image_{name}"] = img
        
        # 添加图像掩码数据
        if "image_mask" in processed_data:
            for name, mask in processed_data["image_mask"].items():
                save_data[f"image_mask_{name}"] = mask
        
        np.savez_compressed(output_path, **save_data)
        
        print(f"Preprocessing completed. Result saved to {output_path}")
    
    return processed_data

def preprocess_from_dir(input_dir, output_dir, adapt_to_pi=True):
    """从目录读取数据并进行前处理（兼容旧API）
    
    参数:
        input_dir: str - 输入数据目录路径
        output_dir: str - 输出结果目录路径
        adapt_to_pi: bool - 是否将数据转换为Pi内部运行时使用的空间格式
    
    返回:
        processed_data: dict - 处理后的数据
    """
    # 读取图片
    cameras = {
        "hight.jpg": "cam_high",
        "low.jpg": "cam_low", 
        "left.jpg": "cam_left_wrist",
        "right.jpg": "cam_right_wrist"
    }
    
    images = {}
    for filename, cam_name in cameras.items():
        img_path = os.path.join(input_dir, filename)
        if os.path.exists(img_path):
            img = cv2.imread(img_path)
            images[cam_name] = img
        else:
            print(f"Warning: {img_path} not found")
    
    # 读取状态数据
    state = None
    state_path = os.path.join(input_dir, "action.txt")
    if os.path.exists(state_path):
        with open(state_path, "r") as f:
            state_data = f.read().strip()
            state = np.array([float(x) for x in state_data.split()])
            if len(state) != 14:
                print(f"Warning: state data has {len(state)} dimensions, expected 14")
                state = None
    
    # 调用主前处理函数
    return preprocess(images, state, adapt_to_pi=adapt_to_pi, save_result=True, output_dir=output_dir)

if __name__ == "__main__":
    input_dir = "data/input"
    output_dir = "data/pre_output"
    preprocess_from_dir(input_dir, output_dir, adapt_to_pi=True)
