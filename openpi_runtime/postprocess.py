import os
import numpy as np

def _make_bool_mask(*dims):
    """创建布尔掩码，用于标记需要处理的维度。
    
    例如: make_bool_mask(6, -1, 6, -1) 对于 14 维数据
    返回: [True, True, True, True, True, True, False, False, False, False, False, False, False, False]
    """
    mask = []
    current_dim = 0
    for dim in dims:
        if dim < 0:
            mask.extend([False] * (14 - current_dim))
        else:
            mask.extend([True] * dim)
            current_dim += dim
    return np.array(mask[:14])

def _unnormalize_actions(actions, mean, std):
    """反归一化动作数据。
    
    参数:
        actions: np.ndarray - 归一化后的动作数据
        mean: np.ndarray - 均值
        std: np.ndarray - 标准差
    
    返回:
        np.ndarray - 反归一化后的动作数据
    """
    # 确保维度匹配
    mean = mean[:actions.shape[-1]]
    std = std[:actions.shape[-1]]
    
    # 应用反归一化公式
    return actions * (std + 1e-6) + mean

def _joint_flip_mask():
    """获取关节角度翻转掩码。
    
    返回:
        np.ndarray - 14维翻转掩码
    """
    return np.array([1, -1, -1, 1, 1, 1, 1, 1, -1, -1, 1, 1, 1, 1])

def _gripper_from_angular(value):
    """将pi0使用的抓取器位置转换为Aloha使用的抓取器位置。
    
    参数:
        value: np.ndarray - pi0使用的抓取器位置
    
    返回:
        np.ndarray - Aloha使用的抓取器位置
    """
    # 我们不缩放输出，因为trossen模型预测已经是弧度制
    # 有关常量的推导，请参见_gripper_to_angular中的注释
    value = value + 0.5476
    
    # 这些值来自Aloha代码：
    # PUPPET_GRIPPER_JOINT_OPEN, PUPPET_GRIPPER_JOINT_CLOSE
    min_val = -0.6213
    max_val = 1.4910
    return (value - min_val) / (max_val - min_val)

def _aloha_outputs(data, adapt_to_pi=False):
    """将输出数据从模型特定格式转换为Aloha机器人可执行的格式。
    
    参数:
        data: np.ndarray - 动作数据，形状为 (..., 14)
        adapt_to_pi: bool - 是否将数据转换为标准Aloha空间
    
    返回:
        np.ndarray - 转换后的动作数据
    """
    # 与 aloha_policy.py 中的 _encode_actions 函数一致
    # 只有在 adapt_to_pi=True 时才进行转换
    if adapt_to_pi:
        data_copy = data.copy()
        # 对关节角度应用翻转掩码
        flip_mask = _joint_flip_mask()
        data_copy = data_copy * flip_mask
        
        # 转换抓取器位置（维度6和13）
        data_copy[..., 6] = _gripper_from_angular(data_copy[..., 6])
        data_copy[..., 13] = _gripper_from_angular(data_copy[..., 13])
        return data_copy
    
    # 如果 adapt_to_pi=False，直接返回原始数据
    return data

def _delta_action_restore(actions, state, mask):
    """Delta 动作还原，将相对动作转换为绝对动作。
    
    参数:
        actions: np.ndarray - 动作数据，形状为 (..., 14)
        state: np.ndarray - 当前状态数据，形状为 (..., 14)
        mask: np.ndarray - 掩码，标记哪些维度需要还原
    
    返回:
        np.ndarray - 还原后的动作数据
    """
    # 使用post.md中的实现方式
    mask = np.asarray(mask)
    
    # 确保输入维度正确
    assert actions.shape[-1] == len(mask), f"动作维度不匹配: {actions.shape[-1]} != {len(mask)}"
    assert state.shape[-1] == len(mask), f"状态维度不匹配: {state.shape[-1]} != {len(mask)}"
    
    for i in range(len(mask)):
        if mask[i]:
            # 每个时间步的动作都加上当前状态
            actions[:, i] += state[i]
    return actions

def postprocess(actions, adapt_to_pi=False, save_result=False, output_dir=None, norm_stats=None, state=None, skip_delta_restore=False):
    """后处理函数（与post.md中的处理流程一致）
    
    参数:
        actions: np.ndarray - 模型输出的动作数据，形状为 (action_horizon, 32) 或 (1, action_horizon, 32)
        adapt_to_pi: bool - 是否将数据从Pi内部运行时空间转换回标准空间格式
            - True: 将动作从Pi内部空间转换为Aloha空间
            - False: 保持原始Aloha空间格式不转换
        save_result: bool - 是否保存处理结果
        output_dir: str - 保存结果的目录路径 (仅当save_result=True时需要)
        norm_stats: dict - 归一化统计信息，包含动作的均值、标准差等
        state: np.ndarray - 当前状态数据，形状为 (14,)，用于绝对动作空间转换
        skip_delta_restore: bool - 是否跳过Delta动作还原步骤
    
    返回:
        processed_actions: np.ndarray - 处理后的动作数据，形状为 (action_horizon, 14)
    """
    if not isinstance(actions, np.ndarray):
        raise TypeError(f"Actions must be numpy array, got {type(actions)}")
    
    # 检查输入形状是否合法
    if actions.ndim < 1:
        raise ValueError(f"Actions must have at least 1 dimension, got {actions.shape}")
    if actions.shape[-1] != 32:
        raise ValueError(f"Actions must have shape (..., 32), got {actions.shape}")
    
    # 处理输入形状，确保是 (action_horizon, 32) 格式
    if actions.ndim == 3 and actions.shape[0] == 1:
        # 处理 (1, action_horizon, 32) 形状的输入
        actions = actions[0]
    
    # 处理一维输入
    if actions.ndim == 1:
        actions = actions.reshape(1, -1)
        was_1d = True
    else:
        was_1d = False

    # 步骤1: 反归一化
    if norm_stats is not None:
        action_mean = np.array(norm_stats["norm_stats"]["actions"]["mean"])
        action_std = np.array(norm_stats["norm_stats"]["actions"]["std"])
        actions = _unnormalize_actions(actions, action_mean, action_std)
    
    # 步骤2: 动作编码 - 提取前14个维度
    actions = np.asarray(actions[..., :14])
    
    # 步骤3: Delta 动作还原
    if state is not None and not skip_delta_restore:
        # 使用post.md中定义的mask
        mask = np.array([True, True, True, True, True, True, False, False, True, True, True, True, False, False])
        actions = _delta_action_restore(actions, state, mask)
    
    # 步骤4: 应用Aloha输出转换（与aloha_policy.py一致）
    actions = _aloha_outputs(actions, adapt_to_pi=adapt_to_pi)
    
    # 处理一维输入的情况
    if was_1d:
        actions = actions.flatten()
    
    # 保存结果（如果需要）
    if save_result and output_dir is not None:
        os.makedirs(output_dir, exist_ok=True)
        output_path = os.path.join(output_dir, "postprocessed_actions.npz")
        np.savez_compressed(output_path, actions=actions)
        
        txt_output_path = os.path.join(output_dir, "postprocessed_actions.txt")
        with open(txt_output_path, "w") as f:
            for i, action in enumerate(actions):
                f.write(f"Action {i}: {action}\n")
        
        print(f"Postprocessing completed. Result saved to {output_path}")
    
    # 确保输出数据类型为float64
    return actions.astype(np.float64)

def postprocess_from_dir(input_dir, output_dir, adapt_to_pi=True):
    """从目录读取模型推理结果并进行后处理（兼容旧API）
    
    参数:
        input_dir: str - 输入数据目录路径
        output_dir: str - 输出结果目录路径
        adapt_to_pi: bool - 是否将数据从Pi内部运行时空间转换回标准空间格式
    
    返回:
        processed_actions: np.ndarray - 处理后的动作数据
    """
    # 读取模型推理结果
    inference_path = os.path.join(input_dir, "inference_result.npz")
    if not os.path.exists(inference_path):
        print(f"Error: {inference_path} not found")
        return None
    
    with np.load(inference_path) as data:
        raw_actions = data["actions"]
    
    # 调用主后处理函数
    return postprocess(raw_actions, adapt_to_pi=adapt_to_pi, save_result=True, output_dir=output_dir)

if __name__ == "__main__":
    input_dir = "data/infer_output"
    output_dir = "data/post_output"
    postprocess_from_dir(input_dir, output_dir, adapt_to_pi=True)
