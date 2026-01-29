#!/usr/bin/env python3

import os
import sys
import numpy as np
import socket
import struct
import time
import cv2
import json
import argparse
import glob
import traceback
import threading

# 添加项目根目录和demo目录到Python路径
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.append(os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'demo'))

# 导入自定义模块
import msg_pb2
from preprocess import preprocess
from postprocess import postprocess

def load_norm_stats(file_path):
    """
    加载归一化统计信息
    """
    try:
        with open(file_path, 'r') as f:
            norm_stats = json.load(f)
        return norm_stats
    except Exception as e:
        print(f"加载norm_stats失败: {e}")
        return None

class Server:
    """
    推理服务器类，用于接收客户端的连接并进行推理
    """
    def __init__(self, port):
        self.port = port
        self.listen_fd = None
        self.sock_fd = None
        self.stop_event = threading.Event()

    def listen(self):
        """
        监听客户端连接
        """
        # 1. 创建套接字（对应 C++ 的 socket()）
        self.listen_fd = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.listen_fd.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

        # 2. 绑定端口（对应 C++ 的 bind()，监听 8888 端口）
        server_addr = ("0.0.0.0", self.port)  # 0.0.0.0 等价于 C++ 的 INADDR_ANY（监听所有网卡）
        try:
            self.listen_fd.bind(server_addr)
        except OSError as e:
            print(f"绑定端口失败：{str(e)}")
            self.listen_fd.close()
            return

        # 3. 开始监听连接（对应 C++ 的 listen()，backlog=5）
        try:
            self.listen_fd.listen(5)  # backlog：等待队列最大长度
            print(f"服务器启动成功，等待客户端连接...（端口：{self.port}")
        except OSError as e:
            print(f"监听失败：{str(e)}")
            self.listen_fd.close()
            return

        # 4. 接受客户端连接（对应 C++ 的 accept()，阻塞直到有连接）
        try:
            # client_addr：存储客户端地址信息，addr_len：地址长度
            self.sock_fd, client_addr = self.listen_fd.accept()
            print(f"客户端已连接：IP={client_addr[0]}, 端口={client_addr[1]}")
        except OSError as e:
            print(f"接受连接失败：{str(e)}")
            self.listen_fd.close()
            return

    def start(self):
        """
        启动服务器
        """
        self.server_thread = threading.Thread(target=self.listen)
        self.server_thread.daemon = True
        self.server_thread.start()
        time.sleep(1)  # 等待服务器启动

    def stop(self):
        """
        停止服务器
        """
        self.stop_event.set()
        if hasattr(self, 'sock_fd') and self.sock_fd:
            self.sock_fd.close()
        if hasattr(self, 'listen_fd') and self.listen_fd:
            self.listen_fd.close()
    
    def close(self):
        """
        关闭服务器（stop方法的别名）
        """
        self.stop()

    def _build_header(self, reset=False):
        """
        构建Header消息
        """
        header = msg_pb2.Header()
        header.seq = 1  # 使用固定序列号1，因为这是单次请求
        
        current = time.time()
        sec = int(current)
        nsec = int((current - sec) * 1e9)
        header.stamp.sec = sec
        header.stamp.nsec = nsec
        header.reset = reset
        return header

    def send(self, obs, dtypes):
        """
        发送数据到客户端
        """
        # 1. 构造消息体
        msg = msg_pb2.MultiModalInput()
        header = self._build_header()
        msg.header.CopyFrom(header)

        # 2. 处理图像张量
        for key, img in obs["images"].items():
            tensor = msg.images.add()
            tensor.dtype = dtypes["images"]
            tensor.shape.extend(img.shape)
            tensor.data = img.tobytes()

        # 3. 处理prompt张量
        tensor = msg.languages.add()
        tensor.dtype = dtypes["prompt"]
        tensor.shape.extend(obs["prompt"].shape)
        tensor.data = obs["prompt"].tobytes()

        # 4. 处理状态张量
        tensor = msg.states.add()
        tensor.dtype = dtypes["state"]
        tensor.shape.extend(obs["state"].shape)
        tensor.data = obs["state"].tobytes()

        # 5. 序列化消息
        serialized_data = msg.SerializeToString()
        data_len = len(serialized_data)

        # 6. 发送数据长度（对应 C++ 的 htonl）
        net_len = socket.htonl(data_len)
        net_len_bytes = struct.pack("<I", net_len)  # 使用小端序
        self.sock_fd.sendall(net_len_bytes)

        # 7. 发送序列化数据
        self.sock_fd.sendall(serialized_data)
        print(f"发送成功，长度：{len(serialized_data)}字节")

    def _parse_tensor(self, tensor_msg):
        """
        解析tensor消息
        """
        # 解析形状
        shape = list(tensor_msg.shape)
        
        # 解析数据
        data = tensor_msg.data
        
        # 计算期望的数据长度
        expected_elements = np.prod(shape)
        
        # 首先根据dtype字段确定数据类型
        dtype_map = {
            msg_pb2.Tensor.FLOAT32: np.float32,
            msg_pb2.Tensor.FLOAT64: np.float64,
            msg_pb2.Tensor.UINT8: np.uint8,
            msg_pb2.Tensor.INT32: np.int32,
            msg_pb2.Tensor.STRING: str,
            msg_pb2.Tensor.FP16: np.float16,  # FP16对应dtype值5
        }
        
        # 尝试使用dtype字段指定的数据类型
        if tensor_msg.dtype in dtype_map:
            dtype = dtype_map[tensor_msg.dtype]
            
            # 字符串类型需要特殊处理
            if dtype == str:
                try:
                    return data.decode('utf-8')
                except Exception as e:
                    print(f"解析字符串类型失败: {e}")
                    return None
            
            element_size = np.dtype(dtype).itemsize
            actual_elements = len(data) // element_size
            
            if actual_elements == expected_elements:
                try:
                    # 解析数据并恢复形状
                    parsed_data = np.frombuffer(data, dtype=dtype)
                    return parsed_data.reshape(shape)
                except Exception as e:
                    print(f"使用指定dtype解析失败: {e}")
        
        # 如果指定类型失败或未知，尝试其他类型
        possible_dtypes = [np.float16, np.float32, np.int32]
        
        for dtype in possible_dtypes:
            try:
                element_size = np.dtype(dtype).itemsize
                actual_elements = len(data) // element_size
                
                if actual_elements == expected_elements:
                    parsed_data = np.frombuffer(data, dtype=dtype)
                    return parsed_data.reshape(shape)
            except Exception as e:
                print(f"尝试{np.dtype(dtype)}解析失败: {e}")
                pass
        
        print(f"无法解析张量，形状: {shape}, 数据长度: {len(data)}, 期望元素数: {expected_elements}")
        return None

    def receive(self):
        """
        从客户端接收数据
        """
        # 1. 接收数据长度（对应 C++ 的 ntohl）
        net_len_data = self.sock_fd.recv(4)
        if not net_len_data:
            print("未收到数据长度")
            return None
        net_len = struct.unpack("<I", net_len_data)[0]
        data_len = socket.ntohl(net_len)

        # 2. 接收实际数据
        received_data = b""
        while len(received_data) < data_len:
            chunk = self.sock_fd.recv(data_len - len(received_data))
            if not chunk:
                print("未收到完整数据")
                return None
            received_data += chunk

        print(f"接收成功，长度：{len(received_data)}字节")

        # 3. 解析消息
        msg = msg_pb2.MultiModalInput()
        msg.ParseFromString(received_data)

        # 4. 解析Header
        print("===== 解析 Header 信息 =====")
        print(f"序列号 : {msg.header.seq}")
        print(f"时间戳: {msg.header.stamp.sec}.{msg.header.stamp.nsec:09d}")
        print(f"重置标志: {msg.header.reset}")

        # 5. 解析Body
        print("====== 解析 Body 信息 ======")
        result = {}

        # 解析图像张量
        print(f"接收到 {len(msg.images)} 个图像张量：")
        for i, img_tensor in enumerate(msg.images):
            print(f"  图像{i+1}：类型={img_tensor.dtype}，维度={img_tensor.shape} ")
            # 如果需要使用图像数据，可以在这里进行处理
            img_data = self._parse_tensor(img_tensor)

        # 解析语言张量
        print(f"接收到 {len(msg.languages)} 个语言张量：")
        for i, language_tensor in enumerate(msg.languages):
            print(f"  语言{i+1}：类型={language_tensor.dtype}，维度={language_tensor.shape} ")
            # 直接比较dtype值为5，而不是依赖msg_pb2.Tensor.FP16
            if language_tensor.dtype == 5:  # 5 对应 FP16
                data = self._parse_tensor(language_tensor)
                result["raw_outputs"] = data  # 推理结果应该是raw_outputs，而不是prompt

        # 解析状态张量
        print(f"接收到 {len(msg.states)} 个状态张量：")
        for i, state_tensor in enumerate(msg.states):
            print(f"  状态{i+1}：类型={state_tensor.dtype}，维度={state_tensor.shape} ")
            if state_tensor.dtype == msg_pb2.Tensor.FLOAT32:
                data = self._parse_tensor(state_tensor)
                result["state"] = data

        print("============================")
        return result

def main():
    """
    主函数，处理单个或多个.npz文件的推理结果一致性验证
    """
    print("==========================================")
    print("后处理结果一致性验证脚本")
    print("==========================================")
    
    # 解析命令行参数
    parser = argparse.ArgumentParser(description='验证推理结果一致性')
    parser.add_argument('--npz-files', nargs='+', help='要处理的.npz文件路径列表')
    parser.add_argument('--output-dir', default='/mnt/wang.liu/project/pi0/data/inference_s600', help='推理结果保存目录')
    parser.add_argument('--server-ip', default='localhost', help='推理服务器IP地址')
    parser.add_argument('--server-port', type=int, default=8888, help='推理服务器端口')
    parser.add_argument('--norm-stats-path', default=os.path.join(os.path.dirname(os.path.abspath(__file__)), '../norm_stats.json'), help='归一化统计信息文件路径')
    args = parser.parse_args()
    
    # 如果没有指定文件，默认处理policy_records目录下的所有step_*.npz文件
    if not args.npz_files:
        npz_files = sorted(glob.glob('/mnt/wang.liu/project/pi0/data/policy_records/step_*.npz'))
    else:
        npz_files = args.npz_files
    
    # 确保输出目录存在
    os.makedirs(args.output_dir, exist_ok=True)
    
    print(f"\n将处理 {len(npz_files)} 个.npz文件")
    print(f"推理结果将保存到: {args.output_dir}")
    
    # 初始化服务器
    print(f"\n[步骤0] 初始化推理服务器...")
    server = Server(args.server_port)
    server.start()
    print(f"  推理服务器已启动，监听 {args.server_ip}:{args.server_port}")
    
    # 等待服务器启动
    time.sleep(1)
    
    # 循环处理每个.npz文件
    for i, npz_file in enumerate(npz_files):
        print(f"\n{'='*50}")
        print(f"处理文件 {i+1}/{len(npz_files)}: {npz_file}")
        print(f"{'='*50}")
        
        try:
            process_single_npz(npz_file, args.output_dir, server, args.norm_stats_path)
        except Exception as e:
            print(f"处理文件 {npz_file} 时出错: {e}")
            traceback.print_exc()
    
    # 关闭服务器
    server.close()
    
    print(f"\n{'='*50}")
    print("所有文件处理完成!")
    print(f"{'='*50}")

def process_single_npz(npz_file, output_dir, server, norm_stats_path):
    """
    处理单个.npz文件的推理结果一致性验证
    """
    # 1. 设置参数
    server_ip = "localhost"
    server_port = 8888
    user_prompt = "put the box"
    
    # 2. 加载数据
    print(f"\n[步骤1] 加载.npz文件中的原始数据...")
    if not os.path.exists(npz_file):
        print(f"错误: 文件 {npz_file} 不存在")
        return
    
    data = np.load(npz_file, allow_pickle=True)
    
    # 从.npz文件中获取原始的输入数据和推理结果
    original_images = {
        'cam_high': data['inputs/images/cam_high'],
        'cam_left_wrist': data['inputs/images/cam_left_wrist'],
        'cam_right_wrist': data['inputs/images/cam_right_wrist']
    }
    original_state = data['inputs/state']
    original_raw_outputs = data['raw_outputs']
    
    print(f"  原始图像形状: {original_images['cam_high'].shape}")
    print(f"  原始状态形状: {original_state.shape}")
    print(f"  原始推理输出形状: {original_raw_outputs.shape}")
    
    # 3. 处理右相机图片
    print(f"\n[步骤2] 处理右相机图片...")
    print(f"  原始右相机图片最大值: {np.max(original_images['cam_right_wrist'])}")
    
    # 检查右相机图片是否已经是纯黑色
    if np.max(original_images['cam_right_wrist']) > 0:
        # 创建一个和左相机图片同样大小的纯黑色图片
        black_image = np.zeros_like(original_images['cam_left_wrist'])
        print(f"  将右相机图片替换为纯黑色图片")
        original_images['cam_right_wrist'] = black_image
    
    print(f"  处理后右相机图片最大值: {np.max(original_images['cam_right_wrist'])}")
    
    # 4. 加载归一化统计信息
    print(f"\n[步骤3] 加载归一化统计信息...")
    norm_stats = load_norm_stats(norm_stats_path)
    if norm_stats is None:
        print(f"错误: 无法加载norm_stats")
        return
    print(f"  norm_stats加载成功")
    
    # 5. 预处理数据
    print(f"\n[步骤4] 预处理数据...")
    # 将CHW格式的图像转换为HWC格式，并确保是BGR颜色空间
    for cam_name, img in original_images.items():
        if img.shape[0] == 3:  # 如果是CHW格式
            # 转换为HWC格式
            original_images[cam_name] = np.transpose(img, (1, 2, 0))
            # 转换为BGR格式（原始数据是RGB格式）
            original_images[cam_name] = cv2.cvtColor(original_images[cam_name], cv2.COLOR_RGB2BGR)
    
    # 调用preprocess.py进行前处理
    processed = preprocess(
        images=original_images,
        state=original_state,
        prompt="put the box",  # 使用新的提示词
        adapt_to_pi=False,  # 使用adapt_to_pi=false的前处理
        norm_stats=norm_stats
    )
    
    preprocessed_images = processed['images']
    preprocessed_state = processed['state']
    preprocessed_token_ids = processed['token_ids']
    
    print(f"  预处理后图像键: {list(preprocessed_images.keys())}")
    for key, img in preprocessed_images.items():
        print(f"    {key}: shape={img.shape}, dtype={img.dtype}")
    print(f"  预处理后状态形状: {preprocessed_state.shape}, dtype={preprocessed_state.dtype}")
    print(f"  预处理后token_ids形状: {preprocessed_token_ids.shape}")
    
    # 6. 准备发送数据
    print(f"\n[步骤5] 准备发送数据...")
    
    # 准备observation数据
    obs = {
        "images": {
            "1": preprocessed_images["base_0_rgb"], 
            "2": preprocessed_images["left_wrist_0_rgb"],
            "3": preprocessed_images["right_wrist_0_rgb"]
        },
        "prompt": preprocessed_token_ids,
        "state": preprocessed_state.astype(np.float32)
    }
    
    # 准备dtypes数据
    dtypes = {
        "images": msg_pb2.Tensor.FLOAT32,
        "prompt": msg_pb2.Tensor.INT32,
        "state": msg_pb2.Tensor.FLOAT32
    }
    
    # 7. 发送推理请求
    print(f"\n[步骤6] 发送推理请求...")
    server.send(obs, dtypes)
    
    # 等待一小段时间，让客户端有足够的时间处理请求并返回结果
    time.sleep(2)
    
    # 8. 接收推理结果
    print(f"\n[步骤7] 接收推理结果...")
    result = server.receive()
    
    if result is None:
        print("接收推理结果失败")
        return
    
    # 10. 获取后处理结果
    print(f"\n[步骤9] 处理推理结果...")
    
    # 从返回结果中获取后处理后的动作数据
    if "prompt" in result:
        postprocessed_actions = result["prompt"]
        print(f"  接收到的推理结果形状: {postprocessed_actions.shape}")
        
        # 11. 对推理结果进行后处理
        print(f"\n[步骤10] 对推理结果进行后处理...")
        start_time = time.time()
        
        # 确保输入形状正确 (action_horizon, 32)
        if postprocessed_actions.ndim == 3 and postprocessed_actions.shape[0] == 1:
            postprocessed_actions = postprocessed_actions[0]
        
        # 执行后处理
        outputs_actions = postprocess(
            postprocessed_actions, 
            adapt_to_pi=True, 
            norm_stats=norm_stats, 
            state=original_state
        )
        
        infer_ms = (time.time() - start_time) * 1000  # 计算推理耗时（毫秒）
        print(f"  后处理完成，形状: {outputs_actions.shape}")
        print(f"  推理耗时: {infer_ms:.2f} 毫秒")
    else:
        print("  未收到推理结果")
        return
    
    # 12. 与原始推理输出比较
    print(f"\n[步骤11] 与原始推理输出比较...")
    
    # 调整原始推理输出的形状，使其与远程推理结果的形状匹配
    if original_raw_outputs.ndim == 3 and postprocessed_actions.ndim == 2:
        original_raw_outputs = original_raw_outputs[0]  # 如果形状是(1, 50, 32)，取第一个维度
        print(f"  注意: 已将原始推理输出的形状从 {(original_raw_outputs.shape[1:])} 调整为 {original_raw_outputs.shape}")
    elif original_raw_outputs.ndim == 2 and postprocessed_actions.ndim == 3:
        original_raw_outputs = original_raw_outputs[np.newaxis, :, :]
        print(f"  注意: 已将原始推理输出的形状从 {(original_raw_outputs.shape[1:])} 调整为 {original_raw_outputs.shape}")
    
    # 原始推理输出和远程推理结果形状比较
    if original_raw_outputs.shape != postprocessed_actions.shape:
        print(f"  警告: 形状不匹配 - 原始推理输出: {original_raw_outputs.shape}, 远程推理结果: {postprocessed_actions.shape}")
    else:
        print(f"  形状匹配: {original_raw_outputs.shape}")
        
        # 计算余弦相似度
        def cosine_similarity(a, b):
            a = a.flatten()
            b = b.flatten()
            dot_product = np.dot(a, b)
            norm_a = np.linalg.norm(a)
            norm_b = np.linalg.norm(b)
            return dot_product / (norm_a * norm_b) if norm_a != 0 and norm_b != 0 else 0
        
        # 计算差异
        def compute_difference(a, b):
            a = a.flatten()
            b = b.flatten()
            return np.max(np.abs(a - b))
    
    # 计算统计信息
    def compute_stats(data):
        data_flat = data.flatten()
        return {
            "min": np.min(data_flat),
            "max": np.max(data_flat),
            "mean": np.mean(data_flat),
            "std": np.std(data_flat)
        }
    
    # 计算统计信息
    original_stats = compute_stats(original_raw_outputs)
    remote_stats = compute_stats(postprocessed_actions)
    
    # 打印结果
    print(f"  原始推理输出统计信息:")
    print(f"    最小值: {original_stats['min']:.6f}, 最大值: {original_stats['max']:.6f}")
    print(f"    均值: {original_stats['mean']:.6f}, 标准差: {original_stats['std']:.6f}")
    
    print(f"  远程推理结果统计信息:")
    print(f"    最小值: {remote_stats['min']:.6f}, 最大值: {remote_stats['max']:.6f}")
    print(f"    均值: {remote_stats['mean']:.6f}, 标准差: {remote_stats['std']:.6f}")
    
    # 如果形状匹配，计算相似度和差异
    if original_raw_outputs.shape == postprocessed_actions.shape:
        # 计算相似度和差异
        avg_similarity = cosine_similarity(original_raw_outputs, postprocessed_actions)
        max_diff = compute_difference(original_raw_outputs, postprocessed_actions)
        
        print(f"  比较结果:")
        print(f"    平均余弦相似度: {avg_similarity:.6f}")
        print(f"    最大差异: {max_diff:.6f}")
    
    # 输出样本数据
    print(f"  样本数据:")
    if original_raw_outputs.ndim == 3 and original_raw_outputs.shape[0] == 1:
        print(f"    原始推理输出[0,0]: {original_raw_outputs[0, 0, :5]}")
    else:
        print(f"    原始推理输出[0,0]: {original_raw_outputs[0, :5]}")
    if postprocessed_actions.ndim == 3 and postprocessed_actions.shape[0] == 1:
        print(f"    远程推理结果[0,0]: {postprocessed_actions[0, 0, :5]}")
    else:
        print(f"    远程推理结果[0,0]: {postprocessed_actions[0, :5]}")
    
    # 12. 保存结果
    print(f"\n[步骤11] 保存比较结果...")
    
    # 从输入文件名中提取编号
    base_name = os.path.basename(npz_file)
    file_prefix = os.path.splitext(base_name)[0]  # 获取不带扩展名的文件名，如 "step_0"
    
    # 构建输出文件名
    output_file = os.path.join(output_dir, f"{file_prefix}_inference.npz")
    
    # 准备保存的数据
    save_data = {
        # 原始输入数据
        'inputs/state': original_state,
        'inputs/images/cam_high': original_images['cam_high'],
        'inputs/images/cam_left_wrist': original_images['cam_left_wrist'],
        'inputs/images/cam_right_wrist': original_images['cam_right_wrist'],
        'inputs/prompt': 'put the box',  # 使用的提示词
        
        # 前处理结果
        'transformed_inputs/image/base_0_rgb': preprocessed_images['base_0_rgb'],
        'transformed_inputs/image/left_wrist_0_rgb': preprocessed_images['left_wrist_0_rgb'],
        'transformed_inputs/image/right_wrist_0_rgb': preprocessed_images['right_wrist_0_rgb'],
        'transformed_inputs/state': preprocessed_state,
        'transformed_inputs/tokenized_prompt': preprocessed_token_ids,
        
        # 原始推理输出
        'raw_outputs': original_raw_outputs,
        
        # 远程推理结果
        'remote_inference_results': postprocessed_actions,
        
        # 后处理结果
        'outputs/actions': outputs_actions,
        'outputs/policy_timing/infer_ms': infer_ms,
        
        # 比较结果
        'comparison/original_stats': original_stats,
        'comparison/remote_stats': remote_stats
    }
    
    # 如果形状匹配，添加相似度和差异
    if original_raw_outputs.shape == postprocessed_actions.shape:
        save_data['comparison/avg_similarity'] = avg_similarity
        save_data['comparison/max_diff'] = max_diff
    
    # 保存比较结果到npz文件
    np.savez_compressed(
        output_file,
        **save_data
    )
    
    print(f"  比较结果已保存到 {output_file}")
    
    # 13. 总结
    print(f"\n[总结]")
    print(f"  后处理结果一致性验证完成")
    print(f"  原始推理输出形状: {original_raw_outputs.shape}")
    print(f"  远程推理结果形状: {postprocessed_actions.shape}")
    
    if original_raw_outputs.shape != postprocessed_actions.shape:
        print(f"  注意: 由于形状不同，无法直接计算余弦相似度")
        print(f"  建议: 比较统计信息和样本数据以评估推理结果的合理性")
    else:
        print(f"  比较结果: 平均余弦相似度为 {avg_similarity:.6f}, 最大差异为 {max_diff:.6f}")
        if avg_similarity > 0.99:
            print(f"  结果: 推理结果一致性高")
        elif avg_similarity > 0.9:
            print(f"  结果: 推理结果一致性良好")
        else:
            print(f"  结果: 推理结果一致性较低")

if __name__ == "__main__":
    main()
