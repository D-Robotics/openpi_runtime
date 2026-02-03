#!/usr/bin/env python3

import socket
import struct
import time
import threading
import numpy as np

# 导入消息协议 - 延迟导入以避免依赖问题
msg_pb2 = None


class Server:
    """
    推理服务器类，用于接收客户端的连接并进行推理
    """

    def __init__(self, port):
        self.port = port
        self.listen_fd = None
        self.sock_fd = None
        self.stop_event = threading.Event()
        self.seq_counter = 0  # 序列号计数器

    def _get_msg_pb2(self):
        """延迟导入msg_pb2模块"""
        global msg_pb2
        if msg_pb2 is None:
            try:
                from common.msg import msg_pb2
            except ImportError:
                # 如果导入失败，尝试从其他位置导入
                import sys
                import os

                sys.path.append(os.path.join(os.path.dirname(__file__), "..", ".."))
                try:
                    from common.msg import msg_pb2
                except ImportError:
                    # 最后尝试从openpi_runtime目录导入
                    sys.path.append(
                        os.path.join(
                            os.path.dirname(__file__), "..", "..", "openpi_runtime"
                        )
                    )
                    try:
                        import msg_pb2
                    except ImportError:
                        raise ImportError(
                            "无法导入msg_pb2模块，请确保msg_pb2.py在正确的路径中"
                        )
        return msg_pb2

    def listen(self):
        """
        监听客户端连接
        """
        # 1. 创建套接字（对应 C++ 的 socket()）
        self.listen_fd = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.listen_fd.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

        # 2. 绑定端口（对应 C++ 的 bind()，监听 8888 端口）
        server_addr = (
            "0.0.0.0",
            self.port,
        )  # 0.0.0.0 等价于 C++ 的 INADDR_ANY（监听所有网卡）
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
        if hasattr(self, "sock_fd") and self.sock_fd:
            self.sock_fd.close()
        if hasattr(self, "listen_fd") and self.listen_fd:
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
        msg_pb2 = self._get_msg_pb2()
        header = msg_pb2.Header()
        self.seq_counter += 1
        header.seq = self.seq_counter  # 使用递增的序列号

        current = time.time()
        sec = int(current)
        nsec = int((current - sec) * 1e9)
        header.stamp.sec = sec
        header.stamp.nsec = nsec
        header.reset = reset
        return header

    def send(self, obs, dtypes):
        """发送数据到客户端"""
        msg_pb2 = self._get_msg_pb2()

        # 如果连接已关闭，重新接受连接
        if self.sock_fd is None or (hasattr(self.sock_fd, '_closed') and self.sock_fd._closed):
            self.sock_fd, client_addr = self.listen_fd.accept()
            print(f"客户端已连接：IP={client_addr[0]}, 端口={client_addr[1]}")

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
        msg_pb2 = self._get_msg_pb2()

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
                    return data.decode("utf-8")
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

        print(
            f"无法解析张量，形状: {shape}, 数据长度: {len(data)}, 期望元素数: {expected_elements}"
        )
        return None

    def receive(self):
        """
        从客户端接收推理结果
        """
        msg_pb2 = self._get_msg_pb2()

        net_len_data = self.sock_fd.recv(4)
        if not net_len_data:
            print("未收到数据长度")
            return None
        net_len = struct.unpack("<I", net_len_data)[0]
        data_len = socket.ntohl(net_len)

        received_data = b""
        while len(received_data) < data_len:
            chunk = self.sock_fd.recv(data_len - len(received_data))
            if not chunk:
                print("未收到完整数据")
                return None
            received_data += chunk

        print(f"接收成功，长度：{len(received_data)}字节")

        msg = msg_pb2.MultiModalInput()
        msg.ParseFromString(received_data)

        print(f"Header - 序列号: {msg.header.seq}")

        result = {}
        if len(msg.languages) > 0:
            language_tensor = msg.languages[0]
            print(f"语言张量：类型={language_tensor.dtype}，维度={language_tensor.shape}")
            if language_tensor.dtype == 5:
                result["raw_outputs"] = self._parse_tensor(language_tensor)

        print("============================")
        return result
