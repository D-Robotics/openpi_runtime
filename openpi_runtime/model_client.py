
# Copyright (c) 2025，D-Robotics.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import socket
import numpy as np
import time
import struct

import openpi_runtime.msg_pb2 as msg_pb2

class NetBase:
    def __init__(self, port=8888):
        self.sock_fd = None
        self.port = port
        self.seq = 0

    def send(
        self,
        observation: dict,
        dtypes,
        reset=False,
    ):
        sock = self.sock_fd
        """发送多模态输入消息"""
        input_msg = msg_pb2.MultiModalInput()

        def build_header(seq):
            """构建Header消息"""
            header = msg_pb2.Header()
            header.seq = seq

            def get_current_time():
                """获取当前时间(秒和纳秒)"""
                current = time.time()
                sec = int(current)
                nsec = int((current - sec) * 1e9)
                return sec, nsec

            sec, nsec = get_current_time()
            header.stamp.sec = sec
            header.stamp.nsec = nsec
            header.reset = reset
            return header

        # 设置Header
        header = build_header(self.seq)
        self.seq += 1

        input_msg.header.CopyFrom(header)

        # 添加图像Tensor
        if "images" in observation:
            for img in observation["images"].values():
                image = input_msg.images.add()
                image.dtype = dtypes["images"]
                image.shape.extend(img.shape)
                image.data = img.tobytes()

        # 添加语言Tensor
        if "prompt" in observation:
            language = input_msg.languages.add()
            language.dtype = dtypes["prompt"]
            language.shape.extend(observation["prompt"].shape)
            language.data = observation["prompt"].tobytes()

        # 添加状态Tensor
        if "state" in observation:
            state = input_msg.states.add()
            state.dtype = dtypes["state"]
            state.shape.extend(observation["state"].shape)
            state.data = observation["state"].tobytes()

        # 发送消息
        def send_proto_message(sock: socket.socket, msg) -> bool:
            try:
                # 1. 序列化 Protobuf 消息
                serialized_data = msg.SerializeToString()
                data_len = len(serialized_data)

                # 2. 关键：长度字段按"大端字节序"打包（与 C++ 网络序一致）
                # struct.pack(">I", data_len) 直接将长度打包为大端格式，无需额外转换
                net_len_bytes = struct.pack(">I", data_len)
                # 验证长度字段是否为 4 字节（必须满足）
                assert len(net_len_bytes) == 4, f"长度字段应为4字节，实际{len(net_len_bytes)}字节"

                # 3. 先发送长度，再发送数据
                sock.sendall(net_len_bytes)  # 发送 4 字节长度
                sock.sendall(serialized_data)  # 发送 Protobuf 数据
                print(f"发送成功，长度：{data_len}字节\n")
                return True
            except Exception as e:
                print(f"发送失败：{str(e)}")
                return False

        send_proto_message(sock, input_msg)

    def receive(self):
        batch = msg_pb2.MultiModalInput()
        sock = self.sock_fd

        def parse_header(header):
            """解析并打印Header信息"""
            print("===== 解析 Header 信息 =====")
            print(f"序列号 : {header.seq}")
            print(f"时间戳: {header.stamp.sec}.{header.stamp.nsec}")
            time_sec = header.stamp.sec + header.stamp.nsec / 1e9
            print("====== 解析 Body 信息 ======")

        def recv_proto_message(sock, msg):
            """接收protobuf消息(先接收长度，再接收数据)"""
            try:
                # 1. 接收 4 字节长度（网络序→大端）
                net_len_data = sock.recv(4)
                if len(net_len_data) != 4:
                    print("未收到完整长度（需4字节，实际收到{}字节）".format(len(net_len_data)))
                    return False

                # 关键：用 ">I"（大端）解析 4 字节无符号整数
                # struct.unpack(">I", ...) 已经返回主机序的长度值
                data_len = struct.unpack(">I", net_len_data)[0]

                # 接收数据
                serialized_data = b""
                while len(serialized_data) < data_len:
                    chunk = sock.recv(min(4096, data_len - len(serialized_data)))
                    if not chunk:
                        print("连接断开")
                        return False
                    serialized_data += chunk

                # 反序列化
                msg.ParseFromString(serialized_data)
                print(f"接收成功，长度：{data_len}字节")
                return True
            except Exception as e:
                print(f"接收失败：{str(e)}")
                return False

        if recv_proto_message(sock, batch):
            # 解析Header
            parse_header(batch.header)

            def parse_type(input):
                if input.dtype == msg_pb2.Tensor.STRING:
                    arr = input.data
                    return arr.decode("utf-8")
                elif input.dtype == msg_pb2.Tensor.UINT8:
                    arr = np.frombuffer(input.data, dtype=np.uint8)
                elif input.dtype == msg_pb2.Tensor.FLOAT64:
                    arr = np.frombuffer(input.data, dtype=np.float64)
                elif input.dtype == msg_pb2.Tensor.FLOAT32:
                    arr = np.frombuffer(input.data, dtype=np.float32)
                elif input.dtype == msg_pb2.Tensor.INT32:
                    arr = np.frombuffer(input.data, dtype=np.int32)
                elif input.dtype == msg_pb2.Tensor.FP16:
                    arr = np.frombuffer(input.data, dtype=np.float16)

                return arr.reshape(input.shape)

            # 解析图片张量
            imgs = []
            img_size = len(batch.images)
            print(f"接收到 {img_size} 个图像张量：")
            for i in range(img_size):
                img = batch.images[i]
                print(f"  语言{i}：类型={img.dtype}，维度=", end="")
                for dim in img.shape:
                    print(f"{dim} ", end="")
                print()
                imgs.append(parse_type(img))

            # 解析语言张量
            langs = []
            lang_size = len(batch.languages)
            print(f"接收到 {lang_size} 个嵌入张量：")
            for i in range(lang_size):
                lang = batch.languages[i]
                print(f"  语言{i}：类型={lang.dtype}，维度=", end="")
                for dim in lang.shape:
                    print(f"{dim} ", end="")
                print()
                langs.append(parse_type(lang))

            # 解析状态张量
            states = []
            state_size = len(batch.states)
            print(f"接收到 {state_size} 个状态张量：")
            for i in range(state_size):
                state = batch.states[i]
                print(f"  语言{i}：类型={state.dtype}，维度=", end="")
                for dim in state.shape:
                    print(f"{dim} ", end="")
                print()
                states.append(parse_type(state))

            print("============================")

            img_keys = ["cam_high", "cam_left_wrist", "cam_right_wrist"]

            obs = {}
            if imgs:
                obs["images"] = dict(zip(img_keys, imgs))
            if states:
                obs["state"] = states[0]
            if langs:
                obs["prompt"] = langs[0]

            return obs

        return None
        
class Client(NetBase):
    def __init__(self, port=8888):
        super().__init__(port)
        self.listen_fd = None
    
    def connect(self):
        def is_connected(sock: socket.socket) -> bool:
            if sock is None:
                return False
            try:
                sock.getpeername()  # 如果未连接，会抛异常
                return True
            except socket.error:
                return False

        if is_connected(self.sock_fd):
            return

        def listen():
            # 1. 创建 TCP Socket（对应 C++ 的 socket(AF_INET, SOCK_STREAM, 0)）
            try:
                # SOCK_STREAM 表示 TCP 协议，AF_INET 表示 IPv4
                self.listen_fd = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                # 设置端口复用（避免 TIME_WAIT 导致端口无法重启，对应 C++ 的 SO_REUSEADDR）
                self.listen_fd.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            except OSError as e:
                print(f"创建 Socket 失败：{str(e)}")
                return

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
                print(f"服务器启动成功，等待客户端连接...（端口：{self.port}）")
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

        listen()
        
if __name__ == "__main__":
    client = Client(port=8888)
    client.connect()
    obs = {
            "images": {"1":np.zeros((1,3,224,224),dtype=np.uint8), "2":np.zeros((1,3,224,224),dtype=np.uint8),"3":np.zeros((1,3,224,224),dtype=np.uint8)},
            "prompt":np.array(("hi")),
            "state": np.zeros((1,14),dtype=np.float64),
        }
    dtypes = {"images":msg_pb2.Tensor.UINT8, "prompt":msg_pb2.Tensor.STRING, "state": msg_pb2.Tensor.FLOAT64}

    while True:
        client.send(obs, dtypes)
        result = client.receive()
        print(result["prompt"].shape, result["prompt"].dtype)
