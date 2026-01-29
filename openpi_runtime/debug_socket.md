# Socket通信问题分析

## 问题描述
运行时节点作为服务端接收推理端客户端的数据时，出现以下错误：
```
未收到完整长度（需4字节，实际收到0字节）
TypeError: 'NoneType' object is not subscriptable
```

## 问题原因
在`model_client.py`文件中，发送和接收数据时的字节序处理不一致：

### 发送端问题（第93-94行）
```python
net_len = socket.htonl(data_len)  # 主机序→网络序（大端）
net_len_bytes = struct.pack("<I", net_len)  # 错误：使用小端字节序打包
```
- `socket.htonl()`已经将数据转换为大端字节序
- 但`struct.pack("<I", net_len)`又使用小端字节序格式，导致字节序混乱

### 接收端问题（第131行）
```python
net_len = struct.unpack("<I", net_len_data)[0]  # 错误：使用小端字节序解析
```
- 应该使用大端字节序（")I"）来解析网络数据

## 解决方案
修改`model_client.py`文件中的以下几行：

### 1. 修复发送端字节序处理
将第94行改为：
```python
net_len_bytes = struct.pack(">I", data_len)  # 直接用大端打包原始长度
```

### 2. 修复接收端字节序处理
将第131行改为：
```python
net_len = struct.unpack(">I", net_len_data)[0]  # 使用大端解析
```

### 3. 优化网络序转换
可以进一步优化，避免重复的网络序转换：

发送端：
```python
# 直接用大端打包原始长度，无需先调用htonl
net_len_bytes = struct.pack(">I", data_len)
```

接收端：
```python
# 用大端解析，无需再调用ntohl
net_len = struct.unpack(">I", net_len_data)[0]
data_len = net_len  # 直接使用解析结果
```

## 额外改进

### 1. 接收超时处理
在`recv_proto_message`函数中添加超时处理，避免长时间阻塞：
```python
# 设置接收超时（可选）
sock.settimeout(5.0)  # 5秒超时
```

### 2. 错误恢复机制
在`receive`方法中添加错误恢复机制，当接收失败时尝试重新连接：
```python
if not recv_proto_message(sock, batch):
    # 尝试重新连接
    self.connect()
    return None
```

## 代码修改总结

```python
# 修改前
net_len = socket.htonl(data_len)
net_len_bytes = struct.pack("<I", net_len)

# 修改后
net_len_bytes = struct.pack(">I", data_len)

# 修改前
net_len = struct.unpack("<I", net_len_data)[0]
data_len = socket.ntohl(net_len)

# 修改后
net_len = struct.unpack(">I", net_len_data)[0]
data_len = net_len
```

这些修改将解决字节序不匹配的问题，使服务端能够正确接收和解析来自推理端客户端的数据。