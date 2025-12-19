# Piper 本体相关脚本说明
包含用于与 **Piper 机械臂** 配合使用的若干 Python 脚本，主要用于与示教臂 **Alicia-D** 协同工作、数据采集、分析及数据集组织。

---

## ⚠️ 重要备注

在运行任何脚本之前，请**务必确认 Alicia-D 与 Piper 之间的通信连接正常**。若通信异常，可能导致同步失败或数据不一致。

---

## 脚本功能说明

### 1. `alicia_techer_piper.py`

- **功能**：实现示教臂 **Alicia-D** 与 **Piper 机械臂** 的实时同步控制。
- **运行方式**：
  ```bash
  python alicia_teach_piper.py
  ```


### 2. `collect_data.py`

- **功能**：采集人机交互过程中的多模态数据（如关节角度、末端位姿、时间戳等），并保存为 **.hdf5** 格式文件。
- **运行方式**：
  ```bash
  source /opt/ros/jazzy/setup.bash
  export ROS_DOMAIN_ID=40
  python collect_data.py --sync
  ```

### 3. analycis_data.py
- **功能**：解析并可视化 .hdf5 数据文件内容，支持查看单次采集的完整轨迹与元信息。
- **运行方式**：（以示例文件为例）：
  ```bash
  python .\analycis_data.py .\episode_1_20251219_142924.hdf5
  ```
请将 .hdf5 文件路径替换为你实际的数据文件路径。

### 4. organize_datasets.py
- **功能**：将多个 .hdf5 数据文件与对应的自然语言指令（来自 instructions.json）进行结构化整合，生成标准化数据集目录。
- **运行方式**：
  ```bash
  python organize_datasets.py
  ```
- **配置要求**：
在脚本内部需手动指定以下两个路径：
- .hdf5 文件所在文件夹目录
- instructions.json 文件的完整路径

建议在运行前检查路径是否正确，避免因路径错误导致脚本中断。