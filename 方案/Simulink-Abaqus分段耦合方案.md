# Simulink-Abaqus 分段耦合方案（保持现有模块结构）

## 设计目标

保持现有 Signal_Generator 和 Response_spectrum_calculation 模块**代码不变**，通过在中间插入"桥接模块"实现与 Abaqus 的分段数据交换。

---

## 系统架构对比

### 当前架构（纯 Simulink）
```
Signal_Generator ──[2×1]──► State-Space ──[2×1]──► Buffer ──[5120×2]──► Response_spectrum
     (每步)                   (每步)               (每0.5s)              (每0.5s)
```

### 目标架构（Abaqus 耦合）
```
Signal_Generator ──[2×1]──► [Force_Accumulator] ──► Abaqus ──► [Accel_Dispatcher] ──► Buffer ──► Response_spectrum
     (每步)                    (累积15s)          (计算15s)      (逐步释放)         (每0.5s)
```

---

## 核心问题 1：如何正确处理 N_samples×2 的加速度数据

### 问题分析

| 模块 | 输入 | 输出 | 频率 |
|------|------|------|------|
| Signal_Generator | L_old [2561×2×2] | u [2×1] | 每步 (0.0001s) |
| **新增: Force_Accumulator** | u [2×1] | u_batch.txt [150000×2] | 每15s一次 |
| **Abaqus** | u_batch.txt | a_batch.txt [150000×2] | 外部调用 |
| **新增: Accel_Dispatcher** | a_batch.txt | y [2×1] | 每步释放一个点 |
| Buffer | y [2×1] → [5120×2] | - | 每0.5s输出 |
| Response_spectrum | y [5120×2] | Syy [2561×2×2] | 每0.5s |

### 解决方案：逐步释放 + 原有 Buffer

**关键洞察**：
- Abaqus 返回的 `a_batch.txt` 是 **连续的时间序列**（假设 15s = 150,000 点 @ 10kHz）
- 你**不需要手动实现重叠**！
- 只需将这些点**逐个释放**给后面的 Buffer 模块
- Buffer 模块会自动完成 50% 重叠和分帧

### Accel_Dispatcher 模块设计

```
输入: a_batch.txt (一次性加载，共 150,000 行 × 2 列)
输出: y [2×1] (每个时间步输出一个点)

工作逻辑:
┌────────────────────────────────────────────────────────┐
│  persistent accel_buffer   % 存储整个 15s 的加速度数据  │
│  persistent read_idx       % 当前读取位置              │
│  persistent segment_loaded % 是否已加载当前段          │
│                                                        │
│  if 需要加载新段:                                       │
│      accel_buffer = load('a_batch.txt');               │
│      read_idx = 1;                                     │
│      segment_loaded = true;                            │
│  end                                                   │
│                                                        │
│  y = accel_buffer(read_idx, :)';  % 取出一个点 [2×1]  │
│  read_idx = read_idx + 1;                              │
│                                                        │
│  if read_idx > size(accel_buffer, 1):                  │
│      segment_loaded = false;  % 触发加载下一段         │
│  end                                                   │
└────────────────────────────────────────────────────────┘
```

### 数据流时序图

```
时间:     0 ──────── 15s ──────── 30s ──────── 45s
          │          │           │           │
Force:    │◄─累积──►│◄─累积───►│           │
          │          │           │           │
Abaqus:   │          [计算15s]   │  [计算]   │
          │               ↓      │     ↓     │
Accel:    │          [逐点释放─────────────►]│
          │               ↓      ↓     ↓     │
Buffer:   │            帧1 帧2 ... 帧30      │
          │               ↓                  │
Response: │            Syy_avg               │
          │               ↓                  │
L_old:    L_init ──────► L_1 ─────────────► L_2
```

---

## 核心问题 2：Simulink 能否等待 Abaqus 计算

### 答案：可以，MATLAB Function 内的外部调用是**同步阻塞**的

当你在 MATLAB Function 中执行以下代码时：

```matlab
% 方法1: 使用 system 命令
status = system('abaqus job=my_analysis interactive');

% 方法2: 使用 Python 脚本
status = system('python run_abaqus.py');
```

**Simulink 会完全停下来等待命令执行完毕**，然后才继续下一步。

### 注意事项

1. **仿真时间 vs 实际时间**
   - Simulink 的仿真时钟会"冻结"在 15s、30s 等时刻
   - 实际墙钟时间可能过去几分钟（Abaqus 计算耗时）
   - 这是正常的，不会影响仿真结果

2. **超时处理**
   - 建议在调用中加入超时检测
   - 如果 Abaqus 运行失败，需要有错误处理机制

3. **文件锁定**
   - 确保 Abaqus 完全写完 `a_batch.txt` 后再读取
   - 可以用一个标志文件（如 `done.flag`）来同步

---

## 详细模块设计

### 模块 1: Force_Accumulator

**位置**: 接在 Signal_Generator 后面

**功能**: 
- 累积力信号
- 到达分段时间点时，输出 txt 文件
- 触发 Abaqus 计算
- 等待 Abaqus 完成

**输入**:
- `u [2×1]`: 每个时间步的力

**输出**:
- `u_pass [2×1]`: 直通（用于后续调试或显示）
- `segment_ready`: 标志位，表示新段数据已准备好

**参数**:
- `segment_duration = 15` (秒)
- `dt = 0.0001` (时间步长)
- `force_file = 'u_batch.txt'`
- `accel_file = 'a_batch.txt'`

**伪代码逻辑**:
```
persistent force_buffer, sample_count, segment_count

每个时间步:
1. 将 u 添加到 force_buffer 末尾
2. sample_count += 1
3. if sample_count >= segment_duration / dt:
      a. 将 force_buffer 写入 u_batch.txt
      b. 调用 Abaqus (阻塞等待)
      c. segment_count += 1
      d. sample_count = 0
      e. force_buffer = []  % 清空
```

---

### 模块 2: Accel_Dispatcher

**位置**: 替代原来的 State-Space，接在 Force_Accumulator 后面

**功能**:
- 加载 Abaqus 计算结果
- 逐点释放给后面的 Buffer

**输入**:
- `segment_ready`: 来自 Force_Accumulator 的触发信号
- （可选）`t`: 当前仿真时间

**输出**:
- `y [2×1]`: 每个时间步的加速度

**伪代码逻辑**:
```
persistent accel_buffer, read_idx, buffer_valid

每个时间步:
1. if segment_ready 且 buffer_valid == false:
      accel_buffer = load('a_batch.txt');
      read_idx = 1;
      buffer_valid = true;
   
2. if buffer_valid:
      y = accel_buffer(read_idx, :)';
      read_idx += 1;
      if read_idx > length(accel_buffer):
          buffer_valid = false;
   else:
      y = [0; 0];  % 初始阶段无数据
```

---

### 模块 3: Buffer（保持不变）

原有的 Simulink Buffer 模块：
- Buffer Size = 5120
- Overlap = 2560
- 接收 Accel_Dispatcher 的 [2×1] 输出
- 自动完成分帧和 50% 重叠

---

### 模块 4: Response_spectrum_calculation（保持不变）

- 输入: [5120×2] 帧数据
- 输出: Syy [2561×2×2], isPeriod
- 代码无需修改

---

## Simulink 模型连接图

```
┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│ Signal_Generator│───►│Force_Accumulator│───►│Accel_Dispatcher │
│    [2×1]        │    │   累积 + 调用   │    │   逐点释放      │
│   (不变)        │    │   Abaqus        │    │    [2×1]        │
└─────────────────┘    └─────────────────┘    └─────────────────┘
                              │                       │
                              │ u_batch.txt           │
                              ▼                       ▼
                       ┌─────────────┐         ┌─────────────┐
                       │   Abaqus    │────────►│   Buffer    │
                       │ (外部进程)  │a_batch  │  [5120×2]   │
                       └─────────────┘  .txt   │   (不变)    │
                                               └──────┬──────┘
                                                      │
                                                      ▼
┌─────────────────┐                            ┌─────────────────┐
│    Controller   │◄───────────────────────────│ Response_spectrum│
│    (不变)       │     Syy, isPeriod          │   calculation    │
└────────┬────────┘                            │    (不变)        │
         │                                     └─────────────────┘
         │ L_new
         ▼
┌─────────────────┐
│   Unit Delay    │
│    (不变)       │
└────────┬────────┘
         │ L_old
         ▼
  回到 Signal_Generator
```

---

## 时序同步细节

### 关键时刻分析

假设 segment_duration = 15s, dt = 0.0001s:

| 仿真时间 | Force_Accumulator | Abaqus | Accel_Dispatcher | Response |
|---------|-------------------|--------|------------------|----------|
| 0~15s | 累积 150,000 点 | 待机 | 输出 0 | 无有效输出 |
| t=15s | 写文件，调用 Abaqus | **开始计算** (阻塞) | 待机 | - |
| t=15s+ | 等待 Abaqus | 计算中... | 待机 | - |
| Abaqus完成 | 继续 | 输出 a_batch.txt | 加载数据 | - |
| 15s~30s | 累积下一段 | 待机 | 逐点释放 | 计算 Syy |
| t=30s | L_old 更新完成 | - | - | isPeriod=true |

### 第一个分段的特殊处理

在 0~15s 期间：
- Force_Accumulator 正在累积
- Accel_Dispatcher 没有数据可以输出
- Response_spectrum 会收到全零输入

**处理方式**:
1. Accel_Dispatcher 输出零值
2. Response_spectrum 的 Syy_max 检查会跳过无效帧
3. 或者，在初始化时预生成第一段数据

---

## Abaqus 端的要求

### 输入文件格式 (u_batch.txt)

```
# 时间(s), 力1(N), 力2(N)
0.0000, 1.234, -0.567
0.0001, 1.245, -0.589
0.0002, 1.256, -0.612
...
14.9999, 2.103, 1.456
```

### 输出文件格式 (a_batch.txt)

```
# 时间(s), 加速度1(m/s²), 加速度2(m/s²)
0.0000, 0.0123, 0.0456
0.0001, 0.0234, 0.0567
...
14.9999, 0.1234, 0.5678
```

### Abaqus Python 脚本框架

```python
# run_abaqus_segment.py
import numpy as np
from abaqus import *

# 1. 读取力时程
force_data = np.loadtxt('u_batch.txt', delimiter=',', skiprows=1)

# 2. 定义载荷幅值曲线
# ... (创建 TabularAmplitude)

# 3. 施加集中力
# ... (在指定节点施加力)

# 4. 提交分析 (动态显式)
# ... (设置分析步、时间增量)

# 5. 等待完成

# 6. 提取加速度历史
# ... (从 ODB 提取)

# 7. 输出到文件
np.savetxt('a_batch.txt', accel_data, header='time, a1, a2')

# 8. 写完成标志
with open('done.flag', 'w') as f:
    f.write('done')
```

---

## 潜在问题与解决方案

### 问题 1: Abaqus 计算失败

**解决方案**:
- 检查 Abaqus 返回的 status code
- 如果失败，用上一段的加速度数据填充（容错）

### 问题 2: 数据采样率不匹配

**场景**: Simulink 步长 0.0001s (10kHz)，但 Abaqus 输出是 0.0002s (5kHz)

**解决方案**:
- 在 Accel_Dispatcher 中加入插值
- 或在 Abaqus 中设置更高的输出频率

### 问题 3: 内存占用

15s × 10kHz × 2通道 = 150,000 × 2 = 300,000 个浮点数 ≈ 2.4 MB

**解决方案**: 这个量级完全可接受

---

## 实施步骤

1. **创建 Force_Accumulator 模块**
   - 使用 MATLAB Function 块
   - 实现力累积和 Abaqus 调用逻辑

2. **创建 Accel_Dispatcher 模块**
   - 使用 MATLAB Function 块
   - 实现文件加载和逐点释放

3. **准备 Abaqus 脚本**
   - 编写 Python 脚本处理输入输出
   - 测试单独运行

4. **连接模块**
   - 移除 State-Space 模块
   - 插入新模块
   - 保持 Buffer / Response_spectrum / Controller 不变

5. **测试**
   - 先用短仿真 (30s) 验证
   - 检查数据流是否正确

---

## 预期性能

| 指标 | 估算 |
|------|------|
| 每段 Abaqus 计算时间 | 1~10 分钟 (取决于模型复杂度) |
| 300s 仿真 = 20 段 | 总计 20~200 分钟 |
| 相比实时 | 比实际慢 100~1000 倍，但**可行** |
