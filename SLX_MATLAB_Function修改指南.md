# SLX 内 MATLAB Function 修改指南

## 修复问题清单
1. [Medium] 起始0.5s输出全零 - Signal_Generator
2. [Medium] NaN/Inf保护缺失 - Controller
3. [Medium] COLA窗口问题 - Signal_Generator 和 Response_spectrum_calculation
4. [Low] 注释滞后 - 所有三个MATLAB Function
5. [Low] DC/Nyquist能量偏低 - Signal_Generator

---

## 1. Signal_Generator 修改 (`control_model.slx`)

**打开方式**：双击 `control_model.slx` → 双击 `Signal_Generator` 模块

### 1.1 修改注释（第29-44行区域）

```matlab
% 修改前
function u = Signal_Generator(L_old, Z)
% Signal_Generator
% 功能：基于矩阵幂次控制变量 L_old，生成 MIMO 时域驱动力信号
% 方法：频域高斯随机化 + IFFT + 重叠相加 (Overlap-Add)
% 输入：
%   L_old: [513x2x2] 加速度域控制因子 (复数下三角)
%   Z:     [513x2x2] 逆系统矩阵 (Force/Acceleration)

    % %% 1. 参数定义
    N = 1024;               % FFT 点数
    Overlap = 512;          % 重叠点数 (50%)

% 修改后
function u = Signal_Generator(L_old, Z)
% Signal_Generator
% 功能：基于矩阵幂次控制变量 L_old，生成 MIMO 时域驱动力信号
% 方法：频域高斯随机化 + IFFT + 重叠相加 (Overlap-Add)
% 配置：1Hz分辨率 (N=5120, N_half=2561)
% 输入：
%   L_old: [2561x2x2] 加速度域控制因子 (复数下三角)
%   Z:     [2561x2x2] 逆系统矩阵 (Force/Acceleration)

    % %% 1. 参数定义
    N = 5120;               % FFT 点数 (1Hz分辨率)
    Overlap = 2560;         % 重叠点数 (50%)
```

### 1.2 修复起始全零问题（初始化部分）

```matlab
% 修改前 (约第53行)
    if isempty(play_idx)
        play_idx = 1;
        Play_Buffer = zeros(Overlap, 2);
        Overlap_Buffer = zeros(Overlap, 2);

% 修改后
    if isempty(play_idx)
        play_idx = Overlap + 1;  % 【修复】强制首次触发生成，避免前0.5s全零
        Play_Buffer = zeros(Overlap, 2);
        Overlap_Buffer = zeros(Overlap, 2);
```

### 1.3 修复COLA窗口问题（第60行）

```matlab
% 修改前
        h_win = 0.5 * (1 - cos(2 * pi * (0:N-1)' / (N-1)));

% 修改后 (周期性Hann窗，满足50% overlap COLA条件)
        h_win = 0.5 * (1 - cos(2 * pi * (0:N-1)' / N));
```

### 1.4 修复DC/Nyquist能量问题（第116-117行区域）

```matlab
% 修改前
        % === B. 构造 Hermitian 对称 (保证时域为实数) ===
        % DC(1) 和 Nyquist(513) 虚部置 0
        U_full(1, :) = real(U_full(1, :));
        U_full(N_half, :) = real(U_full(N_half, :));

% 修改后 (补偿DC/Nyquist的方差减半问题)
        % === B. 构造 Hermitian 对称 (保证时域为实数) ===
        % DC(1) 和 Nyquist(2561) 使用实高斯，并补偿方差 (乘sqrt(2))
        U_full(1, :) = real(U_full(1, :)) * sqrt(2);
        U_full(N_half, :) = real(U_full(N_half, :)) * sqrt(2);
```

---

## 2. Response_spectrum_calculation 修改 (`control_model.slx`)

**打开方式**：双击 `control_model.slx` → 双击 `Response_spectrum_calculation` 模块

### 2.1 修改注释（第27-37行区域）

```matlab
% 修改前
function [Syy, isPeriod] = Response_spectrum_calculation(y)
% Response_spectrum_calculation
% 功能：计算加速度响应的自功率谱矩阵，并进行线性平均
% 输入: y [1024x2] (时域帧)
% 输出: Syy [513x2x2] (平均后的功率谱), isPeriod (触发标志)

    % %% 1. 参数定义 (必须与 prepare_control_model.m 一致)
    N = 1024;           % FFT 点数
    Fs = 5120;          % 采样率
    AvgNum = 60;        % 平均帧数
    N_half = N/2 + 1;   % 513

% 修改后
function [Syy, isPeriod] = Response_spectrum_calculation(y)
% Response_spectrum_calculation
% 功能：计算加速度响应的自功率谱矩阵，并进行线性平均
% 配置：1Hz分辨率 (N=5120, N_half=2561)
% 输入: y [5120x2] (时域帧)
% 输出: Syy [2561x2x2] (平均后的功率谱), isPeriod (触发标志)

    % %% 1. 参数定义 (必须与 prepare_control_model.m 一致)
    N = 5120;           % FFT 点数 (1Hz分辨率)
    Fs = 5120;          % 采样率
    AvgNum = 30;        % 平均帧数 (控制周期约15s)
    N_half = N/2 + 1;   % 2561
```

### 2.2 修复COLA窗口问题（第56行）

```matlab
% 修改前
    w = 0.5 * (1 - cos(2 * pi * (0:N-1)' / (N-1)));

% 修改后 (周期性Hann窗，与Signal_Generator保持一致)
    w = 0.5 * (1 - cos(2 * pi * (0:N-1)' / N));
```

---

## 3. Controller 修改 (`control_model.slx`)

**打开方式**：双击 `control_model.slx` → 双击 `Controller` 模块

### 3.1 修改注释（第29-38行区域）

```matlab
% 修改前
function L_new = Controller(L_old, Syy, R, isPeriod)
% Controller
% 功能：基于矩阵幂次算法更新控制变量
% 输入：
%   L_old:    [513x2x2] 上一时刻的控制变量
%   Syy:      [513x2x2] 平均响应谱 (Response Spectrum)
%   R:        [513x2x2] 参考谱 (Reference Spectrum)

% 修改后
function L_new = Controller(L_old, Syy, R, isPeriod)
% Controller
% 功能：基于矩阵幂次算法更新控制变量
% 配置：1Hz分辨率 (N_half=2561)
% 输入：
%   L_old:    [2561x2x2] 上一时刻的控制变量
%   Syy:      [2561x2x2] 平均响应谱 (Response Spectrum)
%   R:        [2561x2x2] 参考谱 (Reference Spectrum)
```

### 3.2 修复NaN/Inf保护（在Syy_max检查后，约第52-56行）

```matlab
% 修改前
       Syy_max = max(abs(Syy(:)));
       if Syy_max < 1e-20
        % Syy 无效 (全零或接近零)，跳过本次修正
        L_new = L_old;
        return;  % 提前返回
    end

% 修改后
       Syy_max = max(abs(Syy(:)));
       % 【新增】NaN/Inf 保护
       if ~isfinite(Syy_max) || Syy_max < 1e-20
        % Syy 无效 (全零、NaN或Inf)，跳过本次修正
        L_new = L_old;
        return;
    end
```

### 3.3 在循环内添加频点级别的NaN/Inf保护（约第64-66行后）

```matlab
% 在提取 R_k, Syy_k, L_old_k 之后，添加：

            % 【新增】频点级别 NaN/Inf 保护
            if any(~isfinite(Syy_k(:))) || any(~isfinite(R_k(:))) || any(~isfinite(L_old_k(:)))
                L_new(k, :, :) = L_old_k;
                continue;
            end
```

完整的循环开头部分应该变成：

```matlab
        for k = 1:N_half
            % 提取当前频点的矩阵 (2x2)
            R_k = squeeze(R(k, :, :));
            Syy_k = squeeze(Syy(k, :, :));
            L_old_k = squeeze(L_old(k, :, :));
            
            % 【新增】频点级别 NaN/Inf 保护
            if any(~isfinite(Syy_k(:))) || any(~isfinite(R_k(:))) || any(~isfinite(L_old_k(:)))
                L_new(k, :, :) = L_old_k;
                continue;
            end
            
            % --- 步骤 A: 响应谱 Syy 分解 (Ls) ---
            % ... 后续代码不变 ...
```

---

## 4. H_Estimation.slx 中的 SpectralEstimation 修改

**打开方式**：双击 `H_Estimation.slx` → 双击 `SpectralEstimation` 模块

### 4.1 修改注释和参数

```matlab
% 修改前
function [Suu, Syu, Syy, f] = SpectralEstimation(u, y)
% SpectralEstimation 计算MIMO系统的瞬时自功率谱和互功率谱
% 
% 输入:
%   u: 激励力信号 [1024 x 2] (时域)
%   y: 加速度响应信号 [1024 x 2] (时域)
%
% 输出:
%   Suu: 激励力的自功率谱矩阵 [513 x 2 x 2] (频域)
%   Syu: 响应与激励的互功率谱矩阵 [513 x 2 x 2] (频域)
%   f:   频率向量 [513 x 1]

    % %% 1. 参数定义
    Fs = 5120;          % 采样频率 (Hz)
    N = 1024;           % FFT点数 (等于Buffer大小)
    N_half = N/2 + 1;   % 单边谱的谱线数 (513)

% 修改后
function [Suu, Syu, Syy, f] = SpectralEstimation(u, y)
% SpectralEstimation 计算MIMO系统的瞬时自功率谱和互功率谱
% 配置：1Hz分辨率 (N=5120, N_half=2561)
% 
% 输入:
%   u: 激励力信号 [5120 x 2] (时域)
%   y: 加速度响应信号 [5120 x 2] (时域)
%
% 输出:
%   Suu: 激励力的自功率谱矩阵 [2561 x 2 x 2] (频域)
%   Syu: 响应与激励的互功率谱矩阵 [2561 x 2 x 2] (频域)
%   f:   频率向量 [2561 x 1]

    % %% 1. 参数定义
    Fs = 5120;          % 采样频率 (Hz)
    N = 5120;           % FFT点数 (1Hz分辨率)
    N_half = N/2 + 1;   % 单边谱的谱线数 (2561)
```

### 4.2 修复COLA窗口问题（第47行）

```matlab
% 修改前
    w = 0.5 * (1 - cos(2 * pi * (0:N-1)' / (N-1)));

% 修改后 (周期性Hann窗)
    w = 0.5 * (1 - cos(2 * pi * (0:N-1)' / N));
```

---

## 修改检查清单

完成修改后，请逐项确认：

- [ ] **Signal_Generator**
  - [ ] 注释更新为 2561
  - [ ] `play_idx = Overlap + 1` (修复起始全零)
  - [ ] 窗函数改为 `/ N` (COLA)
  - [ ] DC/Nyquist 乘 `sqrt(2)` (能量补偿)

- [ ] **Response_spectrum_calculation**
  - [ ] 注释更新为 5120/2561
  - [ ] `AvgNum = 30`
  - [ ] 窗函数改为 `/ N` (COLA)

- [ ] **Controller**
  - [ ] 注释更新为 2561
  - [ ] `~isfinite(Syy_max)` 检查
  - [ ] 频点级别 `isfinite` 保护

- [ ] **SpectralEstimation** (H_Estimation.slx)
  - [ ] 注释更新为 5120/2561
  - [ ] 窗函数改为 `/ N` (COLA)

---

## 验证测试

修改完成后，建议按以下步骤验证：

1. 运行 `spaceABCD.m`
2. 运行 `H_Estimation.slx` (60s)
3. 运行 `system_model.m`
4. 验证 `size(H)` = [2561, 2, 2]
5. 运行 `prepare_control_model.m`
6. 运行 `control_model.slx` (300s)
7. 观察命令窗口无 NaN/Inf 警告
8. 运行 `verify_control_results.m` 检查收敛效果
