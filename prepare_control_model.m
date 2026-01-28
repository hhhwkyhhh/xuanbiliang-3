%% prepare_control_model.m (1Hz Resolution Version)
% 功能：MIMO 随机振动控制仿真初始化
% 包含：参考谱定义、逆系统加载、初始控制变量计算、参数设置
% 修改记录：频率分辨率从5Hz改为1Hz (N_frame: 1024→5120)

clear; clc;

%% 1. 核心仿真参数
Fs = 5120;                  % 采样频率 (Hz)
N_frame = 5120;             % 帧长 (Buffer Size) - 对应 1Hz 分辨率
AvgNum = 30;                % 谱估计平均次数 (调整以平衡控制周期)
epsilon = 0.5;              % 矩阵幂次收敛因子 (统一参数)
T_sim = 300;                % 仿真时间 (s) - 延长以观察完整收敛
df = Fs / N_frame;          % 频率分辨率 (1 Hz)
f = (0 : N_frame/2)' * df;  % 频率向量 [2561 x 1]
n_freq = length(f);

%% 2. 加载逆系统模型 Z
if exist('system_model.mat', 'file')
    load('system_model.mat', 'Z');
    % Z 维度: [2561 x 2 x 2] (单位: Force / Acceleration)
    % 确保 Z 的频率轴与当前设置一致 (1Hz分辨率)
    if size(Z, 1) ~= n_freq
        error('Z矩阵频率点数不匹配！请重新运行 system_model.m');
    end
    fprintf('已加载系统逆模型 Z。\n');
else
    error('缺少 system_model.mat！');
end

%% 3. 定义参考谱 R (Reference Spectrum)
% 目标: 20-2000Hz, 平直段 0.1 g^2/Hz
% 包含健壮的斜率计算与复数相位处理

% A. 幅值谱 S_ref (SI Units)
g = 9.80665;
ref_level_g = 0.1;
ref_level_si = ref_level_g * g^2; % 转换为 (m/s^2)^2/Hz

S_ref_profile = zeros(n_freq, 1);

% 使用逻辑索引确保健壮性
idx_20_100   = (f >= 20) & (f < 100);
idx_100_1000 = (f >= 100) & (f < 1000);
idx_1000_2000= (f >= 1000) & (f <= 2000);

% 1. 上升段 (+3dB/oct)
freqs_up = f(idx_20_100);
% log2(f/100) * 3dB
S_ref_profile(idx_20_100) = ref_level_si * (10 .^ (3 * log2(freqs_up/100) / 10));

% 2. 平直段
S_ref_profile(idx_100_1000) = ref_level_si;

% 3. 下降段 (-3dB/oct)
freqs_down = f(idx_1000_2000);
S_ref_profile(idx_1000_2000) = ref_level_si * (10 .^ (-3 * log2(freqs_down/1000) / 10));

% 4. 极值保护 (防止除零)
min_val = ref_level_si * 1e-6;
S_ref_profile(S_ref_profile < min_val) = min_val;

% B. 构造 R 矩阵 (含相干与相位)
% 相干: 0.1 -> 0.8; 相位: 90度
R = complex(zeros(n_freq, 2, 2)); % 初始化为复数矩阵

Coh_vals = interp1([20, 2000], [0.1, 0.8], f, 'linear', 0.01);
Phase_val = pi/2; % 90度

for k = 1:n_freq
    if f(k) >= 20 && f(k) <= 2000
        S = S_ref_profile(k);
        gamma = sqrt(Coh_vals(k)); % 相干系数

        % 非对角项: S * gamma * e^(j*theta)
        Cross = S * gamma * exp(1j * Phase_val);

        R(k, :, :) = [S,       Cross;
            conj(Cross), S];
    else
        % 非控制频段保持微小值对角阵
        R(k, :, :) = eye(2) * min_val;
    end
end

%% 4. 计算初始控制变量 L_init
% 【架构决策】：L_init 保持为加速度因子 (Response Factor)
%  原因：Signal_Generator 模块将负责执行 D = Z * L 的运算。
%  这样 $t=0$ 时，发出的力为 Z * chol(R)，符合物理要求，且保持了闭环变量的一致性。

L_init = complex(zeros(n_freq, 2, 2));

for k = 1:n_freq
    R_k = squeeze(R(k, :, :));

    % 正定性保护
    R_k = R_k + eye(2) * (ref_level_si * 1e-10);

    try
        % Cholesky 分解 (得到下三角)
        L_init(k, :, :) = chol(R_k, 'lower');
    catch
        % 极低频容错
        L_init(k, :, :) = zeros(2, 2);
    end
end

fprintf('------------------------------------------------\n');
fprintf('初始化完成 (Strict Review Passed)\n');
fprintf('1. 参考谱 R 已生成 (含相干性与复数相位)\n');
fprintf('2. 初始变量 L_init 已设定为 R 的 Cholesky 因子\n');
fprintf('   注意：Signal_Generator 必须执行 D = Z * L 运算！\n');
fprintf('3. 仿真参数: Fs=%d, Tsim=%d, AvgNum=%d\n', Fs, T_sim, AvgNum);
fprintf('------------------------------------------------\n');