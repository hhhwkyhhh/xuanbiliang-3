%% prepare_control_model_2.m (验证版本 - 不同参考谱, 1Hz分辨率)
% 功能：MIMO 随机振动控制仿真初始化
% 目的：使用不同的参考谱 R 验证矩阵幂次控制算法的有效性
% 修改记录：频率分辨率从5Hz改为1Hz (N_frame: 1024→5120)
% 变化：
%   - 平直段电平: 0.05 g^2/Hz (原为 0.1)
%   - 控制频段: 50-1500 Hz (原为 20-2000)
%   - 斜率: +6dB/oct 上升, -6dB/oct 下降 (原为 ±3dB/oct)
%   - 相干性: 0.5 固定 (原为 0.1→0.8 渐变)

clear; clc;

%% 1. 核心仿真参数 (与原版保持一致 - 1Hz分辨率)
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
    if size(Z, 1) ~= n_freq
        error('Z矩阵频率点数不匹配！请重新运行 system_model.m');
    end
    fprintf('已加载系统逆模型 Z。\n');
else
    error('缺少 system_model.mat！');
end

%% 3. 定义新的参考谱 R (Reference Spectrum) - 验证版本
% ========================================
% 新规格:
%   控制频段: 50 - 1500 Hz
%   平直段: 200 - 800 Hz, 0.05 g^2/Hz
%   上升段: 50 - 200 Hz, +6 dB/oct
%   下降段: 800 - 1500 Hz, -6 dB/oct
%   相干性: 固定 0.5
%   相位差: 45度 (原为 90度)
% ========================================

fprintf('\n========================================\n');
fprintf('【验证版本】新参考谱规格:\n');
fprintf('  频段: 50 - 1500 Hz\n');
fprintf('  平直段: 200 - 800 Hz @ 0.05 g²/Hz\n');
fprintf('  斜率: ±6 dB/oct\n');
fprintf('  相干性: 0.5 (固定)\n');
fprintf('========================================\n\n');

% A. 幅值谱 S_ref (SI Units)
g = 9.80665;
ref_level_g = 0.05;  % 【变化】0.05 g^2/Hz (原为 0.1)
ref_level_si = ref_level_g * g^2;

S_ref_profile = zeros(n_freq, 1);

% 使用逻辑索引定义各段
idx_50_200   = (f >= 50) & (f < 200);     % 上升段
idx_200_800  = (f >= 200) & (f < 800);    % 平直段
idx_800_1500 = (f >= 800) & (f <= 1500);  % 下降段

% 1. 上升段 (+6dB/oct) - 从200Hz反推
%    +6dB/oct 意味着幅值与频率成正比 (因为 6dB ≈ 功率翻倍/oct)
freqs_up = f(idx_50_200);
% 公式: S(f) = S_ref * (f / f_ref)^(slope_dB / 10 * log10(2))
% 简化: 对于 +6dB/oct, S(f) = S_ref * (f / 200)^2
S_ref_profile(idx_50_200) = ref_level_si * (freqs_up / 200).^2;

% 2. 平直段
S_ref_profile(idx_200_800) = ref_level_si;

% 3. 下降段 (-6dB/oct)
freqs_down = f(idx_800_1500);
% 对于 -6dB/oct, S(f) = S_ref * (800 / f)^2
S_ref_profile(idx_800_1500) = ref_level_si * (800 ./ freqs_down).^2;

% 4. 极值保护
min_val = ref_level_si * 1e-6;
S_ref_profile(S_ref_profile < min_val) = min_val;

% B. 构造 R 矩阵 (含相干与相位)
R = complex(zeros(n_freq, 2, 2));

% 【变化】固定相干性 0.5，相位差 45度
Coh_fixed = 0.5;
Phase_val = pi/4;  % 45度 (原为 90度)

for k = 1:n_freq
    if f(k) >= 50 && f(k) <= 1500
        S = S_ref_profile(k);
        gamma = sqrt(Coh_fixed);

        Cross = S * gamma * exp(1j * Phase_val);

        R(k, :, :) = [S,       Cross;
            conj(Cross), S];
    else
        % 非控制频段
        R(k, :, :) = eye(2) * min_val;
    end
end

%% 4. 计算初始控制变量 L_init
L_init = complex(zeros(n_freq, 2, 2));

for k = 1:n_freq
    R_k = squeeze(R(k, :, :));
    R_k = R_k + eye(2) * (ref_level_si * 1e-10);

    try
        L_init(k, :, :) = chol(R_k, 'lower');
    catch
        L_init(k, :, :) = zeros(2, 2);
    end
end

%% 5. 绘制新旧参考谱对比图
figure('Name', '参考谱对比', 'Color', 'w', 'Position', [100, 100, 900, 500]);

% 加载原版参考谱用于对比 (如果可用)
try
    % 临时计算原版谱供对比
    ref_level_g_old = 0.1;
    ref_level_si_old = ref_level_g_old * g^2;
    S_old = zeros(n_freq, 1);

    idx_old_up = (f >= 20) & (f < 100);
    idx_old_flat = (f >= 100) & (f < 1000);
    idx_old_down = (f >= 1000) & (f <= 2000);

    S_old(idx_old_up) = ref_level_si_old * (10 .^ (3 * log2(f(idx_old_up)/100) / 10));
    S_old(idx_old_flat) = ref_level_si_old;
    S_old(idx_old_down) = ref_level_si_old * (10 .^ (-3 * log2(f(idx_old_down)/1000) / 10));
    S_old(S_old < min_val) = min_val;

    % 转换为 g^2/Hz
    loglog(f, S_old / g^2, 'b--', 'LineWidth', 1.5, 'DisplayName', '原版 (0.1 g²/Hz, 20-2000Hz)');
    hold on;
catch
    % 无法计算原版，跳过
end

% 绘制新版参考谱
loglog(f, S_ref_profile / g^2, 'r-', 'LineWidth', 2, 'DisplayName', '新版 (0.05 g²/Hz, 50-1500Hz)');

% 图表装饰
grid on;
xlabel('Frequency (Hz)');
ylabel('PSD (g²/Hz)');
title('参考谱对比: 原版 vs 验证版本');
legend('Location', 'SouthWest');
xlim([10, 3000]);
ylim([1e-5, 1]);

% 标注关键频率
xline(50, 'r:', 'LineWidth', 1);
xline(200, 'r:', 'LineWidth', 1);
xline(800, 'r:', 'LineWidth', 1);
xline(1500, 'r:', 'LineWidth', 1);

text(50, 0.3, '50Hz', 'Color', 'r', 'FontSize', 10);
text(200, 0.3, '200Hz', 'Color', 'r', 'FontSize', 10);
text(800, 0.3, '800Hz', 'Color', 'r', 'FontSize', 10);
text(1500, 0.3, '1500Hz', 'Color', 'r', 'FontSize', 10);

%% 6. 输出汇总
fprintf('------------------------------------------------\n');
fprintf('初始化完成 (验证版本)\n');
fprintf('1. 参考谱 R 已生成:\n');
fprintf('   - 控制频段: 50 - 1500 Hz\n');
fprintf('   - 平直段电平: 0.05 g²/Hz @ 200-800 Hz\n');
fprintf('   - 斜率: ±6 dB/oct\n');
fprintf('   - 相干性: 0.5 (固定)\n');
fprintf('   - 相位差: 45°\n');
fprintf('2. 初始变量 L_init 已计算\n');
fprintf('3. 仿真参数: Fs=%d, Tsim=%d, AvgNum=%d\n', Fs, T_sim, AvgNum);
fprintf('------------------------------------------------\n');
fprintf('\n请运行 control_model.slx 进行仿真验证。\n');
