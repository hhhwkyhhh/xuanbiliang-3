%% verify_time_to_psd.m
% 使用 1 秒时域数据反算频谱，并与目标谱对比
% 输出图：S11 对比、S22 对比、|S12| 对比（全部分开绘制）

clear; clc;

%% 输入文件
spec_txt = 'force_spectrum_0_2000Hz_df1Hz.txt';
time_txt = 'force_time_1s_fs5096.txt';

%% 读入目标谱
spec_data = readmatrix(spec_txt);
f_ref = spec_data(:,1);
S11_ref = spec_data(:,2);
S22_ref = spec_data(:,3);
S12_ref = spec_data(:,4) + 1j*spec_data(:,5);

%% 读入时域信号
time_data = readmatrix(time_txt);
t = time_data(:,1);
u1 = time_data(:,2);
u2 = time_data(:,3);

Fs = 1 / mean(diff(t));
N = length(t);
N_half = N/2 + 1;
f = (0:N_half-1)' * (Fs/N);

%% 计算单边谱（Periodogram）
U1 = fft(u1);
U2 = fft(u2);
U1 = U1(1:N_half);
U2 = U2(1:N_half);

% 计算单边 PSD/CSD
S11 = (abs(U1).^2) / (Fs*N);
S22 = (abs(U2).^2) / (Fs*N);
S12 = (U1 .* conj(U2)) / (Fs*N);

% 单边修正：除 DC 与 Nyquist 外乘 2
if N_half > 2
    S11(2:end-1) = 2*S11(2:end-1);
    S22(2:end-1) = 2*S22(2:end-1);
    S12(2:end-1) = 2*S12(2:end-1);
end

%% 截取 0–2000 Hz
idx = (f >= 0) & (f <= 2000);
f_plot = f(idx);

S11 = S11(idx);
S22 = S22(idx);
S12 = S12(idx);

%% 脚本所在目录（用于保存图）
script_dir = fileparts(mfilename('fullpath'));
if isempty(script_dir)
    script_dir = pwd;
end

%% 绘图：S11
figure('Name','PSD Verify - S11','Color','w','Position',[100 100 1000 450]);
loglog(f_ref, S11_ref, 'k--', 'LineWidth', 1.5); hold on;
loglog(f_plot, S11, 'b-', 'LineWidth', 1.0);
grid on; xlim([1, 2500]);
xlabel('Frequency (Hz)'); ylabel('PSD (kN^2/Hz)');
title('S11：目标谱 vs 时域反算');
legend('目标谱','时域反算','Location','SouthWest');
exportgraphics(gcf, fullfile(script_dir, 'verify_s11.png'));

%% 绘图：S22
figure('Name','PSD Verify - S22','Color','w','Position',[120 120 1000 450]);
loglog(f_ref, S22_ref, 'k--', 'LineWidth', 1.5); hold on;
loglog(f_plot, S22, 'r-', 'LineWidth', 1.0);
grid on; xlim([1, 2500]);
xlabel('Frequency (Hz)'); ylabel('PSD (kN^2/Hz)');
title('S22：目标谱 vs 时域反算');
legend('目标谱','时域反算','Location','SouthWest');
exportgraphics(gcf, fullfile(script_dir, 'verify_s22.png'));

%% 绘图：|S12|
figure('Name','PSD Verify - |S12|','Color','w','Position',[140 140 1000 450]);
loglog(f_ref, abs(S12_ref), 'k--', 'LineWidth', 1.5); hold on;
loglog(f_plot, abs(S12), 'm-', 'LineWidth', 1.0);
grid on; xlim([1, 2500]);
xlabel('Frequency (Hz)'); ylabel('|S12| (kN^2/Hz)');
title('|S12|：目标谱 vs 时域反算');
legend('目标谱','时域反算','Location','SouthWest');
exportgraphics(gcf, fullfile(script_dir, 'verify_s12_mag.png'));

fprintf('验证图已保存到：%s\n', script_dir);
