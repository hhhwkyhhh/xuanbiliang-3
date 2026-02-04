%% verify_time_to_psd_welch.m
% 使用 Welch 方法（分段 + Hann 窗 + 平均）反算频谱并与目标谱对比
% 不加长时间，仅使用 1 秒信号的分段平均

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

%% Welch 参数（分段 + Hann 窗 + 平均）
seg_len = 1024;                 % 每段长度
overlap = seg_len/2;            % 50% 重叠
hop = seg_len - overlap;
win = hann(seg_len, 'periodic');
W = sum(win.^2);

n_seg = floor((N - overlap) / hop);

N_half = seg_len/2 + 1;
f = (0:N_half-1)' * (Fs/seg_len);

S11 = zeros(N_half,1);
S22 = zeros(N_half,1);
S12 = complex(zeros(N_half,1));

for s = 1:n_seg
    idx = (s-1)*hop + (1:seg_len);
    if idx(end) > N
        break;
    end
    x1 = u1(idx) .* win;
    x2 = u2(idx) .* win;

    X1 = fft(x1);
    X2 = fft(x2);
    X1 = X1(1:N_half);
    X2 = X2(1:N_half);

    % 单边 PSD/CSD
    P11 = (abs(X1).^2) / (Fs * W);
    P22 = (abs(X2).^2) / (Fs * W);
    P12 = (X1 .* conj(X2)) / (Fs * W);

    % 单边修正：除 DC 与 Nyquist 外乘 2
    if N_half > 2
        P11(2:end-1) = 2*P11(2:end-1);
        P22(2:end-1) = 2*P22(2:end-1);
        P12(2:end-1) = 2*P12(2:end-1);
    end

    S11 = S11 + P11;
    S22 = S22 + P22;
    S12 = S12 + P12;
end

S11 = S11 / n_seg;
S22 = S22 / n_seg;
S12 = S12 / n_seg;

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
figure('Name','PSD Welch Verify - S11','Color','w','Position',[100 100 1000 450]);
loglog(f_ref, S11_ref, 'k--', 'LineWidth', 1.5); hold on;
loglog(f_plot, S11, 'b-', 'LineWidth', 1.0);
grid on; xlim([1, 2500]);
xlabel('Frequency (Hz)'); ylabel('PSD (kN^2/Hz)');
title('Welch S11：目标谱 vs 时域反算');
legend('目标谱','Welch 反算','Location','SouthWest');
exportgraphics(gcf, fullfile(script_dir, 'verify_welch_s11.png'));

%% 绘图：S22
figure('Name','PSD Welch Verify - S22','Color','w','Position',[120 120 1000 450]);
loglog(f_ref, S22_ref, 'k--', 'LineWidth', 1.5); hold on;
loglog(f_plot, S22, 'r-', 'LineWidth', 1.0);
grid on; xlim([1, 2500]);
xlabel('Frequency (Hz)'); ylabel('PSD (kN^2/Hz)');
title('Welch S22：目标谱 vs 时域反算');
legend('目标谱','Welch 反算','Location','SouthWest');
exportgraphics(gcf, fullfile(script_dir, 'verify_welch_s22.png'));

%% 绘图：|S12|
figure('Name','PSD Welch Verify - |S12|','Color','w','Position',[140 140 1000 450]);
loglog(f_ref, abs(S12_ref), 'k--', 'LineWidth', 1.5); hold on;
loglog(f_plot, abs(S12), 'm-', 'LineWidth', 1.0);
grid on; xlim([1, 2500]);
xlabel('Frequency (Hz)'); ylabel('|S12| (kN^2/Hz)');
title('Welch |S12|：目标谱 vs 时域反算');
legend('目标谱','Welch 反算','Location','SouthWest');
exportgraphics(gcf, fullfile(script_dir, 'verify_welch_s12_mag.png'));

fprintf('Welch 验证图已保存到：%s\n', script_dir);
