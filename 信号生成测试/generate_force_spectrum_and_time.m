%% generate_force_spectrum_and_time.m
% 生成 2x2 力功率谱密度（Suu）与 1 秒时域力信号
% 输出：
%   - TXT 文件：f, S11, S22, S12, S21（复数拆成实部/虚部）
%   - 频谱图与时域信号图（分开绘制）
%
% 说明：
%   - PSD 单位：kN^2/Hz
%   - 频率分辨率：1 Hz（Fs = N = 5096）
%   - 输出频段：0–2000 Hz
%
% 如需调整谱形，请修改“用户参数设置”部分。

clear; clc;

%% 用户参数设置
Fs = 5096;          % 采样频率 (Hz)
N = Fs;             % 1 秒数据 -> df = 1 Hz
f_low  = 20;        % Start of control band (Hz)
f_flat1 = 100;      % Flat-band start (Hz)
f_flat2 = 1000;     % Flat-band end (Hz)
f_high = 2000;      % End of control band (Hz)

S_flat = 10;        % 平直段电平 (kN^2/Hz)
ratio_S22 = 1.0;    % S22 = ratio_S22 * S11

coh_min = 0.1;      % coherence at f_low
coh_max = 0.8;      % coherence at f_high
phase_deg = 90;     % cross-spectrum phase (deg)

out_txt = 'force_spectrum_0_2000Hz_df1Hz.txt';
out_time_txt = 'force_time_1s_fs5096.txt';

%% 导出量
df = Fs / N;
N_half = N/2 + 1;
f = (0:N_half-1)' * df;

% 脚本所在目录（用于保存图）
script_dir = fileparts(mfilename('fullpath'));
if isempty(script_dir)
    script_dir = pwd;
end

%% 构造目标自谱（S11, S22）
S11 = zeros(N_half,1);

idx_up   = (f >= f_low)  & (f < f_flat1);
idx_flat = (f >= f_flat1) & (f < f_flat2);
idx_down = (f >= f_flat2) & (f <= f_high);

% +3 dB/oct up
f_up = f(idx_up);
S11(idx_up) = S_flat * (10 .^ (3 * log2(f_up / f_flat1) / 10));

% flat
S11(idx_flat) = S_flat;

% -3 dB/oct down
f_down = f(idx_down);
S11(idx_down) = S_flat * (10 .^ (-3 * log2(f_down / f_flat2) / 10));

% floor to avoid zeros
min_val = S_flat * 1e-6;
S11(S11 < min_val) = min_val;

% S22 as scaled version
S22 = ratio_S22 * S11;

%% 构造互谱（S12, S21）
S12 = complex(zeros(N_half,1));
phase_rad = phase_deg * pi/180;
coh_vals = interp1([f_low, f_high], [coh_min, coh_max], f, 'linear', coh_min);

for k = 1:N_half
    if f(k) >= f_low && f(k) <= f_high
        gamma = sqrt(max(0, min(1, coh_vals(k))));
        S12(k) = gamma * sqrt(S11(k) * S22(k)) * exp(1j * phase_rad);
    else
        S12(k) = 0;
    end
end

S21 = conj(S12);

%% 组装 Suu(f) = [S11 S12; S21 S22]
Suu = complex(zeros(N_half,2,2));
for k = 1:N_half
    Suu(k,:,:) = [S11(k), S12(k); S21(k), S22(k)];
end

%% 写入 TXT（0–2000 Hz，1 Hz 分辨率）
idx_out = (f >= 0) & (f <= f_high);
f_out = f(idx_out);

fid = fopen(out_txt, 'w');
fprintf(fid, '# Force PSD/CSD (units: kN^2/Hz)\n');
fprintf(fid, '# Columns: f, S11, S22, Re(S12), Im(S12), Re(S21), Im(S21)\n');
for k = find(idx_out).'
    fprintf(fid, '%.0f %.6e %.6e %.6e %.6e %.6e %.6e\n', ...
        f(k), S11(k), S22(k), real(S12(k)), imag(S12(k)), real(S21(k)), imag(S21(k)));
end
fclose(fid);

%% 生成 1 秒时域力信号（2 通道）
% 使用频域合成，目标为 Suu
U_full = complex(zeros(N,2));
rng(1); % fixed seed for repeatability

scale = sqrt(Fs * N / 2);   % for k=2..N/2
scale_dc = sqrt(Fs * N);    % for DC and Nyquist
reg = S_flat * 1e-12;

for k = 1:N_half
    Suu_k = squeeze(Suu(k,:,:));
    Suu_k = (Suu_k + Suu_k')/2;
    [L,p] = chol(Suu_k + eye(2)*reg, 'lower');
    if p > 0
        % fallback: eigen regularization
        [V,D] = eig(Suu_k);
        d = diag(D);
        d(d < reg) = reg;
        L = V * diag(sqrt(d));
    end
    w = (randn(2,1) + 1j*randn(2,1)) / sqrt(2);
    U_k = L * w;
    if k == 1 || k == N_half
        U_full(k,:) = real(U_k).' * scale_dc;
    else
        U_full(k,:) = U_k.' * scale;
    end
end

% 构造共轭对称，保证时域为实数
U_full(N_half+1:end,:) = conj(flipud(U_full(2:N_half-1,:)));

u_time = ifft(U_full, N, 1, 'symmetric'); % N x 2
t = (0:N-1)' / Fs;

%% 写入时域信号 TXT（2 通道）
fid = fopen(out_time_txt, 'w');
fprintf(fid, '# Time-domain force (units: kN)\n');
fprintf(fid, '# Columns: t, u1, u2\n');
for k = 1:N
    fprintf(fid, '%.9f %.6e %.6e\n', t(k), u_time(k,1), u_time(k,2));
end
fclose(fid);

%% 绘图（分开绘制）
figure('Name','Force PSD - Auto Spectrum S11','Color','w','Position',[100 100 1000 450]);
loglog(f, S11, 'b-', 'LineWidth', 1.5);
grid on; xlim([1, 2500]);
xlabel('Frequency (Hz)'); ylabel('PSD (kN^2/Hz)');
title('自谱 S11');
exportgraphics(gcf, fullfile(script_dir, 'psd_s11.png'));

figure('Name','Force PSD - Auto Spectrum S22','Color','w','Position',[120 120 1000 450]);
loglog(f, S22, 'r--', 'LineWidth', 1.5);
grid on; xlim([1, 2500]);
xlabel('Frequency (Hz)'); ylabel('PSD (kN^2/Hz)');
title('自谱 S22');
exportgraphics(gcf, fullfile(script_dir, 'psd_s22.png'));

figure('Name','Force PSD - Cross Spectrum','Color','w','Position',[140 140 1000 450]);
loglog(f, abs(S12), 'k-', 'LineWidth', 1.5);
grid on; xlim([1, 2500]);
xlabel('Frequency (Hz)'); ylabel('|S12| (kN^2/Hz)');
title('互谱幅值 |S12|');
exportgraphics(gcf, fullfile(script_dir, 'psd_s12_mag.png'));

figure('Name','Time-domain Force CH1','Color','w','Position',[160 160 1000 450]);
plot(t, u_time(:,1), 'b-');
grid on; xlim([0, 1]);
xlabel('Time (s)'); ylabel('Force (kN)');
title('1 秒时域力信号 - 通道 1');
exportgraphics(gcf, fullfile(script_dir, 'time_ch1.png'));

figure('Name','Time-domain Force CH2','Color','w','Position',[180 180 1000 450]);
plot(t, u_time(:,2), 'r-');
grid on; xlim([0, 1]);
xlabel('Time (s)'); ylabel('Force (kN)');
title('1 秒时域力信号 - 通道 2');
exportgraphics(gcf, fullfile(script_dir, 'time_ch2.png'));

fprintf('已写入 PSD/CSD TXT：%s\n', out_txt);
fprintf('已写入时域 TXT：%s\n', out_time_txt);
fprintf('Fs = %d Hz, N = %d, df = %.2f Hz\n', Fs, N, df);
