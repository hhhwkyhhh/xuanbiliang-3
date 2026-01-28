%% H1_plot.m (Final Version)
% 功能：绘制 2x2 MIMO 系统的频响函数 (H) 和 相干函数 (Coherence)
% 特性：横轴对数坐标，纵轴工程单位 (g/N)

clc; close all;

%% 1. 加载与预处理
if exist('system_model.mat', 'file')
    load('system_model.mat');
    fprintf('已加载 system_model.mat\n');
else
    error('未找到数据文件，请先运行 system_model.m');
end

% 物理常数与单位转换
g_const = 9.80665; % 重力加速度 m/s^2

% 绘图参数
[n_freq, n_out, n_in] = size(H);
input_labels = {'Input 1 (Node 3 Force)', 'Input 2 (Node 5 Force)'};
output_labels = {'Output 1 (Node 3 Acc)', 'Output 2 (Node 5 Acc)'};

% 对数坐标显示的频率范围 (注意：Log轴不能包含0)
f_min = 10;   % 起始频率 10Hz
f_max = 2000; % 截止频率 2000Hz

%% Figure 1: 频响函数 (Accelerance in g/N)
% 采用 Log-Log (幅值) 和 Semi-Logx (相位)
fig_h = figure('Name', 'FRF (Engineering Units)', ...
               'Color', 'w', 'Position', [100, 100, 1100, 800]);

for i = 1:n_out
    for j = 1:n_in
        idx = (i-1)*n_in + j;
        subplot(n_out, n_in, idx);
        
        % 提取数据
        H_vals = squeeze(H(:, i, j));
        
        % --- 数据转换 ---
        % 幅值：转换为 g/N
        mag_g = abs(H_vals) / g_const; 
        % 相位：角度 (deg)
        phs_deg = unwrap(angle(H_vals)) * 180/pi;
        
        % --- 绘制幅频 (左轴, Log-Log) ---
        yyaxis left
        % 使用 loglog 绘制幅值，能更好展示宽频带动态特性
        loglog(f, mag_g, 'b-', 'LineWidth', 1.5);
        ylabel('Magnitude (g/N)');
        % 设置Y轴颜色为黑色或蓝色，保持清晰
        ax = gca; ax.YColor = 'b';
       
        % %相频屏蔽一下 
       %  % --- 绘制相频 (右轴, Semi-Logx) ---
       %  yyaxis right
       %  semilogx(f, phs_deg, 'r--', 'LineWidth', 1);
       %  ylabel('Phase (deg)');
       %  ylim([-200, 200]); % 相位通常在 +/- 180 范围
       %  ax.YAxis(2).Color = 'r';
        
        % --- 通用装饰 ---
        grid on;
        % 开启次网格 (Minor Grid) 对 Log 坐标很重要
        grid minor; 
        
        xlim([f_min, f_max]);
        xlabel('Frequency (Hz) [Log Scale]');
        
        title(sprintf('H_%d%d: %s -> %s', i, j, input_labels{j}, output_labels{i}), ...
              'Interpreter', 'none', 'FontSize', 10);
          
        if i==1 && j==1
            legend({'Mag (g/N)', 'Phase (deg)'}, 'Location', 'best');
        end
    end
end
sgtitle('MIMO Frequency Response (Units: g/N)');


%% Figure 2: 相干函数 (Coherence)
% 采用 Semi-Logx
fig_coh = figure('Name', 'Coherence (Log Freq)', ...
                 'Color', 'w', 'Position', [150, 150, 1000, 600]);

for i = 1:n_out
    for j = 1:n_in
        idx = (i-1)*n_in + j;
        subplot(n_out, n_in, idx);
        
        C_vals = squeeze(Coh(:, i, j));
        
        % 绘制
        semilogx(f, C_vals, 'k-', 'LineWidth', 1.5);
        
        % 辅助线
        hold on;
        yline(0.8, 'r--', 'Threshold 0.8');
        yline(1.0, 'g:', 'Ideal 1.0');
        
        % 装饰
        grid on; grid minor;
        xlim([f_min, f_max]);
        ylim([0, 1.1]);
        
        xlabel('Frequency (Hz) [Log Scale]');
        ylabel('Coherence \gamma^2');
        title(sprintf('Coh_{%d%d}', i, j));
    end
end
sgtitle('Coherence Function Check');

fprintf('绘图完成：横轴已设为对数坐标，纵轴已转换为 g/N。\n');