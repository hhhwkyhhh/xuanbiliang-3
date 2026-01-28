%% verify_control_results.m
% 功能：对比 MIMO 随机振动控制前后的效果 (复现论文图 4.10 和 4.11)
% 依赖：运行完 control_model.slx，工作区需有 out.Syy_log 和 变量 R

clc; close all;
% 
% % %% 1. 检查数据
% if ~isfield(out, 'Syy_log')
%     error('未找到 out.Syy_log，请在 Simulink 中 Response_spectrum_calculation 的 Syy 端口添加 To Workspace 模块！');
% end

% %% 2. 数据提取
% Syy_log 数据结构: [513 x 2 x 2 x TimeSteps]
Syy_data = out.Syy_log.Data;
Time_data = out.Syy_log.Time;

% 提取 "修正前" 的结果 (Initial Response)
% 取第 10 秒左右的数据 (避开 t=0 的瞬态，取第一次平均完成后的结果)
% 我们的控制周期大约是 6秒 (60帧 * 0.1s)，所以取 t=8s 附近的值
idx_before = find(Time_data >= 55, 1);
if isempty(idx_before), idx_before = 1; end
Syy_before = squeeze(Syy_data(:, :, :, idx_before));

% 提取 "修正后" 的结果 (Final Response)
% 取仿真结束前倒数第2个点
idx_after = length(Time_data) - 1;
Syy_after = squeeze(Syy_data(:, :, :, idx_after));

% %% 3. 单位转换与参考谱准备
g = 9.80665;
f_plot = f; % 频率轴来自 prepare 脚本

% 将 SI 单位 ((m/s^2)^2/Hz) 转换为 工程单位 (g^2/Hz)
scale_g = 1 / g^2;

Syy_before_g = Syy_before * scale_g;
Syy_after_g  = Syy_after * scale_g;
R_g          = R * scale_g; % 参考谱也转换

% %% 4. 绘图参数设置
% 论文中通常使用 PSD/g^2/Hz 为纵轴，对数坐标
xlim_range = [20, 2000];
ylim_range = [1e-5, 1]; % 根据 0.1g^2/Hz 的参考谱调整

% 定义容差带 (Tolerance Lines) +/- 3dB
% +3dB = * 2, -3dB = / 2
tol_upper = 2; 
tol_lower = 0.5;

% %% 5. 绘制对比图 (控制点 1 & 2)

figure('Name', 'MIMO Control Validation', 'Color', 'w', 'Position', [100, 100, 1200, 500]);

for ch = 1:2
    subplot(1, 2, ch);
    
    % 1. 提取曲线
    % 自功率谱: 对角线元素 (ch, ch)
    ref_curve = real(squeeze(R_g(:, ch, ch)));
    before_curve = real(squeeze(Syy_before_g(:, ch, ch)));
    after_curve = real(squeeze(Syy_after_g(:, ch, ch)));
    
    % 2. 绘制参考谱 (黑色加粗)
    loglog(f_plot, ref_curve, 'k-', 'LineWidth', 2, 'DisplayName', '参考谱 (Ref)');
    hold on;
    
    % 3. 绘制容差带 (黑色虚线)
    loglog(f_plot, ref_curve * tol_upper, 'k--', 'LineWidth', 0.5, 'DisplayName', '+3dB');
    loglog(f_plot, ref_curve * tol_lower, 'k--', 'LineWidth', 0.5, 'DisplayName', '-3dB');
    
    % 4. 绘制修正前 (蓝色细线)
    loglog(f_plot, before_curve, 'b-', 'LineWidth', 1, 'DisplayName', '修正前 (Initial)');
    
    % 5. 绘制修正后 (红色实线)
    loglog(f_plot, after_curve, 'r-', 'LineWidth', 1.5, 'DisplayName', '修正后 (Final)');
    
    % 6. 图表美化
    grid on;
    xlabel('Frequency (Hz)');
    ylabel('PSD (g^2/Hz)');
    title(sprintf('控制点 %d 自功率谱控制效果', ch));
    xlim(xlim_range);
    ylim(ylim_range);
    legend('Location', 'SouthWest');
end

% %% 6. 计算均方根误差 (RMSE) 验证收敛性
fprintf('--- 控制精度验证 ---\n');
for ch = 1:2
    % 只统计控制频段 (20-2000Hz)
    ctrl_idx = find(f_plot >= 20 & f_plot <= 2000);
    
    % 计算对数域的误差 (dB Error)
    % Error_dB = 10 * log10(Response / Ref)
    err_dB = 10 * log10(Syy_after_g(ctrl_idx, ch, ch) ./ R_g(ctrl_idx, ch, ch));
    
    rmse_dB = sqrt(mean(err_dB.^2));
    max_err = max(abs(err_dB));
    
    fprintf('通道 %d: RMSE = %.2f dB, 最大误差 = %.2f dB\n', ch, rmse_dB, max_err);
end