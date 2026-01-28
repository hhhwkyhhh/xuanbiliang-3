%% evaluate_control_error.m
% 功能：量化评估 MIMO 随机振动控制精度
% 指标：dB误差谱、RMSE (均方根误差)、最大误差
% 依赖：需先运行 control_model 仿真并保留工作区数据
% 提取基础数据
Syy_data = out.Syy_log.Data;   % [513 x 2 x 2 x Time]
Time_vec = out.Syy_log.Time;
Freq_vec = f;                  % [513 x 1]

% 确定控制频段索引 (20-2000 Hz)
idx_ctrl = find(Freq_vec >= 20 & Freq_vec <= 2000);
f_ctrl = Freq_vec(idx_ctrl);

% %% 2. 提取关键帧 (初始帧 vs 最终帧)
% 初始帧：取第 10 秒 (假设第一次平均已完成，开环控制效果)
t_init = 10;
[~, idx_init] = min(abs(Time_vec - t_init));

% 最终帧：取最后时刻 (闭环收敛效果)
idx_final = length(Time_vec);

% 提取自功率谱 (对角线元素)
% 维度: [Freq x Channel]
Syy_init_diag = squeeze(real([Syy_data(idx_ctrl, 1, 1, idx_init), Syy_data(idx_ctrl, 2, 2, idx_init)]));
Syy_final_diag = squeeze(real([Syy_data(idx_ctrl, 1, 1, idx_final), Syy_data(idx_ctrl, 2, 2, idx_final)]));
R_diag = squeeze(real([R(idx_ctrl, 1, 1), R(idx_ctrl, 2, 2)]));

% %% 3. 计算误差指标 (dB Error)
% Error_dB = 10 * log10( Measure / Ref )
Error_init_dB = 10 * log10(Syy_init_diag ./ R_diag);
Error_final_dB = 10 * log10(Syy_final_diag ./ R_diag);

% 计算统计量 (RMSE & Max Abs)
% RMSE = sqrt( mean( error^2 ) )
RMSE_init  = sqrt(mean(Error_init_dB.^2));
RMSE_final = sqrt(mean(Error_final_dB.^2));

Max_init  = max(abs(Error_init_dB));
Max_final = max(abs(Error_final_dB));

% %% 4. 打印量化评估报告
fprintf('============================================================\n');
fprintf('             MIMO 随机振动控制精度评估报告\n');
fprintf('============================================================\n');
fprintf('评估频段: 20 - 2000 Hz\n');
fprintf('参考谱值: 0.1 g^2/Hz (平直段)\n');
fprintf('------------------------------------------------------------\n');
fprintf('| 通道 | 状态 |   RMSE (dB)   | 最大误差 (dB) | 精度提升倍数 |\n');
fprintf('|------|------|---------------|---------------|--------------|\n');

for ch = 1:2
    fprintf('|  CH%d | 初始 | %6.2f dB      | %6.2f dB      |      -       |\n', ...
        ch, RMSE_init(ch), Max_init(ch));
    fprintf('|  CH%d | 最终 | \b%6.2f dB\b      | %6.2f dB      |    %4.1f 倍   |\n', ...
        ch, RMSE_final(ch), Max_final(ch), RMSE_init(ch)/RMSE_final(ch));
    fprintf('|------|------|---------------|---------------|--------------|\n');
end
fprintf('注: 工业标准通常要求 RMSE < 3dB 且绝大部分谱线在 +/-3dB 内。\n');
fprintf('============================================================\n');

% %% 5. 绘制误差谱对比图 (可视化)
figure('Name', 'Control Error Spectrum', 'Color', 'w', 'Position', [200, 200, 1000, 600]);

for ch = 1:2
    subplot(2, 1, ch);
    
    % 绘制初始误差 (虚线)
    semilogx(f_ctrl, Error_init_dB(:, ch), 'b--', 'LineWidth', 1, 'DisplayName', '初始误差 (开环)');
    hold on;
    
    % 绘制最终误差 (实线)
    semilogx(f_ctrl, Error_final_dB(:, ch), 'r-', 'LineWidth', 1.5, 'DisplayName', '最终误差 (闭环)');
    
    % 绘制 +/- 3dB 容差带 (绿色填充或粗线)
    yline(3, 'g--', 'LineWidth', 2, 'DisplayName', '+/- 3dB 容差');
    yline(-3, 'g--', 'LineWidth', 2, 'HandleVisibility', 'off');
    yline(0, 'k-', 'LineWidth', 0.5, 'HandleVisibility', 'off'); % 0dB 基准线
    
    % 图表设置
    title(sprintf('通道 %d 控制误差谱 (Control Error Spectrum - CH%d)', ch, ch), 'FontWeight', 'bold');
    ylabel('Error (dB)');
    if ch == 2
        xlabel('Frequency (Hz)');
    end
    grid on;
    xlim([20, 2000]);
    ylim([-15, 15]); % 限制Y轴范围以便观察细节，如有大离群点可调整
    legend('Location', 'Best');
end

sgtitle('MIMO 控制前后误差谱对比 (20-2000Hz)', 'FontSize', 14);