%% system_model.m
% 功能：处理Simulink仿真数据，进行谱平均，估计频响函数 H 和 逆系统矩阵 Z
% 输入：Simulink 输出的 out.Suu_ts, out.Syu_ts, out.f_ts
% 输出：保存 system_model.mat

clc; clearvars -except out; % 清除除仿真结果外的变量

% 检查仿真数据是否存在
if ~exist('out', 'var')
    error('请先运行 Simulink 模型生成数据 (out 变量)！');
end

fprintf('开始处理仿真数据...\n');

%% 1. 数据提取
% 从 Timeseries 对象中提取 Data
% 假设 Simulink 设置为 Timeseries, 且维度未被压缩
% 预期维度: [Frequency x Output x Input x Time] -> [513 x 2 x 2 x K]
raw_Suu = out.Suu_ts.Data;
raw_Syu = out.Syu_ts.Data;
raw_Syy = out.Syy_ts.Data;
f = out.f_ts.Data(:, 1, 1); % 提取频率向量 (所有时刻都一样，取第一个)

% 获取维度信息
[n_freq, n_out, n_in, n_frames] = size(raw_Suu);

fprintf('数据维度检测:\n');
fprintf('  频率点数: %d\n', n_freq);
fprintf('  输入通道: %d, 输出通道: %d\n', n_in, n_out);
fprintf('  平均帧数 (Time Steps): %d\n', n_frames);

%% 2. 频谱平均 (Averaging)
% 对第4维 (时间/帧) 求平均
% 得到平均后的自功率谱和互功率谱
Suu_avg = mean(raw_Suu, 4); % [513 x 2 x 2]
Syu_avg = mean(raw_Syu, 4); % [513 x 2 x 2]
Syy_avg = mean(raw_Syy, 4); % [513 x 2 x 2]

fprintf('频谱平均完成。\n');

%% 3. 计算频响函数 H 和 逆矩阵 Z
% H1 估计公式: H(w) = Syu(w) * inv(Suu(w))
% Z(w) = inv(H(w))

% 初始化矩阵
H = complex(zeros(n_freq, n_out, n_in));
Z = complex(zeros(n_freq, n_in, n_out));

% 逐频点计算
% 注意：H 和 Z 是针对每一个频率点 k 的 2x2 矩阵运算
for k = 1:n_freq
    % 提取当前频点的 2x2 矩阵
    % squeeze 将 [1 x 2 x 2] 压缩为 [2 x 2]
    Suu_k = squeeze(Suu_avg(k, :, :));
    Syu_k = squeeze(Syu_avg(k, :, :));
    
    % --- 计算 H (Admittance / Mobility) ---
    % MATLAB 中 / 代表右除，即 A/B = A * inv(B)
    % 这一步对应公式: H = Syu * Suu^-1
    H_k = Syu_k / Suu_k; 
    
    % --- 计算 Z (Impedance / Inverse System) ---
    % 简单的求逆
    % 在实际控制中，为了防止奇异，有时会使用 pinv 或增加正则化项
    % 但在理想仿真中，直接 inv 即可
    if rcond(H_k) < 1e-12
        % 如果矩阵接近奇异 (例如在极低频或反共振点)，给个警告或特殊处理
        % 这里简单处理，防止报错，但在控制中需要小心
        Z_k = pinv(H_k); 
    else
        Z_k = inv(H_k);
    end
    
    % 存入总矩阵
    H(k, :, :) = H_k;
    Z(k, :, :) = Z_k;
end

fprintf('频响函数 H 与 阻抗矩阵 Z 计算完成。\n');
%% 在"计算 H 和 Z"之后，增加"相干函数计算"：
% 初始化相干函数矩阵 [频率点 x 输出通道 x 输入通道]
% Coh(k, i, j) 表示第 j 个输入与第 i 个输出之间的相干性
Coh = zeros(n_freq, n_out, n_in);

for k = 1:n_freq
    for i = 1:n_out
        for j = 1:n_in
            % 提取对应的平均谱分量
            Syjui = Syu_avg(k, i, j);    % 互谱 (i,j)
            Syiyi = Syy_avg(k, i, i);    % 输出 i 的自谱
            Sujuj = Suu_avg(k, j, j);    % 输入 j 的自谱
            
            % 计算普通相干函数 (取绝对值平方)
            % 注意：必须使用平均后的谱进行计算，否则相干项恒等于 1
            Coh(k, i, j) = (abs(Syjui)^2) / (real(Syiyi) * real(Sujuj));
        end
    end
end

%% 4. 保存模型数据
% 保存用于后续控制仿真和绘图的数据
% save 包含: H, Z, f, Suu_avg, Syu_avg
save('system_model.mat', 'H', 'Z', 'f', 'Suu_avg', 'Syu_avg', 'Syy_avg', 'Coh');

fprintf('数据已保存至 system_model.mat\n');

