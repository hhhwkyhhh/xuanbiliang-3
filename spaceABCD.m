%% spaceABCD.m
% 功能：生成悬臂梁有限元模型及其状态空间矩阵 (A, B, C, D)
% 模型说明：
%   - 几何：1m x 0.1m x 0.01m
%   - 材料：铝 (Aluminum)
%   - 单元：5个欧拉-伯努利梁单元
%   - 节点编号：从右(自由端)向左(固支端)编号。Node 1=Free, Node 6=Fixed
%   - 自由度：每个节点2个 (v, theta)。DOF 11,12 被约束。
%   - 输入：力 (Force) @ Node 3 (DOF 5) & Node 5 (DOF 9)
%   - 输出：速度 (Velocity) @ Node 3 (DOF 5) & Node 5 (DOF 9)
%   - 阻尼：瑞利阻尼 (前两阶阻尼比 0.02)

clear; clc;

%% 1. 物理参数定义 (SI Units)
L = 1.0;           % 长度 (m) [cite: 2182]
b = 0.1;            % 宽度 (m) [cite: 2182]
h = 0.01;           % 厚度 (m) [cite: 2182]
E = 7.0e10;         % 弹性模量 (Pa) - Aluminum
rho = 2700;         % 密度 (kg/m^3) - Aluminum
zeta = 0.02;        % 前两阶模态阻尼比

% 截面属性
Area = b * h;           % 截面面积
I = (b * h^3) / 12;     % 截面惯性矩

%% 2. 有限元网格参数
Ne = 5;                 % 单元数量 
le = L / Ne;            % 单元长度 (0.2 m)
N_node = Ne + 1;        % 节点数量 (6)
Total_DOF = N_node * 2; % 总自由度 (12)

%% 3. 组装质量矩阵 M 和 刚度矩阵 K
% 初始化全局矩阵
M_global = zeros(Total_DOF, Total_DOF);
K_global = zeros(Total_DOF, Total_DOF);

% 单元刚度矩阵 (ke) 和 质量矩阵 (me) - 欧拉伯努利梁
% 局部自由度顺序: [v1, theta1, v2, theta2]
c1 = E * I / le^3;
ke = c1 * [12,      6*le,    -12,     6*le;
           6*le,    4*le^2,  -6*le,   2*le^2;
          -12,     -6*le,     12,    -6*le;
           6*le,    2*le^2,  -6*le,   4*le^2];

c2 = rho * Area * le / 420;
me = c2 * [156,     22*le,    54,     -13*le;
           22*le,   4*le^2,   13*le,  -3*le^2;
           54,      13*le,    156,    -22*le;
          -13*le,  -3*le^2,  -22*le,   4*le^2];

% 循环组装
for i = 1:Ne
    % 定义当前单元的全局自由度索引
    % 单元1 连接 Node 1 (DOF 1,2) 和 Node 2 (DOF 3,4)
    % 单元i 连接 Node i 和 Node i+1
    idx = [2*i-1, 2*i, 2*i+1, 2*i+2]; 
    
    M_global(idx, idx) = M_global(idx, idx) + me;
    K_global(idx, idx) = K_global(idx, idx) + ke;
end

%% 4. 施加边界条件 (Boundary Conditions)
% Node 6 (Fixed end) 对应 DOF 11 和 12
% 我们需要移除最后两行两列
active_dof_idx = 1:10; % 保留 DOF 1 到 10

M = M_global(active_dof_idx, active_dof_idx);
K = K_global(active_dof_idx, active_dof_idx);

Dimension = length(M); % 系统维度 (10)

%% 5. 阻尼矩阵 C (Rayleigh Damping)
% C = alpha*M + beta*K
% 求解 alpha 和 beta 需要前两阶固有频率

% 求解广义特征值问题 K*phi = w^2*M*phi
[Phi, D_eig] = eig(K, M);
omega2 = diag(D_eig);
[omega2, sort_idx] = sort(omega2); % 排序
omega = sqrt(omega2);              % 固有频率 (rad/s)
Phi = Phi(:, sort_idx);            % 对应的振型

% 获取前两阶频率
w1 = omega(1);
w2 = omega(2);

% 根据公式: zeta = 0.5 * (alpha/w + beta*w) 求解
% [ 1/(2w1)  w1/2 ] [ alpha ] = [ zeta ]
% [ 1/(2w2)  w2/2 ] [ beta  ]   [ zeta ]
A_damp = [1/(2*w1), w1/2; 
          1/(2*w2), w2/2];
b_damp = [zeta; zeta];
x_damp = A_damp \ b_damp;

alpha = x_damp(1);
beta = x_damp(2);

C_damp = alpha * M + beta * K; % 阻尼矩阵

fprintf('系统前两阶固有频率: %.2f Hz, %.2f Hz\n', w1/(2*pi), w2/(2*pi));
fprintf('Rayleigh阻尼系数: alpha = %.4f, beta = %.6f\n', alpha, beta);

%% 6. 构建状态空间矩阵 (State-Space Matrices)
% 状态向量 x = [displacement; velocity] (20x1)
% 状态方程: dx/dt = A*x + B*u
% 输出方程: y     = C*x + D*u

% --- 构建 A 矩阵 (20x20) ---
% A = [   0       I   ]
%     [ -M\K    -M\C  ]
Minv = inv(M);
A = [zeros(Dimension), eye(Dimension);
    -Minv*K,          -Minv*C_damp];

% --- 构建 B 矩阵 (20x2) ---
% 输入 u = [F_node3; F_node5] (2x1)
% 力作用在 Node 3 (DOF 5) 和 Node 5 (DOF 9)
% 对应的方程是动力学方程(下半部分)
B_force = zeros(Dimension, 2);
B_force(5, 1) = 1; % DOF 5 (Node 3) corresponds to Input 1
B_force(9, 2) = 1; % DOF 9 (Node 5) corresponds to Input 2

B = [zeros(Dimension, 2);
     Minv * B_force];

% --- 构建 C 矩阵 (2x20) ---
% 输出 y = [Velocity_node3; Velocity_node5] (2x1)
% 速度位于状态向量的下半部分 (索引 11-20)
% DOF 5 的速度对应状态向量索引 10 + 5 = 15
% DOF 9 的速度对应状态向量索引 10 + 9 = 19
C = zeros(2, 2*Dimension);
C(1, Dimension + 5) = 1; % Output 1: Velocity at DOF 5
C(2, Dimension + 9) = 1; % Output 2: Velocity at DOF 9

% --- 构建 D 矩阵 (2x2) ---
% 速度输出对于力输入，直通项为0
D = zeros(2, 2);

%% 7. 保存数据
% 保存 A, B, C, D 矩阵供 Simulink 使用
save('spaceABCD.mat', 'A', 'B', 'C', 'D');

fprintf('状态空间矩阵已生成并保存至 spaceABCD.mat\n');
fprintf('A矩阵大小: %dx%d\n', size(A));
fprintf('B矩阵大小: %dx%d\n', size(B));
fprintf('C矩阵大小: %dx%d\n', size(C));
fprintf('D矩阵大小: %dx%d\n', size(D));