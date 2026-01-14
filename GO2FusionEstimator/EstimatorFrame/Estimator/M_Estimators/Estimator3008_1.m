function StateSpaceModelN = Estimator3008_1(StateSpaceModelN)
% Estimator3008_1 — ASTCKF（Adaptive Spherical-Cubature Kalman Filter，自适应球面容积卡尔曼滤波）
%
% 功能
%   对非线性系统执行一次基于容积点（±sqrt(n)·S）的状态估计与协方差更新，并结合 ST 策略进行
%   观测残差驱动的自适应调整；随后从后验状态出发前视 PredictStep 步，给出 PredictedState 与 PredictedObservation。
%
% 数学模型
%   x_{k+1} = f(x_k) + w_k
%   z_k     = h(x_k) + v_k
%   其中 w_k ~ N(0, Q)、v_k ~ N(0, R)。
% 
% 自适应策略（ST）
%   残差 e_k = z_k − ẑ_k；以衰减系数 Rou=0.95、阈值参数 Beta=3 构造 Vk 并形成自适应项：
%       Vk ← (Rou·V_{k−1} + e_k e_k^T)/(1+Rou)（首次时刻）或 Vk ← e_k e_k^T（之后）
%       N  = Vk − Beta·R − P_xz^T / P_e · Q / P_e · P_xz
%       M  = P_zz + N − Vk − (1−Beta)·R
%       若 trace(N)/trace(N) > 1，则对 P_e 施加同尺度放大（与原式一致）
%   为保持“无中间变量持久化”规范，上一时刻 Vk 保存在 StateSpaceModelN.Double_Par(100)（初值 0）。
%
% 主要步骤
%   1) 生成容积点：S = chol(Matrix_P, 'lower')，X_i = ±sqrt(n)·S + EstimatedState
%   2) 传播至 f：得到 RJD_i，计算 Xe、P_e
%   3) 传播至 h：得到量测容积点，计算 ẑ（=Zei）、P_zz、P_xz
%   4) ST 自适应：根据 e_k、Rou、Beta 更新 Vk 和放缩因子，对 P_e 进行尺度修正（如需）
%   5) 更新：K = P_xz / P_zz；EstimatedState ← Xe + K e_k；Matrix_P ← P_e − K P_zz K^T
%   6) 前视：从后验出发迭代 StateTransitionEquation PredictStep 次；PredictedObservation = Matrix_H * PredictedState（若为纯非线性量测，则可由 ObservationEquation 生成）
%
% 复杂度
%   每次递推主要开销为两次 Cholesky 分解与 O(n·m) 的二阶矩运算，整体近似 O(n^3)。
%
% 参考文献
%   10.1109/ACCESS.2020.3013561
%
% 作者与联系方式
%   光电所一室2020级  孙敏行
%   401435318@qq.com

if isempty(StateSpaceModelN)
    error('State space model is not initialized');
end

% ===== (1) 立方球容积点（基于后验 P） =====
Ma = [eye(StateSpaceModelN.Nx), -eye(StateSpaceModelN.Nx)];
Si = chol(StateSpaceModelN.Matrix_P, 'lower');
Temp = sqrt(StateSpaceModelN.Nx) * Si * Ma + ...
       repmat(StateSpaceModelN.EstimatedState, 1, 2*StateSpaceModelN.Nx);

% 传播容积点：x_k → f(x_k)
RJD = zeros(StateSpaceModelN.Nx, 2*StateSpaceModelN.Nx);
for j = 1:2*StateSpaceModelN.Nx
    [RJD(:,j), StateSpaceModelN] = StateSpaceModelN.StateTransitionEquation(Temp(:,j), StateSpaceModelN);
end

% 状态预测均值与协方差
Xe = mean(RJD, 2);
Pe = (RJD * RJD.')/(2*StateSpaceModelN.Nx) - Xe*Xe.' + StateSpaceModelN.Matrix_Q;

% ===== (2) 观测预测 =====
Si = chol(Pe, 'lower');
Ksi = sqrt(StateSpaceModelN.Nx) * Si * Ma + repmat(Xe, 1, 2*StateSpaceModelN.Nx);

Zpts = zeros(StateSpaceModelN.Nz, 2*StateSpaceModelN.Nx);
for k = 1:2*StateSpaceModelN.Nx
    [Zpts(:,k), StateSpaceModelN] = StateSpaceModelN.ObservationEquation(Ksi(:,k), StateSpaceModelN);
end
Zei  = mean(Zpts, 2);
Pizz = (Zpts * Zpts.')/(2*StateSpaceModelN.Nx) - Zei*Zei.' + StateSpaceModelN.Matrix_R;
Pixz = (Ksi * Zpts.')/(2*StateSpaceModelN.Nx) - Xe*Zei.';

% ===== (3) ST 自适应（Rou=0.95, Beta=3；Vk 持久化在 Double_Par(1)） =====
e = StateSpaceModelN.CurrentObservation - Zei;
Rou  = 0.95;
Beta = 3;

if ~StateSpaceModelN.Double_Par(1)
    Vk = (Rou * StateSpaceModelN.Double_Par(1)+ e*e.')/(1 + Rou);
    StateSpaceModelN.Double_Par= Vk;
else
    Vk = e*e.';
    StateSpaceModelN.Double_Par= Vk;
end

N_adapt = Vk - Beta*StateSpaceModelN.Matrix_R ...
          - (Pixz.'/Pe) * StateSpaceModelN.Matrix_Q / Pe * Pixz;
M_adapt = Pizz + N_adapt - Vk - (1 - Beta)*StateSpaceModelN.Matrix_R;

Lambda = trace(N_adapt) / trace(N_adapt);
if Lambda > 1
    Pe = Pe * Lambda;
end

% ===== (4) 更新 =====
Kk = Pixz / (Pizz);
StateSpaceModelN.EstimatedState = Xe + Kk * e;
StateSpaceModelN.Matrix_P       = Pe - Kk * Pizz * Kk.';

% ===== (5) 前视预测（与 M_Demo 的真值前移对齐）=====
StateSpaceModelN.PredictedState = StateSpaceModelN.EstimatedState;
if StateSpaceModelN.PredictStep
    for j = 1:StateSpaceModelN.PredictStep
        [StateSpaceModelN.PredictedState, StateSpaceModelN] = ...
            StateSpaceModelN.StateTransitionEquation(StateSpaceModelN.PredictedState, StateSpaceModelN);
    end
end
[StateSpaceModelN.PredictedObservation, StateSpaceModelN] = ...
    StateSpaceModelN.ObservationEquation(StateSpaceModelN.PredictedState, StateSpaceModelN);
end
