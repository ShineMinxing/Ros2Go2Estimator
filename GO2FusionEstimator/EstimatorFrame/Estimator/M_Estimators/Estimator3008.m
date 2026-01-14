function StateSpaceModelN = Estimator3008(StateSpaceModelN)
% Estimator3008 — CKF（Spherical-Cubature Kalman Filter，球面容积卡尔曼滤波）
%
% 功能
%   对非线性系统执行一次基于球面容积点（±sqrt(n)·S）的状态估计与协方差更新；随后从后验状态
%   出发前视 PredictStep 步，返回 PredictedState 与 PredictedObservation。
%
% 数学模型
%   x_{k+1} = f(x_k) + w_k
%   z_k     = h(x_k) + v_k
%   w_k ~ N(0, Q)，v_k ~ N(0, R)。
%
% 结构体字段映射
%   状态维数/量测维数：Nx, Nz
%   当前后验：EstimatedState（n×1）、Matrix_P（n×n）
%   噪声协方差：Matrix_Q（n×n）、Matrix_R（m×m）
%   非线性方程：StateTransitionEquation(x, S) ≡ f(x)，ObservationEquation(x, S) ≡ h(x)
%   输入观测：CurrentObservation（m×1）
%   前视步数：PredictStep
%
% 计算流程
%   1) 生成容积点：S = chol(Matrix_P,'lower')，Xi = ±sqrt(n)·S + EstimatedState
%   2) 传播至 f 得到 RJD_i，计算 Xe、Pe
%   3) 以 Pe 生成新容积点并传播至 h，得 Zei、P_zz、P_xz
%   4) K = P_xz / P_zz，x̂ ← Xe + K (z − Zei)，P ← Pe − K P_zz K^T
%   5) 从后验出发前视 PredictStep 次：PredictedState；并由 ObservationEquation 给出 PredictedObservation
%
% 作者与联系方式
%   光电所一室2020级  孙敏行
%   401435318@qq.com

if isempty(StateSpaceModelN)
    error('State space model is not initialized');
end

% === 1) 构造容积点（基于当前后验协方差） ===
Ma = [eye(StateSpaceModelN.Nx), -eye(StateSpaceModelN.Nx)];                                  % n×2n
Si = chol(StateSpaceModelN.Matrix_P, 'lower');                                               % n×n
Temp = sqrt(StateSpaceModelN.Nx) * Si * Ma + ...
       repmat(StateSpaceModelN.EstimatedState, 1, 2*StateSpaceModelN.Nx);                    % n×2n

% 传播至状态方程 f：RJD(:,j) = f(Temp(:,j))
RJD = zeros(StateSpaceModelN.Nx, 2*StateSpaceModelN.Nx);
for j = 1:2*StateSpaceModelN.Nx
    [RJD(:,j), StateSpaceModelN] = StateSpaceModelN.StateTransitionEquation(Temp(:,j), StateSpaceModelN);
end

% 状态预测均值/协方差
Xe = mean(RJD, 2);                                                                           % n×1
Pe = (RJD * RJD.')/(2*StateSpaceModelN.Nx) - Xe*Xe.' + StateSpaceModelN.Matrix_Q;            % n×n

% === 2) 观测预测（以 Pe 生成新容积点并传播至 h） ===
Si = chol(Pe, 'lower');
Ksi = sqrt(StateSpaceModelN.Nx) * Si * Ma + repmat(Xe, 1, 2*StateSpaceModelN.Nx);            % n×2n

Zpts = zeros(StateSpaceModelN.Nz, 2*StateSpaceModelN.Nx);
for k = 1:2*StateSpaceModelN.Nx
    [Zpts(:,k), StateSpaceModelN] = StateSpaceModelN.ObservationEquation(Ksi(:,k), StateSpaceModelN);
end
Zei  = mean(Zpts, 2);                                                                         % m×1
Pizz = (Zpts * Zpts.')/(2*StateSpaceModelN.Nx) - Zei*Zei.' + StateSpaceModelN.Matrix_R;       % m×m
Pixz = (Ksi * Zpts.')/(2*StateSpaceModelN.Nx) - Xe*Zei.';                                     % n×m

% === 3) 更新 ===
e  = StateSpaceModelN.CurrentObservation - Zei;                                               % m×1
Kk = Pixz / (Pizz);                                                                           % n×m
StateSpaceModelN.EstimatedState = Xe + Kk * e;                                                % n×1
StateSpaceModelN.Matrix_P       = Pe - Kk * Pizz * Kk.';                                      % n×n

% === 4) 前视预测（与 M_Demo 的时标对齐） ===
StateSpaceModelN.PredictedState = StateSpaceModelN.EstimatedState;
if StateSpaceModelN.PredictStep
    for t = 1:StateSpaceModelN.PredictStep
        [StateSpaceModelN.PredictedState, StateSpaceModelN] = ...
            StateSpaceModelN.StateTransitionEquation(StateSpaceModelN.PredictedState, StateSpaceModelN);
    end
end
[StateSpaceModelN.PredictedObservation, StateSpaceModelN] = ...
    StateSpaceModelN.ObservationEquation(StateSpaceModelN.PredictedState, StateSpaceModelN);
end
