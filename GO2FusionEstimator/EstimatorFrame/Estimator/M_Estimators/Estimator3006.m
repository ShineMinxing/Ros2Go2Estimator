function StateSpaceModelN = Estimator3006(StateSpaceModelN)
% Estimator3006 — Robust Filter with Structured Uncertainty
%
% 数学模型
%   x_{k+1} = (A + dA) x_k + (B + dB) w_k + u_k
%   z_k     =  C x_k + v_k
%   [dA  dB] = M · δ · [Ef  Eg]
% 其中：
%   A,B,C 为离散系统矩阵；w_k、v_k 为高斯噪声；
%   M ∈ R^{n×1}，Ef, Eg ∈ R^{1×n}；δ 为标量不确定项。
%
% Ef、Eg、M 的读取方式
%   令 n = Nx：
%     Ef = Double_Par(1:n)          作为 1×n 行向量（代码中记为 Ef_row）
%     Eg = Double_Par(n+1:2n)       作为 1×n 行向量（代码中记为 Eg_row）
%     M  = Double_Par(2n+1:3n)      作为 n×1 列向量（代码中记为 M_col）
%
% 参考文献：doi:10.1109/9.935054
% 作者：光电所一室2020级 孙敏行    联系方式：401435318@qq.com

if isempty(StateSpaceModelN)
    error('State space model is not initialized');
end

% —— 1) 从 Double_Par 读取 Ef, Eg, M（按顺序与形状要求） ——
Ef_row = (StateSpaceModelN.Double_Par(1:StateSpaceModelN.Nx)).';                        % 1×n
Eg_row = (StateSpaceModelN.Double_Par(StateSpaceModelN.Nx+1:2*StateSpaceModelN.Nx)).';  % 1×n
M_col  =  StateSpaceModelN.Double_Par(2*StateSpaceModelN.Nx+1:3*StateSpaceModelN.Nx);   % n×1

% —— 2) 先验等价项 Ppre ——
% Ppre = P − P C' (R + C P C')^{-1} C P
Ppre = StateSpaceModelN.Matrix_P ...
     - StateSpaceModelN.Matrix_P * StateSpaceModelN.Matrix_H' ...
       / (StateSpaceModelN.Matrix_R + StateSpaceModelN.Matrix_H * StateSpaceModelN.Matrix_P * StateSpaceModelN.Matrix_H') ...
       * StateSpaceModelN.Matrix_H * StateSpaceModelN.Matrix_P;

% —— 3) 鲁棒等效参数（Qe, Rep1, Ae, Be, Λ） ——
Qe      = StateSpaceModelN.Matrix_Q;      % 等效过程噪声
Ae      = StateSpaceModelN.Matrix_F;      % 等效系统矩阵
Be      = StateSpaceModelN.Matrix_G;      % 等效过程通道（对应 B）
Lambdae = 0;                              % 鲁棒权重

if ~StateSpaceModelN.Matrix_Par(1)
    Rep1    = StateSpaceModelN.Matrix_R;  % 等效量测噪声（基于 R，而非 Re）
else
    Rep1 = StateSpaceModelN.Matrix_Par;
end
Re      = Rep1;
if any( (StateSpaceModelN.Matrix_H * M_col) ~= 0 )
    % Λ_min = | M' C' (R \ (C M)) |
    Lambdamin = abs( M_col.' * ( StateSpaceModelN.Matrix_H' * ...
                   ( StateSpaceModelN.Matrix_R \ (StateSpaceModelN.Matrix_H * M_col) ) ) );
    Lambdae   = 1.5 * Lambdamin;

    EgEgT = Eg_row * Eg_row';   % 标量
    EfEfT = Ef_row * Ef_row';   % 标量
    EfEgT = Ef_row * Eg_row';   % 标量

    if EgEgT == 0
        Qe   = StateSpaceModelN.Matrix_Q;
        Rep1 = StateSpaceModelN.Matrix_R - (1/max(Lambdae,eps)) * ...
               (StateSpaceModelN.Matrix_H*M_col) * (StateSpaceModelN.Matrix_H*M_col)';
        Ae   = StateSpaceModelN.Matrix_F * ( eye(StateSpaceModelN.Nx) - ...
               Lambdae * Ppre * (Ef_row' * Ef_row) );
        Be   = StateSpaceModelN.Matrix_G;

    elseif EfEfT == 0
        Qe   = inv( inv(StateSpaceModelN.Matrix_Q) + Lambdae * (Eg_row' * Eg_row) );
        Rep1 = StateSpaceModelN.Matrix_R - (1/max(Lambdae,eps)) * ...
               (StateSpaceModelN.Matrix_H*M_col) * (StateSpaceModelN.Matrix_H*M_col)';
        Ae   = StateSpaceModelN.Matrix_F;
        Be   = StateSpaceModelN.Matrix_G;

    elseif EfEgT == 0
        Qe   = inv( inv(StateSpaceModelN.Matrix_Q) + Lambdae * (Eg_row' * Eg_row) );
        Rep1 = StateSpaceModelN.Matrix_R - (1/max(Lambdae,eps)) * ...
               (StateSpaceModelN.Matrix_H*M_col) * (StateSpaceModelN.Matrix_H*M_col)';
        Ae   = StateSpaceModelN.Matrix_F * ( eye(StateSpaceModelN.Nx) - ...
               Lambdae * Ppre * (Ef_row' * Ef_row) );
        Be   = StateSpaceModelN.Matrix_G;

    else
        % Qe = [inv(Q) + Λ·Eg'·inv(I + Λ·Ef·Ppre·Ef')·Eg]^{-1}
        denom = 1 + Lambdae * (Ef_row * Ppre * Ef_row');  % 标量
        Qe    = inv( inv(StateSpaceModelN.Matrix_Q) + (Lambdae/denom) * (Eg_row' * Eg_row) );

        Rep1  = StateSpaceModelN.Matrix_R - (1/max(Lambdae,eps)) * ...
               (StateSpaceModelN.Matrix_H*M_col) * (StateSpaceModelN.Matrix_H*M_col)';

        % Ae = (A − Λ·B_e·Qe·Eg'·Ef) · (I − Λ·Ppre·Ef'·Ef)
        Ae    = ( StateSpaceModelN.Matrix_F ...
                - Lambdae * Be * Qe * (Eg_row' * Ef_row) ) ...
                * ( eye(StateSpaceModelN.Nx) - Lambdae * Ppre * (Ef_row' * Ef_row) );

        % Be = B − Λ·A·Ppre·Ef'·Eg
        Be    = StateSpaceModelN.Matrix_G ...
              - Lambdae * StateSpaceModelN.Matrix_F * Ppre * (Ef_row' * Eg_row);
    end
end

StateSpaceModelN.Matrix_Par = Rep1;

% —— 4) 后验状态更新（u_k 取 0） ——
% x̂ = Ae x̂ + Ae Ppre C'^{-1} (z − C x̂)
innovation = StateSpaceModelN.CurrentObservation ...
           - StateSpaceModelN.Matrix_H * StateSpaceModelN.EstimatedState;
StateSpaceModelN.EstimatedState = ...
    Ae * StateSpaceModelN.EstimatedState ...
  + Ae * ( Ppre * StateSpaceModelN.Matrix_H' * ( Re \ innovation ) );

% —— 5) 协方差更新 ——
% Hb = [ C' R^{-(T/2)} , √Λ · Ef' ]'  ∈ R^{(Nz+1)×n}
Hb  = [ StateSpaceModelN.Matrix_H' * ( StateSpaceModelN.Matrix_R^(-StateSpaceModelN.Intervel/2) ), ...
        sqrt(Lambdae) * (Ef_row') ]';
Rbe = eye(size(Hb,1)) + Hb * StateSpaceModelN.Matrix_P * Hb';
Kb  = StateSpaceModelN.Matrix_F * Ppre * Hb';
StateSpaceModelN.Matrix_P = ...
    StateSpaceModelN.Matrix_F * StateSpaceModelN.Matrix_P * StateSpaceModelN.Matrix_F' ...
  - Kb / Rbe * Kb' ...
  + Be * Qe * Be';

% —— 6) 前视预测（无确定性输入） ——
StateSpaceModelN.PredictedState = StateSpaceModelN.EstimatedState;
if StateSpaceModelN.PredictStep
    for j = 1:StateSpaceModelN.PredictStep
        StateSpaceModelN.PredictedState = ...
            StateSpaceModelN.Matrix_F * StateSpaceModelN.PredictedState;
    end
end
StateSpaceModelN.PredictedObservation = ...
    StateSpaceModelN.Matrix_H * StateSpaceModelN.PredictedState;
end
