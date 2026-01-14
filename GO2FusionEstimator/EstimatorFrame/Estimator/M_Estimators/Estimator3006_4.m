function StateSpaceModelN = Estimator3006_4(StateSpaceModelN)
% Estimator3006_4 — Robust Filter + RMA（Root-Mahalanobis）自适应量测协方差
%
% 数学模型
%   x_{k+1} = (A + dA) x_k + (B + dB) w_k + u_k
%   z_k     =  C x_k + v_k
%   [dA  dB] = M · δ · [Ef  Eg]
% 其中：
%   A,B,C 为离散系统矩阵；w_k、v_k 为高斯噪声；
%   M ∈ R^{n×1}，Ef, Eg ∈ R^{1×n}；δ 为标量不确定项。
%
% RMA 自适应
%   残差 e = z − C x̂，S = Re + C P C'，Γ = sqrt( e' S^{-1} e + 1e-3 )。
%   设门限 Kai2（常量赋值），当 Γ > Kai2 时放大量测协方差：Re ← Γ · Re；否则 Re = Rep1。
%
% 鲁棒结构化不确定性（基于 doi:10.1109/9.935054）
%   鲁棒项使用基线量测噪声 Rep1（由 Matrix_Par 持久化或回退到 R）；
%   自适应仅影响 Re（用于 Ppre、后验更新与 Hb 中的 Re^{-(T/2) }）。

% —— 1) 读取 Ef, Eg, M；设定 RMA 门限 ——
Ef_row = (StateSpaceModelN.Double_Par(1:StateSpaceModelN.Nx)).';                        % 1×n
Eg_row = (StateSpaceModelN.Double_Par(StateSpaceModelN.Nx+1:2*StateSpaceModelN.Nx)).';  % 1×n
M_col  =  StateSpaceModelN.Double_Par(2*StateSpaceModelN.Nx+1:3*StateSpaceModelN.Nx);   % n×1
Kai2   =  2.5;  % Root-Mahalanobis 门限（≈ sqrt(χ^2_{dof=2,95%}=2.448）可按需求调整）

% —— 2) 基线量测协方差 Rep1；初始化 Re ——（沿用 3006_1 的约定）
if ~StateSpaceModelN.Matrix_Par(1)
    Rep1 = StateSpaceModelN.Matrix_R;         % 基线（鲁棒项使用）
else
    Rep1 = StateSpaceModelN.Matrix_Par;       % 复用历史/外部设定
end
Re = Rep1;

% —— 3) RMA 自适应得到 Re ——（仅 Re 参与后续）
innovation = StateSpaceModelN.CurrentObservation - ...
             StateSpaceModelN.Matrix_H * StateSpaceModelN.EstimatedState;                % e
S      = Re + StateSpaceModelN.Matrix_H * StateSpaceModelN.Matrix_P * StateSpaceModelN.Matrix_H';
Gamma  = sqrt( innovation' * (S \ innovation) + 1e-3 );                                  % sqrt(e' S^{-1} e + 1e-3)
if Gamma > Kai2
    Re = Gamma * Re;
end

% —— 4) 先验等价项 Ppre（以 Re 加权）——
% Ppre = P − P C' (Re + C P C')^{-1} C P
Ppre = StateSpaceModelN.Matrix_P ...
     - StateSpaceModelN.Matrix_P * StateSpaceModelN.Matrix_H' ...
       / (Re + StateSpaceModelN.Matrix_H * StateSpaceModelN.Matrix_P * StateSpaceModelN.Matrix_H') ...
       * StateSpaceModelN.Matrix_H * StateSpaceModelN.Matrix_P;

% —— 5) 鲁棒结构化不确定性项（Qe, Rep1, Ae, Be, Λ）——
Qe      = StateSpaceModelN.Matrix_Q;      % 等效过程噪声
Ae      = StateSpaceModelN.Matrix_F;      % 等效系统矩阵
Be      = StateSpaceModelN.Matrix_G;      % 等效过程通道（对应 B）
Lambdae = 0;

if any( (StateSpaceModelN.Matrix_H * M_col) ~= 0 )
    % Λ_min = | M' C' (R \ (C M)) |（鲁棒项基于基线 R 计算）
    Lambdamin = abs( M_col.' * ( StateSpaceModelN.Matrix_H' * ...
                   ( StateSpaceModelN.Matrix_R \ ( StateSpaceModelN.Matrix_H * M_col ) ) ) );
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

% 持久化基线 Rep1（与 3006_1/3006_2/3006_3 保持一致）
StateSpaceModelN.Matrix_Par = Rep1;

% —— 6) 后验状态更新（使用 Re）——
% x̂ ← Ae x̂ + Ae Ppre C' Re^{-1} (z − C x̂)
StateSpaceModelN.EstimatedState = ...
    Ae * StateSpaceModelN.EstimatedState ...
  + Ae * ( Ppre * StateSpaceModelN.Matrix_H' * ( Re \ innovation ) );

% —— 7) 协方差更新（Hb 用 Re^{-(T/2)}）——
% Hb = [ C' Re^{-(T/2)} , √Λ · Ef' ]'  ∈ R^{(m+1)×n}
Hb  = [ StateSpaceModelN.Matrix_H' * ( Re^(-StateSpaceModelN.Intervel/2) ), ...
        sqrt(Lambdae) * (Ef_row') ]';
Rbe = eye(size(Hb,1)) + Hb * StateSpaceModelN.Matrix_P * Hb';
Kb  = StateSpaceModelN.Matrix_F * Ppre * Hb';
StateSpaceModelN.Matrix_P = ...
    StateSpaceModelN.Matrix_F * StateSpaceModelN.Matrix_P * StateSpaceModelN.Matrix_F' ...
  - Kb / Rbe * Kb' ...
  + Be * Qe * Be';

% —— 8) 前视预测（u=0）——
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
