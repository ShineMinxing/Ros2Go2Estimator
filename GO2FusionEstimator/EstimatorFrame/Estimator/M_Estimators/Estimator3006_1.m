function StateSpaceModelN = Estimator3006_1(StateSpaceModelN)
% Estimator3006_1 — Robust Filter with Huber Factor（结构化不确定性 + Huber 自适应量测协方差）
%
% 数学模型
%   x_{k+1} = (A + dA) x_k + (B + dB) w_k + u_k
%   z_k     =  C x_k + v_k
%   [dA  dB] = M · δ · [Ef  Eg]
% 其中：
%   A,B,C 为离散系统矩阵；w_k、v_k 为高斯噪声；
%   M ∈ R^{n×1}，Ef, Eg ∈ R^{1×n}；δ 为标量不确定项。
%
% Huber 自适应（对量测协方差的门限缩放）
%   令残差 e = z − C x̂，S = sqrtm(R + C P C')，γ = ‖S \ e‖₂；
%   给定门限 Kai2，当 γ > Kai2 时，量测协方差自适应为 Re = (γ/Kai2)·R，否则 Re = R。
%
% 字段映射与参数读取
%   A ≡ Matrix_F，C ≡ Matrix_H，P ≡ Matrix_P，Q ≡ Matrix_Q，R ≡ Matrix_R
%   B（过程噪声通道） ≡ Matrix_G；G（确定性输入通道） ≡ Matrix_B（本实现不使用 u，等效 u=0）
%   采样时间 T ≡ Intervel；前视步数 ≡ PredictStep；观测 ≡ CurrentObservation
%   Ef、Eg、M 从 Double_Par 依次读取：
%       Ef = Double_Par(1:Nx)   （1×n 行向量）
%       Eg = Double_Par(Nx+1:2Nx)
%       M  = Double_Par(2Nx+1:3Nx)（n×1 列向量）
%   Huber 门限 Kai2 = Double_Par(3*Nx + 1)
%
% 参考文献：doi:10.1109/9.935054
% 作者：光电所一室2020级  孙敏行   联系方式：401435318@qq.com

% —— 1) 读取 Ef, Eg, M, Kai2 ——
Ef_row = (StateSpaceModelN.Double_Par(1:StateSpaceModelN.Nx)).';                             % 1×n
Eg_row = (StateSpaceModelN.Double_Par(StateSpaceModelN.Nx+1:2*StateSpaceModelN.Nx)).';       % 1×n
M_col  =  StateSpaceModelN.Double_Par(2*StateSpaceModelN.Nx+1:3*StateSpaceModelN.Nx);        % n×1
Kai2   =  5.991;                                                                             % 标量

% —— 2) 结构化不确定性鲁棒项（Qe, Rep1, Ae, Be, Λ）——
Qe       = StateSpaceModelN.Matrix_Q;      % 等效过程噪声
Ae       = StateSpaceModelN.Matrix_F;      % 等效系统矩阵
Be       = StateSpaceModelN.Matrix_G;      % 等效过程通道（对应 B）
Lambdae  = 0;

if ~StateSpaceModelN.Matrix_Par(1)
    Rep1 = StateSpaceModelN.Matrix_R;  % 等效量测噪声（基于 R，而非 Re）
else
    Rep1 = StateSpaceModelN.Matrix_Par;
end
Re       = Rep1;

% —— 4) Huber 自适应量测协方差 Re ——
innovation = StateSpaceModelN.CurrentObservation - StateSpaceModelN.Matrix_H * StateSpaceModelN.EstimatedState;
S          = Re + StateSpaceModelN.Matrix_H * StateSpaceModelN.Matrix_P * StateSpaceModelN.Matrix_H';
gamma      = abs(innovation'/sqrt(S));
if gamma > Kai2
    Re = (gamma / Kai2) * Re;  % 按比例放大量测协方差
end

% —— 3) 先验等价项 Ppre（以 Re 加权）——
% Ppre = P − P C' (Re + C P C')^{-1} C P
Ppre = StateSpaceModelN.Matrix_P ...
     - StateSpaceModelN.Matrix_P * StateSpaceModelN.Matrix_H' ...
       / (Re + StateSpaceModelN.Matrix_H * StateSpaceModelN.Matrix_P * StateSpaceModelN.Matrix_H') ...
       * StateSpaceModelN.Matrix_H * StateSpaceModelN.Matrix_P;


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

% —— 5) 后验状态更新（使用 Re）——
% x̂ = Ae x̂ + Ae Ppre C' Re^{-1} (z − C x̂)
StateSpaceModelN.EstimatedState = ...
    Ae * StateSpaceModelN.EstimatedState ...
  + Ae * ( Ppre * StateSpaceModelN.Matrix_H' * ( Re \ innovation ) );

% —— 6) 协方差更新（Hb 用 Re^{-(T/2)}）——
% Hb = [ C' Re^{-(T/2)} , √Λ · Ef' ]'  ∈ R^{(Nz+1)×n}
Hb  = [ StateSpaceModelN.Matrix_H' * ( Re^(-StateSpaceModelN.Intervel/2) ), ...
        sqrt(Lambdae) * (Ef_row') ]';
Rbe = eye(size(Hb,1)) + Hb * StateSpaceModelN.Matrix_P * Hb';
Kb  = StateSpaceModelN.Matrix_F * Ppre * Hb';
StateSpaceModelN.Matrix_P = ...
    StateSpaceModelN.Matrix_F * StateSpaceModelN.Matrix_P * StateSpaceModelN.Matrix_F' ...
  - Kb / Rbe * Kb' ...
  + Be * Qe * Be';

% —— 7) 前视预测（u=0）——
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
