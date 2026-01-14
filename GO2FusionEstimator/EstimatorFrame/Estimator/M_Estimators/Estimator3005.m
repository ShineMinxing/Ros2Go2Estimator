function StateSpaceModelN = Estimator3005(StateSpaceModelN)
% Estimator3005 — IMCKR 插值鲁棒卡尔曼滤波（名义性能/鲁棒性权衡）
%
% 模型
%   x_{k+1} = (A + dA) x_k + (B + dB) w_k + u_k
%   z_k     =  C x_k + v_k
%   [dA  dB] = M · δ · [Ef  Eg]
%
% 关键思想
%   通过标量 α∈[0,1] 在线权衡名义性能（Kalman）与鲁棒性：
%   通过最小化 GLF(Λ)（一维凸问题）得到 Λo，构造等效量测协方差 Re 以及等效
%   (Qe, Fe, Ge)，完成后验更新与前视预测。该实现对应 IMCKR 的单步在线形式。
%
% 字段映射（与模板一致）
%   A ≡ Matrix_F，B ≡ Matrix_G，C ≡ Matrix_H
%   P ≡ Matrix_P，Q ≡ Matrix_Q，R ≡ Matrix_R
%   采样时间 T ≡ Intervel，预测步数 ≡ PredictStep
%   当前观测 ≡ CurrentObservation，确定性输入通道 ≡ Matrix_B（本实现取 u=0）
%
% 结构化不确定性参数读取（自 Double_Par）
%   令 n = Nx：
%     Ef = Double_Par(1:n)            （1×n 行向量）
%     Eg = Double_Par(n+1:2n)         （1×n 行向量）
%     M  = Double_Par(2n+1:3n)        （n×1 列向量）
%
% 参考：Huan Xu, Shie Mannor, “A Kalman Filter Design Based on the
%       Performance/Robustness Tradeoff”, IEEE TAC, 54(5):1171–1179, 2009.
% 作者：光电所一室2020级  孙敏行   联系方式：401435318@qq.com

% ===== 读取 Ef, Eg, M（与原式含义一致）=====
Ef_row = (StateSpaceModelN.Double_Par(1:StateSpaceModelN.Nx)).';                        % 1×n
Eg_row = (StateSpaceModelN.Double_Par(StateSpaceModelN.Nx+1:2*StateSpaceModelN.Nx)).';  % 1×n
M_col  =  StateSpaceModelN.Double_Par(2*StateSpaceModelN.Nx+1:3*StateSpaceModelN.Nx);   % n×1

% α 固定为 0.5（原函数做为入参；此处为最小实现，不做外部存取/写回）
alpha = 0.5;

% ===== Re{i}：沿用上一时刻等效量测噪声（与原式 Re{1} = R{1} 对应）=====
if ~StateSpaceModelN.Matrix_Par(1)
    Re = StateSpaceModelN.Matrix_R;
else
    Re = StateSpaceModelN.Matrix_Par;
end

% ===== P{i} = PP − PP H' (Re + H PP H')^{-1} H PP  （原式 P{i}）=====
Ppre = StateSpaceModelN.Matrix_P ...
     - StateSpaceModelN.Matrix_P * StateSpaceModelN.Matrix_H' ...
       / (Re + StateSpaceModelN.Matrix_H * StateSpaceModelN.Matrix_P * StateSpaceModelN.Matrix_H') ...
       * StateSpaceModelN.Matrix_H * StateSpaceModelN.Matrix_P;

% ===== 原式 A、b、T、W、D、Ea、Eb =====
A_t  = StateSpaceModelN.Matrix_H * [StateSpaceModelN.Matrix_F, StateSpaceModelN.Matrix_G];       % H*[F G]
bvec = StateSpaceModelN.CurrentObservation ...
     - StateSpaceModelN.Matrix_H * StateSpaceModelN.Matrix_F * StateSpaceModelN.EstimatedState;  % Y - H*F*x
Tblk = blkdiag(inv(Ppre), inv(StateSpaceModelN.Matrix_Q));                                       % T
Wmat = inv(StateSpaceModelN.Matrix_R);                                                           % W = 1/R
Dvec = StateSpaceModelN.Matrix_H * M_col;                                                        % D = H*M
Ea   = [Ef_row, Eg_row];
Eb   = -Ef_row * StateSpaceModelN.EstimatedState;

% Λ_min = |M' C' (R \ (C M))|
Lambdamin = abs( M_col.' * ( StateSpaceModelN.Matrix_H' * ...
               ( StateSpaceModelN.Matrix_R \ (StateSpaceModelN.Matrix_H * M_col) ) ) );

% ===== 一维优化：GLF(Λ)（与原式等价重写）=====
GLF = @(Lambda) local_glf_imckr(Lambda, alpha, Tblk, A_t, bvec, Wmat, Dvec, Ea, Eb);
Lambdao = fminbnd(GLF, Lambdamin, 100*Lambdamin);
Lambdae = (1 - alpha) * Lambdao;

% ===== Re{i+1}（原式插值）=====
Re_next = inv( alpha * inv(StateSpaceModelN.Matrix_R) ...
            + (1 - alpha) * inv( StateSpaceModelN.Matrix_R ...
                - (1/max(Lambdao,eps)) * (StateSpaceModelN.Matrix_H*M_col) * (StateSpaceModelN.Matrix_H*M_col)' ) );

% ===== (Qe, Fe, Ge) 四种情形（与原式等价）=====
Qe = StateSpaceModelN.Matrix_Q;
Fe = StateSpaceModelN.Matrix_F;
Ge = StateSpaceModelN.Matrix_G;

EgEgT = Eg_row * Eg_row';   % 标量
EfEfT = Ef_row * Ef_row';   % 标量
EfEgT = Ef_row * Eg_row';   % 标量

if EgEgT == 0
    Qe = StateSpaceModelN.Matrix_Q;
    Fe = StateSpaceModelN.Matrix_F * ( eye(StateSpaceModelN.Nx) ...
        - Lambdae * Ppre * (Ef_row' * Ef_row) );
    Ge = StateSpaceModelN.Matrix_G;

elseif EfEfT == 0
    Qe = inv( inv(StateSpaceModelN.Matrix_Q) + Lambdae * (Eg_row' * Eg_row) );
    Fe = StateSpaceModelN.Matrix_F;
    Ge = StateSpaceModelN.Matrix_G;

elseif EfEgT == 0
    Qe = inv( inv(StateSpaceModelN.Matrix_Q) + Lambdae * (Eg_row' * Eg_row) );
    Fe = StateSpaceModelN.Matrix_F * ( eye(StateSpaceModelN.Nx) ...
        - Lambdae * Ppre * (Ef_row' * Ef_row) );
    Ge = StateSpaceModelN.Matrix_G;

else
    Qe = inv( inv(StateSpaceModelN.Matrix_Q) ...
          + (Lambdae / (1 + Lambdae * (Ef_row * Ppre * Ef_row'))) * (Eg_row' * Eg_row) );

    Fe = ( StateSpaceModelN.Matrix_F ...
         - Lambdae * StateSpaceModelN.Matrix_G * Qe * (Eg_row' * Ef_row) ) ...
         * ( eye(StateSpaceModelN.Nx) - Lambdae * Ppre * (Ef_row' * Ef_row) );

    Ge = StateSpaceModelN.Matrix_G ...
       - Lambdae * StateSpaceModelN.Matrix_F * Ppre * (Ef_row' * Eg_row);
end

% ===== 状态更新（原式：Xe{i+1} = Fe*x + Fe*P*H'/Re*(Y - H*x)；此处 P=Ppre）=====
innovation = StateSpaceModelN.CurrentObservation - StateSpaceModelN.Matrix_H * StateSpaceModelN.EstimatedState;
StateSpaceModelN.EstimatedState = ...
    Fe * StateSpaceModelN.EstimatedState ...
  + Fe * ( Ppre * StateSpaceModelN.Matrix_H' * ( Re \ innovation ) );

% ===== 协方差更新（原式：Hb 用 Re^{-(Time/2)}；PP 走 F，Kb 用 P{i}=Ppre）=====
Hb  = [ StateSpaceModelN.Matrix_H' * ( Re^(-StateSpaceModelN.Intervel/2) ), ...
        sqrt(Lambdae) * (Ef_row') ]';
Rbe = eye(size(Hb,1)) + Hb * StateSpaceModelN.Matrix_P * Hb';
Kb  = StateSpaceModelN.Matrix_F * Ppre * Hb';
StateSpaceModelN.Matrix_P = ...
    StateSpaceModelN.Matrix_F * StateSpaceModelN.Matrix_P * StateSpaceModelN.Matrix_F' ...
  - Kb / Rbe * Kb' ...
  + Ge * Qe * Ge';

% ===== 写回 Re{i+1} 供下一步使用（与原式 Re{i+1} 对齐）=====
StateSpaceModelN.Matrix_Par = Re_next;

% ===== 最小化输出：不做多步预测；仅给出“当前时刻”的预测量（等同设为估计量）=====
StateSpaceModelN.PredictedState       = StateSpaceModelN.EstimatedState;
StateSpaceModelN.PredictedObservation = StateSpaceModelN.Matrix_H * StateSpaceModelN.PredictedState;

end

% -------- GLF(Λ)：与原式等价（WL, ZL, 目标函数三项）--------
function val = local_glf_imckr(Lambda, alpha, Tblk, A_t, bvec, Wmat, Dvec, Ea, Eb)
WL = Wmat + (1 - alpha) * ( Wmat * Dvec * pinv( Lambda - (Dvec' * Wmat * Dvec) ) * Dvec' * Wmat );
K   = Tblk + (A_t' * WL * A_t) + (1 - alpha) * Lambda * (Ea' * Ea);
rhs = (A_t' * WL * bvec) + (1 - alpha) * Lambda * (Ea' * Eb);
ZL  = K \ rhs;
resA = (A_t * ZL - bvec);
resE = (Ea  * ZL - Eb);
val = (ZL' * Tblk * ZL) + (resA' * WL * resA) + (1 - alpha) * Lambda * (resE' * resE);
end