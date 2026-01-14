function StateSpaceModelN = Estimator3007(StateSpaceModelN)
% Estimator3007 — RESP（Error Sensitivity Penalizing）鲁棒状态估计
%
% 数学模型
%   x_{k+1} = F x_k + G w_k ,   z_k = H x_k + v_k
%   参数不确定性通过灵敏度项(S,T)进入等效噪声设计与滤波器系数中
%
% 方法要点（参见：Robust State Estimation Using Error Sensitivity Penalizing， Tong Zhou, CDC 2008）
%   同时最小化名义估计误差与误差对参数不确定性的灵敏度：
%     S = blkrow_k [ PH_k (F + Fep1_k·ε_k·Fep2_k) ;
%                    (H + Hep1_k·ε_k·Hep2_k) · PF_k ]
%     T = blkrow_k [ PH_k (G + Gep1_k·ε_k·Gep2_k) ;
%                    (H + Hep1_k·ε_k·Hep2_k) · PG_k ]
%   令 ρ = (1-Γ)/Γ：
%     Q_e = [ Q^{-1} + ρ·T' (I + ρ·S P S') T ]^{-1}
%     P_e = [ P^{-1} + ρ·S' S ]^{-1}
%     G_e = G − ρ·F P_e S' S
%     F_e = (F − ρ·G_e Q_e T' S) · (I − ρ·P_e S' S)
%     P⁺  = F P_e F' + G_e Q_e G_e'
%     R_e = R + H P⁺ H'
%     P   = P⁺ − P⁺ H' / R_e · H P⁺
%     x̂   = F_e x̂ + P H' / R · ( z − H F_e x̂ )
%
% 作者：光电所一室2020级 孙敏行    联系方式：401435318@qq.com

if isempty(StateSpaceModelN)
    error('State space model is not initialized');
end
if StateSpaceModelN.Nx ~= 4
    error('Estimator3007:Nx','This Estimator3007 implementation requires StateSpaceModelN.Nx == 4.');
end

% ===== 1) 固定 RESP 参数（显式字面量）=====
% 折中参数 Γ 与 ρ
Gamma_local = 0.8;
rho         = (1 - Gamma_local) / Gamma_local;

% 不确定性标量 ε
eps_local   = -0.8508;

% 灵敏度矩阵（显式写法；与 2×2 示例一致的“位置取值”扩展到 4×4）
% PF0 = blkdiag([0 0.0990; 0 0], [0 0.0990; 0 0]);
PF_k = [ 0      0.0990   0       0;
         0      0        0       0;
         0      0        0       0.0990;
         0      0        0       0 ];

% FEP1 = [0.0198; 0] 扩展为作用在 X、Y 两个子系统的列向量
Fep1_k = [ 0.0198; 0; 0.0198; 0 ];

% FEP2 = [0, 5] 扩展为作用在第2列与第4列的行向量
Fep2_k = [ 0, 5, 0, 5 ];

% 其余灵敏度项按脚本设为 0（显式字面量）
PH_k   = [ 0 0 0 0;
           0 0 0 0;
           0 0 0 0;
           0 0 0 0 ];

PG_k   = [ 0 0 0 0;
           0 0 0 0;
           0 0 0 0;
           0 0 0 0 ];

Gep1_k = [ 0; 0; 0; 0 ];
Gep2_k = [ 0, 0, 0, 0 ];

% H 属于 2×4，本实现不对 H 做秩一扰动（与脚本设为 0 保持一致）
Hep1_k = [ 0; 0 ];
Hep2_k = [ 0, 0, 0, 0 ];

% ===== 2) 构造 S、T =====
Fk = StateSpaceModelN.Matrix_F + Fep1_k * eps_local * Fep2_k;   % 4×4
Gk = StateSpaceModelN.Matrix_G + Gep1_k * eps_local * Gep2_k;   % 4×4（与常见 G=I_4 一致）
Hk = StateSpaceModelN.Matrix_H + Hep1_k * eps_local * Hep2_k;   % 2×4（此处即为 H）

% S（(4+2)×4）与 T（(4+2)×4），直接按公式堆叠
Si = [ PH_k * Fk ;
       Hk   * PF_k ];

Ti = [ PH_k * Gk ;
       Hk   * PG_k ];

% ===== 3) 等效噪声/增益（RESP 主公式）=====
Qe = inv( inv(StateSpaceModelN.Matrix_Q) ...
      + rho * ( Ti' * ( eye(size(Si,1)) + rho * Si * StateSpaceModelN.Matrix_P * Si' ) * Ti ) );

Pe = inv( inv(StateSpaceModelN.Matrix_P) + rho * ( Si' * Si ) );

Ge = StateSpaceModelN.Matrix_G - rho * StateSpaceModelN.Matrix_F * Pe * (Si' * Si);

Fe = ( StateSpaceModelN.Matrix_F - rho * Ge * Qe * (Ti' * Si) ) ...
   * ( eye(4) - rho * Pe * (Si' * Si) );

PP = StateSpaceModelN.Matrix_F * Pe * StateSpaceModelN.Matrix_F' + Ge * Qe * Ge';

Re = StateSpaceModelN.Matrix_R + StateSpaceModelN.Matrix_H * PP * StateSpaceModelN.Matrix_H';

StateSpaceModelN.Matrix_P = PP - PP * StateSpaceModelN.Matrix_H' / Re * StateSpaceModelN.Matrix_H * PP;

% ===== 4) 状态更新（测量校正使用名义 R）=====
innovation = StateSpaceModelN.CurrentObservation - StateSpaceModelN.Matrix_H * (Fe * StateSpaceModelN.EstimatedState);
StateSpaceModelN.EstimatedState = Fe * StateSpaceModelN.EstimatedState ...
    + StateSpaceModelN.Matrix_P * StateSpaceModelN.Matrix_H' / StateSpaceModelN.Matrix_R * innovation;

% ===== 5) 前视预测（u=0，保持模板接口一致）=====
StateSpaceModelN.PredictedState = StateSpaceModelN.EstimatedState;
if StateSpaceModelN.PredictStep
    for j = 1:StateSpaceModelN.PredictStep
        StateSpaceModelN.PredictedState = StateSpaceModelN.Matrix_F * StateSpaceModelN.PredictedState;
    end
end
StateSpaceModelN.PredictedObservation = StateSpaceModelN.Matrix_H * StateSpaceModelN.PredictedState;
end