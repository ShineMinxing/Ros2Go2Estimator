function StateSpaceModelN = Estimator3009(StateSpaceModelN)
% Estimator3009 — Discrete-Time H-infinity Filter (经典 H∞ 估计器)
%
% 模型
%   x_{k+1} = F x_k + w_k,    w_k 为能量有界扰动
%   z_k     = H x_k + v_k,    v_k 为能量有界扰动
%
% 关键步骤（D=0）
%   P_pre = F P F' + Q
%   S     = H P_pre H' + R
%   S_g   = S - (1/gamma^2) * I         (需 S_g ≻ 0)
%   K     = P_pre H' / S_g
%   x̂     ← x̂_pre + K (z - H x̂_pre)
%   P     ← P_pre - K S K'
%
%   gamma 读取自 Double_Par(1)（<=0 或未设则取 1.5），并回写有效值
%
% 作者与联系方式
%   光电所一室2020级  孙敏行
%   401435318@qq.com

    if isempty(StateSpaceModelN)
        error('State space model is not initialized');
    end

    % === 读取/设置 gamma ===
    gamma = 1.5;
    if numel(StateSpaceModelN.Double_Par) >= 1 && StateSpaceModelN.Double_Par(1) > 0
        gamma = StateSpaceModelN.Double_Par(1);
    end

    % === 预测 ===
    x_pre = StateSpaceModelN.Matrix_F * StateSpaceModelN.EstimatedState;
    P_pre = StateSpaceModelN.Matrix_F * StateSpaceModelN.Matrix_P * StateSpaceModelN.Matrix_F' ...
          + StateSpaceModelN.Matrix_Q;

    % === H∞ 增益 ===
    S  = StateSpaceModelN.Matrix_H * P_pre * StateSpaceModelN.Matrix_H' + StateSpaceModelN.Matrix_R;
    % 确保 S_g 正定：需要 1/gamma^2 < lambda_min(S)
    % 若不满足，则调大 gamma 到阈值之上
    Ssym = (S + S')/2;
    lam_min = min(eig(Ssym));
    if ~(lam_min > 0)
        % 极端情况下数值修正（理论上 R>0 时不应出现）
        epsJ = 1e-9;
        Ssym = Ssym + epsJ*eye(size(Ssym));
        lam_min = min(eig(Ssym));
    end
    gamma_needed = 1/sqrt(max(lam_min, eps));
    if gamma < gamma_needed * (1 + 1e-6)
        gamma = gamma_needed * (1 + 1e-6);  % 略高于阈值，保证 S_g ≻ 0
    end
    S_g = Ssym - (1/gamma^2) * eye(size(Ssym));

    % 增益与更新
    K = (P_pre * StateSpaceModelN.Matrix_H') / S_g;
    innov = StateSpaceModelN.CurrentObservation - StateSpaceModelN.Matrix_H * x_pre;
    StateSpaceModelN.EstimatedState = x_pre + K * innov;

    % P 更新时用 S（非 S_g），以保证半正定（经典实现）
    StateSpaceModelN.Matrix_P = P_pre - K * S * K';

    % === 前视预测 ===
    StateSpaceModelN.PredictedState = StateSpaceModelN.EstimatedState;
    if StateSpaceModelN.PredictStep
        for i = 1:StateSpaceModelN.PredictStep
            StateSpaceModelN.PredictedState = StateSpaceModelN.Matrix_F * StateSpaceModelN.PredictedState;
        end
    end
    StateSpaceModelN.PredictedObservation = ...
        StateSpaceModelN.Matrix_H * StateSpaceModelN.PredictedState;

    % 回写有效 gamma
    StateSpaceModelN.Double_Par(1) = gamma;
end
