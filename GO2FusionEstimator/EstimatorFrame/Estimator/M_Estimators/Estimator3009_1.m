function StateSpaceModelN = Estimator3009_1(StateSpaceModelN)
% Estimator3009_1 — H-infinity（风险敏感/信息型）线性鲁棒估计器
% x_{k+1}=F x_k + G w_k,  z_k = H x_k + v_k
% P_pre = F P F' + (G Q G' 或 Q)
% P_plus = (P_pre^{-1} + H'R^{-1}H - θ S_b)^{-1}
% K = P_plus H' R^{-1}
% x_hat = x_pre + K (z - H x_pre)
% 作者：光电所一室2020级  孙敏行    联系方式：401435318@qq.com

    % —— 安全检查 ——
    if isempty(StateSpaceModelN)
        error('State space model is not initialized');
    end

    if isempty(StateSpaceModelN)
        error('State space model is not initialized');
    end

    % —— 风险敏感权重 —— 
    L = eye(StateSpaceModelN.Nx);
    if StateSpaceModelN.Nx == 4
        % 位置权重大于速度（可按需调整）
        S = diag([10,1,10,1]);
    else
        S = eye(StateSpaceModelN.Nx);
    end
    theta = 1e-2;  % 建议 1e-4 ~ 1e-2

    % —— 先验（时间更新）——
    x_pre = StateSpaceModelN.Matrix_F * StateSpaceModelN.EstimatedState;

    % 对称化 P、R 并做微抖动保护 R
    P = 0.5*(StateSpaceModelN.Matrix_P + StateSpaceModelN.Matrix_P.');
    R = 0.5*(StateSpaceModelN.Matrix_R + StateSpaceModelN.Matrix_R.') + 1e-12*eye(StateSpaceModelN.Nz);
    Rinv = R \ eye(StateSpaceModelN.Nz);

    % P_pre
    P_pre = StateSpaceModelN.Matrix_F * P * StateSpaceModelN.Matrix_F.' ...
            + StateSpaceModelN.Matrix_G * StateSpaceModelN.Matrix_Q * StateSpaceModelN.Matrix_G.';
    P_pre = 0.5*(P_pre + P_pre.');

    % —— 信息型量测更新（基于 P_pre）——
    Sb = L.' * S * L;
    U  = StateSpaceModelN.Matrix_H.' * (Rinv * StateSpaceModelN.Matrix_H) - theta * Sb;
    M  = eye(StateSpaceModelN.Nx) + P_pre * U;

    % 防近奇异：必要时缩小 theta
    if rcond(M) < 1e-12
        theta_min = 1e-8; shrink = 10;
        while rcond(M) < 1e-12 && theta > theta_min
            theta = theta / shrink;
            U = StateSpaceModelN.Matrix_H.' * (Rinv * StateSpaceModelN.Matrix_H) - theta * Sb;
            M = eye(StateSpaceModelN.Nx) + P_pre * U;
        end
        if rcond(M) < 1e-12
            warning('Estimator3009_1: M nearly singular; reduce theta and/or check R,P.');
        end
    end

    % P_plus = (I + P_pre U) \ P_pre  （避免显式逆）
    P_plus = M \ P_pre;
    P_plus = 0.5*(P_plus + P_plus.');

    % K = P_plus H' R^{-1}
    K = P_plus * (StateSpaceModelN.Matrix_H.' * Rinv);

    % 状态后验
    innov = StateSpaceModelN.CurrentObservation - StateSpaceModelN.Matrix_H * x_pre;
    x_hat = x_pre + K * innov;

    % —— 写回（保持“后验”存入 Matrix_P，与库内其他滤波器一致）——
    StateSpaceModelN.EstimatedState = x_hat;
    StateSpaceModelN.Matrix_P       = P_plus;

    % —— 前视预测（可选）——
    StateSpaceModelN.PredictedState = x_hat;
    if StateSpaceModelN.PredictStep
        for t = 1:StateSpaceModelN.PredictStep
            StateSpaceModelN.PredictedState = StateSpaceModelN.Matrix_F * StateSpaceModelN.PredictedState;
        end
    end
    StateSpaceModelN.PredictedObservation = StateSpaceModelN.Matrix_H * StateSpaceModelN.PredictedState;
end