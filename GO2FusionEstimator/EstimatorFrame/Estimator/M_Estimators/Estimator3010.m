function StateSpaceModelN = Estimator3010(StateSpaceModelN)
% Estimator3010 — Interactive Multiple Model (IMM) 交互式多模型估计器
%
% 模型假设
%   顶层 StateSpaceModelN 维护一个“融合模型”，内部包含若干子模型：
%     StateSpaceModelN.IMM_ModeNames : 子模型名 cell，如 {'SubModel1','SubModel2', ...}
%     StateSpaceModelN.IMM_Mn        : 子模型数量 Mn
%     StateSpaceModelN.IMM_Ptrans    : 模式转移矩阵 P_ij = P(mode_j | mode_i)   (Mn×Mn)
%     StateSpaceModelN.IMM_ModeProb  : 上一时刻模式概率 μ_i(k-1)               (Mn×1)
%
%   每个子模型 StateSpaceModelN.(IMM_ModeNames{j}) 自身是一个完整的状态空间模型+估计器：
%     .EstimatedState     : x̂_j(k-1)
%     .Matrix_P           : P_j(k-1)
%     .Matrix_H, .Matrix_R: 量测矩阵与噪声协方差
%     .EstimatorPort      : 单模型滤波函数句柄，调用时完成一次从 k-1 -> k 的更新
%
% IMM 一步递推流程
%   1) 收集各模式 k-1 时刻后验 (x̂_i, P_i)
%   2) 输入交互：利用转移矩阵和 μ_i(k-1) 计算混合权 μ_ij(i,j)，得到每个模式的混合先验 (x̂_0j, P_0j)
%   3) 对每个子模型 j：
%        - 以 (x̂_0j, P_0j) 为初值调用其 EstimatorPort 做一步滤波
%        - 基于混合先验 (x̂_0j, P_0j) 计算模式似然 A_j
%   4) 更新模式概率 μ_j(k) ∝ A_j · c_j
%   5) 融合所有子模型的后验，得到顶层的 x̂(k), P(k)
%   6) 顶层再做统一的前视预测 PredictedState / PredictedObservation
%
% 作者与联系方式
%   光电所一室2020级  孙敏行
%   401435318@qq.com

    % ========= 1) 收集各子模型上一时刻后验 =========
    X_post = zeros(StateSpaceModelN.Nx, StateSpaceModelN.IMM_Mn);
    P_post = zeros(StateSpaceModelN.Nx, StateSpaceModelN.Nx, StateSpaceModelN.IMM_Mn);

    for j = 1:StateSpaceModelN.IMM_Mn
        sub = StateSpaceModelN.(StateSpaceModelN.IMM_ModeNames{j});
        X_post(:,j)   = sub.EstimatedState;
        P_post(:,:,j) = 0.5 * (sub.Matrix_P + sub.Matrix_P.');   % 对称化以防数值非对称
    end

    mu_prev = StateSpaceModelN.IMM_ModeProb(:);   % μ_i(k-1) 作为列向量

    % ========= 2) 输入交互：混合权 μ_ij 与混合先验 =========
    % 2.1 马尔可夫预测概率 c_j = ∑_i P_ij μ_i(k-1)
    c_j = StateSpaceModelN.IMM_Ptrans.' * mu_prev;         % (Mn×1)
    c_j = c_j + eps;                                       % 防止出现精确 0

    % 2.2 条件混合概率 μ_ij(i,j) = P_ij μ_i / c_j(j)
    mu_ij = (StateSpaceModelN.IMM_Ptrans .* (mu_prev * ones(1, StateSpaceModelN.IMM_Mn))) ...
          ./ (ones(StateSpaceModelN.IMM_Mn,1) * c_j.');

    % 2.3 混合先验 (x̂_0j, P_0j)
    X_mix = zeros(StateSpaceModelN.Nx, StateSpaceModelN.IMM_Mn);
    P_mix = zeros(StateSpaceModelN.Nx, StateSpaceModelN.Nx, StateSpaceModelN.IMM_Mn);

    for j = 1:StateSpaceModelN.IMM_Mn
        % 状态混合
        X_mix(:,j) = X_post * mu_ij(:,j);

        % 协方差混合
        Ptemp = zeros(StateSpaceModelN.Nx, StateSpaceModelN.Nx);
        for i = 1:StateSpaceModelN.IMM_Mn
            dx   = X_post(:,i) - X_mix(:,j);
            Ptemp = Ptemp + mu_ij(i,j) * (P_post(:,:,i) + dx * dx.');
        end
        P_mix(:,:,j) = 0.5 * (Ptemp + Ptemp.');
    end

    % ========= 3) 子滤波器更新 + 模式似然 Aj =========
    Aj = zeros(StateSpaceModelN.IMM_Mn, 1);
    z_k = StateSpaceModelN.CurrentObservation(:);

    for j = 1:StateSpaceModelN.IMM_Mn
        name = StateSpaceModelN.IMM_ModeNames{j};
        sub  = StateSpaceModelN.(name);

        % 3.1 把混合先验写回子模型
        sub.EstimatedState     = X_mix(:,j);
        sub.Matrix_P           = P_mix(:,:,j);
        sub.CurrentObservation = z_k;

        % 3.2 调用该子模型自己的 EstimatorPort（可以是 KF / EKF / H∞ 等）
        sub = sub.EstimatorPort(sub);

        % 3.3 子模型后验写回顶层缓存
        StateSpaceModelN.(name) = sub;
        X_post(:,j)             = sub.EstimatedState;
        P_post(:,:,j)           = 0.5 * (sub.Matrix_P + sub.Matrix_P.');

        % 3.4 模式似然 A_j — 基于“混合先验的预测测量”
        %     ẑ_j^- = H_j x̂_0j,  S_j = H_j P_0j H_j' + R_j
        H_j = sub.Matrix_H;
        R_j = 0.5 * (sub.Matrix_R + sub.Matrix_R.') + 1e-12 * eye(StateSpaceModelN.Nz);

        innov_j = z_k - H_j * X_mix(:,j);
        S_j     = H_j * P_mix(:,:,j) * H_j.' + R_j;
        S_j     = 0.5 * (S_j + S_j.') + 1e-12 * eye(StateSpaceModelN.Nz);

        % 用 Cholesky 计算似然（避免显式求逆和 det）
        [L, pd] = chol(S_j, 'lower');
        if pd ~= 0
            S_j = S_j + 1e-6 * eye(StateSpaceModelN.Nz);
            [L, pd] = chol(S_j, 'lower');
        end

        if pd == 0
            logdetS = 2 * sum(log(diag(L)));
            quad    = innov_j' * (S_j \ innov_j);
            % 高斯似然的常数 (2π)^(m/2) 在各模式间相同，可忽略
            Aj(j)   = exp(-0.5 * (quad + logdetS));
        else
            Aj(j)   = realmin;
        end
    end

    % ========= 4) 模式概率更新 μ_j(k) ∝ A_j · c_j =========
    num   = Aj .* c_j;
    den   = sum(num);
    mu_new = num / den;
    StateSpaceModelN.IMM_ModeProb = mu_new;

    % ========= 5) 模式融合输出 =========
    % 融合后的后验状态
    X_fused = X_post * mu_new;

    % 融合后的后验协方差
    P_fused = zeros(StateSpaceModelN.Nx, StateSpaceModelN.Nx);
    for j = 1:StateSpaceModelN.IMM_Mn
        dx = X_post(:,j) - X_fused;
        P_fused = P_fused + mu_new(j) * (P_post(:,:,j) + dx * dx.');
    end
    P_fused = 0.5 * (P_fused + P_fused.');

    StateSpaceModelN.EstimatedState = X_fused;
    StateSpaceModelN.Matrix_P       = P_fused;

    % ========= 6) 顶层统一前视预测 =========
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
