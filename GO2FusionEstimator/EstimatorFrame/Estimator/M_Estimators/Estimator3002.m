function StateSpaceModelN = Estimator3002(StateSpaceModelN)
% Unscented Kalman Estiamtor
% 作者：光电所一室2020级  孙敏行   联系方式：401435318@qq.com

    % 确保结构体已初始化
    if isempty(StateSpaceModelN)
        error('State space model is not initialized');
    end

    % 设置UKF参数
    alpha = 1e-2;
    ki = 0;
    beta = 2;
    lambda = alpha^2 * (StateSpaceModelN.Nx + ki) - StateSpaceModelN.Nx;
    Wm = [lambda / (StateSpaceModelN.Nx + lambda), 0.5 / (StateSpaceModelN.Nx + lambda) * ones(1, 2 * StateSpaceModelN.Nx)];  % 均值权重
    Wc = Wm;
    Wc(1) = Wc(1) + 1 - alpha^2 + beta;

    % 对观测值序列进行估计

    % 1. 生成Sigma点集
    A = sqrt(StateSpaceModelN.Nx + lambda) * chol(StateSpaceModelN.Matrix_P)';
    Xsigmaset = [StateSpaceModelN.EstimatedState, StateSpaceModelN.EstimatedState + A, StateSpaceModelN.EstimatedState - A];

    % 2. 对状态进行UT变换，计算均值和协方差
    LL = size(Xsigmaset, 2);
    X1means = zeros(StateSpaceModelN.Nx, 1);
    X1 = zeros(StateSpaceModelN.Nx, LL);
    for j = 1:LL
        [X1(:, j), StateSpaceModelN] = StateSpaceModelN.StateTransitionEquation(Xsigmaset(:, j), StateSpaceModelN);  % 状态转移函数
        X1means = X1means + Wm(j) * X1(:, j);
    end
    X2 = X1 - X1means;
    P1 = X2 * diag(Wc) * X2' + StateSpaceModelN.Matrix_Q;

    % 3. 对观测进行UT变换，计算均值和协方差
    Zpre = zeros(StateSpaceModelN.Nz, 1);
    Z1 = zeros(StateSpaceModelN.Nz, LL);
    for j = 1:LL
        [Z1(:, j), StateSpaceModelN] = StateSpaceModelN.ObservationEquation(X1(:, j), StateSpaceModelN);
        Zpre = Zpre + Wm(j) * Z1(:, j);
    end
    Z2 = Z1 - Zpre;
    Pzz = Z2 * diag(Wc) * Z2' + StateSpaceModelN.Matrix_R;
    Pxz = X2 * diag(Wc) * Z2';

    % 4. 计算卡尔曼增益
    K = Pxz / Pzz;

    % 5. 状态和协方差更新
    Z_diff = StateSpaceModelN.CurrentObservation - Zpre;
    StateSpaceModelN.EstimatedState = X1means + K * Z_diff;
    StateSpaceModelN.Matrix_P = P1 - K * Pxz';

    % 6. 状态和观测预测
    [PredictedState, StateSpaceModelN] = StateSpaceModelN.PredictionEquation(StateSpaceModelN.EstimatedState, StateSpaceModelN);
    [PredictedObservation, StateSpaceModelN] = StateSpaceModelN.ObservationEquation(PredictedState, StateSpaceModelN);
    StateSpaceModelN.PredictedState = PredictedState';
    StateSpaceModelN.PredictedObservation = PredictedObservation';
end
