function StateSpaceModelN = Estimator3001(StateSpaceModelN)
% Linear Kalman
% 作者：光电所一室2020级  孙敏行   联系方式：401435318@qq.com

    % 确保结构体已初始化
    if isempty(StateSpaceModelN)
        error('State space model is not initialized');
    end

    % 状态预测 E3001_Xe
    StateSpaceModelN.PredictedState = StateSpaceModelN.Matrix_F * StateSpaceModelN.EstimatedState;
    StateSpaceModelN.EstimatedState = StateSpaceModelN.PredictedState;

    % 协方差预测 E3001_P_pre
    P_pre = StateSpaceModelN.Matrix_F * StateSpaceModelN.Matrix_P * StateSpaceModelN.Matrix_F' + StateSpaceModelN.Matrix_Q;

    % 计算卡尔曼增益 E3001_K
    ZZ1 = StateSpaceModelN.Matrix_H * P_pre * StateSpaceModelN.Matrix_H' + StateSpaceModelN.Matrix_R;
    K = P_pre * StateSpaceModelN.Matrix_H' / ZZ1;

    % 状态更新 E3001_Xe
    Z12 = StateSpaceModelN.CurrentObservation - StateSpaceModelN.Matrix_H * StateSpaceModelN.EstimatedState;
    StateSpaceModelN.EstimatedState = StateSpaceModelN.EstimatedState + K * Z12;

    % 协方差更新 E3001_P
    StateSpaceModelN.Matrix_P = (eye(StateSpaceModelN.Nx) - K * StateSpaceModelN.Matrix_H) * P_pre;

    % 状态和观测预测 E3001_Z
    StateSpaceModelN.PredictedState = StateSpaceModelN.EstimatedState;
    if(StateSpaceModelN.PredictStep)
        for i = 1:StateSpaceModelN.PredictStep
            StateSpaceModelN.PredictedState = StateSpaceModelN.Matrix_F * StateSpaceModelN.PredictedState;
        end
    end
    StateSpaceModelN.PredictedObservation = StateSpaceModelN.Matrix_H * StateSpaceModelN.PredictedState;
end
