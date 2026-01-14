function StateSpaceModelN = ModelErrorLinearTrajectoryGeneration(StateSpaceModelN)
    % 仿真有模型误差的线性轨迹
    % 确保结构体已初始化
    if isempty(StateSpaceModelN)
        error('State space model is not initialized');
    end

    Ef = [0 1 0 0];
    Eg = [0 0 0 0];
    M  = [0.0198;0;0;0];

    StateSpaceModelN.EstimatedState = (StateSpaceModelN.Matrix_F + M*(1-2*rand)*Ef) * StateSpaceModelN.EstimatedState + ...
        (StateSpaceModelN.Matrix_G + M*(1-2*rand)*Eg) * StateSpaceModelN.Matrix_B + sqrtm(StateSpaceModelN.Matrix_Q) * randn(StateSpaceModelN.Nx,1);
    
    StateSpaceModelN.CurrentObservation = StateSpaceModelN.Matrix_H * StateSpaceModelN.EstimatedState + ...
        sqrtm(StateSpaceModelN.Matrix_R) * randn(StateSpaceModelN.Nz,1);
end