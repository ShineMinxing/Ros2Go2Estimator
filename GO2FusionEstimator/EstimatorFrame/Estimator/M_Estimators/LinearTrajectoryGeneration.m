function StateSpaceModelN = LinearTrajectoryGeneration(StateSpaceModelN)
    % 仿真线性轨迹
    % 确保结构体已初始化
    if isempty(StateSpaceModelN)
        error('State space model is not initialized');
    end

    StateSpaceModelN.EstimatedState =  StateSpaceModelN.Matrix_F *  StateSpaceModelN.EstimatedState + ...
        StateSpaceModelN.Matrix_G * StateSpaceModelN.Matrix_B + sqrtm(StateSpaceModelN.Matrix_Q) * randn(StateSpaceModelN.Nx,1);


    StateSpaceModelN.CurrentObservation = StateSpaceModelN.Matrix_H *  StateSpaceModelN.EstimatedState + ...
        sqrtm(StateSpaceModelN.Matrix_R) * randn(StateSpaceModelN.Nz,1);

end