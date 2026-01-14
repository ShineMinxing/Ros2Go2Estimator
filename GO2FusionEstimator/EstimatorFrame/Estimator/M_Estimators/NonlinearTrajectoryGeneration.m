function StateSpaceModelN = NonlinearTrajectoryGeneration(StateSpaceModelN)
    % 仿真非线性轨迹
    % 确保结构体已初始化
    if isempty(StateSpaceModelN)
        error('State space model is not initialized');
    end
    
    EstimatedState = StateSpaceModelN.EstimatedState;
    [EstimatedState, ~] = StateSpaceModelN.StateTransitionEquation(EstimatedState, StateSpaceModelN);
    EstimatedState = EstimatedState + sqrtm(StateSpaceModelN.Matrix_Q) * randn(StateSpaceModelN.Nx,1);
    StateSpaceModelN.EstimatedState = EstimatedState;

    [CurrentObservation, ~] = StateSpaceModelN.ObservationEquation(EstimatedState, StateSpaceModelN);
    CurrentObservation = CurrentObservation + sqrtm(StateSpaceModelN.Matrix_R) * randn(StateSpaceModelN.Nz,1);
    StateSpaceModelN.CurrentObservation = CurrentObservation;

end