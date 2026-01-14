function StateSpaceModelN = Estimator3004(StateSpaceModelN)
% Extended Kalman Filter Estimator
% 作者：光电所一室2020级  孙敏行   联系方式：401435318@qq.com

    % Ensure the structure is initialized
    if isempty(StateSpaceModelN)
        error('State space model is not initialized');
    end

    % Predict the next state using the state transition function
    [StateSpaceModelN.EstimatedState, ~] = StateSpaceModelN.StateTransitionEquation(StateSpaceModelN.EstimatedState, StateSpaceModelN);
    
    % Compute the Jacobian of the state transition function
    [F, ~] = StateSpaceModelN.StateTransitionDiffEquation(StateSpaceModelN.EstimatedState, StateSpaceModelN);
    
    % Predict the error covariance
    P_pre = F * StateSpaceModelN.Matrix_P * F' + StateSpaceModelN.Matrix_Q;
    
    % Compute the predicted observation
    [Zpre, ~] = StateSpaceModelN.ObservationEquation(StateSpaceModelN.EstimatedState, StateSpaceModelN);
    
    % Compute the Jacobian of the observation function
    [H, ~] = StateSpaceModelN.ObservationDiffEquation(StateSpaceModelN.EstimatedState, StateSpaceModelN);

    % Compute the innovation (observation residual)
    Innovation = StateSpaceModelN.CurrentObservation - Zpre;
    
    % Compute the innovation covariance
    InnovationCovariance = H * P_pre * H' + StateSpaceModelN.Matrix_R;

    % Compute the Kalman gain
    K = P_pre * H' / InnovationCovariance;

    % Update the state estimate
    StateSpaceModelN.EstimatedState = StateSpaceModelN.EstimatedState + K * Innovation;

    % Update the error covariance
    StateSpaceModelN.Matrix_P = (eye(size(P_pre)) - K * H) * P_pre;

    % 3. Predict the next state and observation for the next time step
    [PredictedState, ~] = StateSpaceModelN.PredictionEquation(StateSpaceModelN.EstimatedState, StateSpaceModelN);

    StateSpaceModelN.PredictedState = PredictedState;
    StateSpaceModelN.PredictedObservation = PredictedState;
end