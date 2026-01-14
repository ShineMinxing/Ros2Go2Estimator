function StateSpaceModelN = StateSpaceModel3(StateSpaceModelN)
    % 初始化结构体中的变量
    StateSpaceModelN.PortName = '3D Uniform Acceleration Model';
    StateSpaceModelN.PortIntroduction = 'For Reference';

    StateSpaceModelN.Nx = 9;
    StateSpaceModelN.Nz = 3;
    StateSpaceModelN.PredictStep = 3;
    StateSpaceModelN.Intervel = 0.005;
    StateSpaceModelN.PredictTime = StateSpaceModelN.PredictStep * StateSpaceModelN.Intervel;

    StateSpaceModelN.EstimatedState = [1; 0; 0; 1; 0; 0; 1; 0; 0];
    StateSpaceModelN.PredictedState = zeros(StateSpaceModelN.Nx, 1);
    StateSpaceModelN.CurrentObservation = zeros(StateSpaceModelN.Nz, 1);
    StateSpaceModelN.PredictedObservation = zeros(StateSpaceModelN.Nz, 1);
    StateSpaceModelN.Matrix_F = [
        1, StateSpaceModelN.Intervel, StateSpaceModelN.Intervel*StateSpaceModelN.Intervel/2, 0, 0, 0, 0, 0, 0;
        0, 1, StateSpaceModelN.Intervel, 0, 0, 0, 0, 0, 0;
        0, 0, 1, 0, 0, 0, 0, 0, 0;
        0, 0, 0, 1, StateSpaceModelN.Intervel, StateSpaceModelN.Intervel*StateSpaceModelN.Intervel/2, 0, 0, 0;
        0, 0, 0, 0, 1, StateSpaceModelN.Intervel, 0, 0, 0;
        0, 0, 0, 0, 0, 1, 0, 0, 0;
        0, 0, 0, 0, 0, 0, 1, StateSpaceModelN.Intervel, StateSpaceModelN.Intervel*StateSpaceModelN.Intervel/2;
        0, 0, 0, 0, 0, 0, 0, 1, StateSpaceModelN.Intervel;
        0, 0, 0, 0, 0, 0, 0, 0, 1;
    ];
    StateSpaceModelN.Matrix_G = eye(StateSpaceModelN.Nx);
    StateSpaceModelN.Matrix_B = [0; 0; 0; 0; 0; 0; 0; 0; 0;];
    StateSpaceModelN.Matrix_H = [
        1, 0, 0, 0, 0, 0, 0, 0, 0;
        0, 0, 0, 1, 0, 0, 0, 0, 0
        0, 0, 0, 0, 0, 0, 1, 0, 0
    ];
    StateSpaceModelN.Matrix_P = eye(StateSpaceModelN.Nx);
    StateSpaceModelN.Matrix_Q = eye(StateSpaceModelN.Nx);
    StateSpaceModelN.Matrix_R = eye(StateSpaceModelN.Nz);

    StateSpaceModelN.Int_Par = zeros(100,1);
    StateSpaceModelN.Double_Par = zeros(100,1);

    % 定义结构体中的函数句柄
    StateSpaceModelN.StateTransitionEquation = @(In_State, StateSpaceModelN) StateSpaceModel3StateTransitionFunction(In_State, StateSpaceModelN);
    StateSpaceModelN.ObservationEquation = @(In_State, StateSpaceModelN) StateSpaceModel3ObservationFunction(In_State, StateSpaceModelN);
    StateSpaceModelN.PredictionEquation = @(In_State, StateSpaceModelN) StateSpaceModel3PredictionFunction(In_State, StateSpaceModelN);
    StateSpaceModelN.EstimatorPort = @(StateSpaceModelN) StateSpaceModel3EstimatorPort(StateSpaceModelN);
    StateSpaceModelN.EstimatorPortTermination = @(StateSpaceModelN) StateSpaceModel3EstimatorPortTermination();
end

% 定义各个函数的实现
function [Out_State, StateSpaceModelN] = StateSpaceModel3StateTransitionFunction(In_State, StateSpaceModelN)
    Out_State = zeros(StateSpaceModelN.Nx,1);
    Out_State(1) = In_State(1) + StateSpaceModelN.Intervel * In_State(2) * StateSpaceModelN.Intervel * StateSpaceModelN.Intervel * In_State(3) / 2;
    Out_State(2) = In_State(2) + StateSpaceModelN.Intervel * In_State(3);
    Out_State(3) = In_State(3);
    Out_State(4) = In_State(4) + StateSpaceModelN.Intervel * In_State(5) * StateSpaceModelN.Intervel * StateSpaceModelN.Intervel * In_State(6) / 2;
    Out_State(5) = In_State(5) + StateSpaceModelN.Intervel * In_State(6);
    Out_State(6) = In_State(6);
    Out_State(7) = In_State(7) + StateSpaceModelN.Intervel * In_State(8) * StateSpaceModelN.Intervel * StateSpaceModelN.Intervel * In_State(9) / 2;
    Out_State(8) = In_State(8) + StateSpaceModelN.Intervel * In_State(9);
    Out_State(9) = In_State(9);
end

function [Out_Observation, StateSpaceModelN] = StateSpaceModel3ObservationFunction(In_State, StateSpaceModelN)
    Out_Observation = zeros(StateSpaceModelN.Nz,1);
    Out_Observation(1) = In_State(1);
    Out_Observation(2) = In_State(4);
    Out_Observation(3) = In_State(7);
end

function [Out_PredictedState, StateSpaceModelN] = StateSpaceModel3PredictionFunction(In_State, StateSpaceModelN)
    Out_PredictedState = zeros(StateSpaceModelN.Nx,1);
    
    Out_PredictedState(1) = In_State(1) + StateSpaceModelN.PredictTime * In_State(2) * StateSpaceModelN.PredictTime * StateSpaceModelN.PredictTime * In_State(3) / 2;
    Out_PredictedState(2) = In_State(2) + StateSpaceModelN.PredictTime * In_State(3);
    Out_PredictedState(3) = In_State(3);
    Out_PredictedState(4) = In_State(4) + StateSpaceModelN.PredictTime * In_State(5) * StateSpaceModelN.PredictTime * StateSpaceModelN.PredictTime * In_State(6) / 2;
    Out_PredictedState(5) = In_State(5) + StateSpaceModelN.PredictTime * In_State(6);
    Out_PredictedState(6) = In_State(6);
    Out_PredictedState(7) = In_State(7) + StateSpaceModelN.PredictTime * In_State(8) * StateSpaceModelN.PredictTime * StateSpaceModelN.PredictTime * In_State(9) / 2;
    Out_PredictedState(8) = In_State(8) + StateSpaceModelN.PredictTime * In_State(9);
    Out_PredictedState(9) = In_State(9);
end

function StateSpaceModelN = StateSpaceModel3EstimatorPort(StateSpaceModelN)
    StateSpaceModelN = Estimator3002(StateSpaceModelN);
end

function StateSpaceModel3EstimatorPortTermination(StateSpaceModelN)
    fprintf('EstimatorPort terminated.\n');
end