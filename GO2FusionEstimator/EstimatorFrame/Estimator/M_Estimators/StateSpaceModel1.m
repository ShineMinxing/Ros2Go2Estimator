function StateSpaceModelN = StateSpaceModel1(StateSpaceModelN)
    % 初始化结构体中的变量
    StateSpaceModelN.PortName = '2D Uniform Acceleration Model';
    StateSpaceModelN.PortIntroduction = 'For Reference';

    StateSpaceModelN.Nx = 6;
    StateSpaceModelN.Nz = 2;
    StateSpaceModelN.PredictStep = 3;
    StateSpaceModelN.Intervel = 0.005;
    StateSpaceModelN.PredictTime = StateSpaceModelN.PredictStep * StateSpaceModelN.Intervel;

    StateSpaceModelN.EstimatedState = [1; 0; 0; 1; 0; 0];
    StateSpaceModelN.PredictedState = zeros(StateSpaceModelN.Nx, 1);
    StateSpaceModelN.CurrentObservation = zeros(StateSpaceModelN.Nz, 1);
    StateSpaceModelN.PredictedObservation = zeros(StateSpaceModelN.Nz, 1);
    
    StateSpaceModelN.Matrix_F = eye(StateSpaceModelN.Nx);
    Indices = sub2ind(size(StateSpaceModelN.Matrix_F), [1,2,4,5],[2,3,5,6]);
    StateSpaceModelN.Matrix_F(Indices) = StateSpaceModelN.Intervel;
    Indices = sub2ind(size(StateSpaceModelN.Matrix_F), [1,4],[3,6]);
    StateSpaceModelN.Matrix_F(Indices) = StateSpaceModelN.Intervel^2/2;

    StateSpaceModelN.Matrix_G = eye(StateSpaceModelN.Nx);
    StateSpaceModelN.Matrix_B = [0; 0; 0; 0; 0; 0];

    StateSpaceModelN.Matrix_H = zeros(StateSpaceModelN.Nz, StateSpaceModelN.Nx);
    Indices = sub2ind(size(StateSpaceModelN.Matrix_H), [1,2],[1,4]);
    StateSpaceModelN.Matrix_H(Indices) = 1;

    StateSpaceModelN.Matrix_P = eye(StateSpaceModelN.Nx);
    StateSpaceModelN.Matrix_Q = eye(StateSpaceModelN.Nx);
    StateSpaceModelN.Matrix_R = eye(StateSpaceModelN.Nz);

    StateSpaceModelN.Int_Par = zeros(100,1);
    StateSpaceModelN.Double_Par = zeros(100,1);
    StateSpaceModelN.Matrix_Par = zeros(100,1);

    % 定义结构体中的函数句柄
    StateSpaceModelN.StateTransitionEquation = @(In_State, StateSpaceModelN) StateSpaceModel1StateTransitionFunction(In_State, StateSpaceModelN);
    StateSpaceModelN.ObservationEquation = @(In_State, StateSpaceModelN) StateSpaceModel1ObservationFunction(In_State, StateSpaceModelN);
    StateSpaceModelN.PredictionEquation = @(In_State, StateSpaceModelN) StateSpaceModel1PredictionFunction(In_State, StateSpaceModelN);
    StateSpaceModelN.EstimatorPort = @(StateSpaceModelN) StateSpaceModel1EstimatorPort(StateSpaceModelN);
    StateSpaceModelN.EstimatorPortTermination = @(StateSpaceModelN) StateSpaceModel1EstimatorPortTermination();
end

% 定义各个函数的实现
function [Out_State, StateSpaceModelN] = StateSpaceModel1StateTransitionFunction(In_State, StateSpaceModelN)
    Out_State = StateSpaceModelN.Matrix_F * In_State;
end

function [Out_Observation, StateSpaceModelN] = StateSpaceModel1ObservationFunction(In_State, StateSpaceModelN)
    Out_Observation = StateSpaceModelN.Matrix_H * In_State;
end

function [Out_PredictedState, StateSpaceModelN] = StateSpaceModel1PredictionFunction(In_State, StateSpaceModelN)
    Matrix_F = eye(StateSpaceModelN.Nx);
    Indices = sub2ind(size(Matrix_F), [1,2,4,5],[2,3,5,6]);
    StateSpaceModelN.Matrix_F(Indices) = StateSpaceModelN.PredictTime;
    Indices = sub2ind(size(Matrix_F), [1,4],[3,6]);
    StateSpaceModelN.Matrix_F(Indices) = StateSpaceModelN.PredictTime^2/2;
    Out_PredictedState =  Matrix_F * In_State;
end

function StateSpaceModelN = StateSpaceModel1EstimatorPort(StateSpaceModelN)
    StateSpaceModelN = Estimator3005(StateSpaceModelN);
end

function StateSpaceModel1EstimatorPortTermination(StateSpaceModelN)
    fprintf('EstimatorPort terminated.\n');
end