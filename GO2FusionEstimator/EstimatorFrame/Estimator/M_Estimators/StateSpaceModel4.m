function StateSpaceModelN = StateSpaceModel4(StateSpaceModelN)
    % 2D Sine-Driven Model
    % 状态: [x; vx; phix; y; vy; phiy]

    StateSpaceModelN.PortName        = '2D Sine-Driven Model';
    StateSpaceModelN.PortIntroduction= 'x,y driven by sinusoidal acceleration';

    StateSpaceModelN.Nx          = 6;
    StateSpaceModelN.Nz          = 2;
    StateSpaceModelN.PredictStep = 3;
    StateSpaceModelN.Intervel    = 0.005;
    StateSpaceModelN.PredictTime = StateSpaceModelN.PredictStep * StateSpaceModelN.Intervel;

    % 初始状态: [x0; vx0; phix0; y0; vy0; phiy0]
    StateSpaceModelN.EstimatedState       = [1; 0; 0; 1; 0; 0];
    StateSpaceModelN.PredictedState       = zeros(StateSpaceModelN.Nx, 1);
    StateSpaceModelN.CurrentObservation   = zeros(StateSpaceModelN.Nz, 1);
    StateSpaceModelN.PredictedObservation = zeros(StateSpaceModelN.Nz, 1);

    % ===== 线性 F：匀速模型（仅位置-速度） =====
    StateSpaceModelN.Matrix_F = eye(StateSpaceModelN.Nx);
    % x(k+1) = x + vx*StateSpaceModelN.Intervel
    StateSpaceModelN.Matrix_F(1,2) = StateSpaceModelN.Intervel;
    % y(k+1) = y + vy*StateSpaceModelN.Intervel
    StateSpaceModelN.Matrix_F(4,5) = StateSpaceModelN.Intervel;
    % 相位在 F 里保持常值（真正的相位推进在非线性函数里处理）

    StateSpaceModelN.Matrix_G = eye(StateSpaceModelN.Nx);
    StateSpaceModelN.Matrix_B = zeros(StateSpaceModelN.Nx,1);

    % 观测: 只量 x,y
    StateSpaceModelN.Matrix_H = zeros(StateSpaceModelN.Nz, StateSpaceModelN.Nx);
    StateSpaceModelN.Matrix_H(1,1) = 1;   % 观测 x
    StateSpaceModelN.Matrix_H(2,4) = 1;   % 观测 y

    StateSpaceModelN.Matrix_P = eye(StateSpaceModelN.Nx);
    StateSpaceModelN.Matrix_Q = eye(StateSpaceModelN.Nx);
    StateSpaceModelN.Matrix_R = eye(StateSpaceModelN.Nz);

    StateSpaceModelN.Int_Par    = zeros(100,1);
    StateSpaceModelN.Double_Par = zeros(100,1);
    StateSpaceModelN.Matrix_Par = zeros(100,1);

    % 默认正弦参数（可在外部覆盖）
    StateSpaceModelN.Double_Par(1:4) = [2.0; 2.0*pi*1.0; 1.0; 2.0*pi*0.5];

    % 函数句柄
    StateSpaceModelN.StateTransitionEquation = @(In_State, StateSpaceModelN) StateSpaceModel4StateTransitionFunction(In_State, StateSpaceModelN);
    StateSpaceModelN.ObservationEquation = @(In_State, StateSpaceModelN) StateSpaceModel4ObservationFunction(In_State, StateSpaceModelN);
    StateSpaceModelN.PredictionEquation = @(In_State, StateSpaceModelN) StateSpaceModel4PredictionFunction(In_State, StateSpaceModelN);
    StateSpaceModelN.EstimatorPort = @(StateSpaceModelN) StateSpaceModel4EstimatorPort(StateSpaceModelN);
    StateSpaceModelN.EstimatorPortTermination = @(StateSpaceModelN) StateSpaceModel4EstimatorPortTermination();
end

function [Out_State, StateSpaceModelN] = StateSpaceModel4StateTransitionFunction(In_State, StateSpaceModelN)
    % 状态: [x; vx; phix; y; vy; phiy]
    dt = StateSpaceModelN.Intervel;
    Out_State = zeros(StateSpaceModelN.Nx,1);

    x    = In_State(1);
    vx   = In_State(2);
    phix = In_State(3);
    y    = In_State(4);
    vy   = In_State(5);
    phiy = In_State(6);

    % 正弦参数
    Ax = StateSpaceModelN.Double_Par(1);   % x 方向加速度幅值
    wx = StateSpaceModelN.Double_Par(2);   % x 方向角频率
    Ay = StateSpaceModelN.Double_Par(3);   % y 方向加速度幅值
    wy = StateSpaceModelN.Double_Par(4);   % y 方向角频率

    % 正弦加速度
    ax = Ax * sin(phix);
    ay = Ay * sin(phiy);

    % 一步积分（半隐式 Euler，比完全显式稳定一点）
    vx_new = vx + ax * dt;
    x_new  = x  + vx_new * dt;

    vy_new = vy + ay * dt;
    y_new  = y  + vy_new * dt;

    phix_new = phix + wx * dt;
    phiy_new = phiy + wy * dt;

    Out_State(1) = x_new;
    Out_State(2) = vx_new;
    Out_State(3) = phix_new;
    Out_State(4) = y_new;
    Out_State(5) = vy_new;
    Out_State(6) = phiy_new;
end

function [Out_Observation, StateSpaceModelN] = StateSpaceModel4ObservationFunction(In_State, StateSpaceModelN)
    % 观测: z = [x; y]
    Out_Observation = StateSpaceModelN.Matrix_H * In_State;
end
function [Out_PredictedState, StateSpaceModelN] = StateSpaceModel4PredictionFunction(In_State, StateSpaceModelN)
    % 使用同一个正弦非线性模型，前视 PredictStep 步
    Out_PredictedState = In_State;
    if StateSpaceModelN.PredictStep
        for k = 1:StateSpaceModelN.PredictStep
            [Out_PredictedState, StateSpaceModelN] = ...
                StateSpaceModel4StateTransitionFunction(Out_PredictedState, StateSpaceModelN);
        end
    end
end

function StateSpaceModelN = StateSpaceModel4EstimatorPort(StateSpaceModelN)
    StateSpaceModelN = Estimator3002(StateSpaceModelN);
end

function StateSpaceModel4EstimatorPortTermination(StateSpaceModelN)
    fprintf('EstimatorPort terminated.\n');
end
