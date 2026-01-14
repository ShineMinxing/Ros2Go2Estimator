function StateSpaceModelN = StateSpaceModel2(StateSpaceModelN)
    % 2D Quadratic Drag Motion Model with Acceleration States
    % 状态: [x; vx; ax; y; vy; ay]
    % Double_Par(1:4) = [m, c_d, a_ext_x, a_ext_y]

    StateSpaceModelN.PortName        = '2D Quadratic Drag Motion Model (with accel states)';
    StateSpaceModelN.PortIntroduction= 'For Reference';
    
    StateSpaceModelN.Nx          = 6;
    StateSpaceModelN.Nz          = 2;
    StateSpaceModelN.PredictStep = 3;
    StateSpaceModelN.Intervel    = 0.005;
    StateSpaceModelN.PredictTime = StateSpaceModelN.PredictStep * StateSpaceModelN.Intervel;

    % 初始状态: [x0; vx0; ax0; y0; vy0; ay0]
    StateSpaceModelN.EstimatedState       = [1; 10; 0; 1; -10; 0];
    StateSpaceModelN.PredictedState       = zeros(StateSpaceModelN.Nx, 1);
    StateSpaceModelN.CurrentObservation   = zeros(StateSpaceModelN.Nz, 1);
    StateSpaceModelN.PredictedObservation = zeros(StateSpaceModelN.Nz, 1);

    % ===== 线性 F：匀加速模型的常矩阵（供线性滤波器用） =====
    dt = StateSpaceModelN.Intervel;
    StateSpaceModelN.Matrix_F = eye(StateSpaceModelN.Nx);
    % x(k+1) = x + vx*dt + 0.5*ax*dt^2
    StateSpaceModelN.Matrix_F(1,2) = dt;
    StateSpaceModelN.Matrix_F(1,3) = 0.5 * dt^2;
    % vx(k+1) = vx + ax*dt
    StateSpaceModelN.Matrix_F(2,3) = dt;
    % y(k+1) = y + vy*dt + 0.5*ay*dt^2
    StateSpaceModelN.Matrix_F(4,5) = dt;
    StateSpaceModelN.Matrix_F(4,6) = 0.5 * dt^2;
    % vy(k+1) = vy + ay*dt
    StateSpaceModelN.Matrix_F(5,6) = dt;
    % ax, ay 在该线性模型中视为常值: ax(k+1)=ax, ay(k+1)=ay

    StateSpaceModelN.Matrix_G = eye(StateSpaceModelN.Nx);
    StateSpaceModelN.Matrix_B = zeros(StateSpaceModelN.Nx,1);

    % 观测: z = [x; y]
    StateSpaceModelN.Matrix_H = zeros(StateSpaceModelN.Nz, StateSpaceModelN.Nx);
    StateSpaceModelN.Matrix_H(1,1) = 1;   % x
    StateSpaceModelN.Matrix_H(2,4) = 1;   % y

    StateSpaceModelN.Matrix_P = eye(StateSpaceModelN.Nx);
    StateSpaceModelN.Matrix_Q = eye(StateSpaceModelN.Nx);
    StateSpaceModelN.Matrix_R = eye(StateSpaceModelN.Nz);

    StateSpaceModelN.Int_Par    = zeros(100,1);
    StateSpaceModelN.Double_Par = zeros(100,1);
    % Double_Par: [质量, 阻力系数, x 方向外部加速度, y 方向外部加速度]
    StateSpaceModelN.Double_Par(1:4) = [100.0; 0.1; 0.0; 0.0];
    StateSpaceModelN.Matrix_Par = zeros(100,1);

    % 函数句柄
    StateSpaceModelN.StateTransitionEquation     = ...
        @(In_State, StateSpaceModelN) StateSpaceModel2StateTransitionFunction(In_State, StateSpaceModelN);
    StateSpaceModelN.StateTransitionDiffEquation = ...
        @(In_State, StateSpaceModelN) StateSpaceModel2StateTransitionDiffFunction(In_State, StateSpaceModelN);
    StateSpaceModelN.ObservationEquation         = ...
        @(In_State, StateSpaceModelN) StateSpaceModel2ObservationFunction(In_State, StateSpaceModelN);
    StateSpaceModelN.ObservationDiffEquation     = ...
        @(In_State, StateSpaceModelN) StateSpaceModel2ObservationDiffFunction(In_State, StateSpaceModelN);
    StateSpaceModelN.PredictionEquation          = ...
        @(In_State, StateSpaceModelN) StateSpaceModel2PredictionFunction(In_State, StateSpaceModelN);
    StateSpaceModelN.EstimatorPort               = ...
        @(StateSpaceModelN) StateSpaceModel2EstimatorPort(StateSpaceModelN);
    StateSpaceModelN.EstimatorPortTermination    = ...
        @(StateSpaceModelN) StateSpaceModel2EstimatorPortTermination(StateSpaceModelN);
end


% ============ 非线性状态转移（考虑阻力 + 显式加速度） ============
function [Out_State, StateSpaceModelN] = StateSpaceModel2StateTransitionFunction(In_State, StateSpaceModelN)
    % 状态定义: x = [x; vx; ax; y; vy; ay]
    Out_State = zeros(StateSpaceModelN.Nx,1);
    dt = StateSpaceModelN.Intervel;

    x   = In_State(1);
    vx  = In_State(2);
    % ax_old = In_State(3);
    y   = In_State(4);
    vy  = In_State(5);
    % ay_old = In_State(6);

    % 参数
    m   = StateSpaceModelN.Double_Par(1);   % 质量
    cd  = StateSpaceModelN.Double_Par(2);   % 二次阻力系数
    ax_ext = StateSpaceModelN.Double_Par(3);% x 外部加速度
    ay_ext = StateSpaceModelN.Double_Par(4);% y 外部加速度

    k = cd / max(m, eps);                  % 阻力系数/m
    v = sqrt(vx*vx + vy*vy + 1e-12);       % 速度模长

    % 二次阻力加速度
    ax = ax_ext - k * v * vx;
    ay = ay_ext - k * v * vy;

    % 一步离散积分（匀加速近似）
    vx_new = vx + ax * dt;
    x_new  = x  + vx * dt + 0.5 * ax * dt^2;

    vy_new = vy + ay * dt;
    y_new  = y  + vy * dt + 0.5 * ay * dt^2;

    Out_State(1) = x_new;
    Out_State(2) = vx_new;
    Out_State(3) = ax;      % 显式保存当前步的加速度
    Out_State(4) = y_new;
    Out_State(5) = vy_new;
    Out_State(6) = ay;
end


% ============ 状态转移雅可比矩阵 F = ∂f/∂x（在 In_State 处） ============
function [Out_State, StateSpaceModelN] = StateSpaceModel2StateTransitionDiffFunction(In_State, StateSpaceModelN)
    % 输出为 6x6 雅可比矩阵
    dt = StateSpaceModelN.Intervel;

    vx  = In_State(2);
    % ax_old = In_State(3);
    vy  = In_State(5);
    % ay_old = In_State(6);

    m   = StateSpaceModelN.Double_Par(1);
    cd  = StateSpaceModelN.Double_Par(2);
    k   = cd / max(m, eps);

    v   = sqrt(vx*vx + vy*vy + 1e-12);

    vx2 = vx*vx;
    vy2 = vy*vy;

    % ∂ax/∂vx, ∂ax/∂vy
    dax_dvx = -k * ( (2*vx2 + vy2 + 1e-12) / v );
    dax_dvy = -k * ( (vx*vy) / v );

    % ∂ay/∂vx, ∂ay/∂vy
    day_dvx = -k * ( (vx*vy) / v );
    day_dvy = -k * ( (vx2 + 2*vy2 + 1e-12) / v );

    F = zeros(StateSpaceModelN.Nx);

    % f1 = x + vx*dt + 0.5*ax*dt^2, ax = ax(vx,vy)
    F(1,1) = 1;
    F(1,2) = dt + 0.5 * dt^2 * dax_dvx;
    F(1,3) = 0;
    F(1,4) = 0;
    F(1,5) = 0.5 * dt^2 * dax_dvy;
    F(1,6) = 0;

    % f2 = vx + ax*dt
    F(2,1) = 0;
    F(2,2) = 1 + dt * dax_dvx;
    F(2,3) = 0;
    F(2,4) = 0;
    F(2,5) = dt * dax_dvy;
    F(2,6) = 0;

    % f3 = ax
    F(3,1) = 0;
    F(3,2) = dax_dvx;
    F(3,3) = 0;
    F(3,4) = 0;
    F(3,5) = dax_dvy;
    F(3,6) = 0;

    % f4 = y + vy*dt + 0.5*ay*dt^2
    F(4,1) = 0;
    F(4,2) = 0.5 * dt^2 * day_dvx;
    F(4,3) = 0;
    F(4,4) = 1;
    F(4,5) = dt + 0.5 * dt^2 * day_dvy;
    F(4,6) = 0;

    % f5 = vy + ay*dt
    F(5,1) = 0;
    F(5,2) = dt * day_dvx;
    F(5,3) = 0;
    F(5,4) = 0;
    F(5,5) = 1 + dt * day_dvy;
    F(5,6) = 0;

    % f6 = ay
    F(6,1) = 0;
    F(6,2) = day_dvx;
    F(6,3) = 0;
    F(6,4) = 0;
    F(6,5) = day_dvy;
    F(6,6) = 0;

    Out_State = F;
end

function [Out_Observation, StateSpaceModelN] = StateSpaceModel2ObservationFunction(In_State, StateSpaceModelN)
    Out_Observation = StateSpaceModelN.Matrix_H * In_State;
end

function [Out_Observation, StateSpaceModelN] = StateSpaceModel2ObservationDiffFunction(In_State, StateSpaceModelN)
    % 观测只量 x,y -> 对应状态 1 和 4
    Out_Observation = zeros(StateSpaceModelN.Nz, StateSpaceModelN.Nx);
    Out_Observation(1,1) = 1;
    Out_Observation(2,4) = 1;
end

function [Out_PredictedState, StateSpaceModelN] = StateSpaceModel2PredictionFunction(In_State, StateSpaceModelN)
    Out_PredictedState = In_State;
    if StateSpaceModelN.PredictStep
        for k = 1:StateSpaceModelN.PredictStep
            [Out_PredictedState, StateSpaceModelN] = ...
                StateSpaceModel2StateTransitionFunction(Out_PredictedState, StateSpaceModelN);
        end
    end
end

function StateSpaceModelN = StateSpaceModel2EstimatorPort(StateSpaceModelN)
    StateSpaceModelN = Estimator3009(StateSpaceModelN);
end

function StateSpaceModel2EstimatorPortTermination(StateSpaceModelN)
    fprintf('EstimatorPort terminated.\n');
end
