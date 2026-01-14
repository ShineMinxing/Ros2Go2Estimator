function StateSpaceModelN = Estimator3003(StateSpaceModelN)
% Particle Filter Estimator
% 作者：光电所一室2020级  孙敏行   联系方式：401435318@qq.com

    % Ensure the structure is initialized
    if isempty(StateSpaceModelN)
        error('State space model is not initialized');
    end

    % Cholesky decompositions for sampling noise
    N_Particles = 1000;
    X_Particles = zeros(StateSpaceModelN.Nx, N_Particles);
    Z_Particles = zeros(StateSpaceModelN.Nz, N_Particles);

    % 1. Particle prediction step
    chol_Q = chol(StateSpaceModelN.Matrix_Q, 'lower');
    for N = 1:N_Particles
        % State transition for each particle with process noise
        X_Particles(:,N) = StateSpaceModelN.EstimatedState + chol_Q * randn(StateSpaceModelN.Nx,1);
        [X_Particles(:,N), StateSpaceModelN] = StateSpaceModelN.StateTransitionEquation(X_Particles(:,N), StateSpaceModelN);
    end

    % 2. Compute particle weights based on observation likelihood
    Weights = zeros(N_Particles,1);
    for N = 1:N_Particles
        [Z_Particles(:,N), StateSpaceModelN] = StateSpaceModelN.ObservationEquation(X_Particles(:,N), StateSpaceModelN);
        % Compute innovation
        innovation = StateSpaceModelN.CurrentObservation - Z_Particles(:,N);
        % Compute log-likelihood
        Weights(N) = exp( -0.5 * (innovation' / StateSpaceModelN.Matrix_R * innovation) )  + eps;
    end

    % Subtract maximum log-weight to prevent numerical underflow
    Weights = Weights/sum(Weights);

    % 3. Resample particles if necessary
    Xparticles_New = zeros(StateSpaceModelN.Nx,N_Particles);
    Weight_res = zeros(1,N_Particles);
    Index = 1;
    for N=1:N_Particles
        if(floor(Weights(N)*N_Particles))
            Xparticles_New(:,Index:(Index-1+floor(Weights(N)*N_Particles))) = repmat(X_Particles(:,N),1,floor(Weights(N)*N_Particles));
            Weight_res(N) = rem(Weights(N)*N_Particles,1);
            Index = Index+floor(Weights(N)*N_Particles);
        end
    end
    Weight_res = Weight_res/sum(Weight_res);
    cumsum_Weight_res = cumsum(Weight_res);
    for N=Index:N_Particles
        Select = find(rand <= cumsum_Weight_res, 1);
        Xparticles_New(:, N) = X_Particles(:, Select);  % 从剩余部分中进行多项式重采样
    end
	
    % 4. Estimate the state as the weighted mean of particles
    StateSpaceModelN.EstimatedState = mean(Xparticles_New,2);

    % 5. Predict next state and observation
    [PredictedState, StateSpaceModelN] = StateSpaceModelN.PredictionEquation(StateSpaceModelN.EstimatedState, StateSpaceModelN);
    [PredictedObservation, StateSpaceModelN] = StateSpaceModelN.ObservationEquation(PredictedState, StateSpaceModelN);
    StateSpaceModelN.PredictedState = PredictedState;
    StateSpaceModelN.PredictedObservation = PredictedObservation;
end