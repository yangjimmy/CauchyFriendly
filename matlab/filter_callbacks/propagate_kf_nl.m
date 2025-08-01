function [xs_kf, Ps_kf] = propagate_kf_nl(x0_kf,P0_kf,zs,propagations, disc_taylor_order)
    arguments
        x0_kf;
        P0_kf;
        zs;
        propagations = size(zs,1);
        disc_taylor_order = 2;
    end

    global mp
    % initialize
    data_length = propagations + 1;
    num_states = size(x0_kf,2);
    I = eye(num_states);
    xs_kf = zeros(num_states,data_length);
    Ps_kf = zeros(data_length, 2, 2);
    W = mp.w_PSD;
    V = mp.v_PSD;
    H = mp.H;
    Gamma_c = mp.Gamma_c;
    dt = mp.dt;
    
    % initial condition
    xs_kf(:,1) = x0_kf';
    Ps_kf(1, :, :) = P0_kf;
    
    % propagation vectors
    x_kf = x0_kf';
    P_kf = P0_kf;
    
    for k = 1:propagations
        % characterize nonlinear dynamics
        Jac_F = jacobian_mp_ode(x_kf);
        [Phi_k, W_k] = discretize_nl_sys(Jac_F, Gamma_c, W, dt, disc_taylor_order, false, true);
    
        % Propagate covariance and state estimates
        P_kf = Phi_k * P_kf * Phi_k' + W_k;
        x_kf = nonlin_transition_model(x_kf);
        % [~,x_kf] = ode45(@nonlin_transition_model_ode45,[0, dt],x_kf);
        % x_kf = x_kf(end,:).';
    
        % Form Kalman Gain, update estimate and covariance
        K = (H * P_kf * H' + V) \ (H * P_kf)';
        zbar = H * x_kf;            % estimated measurement
        r = zs(k+1) - zbar;         % residue
        x_kf = x_kf + K * r;        % updated state
        P_kf = (I - K * H) * P_kf * (I - K * H)' + K * V * K';    % covariance update
    
        % Store estimates
        xs_kf(:,k+1) = x_kf;
        Ps_kf(k, :, :) = P_kf;
    end
    xs_kf = xs_kf.';
end