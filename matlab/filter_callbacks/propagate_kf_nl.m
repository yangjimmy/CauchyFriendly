function [xs_kf, Ps_kf] = propagate_kf_nl(x0_kf,P0_kf,zs,propagations,disc_taylor_order)
    global mp
    % initialize
    xs_kf = zeros(num_state,data_length);
    Ps_kf = zeros(data_length, 2, 2);
    W = mp.w_PSD;
    V = mp.v_PSD;
    H = mp.H;
    Gamma_c = mp.Gamma_c;
    dt = mp.dt;
    
    % initial condition
    xs_kf(:,1) = x0_kf;
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
        % x_kf = nonlin_transition_model(x_kf);
        [~,x_kf] = ode45(@nonlin_transition_model_ode45,[0, dt],x_kf);
        x_kf = x_kf(end,:).';
    
        % Form Kalman Gain, update estimate and covariance
        K = (H * P_kf * H' + V) \ (H * P_kf)';
        zbar = H * x_kf;            % estimated measurement
        r = zs(k+1) - zbar;         % residue
        x_kf = x_kf + K * r;        % updated state
        P_kf = (I2 - K * H) * P_kf * (I2 - K * H)' + K * V * K';    % covariance update
    
        % Store estimates
        xs_kf(:,k+1) = x_kf;
        Ps_kf(k, :, :) = P_kf;
    end
    xs_kf = xs_kf.';
end