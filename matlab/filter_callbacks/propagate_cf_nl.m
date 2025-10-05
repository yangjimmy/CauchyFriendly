function cauchyEst = propagate_cf_nl(x0_kf,P0_kf,zs,scale_g2c,propagations,num_controls,sanity_check)
    arguments
        x0_kf;
        P0_kf;
        zs;scale_g2c;
        propagations = size(zs,1);
        num_controls = 0;
        sanity_check = false;
    end

    global mp
    
     % scale factor to fit the cauchy to the gaussian, found by least squares; needs tuning
    num_states = size(x0_kf,2);
    W = mp.w_PSD;
    V = mp.v_PSD;
    dt = mp.dt;

    beta = sqrt(W / dt) * scale_g2c;
    gamma = sqrt(V(1, 1)) * scale_g2c;
    
    beta = mp.beta;
    gamma = mp.gamma;

    x0_ce = x0_kf;
    A0 = eye(num_states);
    p0 = sqrt(diag(P0_kf)) * scale_g2c;
    b0 = zeros(num_states, 1);
    sanity_check_steps = 5;
    % num_controls = 0;
    print_debug = true;
    
    % Uncomment for sanity check and test using MCauchyEstimator
    
    if sanity_check
        cauchyEst = MCauchyEstimator("nonlin", sanity_check_steps, print_debug);
        
        cauchyEst.initialize_nonlin(x0_ce, A0, p0, b0, beta, gamma, 'dynamics_update', 'nonlinear_msmt_model', 'msmt_model_jacobian', num_controls, dt)
        
        for i = 1:sanity_check_steps
            cauchyEst.step(zs(i));
        end
        cauchyEst.shutdown();
    else
        % Sliding window
        swm_print_debug = false; 
        win_print_debug = false;
        num_windows = 2;
        % 
        cauchyEst = MSlidingWindowManager("nonlin", num_windows, swm_print_debug, win_print_debug);
        cauchyEst.initialize_nonlin(x0_ce, A0, p0, b0, beta, gamma, 'dynamics_update', 'nonlinear_msmt_model', 'msmt_model_jacobian', num_controls, dt);
        % charACTERIZE the rate
        tic;
        for k = 1:propagations
            zk = zs(k);
            [xhat, Phat, wavg_xhat, wavg_Phat] = cauchyEst.step(zk, []);
        end
        toc;
        cauchyEst.shutdown();
    end
end