% File: simulate_mixed_ltiv_system.m

function [xs, zs, ws, vs] = simulate_mixed_ltiv_system(num_steps, x0_truth, us, Phi, B, Gamma, W, H, V, dt, with_zeroth_step_msmt, dynamics_update_callback, other_params)
    if nargin < 11
        with_zeroth_step_msmt = true;
    end
    if nargin < 12
        dynamics_update_callback = [];
    end
    
    if ~isempty(B)
        assert(~isempty(us));
        assert(size(us,1) == num_steps);
        if numel(B) == numel(x0_truth)
            us = reshape(us, num_steps, 1);
        else
            assert(size(us,2) == size(B,2));
        end
    else
        assert(isempty(us));
        B = zeros(length(x0_truth), 1);
        us = zeros(num_steps, 1);
    end
    
    if isscalar(W)
        W = reshape(W, 1, 1);
    else
        assert(size(W,1) == size(W,2));
    end
    
    if isscalar(V)
        V = reshape(V, 1, 1);
    else
        assert(size(V,1) == size(V,2));
    end
    
    assert(~isempty(Gamma));
    assert(~isempty(W));
    Gamma = reshape(Gamma, numel(x0_truth), size(W,1));
    v_zero_vec = zeros(size(V,1), 1);
    w_zero_vec = zeros(size(W,1), 1);
    xk = x0_truth;
    
    xs = xk'; % Initialize with the first state
    zs = []; % Initialize measurement history
    ws = []; % Initialize process noise history
    vs = []; % Initialize measurement noise history
    thetas = [];
    
    if with_zeroth_step_msmt
        v0 = unifrnd(-pi/200,pi/200)';
        theta0 = H * xk + v0;
        thetas = theta0; % Add initial measurement
        vs = v0; % Add initial measurement noise
    end
    
    for i = 1:num_steps
        if ~isempty(dynamics_update_callback)
            dynamics_update_callback(Phi, B, Gamma, H, W, V, i - 1, other_params);
        end
        uk = us(i,:)';
        wk = mvnrnd(w_zero_vec, W, 1)';
        xk = Phi * xk + B * uk + Gamma * wk;
        
        xs = [xs; xk']; % Concatenate to states matrix
        ws = [ws wk]; % Concatenate to process noise matrix
        
        vk = unifrnd(-pi/200,pi/200)'; % sample from uniform distribution, v is in theta, but need to calculate omega for zk
        thetak = H * xk + vk;
        zk = (thetak-thetas(i))/dt; % backward difference for now; will convert to not use derivative at a later date
        
        thetas = [thetas; thetak]; % Concatenate to theta matrix
        zs = [zs; zk]; % Concatenate to measurement matrix
        vs = [vs vk]; % Concatenate to measurement noise matrix
    end
    ws = ws';
    vs = vs';
end