% takes the current state estimate and forms H_k
function msmt_model_jacobian(c_duc)
    H = [1.0, 0.0, 0.0];
    mduc = M_CauchyDynamicsUpdateContainer(c_duc);
    % Set H: for other systems, call your nonlinear jacobian function H(x)
    mduc.cset_H(H); % we could write some if condition to only set this once, but its such a trivial overhead, who cares
end