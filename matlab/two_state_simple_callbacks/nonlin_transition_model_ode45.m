function dx_dt = nonlin_transition_model_ode45(t, x)
    % extra argument t to align with ode45 convention
    dx_dt = zeros(2, 1);
    % x1 = theta, x2 = omega
    dx_dt(1) = x(2);
    dx_dt(2) = 0; % currently u does not depend on x
end