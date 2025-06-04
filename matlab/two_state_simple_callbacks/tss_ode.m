function dx_dt = tss_ode(x)
    dx_dt = zeros(2, 1);
    % x1 = theta, x2 = omega
    dx_dt(1) = x(2);
    dx_dt(2) = 0; % currently u does not depend on x
end