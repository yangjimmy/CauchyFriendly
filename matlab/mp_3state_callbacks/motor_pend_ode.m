function dx_dt = motor_pend_ode(x)
    global mp;
    dx_dt = zeros(3, 1);
    % x1 = theta, x2 = omega, x3 = i
    dx_dt(1) = x(2);
    dx_dt(2) = (-mp.B*x(2)+mp.K_m*x(3)-mp.m*mp.g*mp.l_c*sin(x(1)))/mp.J; % currently u does not depend on x
    dx_dt(3) = (-mp.K_m*x(2)-mp.R*x(3))/mp.L;
end