function dx_dt = motor_pend_ode(x,u)
    global mp;
    dx_dt = zeros(2, 1);
    % x1 = theta, x2 = omega
    % currently u does not depend on x
    dx_dt(1) = x(2);
    % dx_dt(2) = -(mp.B/mp.J)*x(2)-mp.m*mp.g*mp.l_c*sin(x(1))/mp.J;
    dx_dt(2) = -(mp.B/mp.J + mp.K_m^2/(mp.J*mp.R))*x(2)-mp.m*mp.g*mp.l_c*sin(x(1))/mp.J;
    dx_dt(2) = dx_dt(2) + mp.K_m * mp.V_s / mp.R / mp.J * u;
    
end