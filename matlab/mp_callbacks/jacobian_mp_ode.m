function Jac = jacobian_mp_ode(x)
    global mp;
    Jac = zeros(2);
    Jac(1,2) = 1;
    Jac(2,1) = -mp.m*mp.g*mp.l_c*cos(x(1))/mp.J;
    Jac(2,2) = -(mp.B/mp.J + mp.K_m^2/(mp.J*mp.R));
end