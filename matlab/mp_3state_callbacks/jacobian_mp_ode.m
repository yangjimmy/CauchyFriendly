function Jac = jacobian_mp_ode(x)
    global mp;
    Jac = zeros(3);
    Jac(1,2) = 1;
    Jac(2,1) = -mp.m*mp.g*mp.l_c*cos(x(1))/mp.J;
    Jac(2,2) = -mp.B/mp.J;
    Jac(2,3) = mp.K_m/mp.J;
    Jac(3,2) = -mp.K_m/mp.L;
    Jac(3,3) = -mp.R/mp.L;
end