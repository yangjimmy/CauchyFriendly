function x_new = nonlin_transition_model(x,u)
    global mp;
    x_new = runge_kutta4(@(x,u)motor_pend_ode(x,u), x, u, mp.dt);
end