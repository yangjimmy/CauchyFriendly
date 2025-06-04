function x_new = nonlin_transition_model(x)
    global mp;
    x_new = runge_kutta4(@(x)tss_ode(x), x, mp.dt);
end