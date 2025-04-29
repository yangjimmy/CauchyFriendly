function x_new = nonlin_transition_model_2(x)
    global mp;
    x_new = runge_kutta4(@(x)motor_pend_ode(x), x, mp.dt);
end