function [Ts,xs] = nonlin_transition_model(x0, tspan)
    global mp;
    [Ts,xs] = ode15s(@motor_pend_ode, tspan, x0);
end