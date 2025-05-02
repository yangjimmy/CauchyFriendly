% takes the current state estimate and hypothesizes what the measurement should be
function nonlinear_msmt_model(c_duc, c_zbar)
    H = [1.0, 0.0, 0.0]; % meausrement model
    mduc = M_CauchyDynamicsUpdateContainer(c_duc);
    %% Set zbar
    xbar = mduc.cget_x(); % xbar
    zbar = H * xbar; % for other systems, call your nonlinear h(x) function
    mduc.cset_zbar(c_zbar, zbar);
end
