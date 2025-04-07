# Cauchy (Multivariate)
## Functions
``` 
MSlidingWindowManager(system_type, num_windows, swm_debug_print, win_debug_print)
```
Constructor; Initialize the sliding window of the Cauchy filter
`system_type` either "lti" or "nonlin" or "ltv" (linear time varying; not implemented yet)
`num_windows` number of windows
`swm_debug_print` TBD: generic debug print for the sliding window manager
`win_debug_print` TBD: debug print for each window for the sliding window manager


```
initialize_lti(A0, p0, b0, Phi, B, Gamma, beta, H, gamma)
initialize_nonlin(x0, A0, p0, b0, beta, gamma, dynamics_update_callback, nonlinear_msmt_model, extended_msmt_update_callback, cmcc, dt, step, reinit_func)

beta = [0.1]; % Cauchy process noise scaling parameter(s)
gamma = [0.2]; % Cauchy measurement noise scaling parameter(s)
A0 = eye(ndim); % Unit directions of the initial state uncertainty
p0 = [0.10; 0.08; 0.05]; % Initial state uncertainty cauchy scaling parameter(s)
b0 = zeros(ndim,1); % Initial median of system state
```
Initialize an lti system
Note: all matrices need to be discretized by dt
`Phi` n by n
`B` n by m (m controls)
`Gamma` n by 1
`beta` 1 by 1
`H` 1 by n Question: can we have multiple measurements?
`gamma` double scalar; Cauchy noise scaling param

```
step(zk, u, reinit_args)
```
`u` either `[]` for no controls, or double for control at step k
`zk` measurement at time t=k
`reinit_args` TBD

```
shutdown()
```
Call this to free up memory after finishing run

# Cauchy (Single Variable)


# Gaussian