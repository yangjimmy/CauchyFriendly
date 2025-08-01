#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

#define KALMAN_STATE_DIM 2
#define KALMAN_INPUT_DIM 1
#define KALMAN_OUTPUT_DIM 1

// Function prototypes
void kalman_init(const float* x0, const float* P0);
void kalman_step(
    const float* Phi,    // [2x2]
    const float* Gamma_W,    // [2x1] 
    const float* Gamma_u,    // [2x1]
    const float* H,    // [1x2]
    const float* W,    // [2x2]
    const float* V,    // scalar
    const float* x_prev,
    const float* P_prev,
    float u,           // input
    float meas,      // measurement
    float* x_kf,       // [2x1] output estimate
    float* P_out
);

#endif
