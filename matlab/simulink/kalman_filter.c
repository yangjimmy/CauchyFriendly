#include "kalman_filter.h"

// Internal state
static float x[KALMAN_STATE_DIM];            // Estimated state
static float P[KALMAN_STATE_DIM][KALMAN_STATE_DIM];  // Error covariance

void kalman_init(const float* x0, const float* P0)
{
    for (int i = 0; i < KALMAN_STATE_DIM; ++i) {
        x[i] = x0[i];
        for (int j = 0; j < KALMAN_STATE_DIM; ++j) {
            P[i][j] = P0[i * KALMAN_STATE_DIM + j];
        }
    }
}

// Dynamics
// dx = Phi * x * dt + Gamma_W * dw_k
// dz = H * x * dt + dv_k

// Propagation
// x_bar_k+1 = Phi_k *x_hat_k
// M_k+1 = (Phi_k * P_k * Phi_k^T) + W
// K_k = (H * M_k * H^T + V) \ (H * M_k)
// P_k = (I - K_k*H) * M_k * (I - K_k*H)^T + K_k * V * K_k^T
// x_hat_k = x_bar_k + K_k*(z - H * x_bar_k)


void kalman_step(
    const float* Phi, const float* Gamma_W, const float* Gamma_u,
    const float* H, const float* W, const float* V,
    const float* x_prev, const float* P_prev,
    float u, float meas,
    float* x_kf, float* P_out)
{
    float x_bar[KALMAN_STATE_DIM] = {0.0f};
    float M[KALMAN_STATE_DIM * KALMAN_STATE_DIM] = {0.0f};
    float HM[KALMAN_STATE_DIM] = {0.0f};
    float HMHT = 0.0f;
    float K[KALMAN_STATE_DIM] = {0.0f};
    float I_KH[KALMAN_STATE_DIM * KALMAN_STATE_DIM] = {0.0f};
    float temp[KALMAN_STATE_DIM * KALMAN_STATE_DIM] = {0.0f};

    // Predict x_bar = Phi * x_prev + Gamma_u * u
    // for (int i = 0; i < KALMAN_STATE_DIM; ++i) {
    //     x_bar[i] = 0.0f;
    //     for (int j = 0; j < KALMAN_STATE_DIM; ++j) {
    //         x_bar[i] += Phi[i * KALMAN_STATE_DIM + j] * x_prev[j];
    //     }
    //     x_bar[i] += Gamma_u[i] * u;
    // }
    for (int i = 0; i < KALMAN_STATE_DIM; ++i){
        x_bar[i] = x_prev[i];
    }

    // Predict M = Phi * P_prev * Phi^T + Gamma_W * W * Gamma_W^T
    for (int i = 0; i < KALMAN_STATE_DIM; ++i) {
        for (int j = 0; j < KALMAN_STATE_DIM; ++j) {
            M[i * KALMAN_STATE_DIM + j] = 0.0f;
            for (int k = 0; k < KALMAN_STATE_DIM; ++k) {
                for (int l = 0; l < KALMAN_STATE_DIM; ++l) {
                    M[i * KALMAN_STATE_DIM + j] += Phi[i * KALMAN_STATE_DIM + k] *
                        P_prev[k * KALMAN_STATE_DIM + l] * Phi[j * KALMAN_STATE_DIM + l];
                }
            }
            // M[i * KALMAN_STATE_DIM + j] += Gamma_W[i] * W[0] * Gamma_W[j];
            M[i * KALMAN_STATE_DIM + j] += W[i * KALMAN_STATE_DIM + j];
        }
    }

    // Compute HM = H * M
    for (int i = 0; i < KALMAN_STATE_DIM; ++i) {
        HM[i] = 0.0f;
        for (int j = 0; j < KALMAN_STATE_DIM; ++j) {
            HM[i] += H[j] * M[j * KALMAN_STATE_DIM + i];
        }
    }

    // Compute HMHT = HM * H^T + V
    HMHT = 0.0f;
    for (int i = 0; i < KALMAN_STATE_DIM; ++i) {
        HMHT += HM[i] * H[i];
    }
    HMHT += *V;

    // Compute Kalman gain K = (H*M)^T / HMHT
    for (int i = 0; i < KALMAN_STATE_DIM; ++i) {
        K[i] = HM[i] / HMHT;
    }

    // Compute x_kf = x_bar + K * (meas - H * x_bar)
    float residual = meas;
    for (int i = 0; i < KALMAN_STATE_DIM; ++i) {
        residual -= H[i] * x_bar[i];
    }
    for (int i = 0; i < KALMAN_STATE_DIM; ++i) {
        x_kf[i] = x_bar[i] + K[i] * residual;
    }

    // Compute I - K*H
    for (int i = 0; i < KALMAN_STATE_DIM; ++i) {
        for (int j = 0; j < KALMAN_STATE_DIM; ++j) {
            I_KH[i * KALMAN_STATE_DIM + j] = (i == j ? 1.0f : 0.0f) - K[i] * H[j];
        }
    }

    // temp = (I - K*H) * M
    for (int i = 0; i < KALMAN_STATE_DIM; ++i) {
        for (int j = 0; j < KALMAN_STATE_DIM; ++j) {
            temp[i * KALMAN_STATE_DIM + j] = 0.0f;
            for (int k = 0; k < KALMAN_STATE_DIM; ++k) {
                temp[i * KALMAN_STATE_DIM + j] += I_KH[i * KALMAN_STATE_DIM + k] * M[k * KALMAN_STATE_DIM + j];
            }
        }
    }

    // P_out = temp * (I - K*H)^T + K*V*K^T
    for (int i = 0; i < KALMAN_STATE_DIM; ++i) {
        for (int j = 0; j < KALMAN_STATE_DIM; ++j) {
            P_out[i * KALMAN_STATE_DIM + j] = 0.0f;
            for (int k = 0; k < KALMAN_STATE_DIM; ++k) {
                P_out[i * KALMAN_STATE_DIM + j] += temp[i * KALMAN_STATE_DIM + k] * I_KH[j * KALMAN_STATE_DIM + k];
            }
            P_out[i * KALMAN_STATE_DIM + j] += K[i] * (*V) * K[j];
        }
    }
}
