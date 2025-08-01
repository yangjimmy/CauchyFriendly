#include "customcode_PuaueSri2jojBvU9Mpy1EC.h"
#ifdef __cplusplus
extern "C" {
#endif


/* Type Definitions */

/* Named Constants */

/* Variable Declarations */

/* Variable Definitions */

/* Function Declarations */
DLL_EXPORT_CC extern const char_T *get_dll_checksum_PuaueSri2jojBvU9Mpy1EC(void);
DLL_EXPORT_CC extern void kalman_init_PuaueSri2jojBvU9Mpy1EC(const real32_T *x0, const real32_T *P0);
DLL_EXPORT_CC extern void kalman_step_PuaueSri2jojBvU9Mpy1EC(const real32_T *Phi, const real32_T *Gamma_W, const real32_T *Gamma_u, const real32_T *H, const real32_T *W, const real32_T *V, const real32_T *x_prev, const real32_T *P_prev, real32_T u, real32_T meas, real32_T *x_kf, real32_T *P_out);

/* Function Definitions */
DLL_EXPORT_CC const uint8_T *get_checksum_source_info(int32_T *size);
#ifdef __cplusplus
}
#endif

