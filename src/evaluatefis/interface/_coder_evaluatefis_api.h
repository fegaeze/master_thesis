/*
 * File: _coder_evaluatefis_api.h
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 21-Dec-2023 10:11:11
 */

#ifndef _CODER_EVALUATEFIS_API_H
#define _CODER_EVALUATEFIS_API_H

/* Include Files */
#include "emlrt.h"
#include "mex.h"
#include "tmwtypes.h"
#include <string.h>

/* Variable Declarations */
extern emlrtCTX emlrtRootTLSGlobal;
extern emlrtContext emlrtContextGlobal;

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
real_T evaluatefis(real_T x);

void evaluatefis_api(const mxArray *prhs, const mxArray **plhs);

void evaluatefis_atexit(void);

void evaluatefis_initialize(void);

void evaluatefis_terminate(void);

void evaluatefis_xil_shutdown(void);

void evaluatefis_xil_terminate(void);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for _coder_evaluatefis_api.h
 *
 * [EOF]
 */