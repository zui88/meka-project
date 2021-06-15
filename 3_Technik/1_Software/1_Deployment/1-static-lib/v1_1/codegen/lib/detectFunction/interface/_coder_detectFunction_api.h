/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: _coder_detectFunction_api.h
 *
 * GPU Coder version                    : 2.0
 * CUDA/C/C++ source code generated on  : 12-Mar-2021 17:27:33
 */

#ifndef _CODER_DETECTFUNCTION_API_H
#define _CODER_DETECTFUNCTION_API_H

/* Include Files */
#include "emlrt.h"
#include "tmwtypes.h"
#include <string.h>

/* Variable Declarations */
extern emlrtCTX emlrtRootTLSGlobal;
extern emlrtContext emlrtContextGlobal;

#ifdef __cplusplus

extern "C" {

#endif

  /* Function Declarations */
  void detectFunction(real_T *x_, real_T *y_, real_T *width_, real_T *height_,
                      real32_T *score_);
  void detectFunction_api(int32_T nlhs, const mxArray *plhs[5]);
  void detectFunction_atexit(void);
  void detectFunction_initialize(void);
  void detectFunction_terminate(void);
  void detectFunction_xil_shutdown(void);
  void detectFunction_xil_terminate(void);

#ifdef __cplusplus

}
#endif
#endif

/*
 * File trailer for _coder_detectFunction_api.h
 *
 * [EOF]
 */
