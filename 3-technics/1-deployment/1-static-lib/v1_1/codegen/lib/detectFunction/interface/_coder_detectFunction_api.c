/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: _coder_detectFunction_api.c
 *
 * GPU Coder version                    : 2.0
 * CUDA/C/C++ source code generated on  : 12-Mar-2021 17:27:33
 */

/* Include Files */
#include "_coder_detectFunction_api.h"
#include "_coder_detectFunction_mex.h"

/* Variable Definitions */
emlrtCTX emlrtRootTLSGlobal = NULL;
emlrtContext emlrtContextGlobal = { true,/* bFirstTime */
  false,                               /* bInitialized */
  131595U,                             /* fVersionInfo */
  NULL,                                /* fErrorFunction */
  "detectFunction",                    /* fFunctionName */
  NULL,                                /* fRTCallStack */
  false,                               /* bDebugMode */
  { 2045744189U, 2170104910U, 2743257031U, 4284093946U },/* fSigWrd */
  NULL                                 /* fSigMem */
};

/* Function Declarations */
static const mxArray *b_emlrt_marshallOut(const real32_T u);
static const mxArray *emlrt_marshallOut(const real_T u);

/* Function Definitions */
/*
 * Arguments    : const real32_T u
 * Return Type  : const mxArray *
 */
static const mxArray *b_emlrt_marshallOut(const real32_T u)
{
  const mxArray *m;
  const mxArray *y;
  y = NULL;
  m = emlrtCreateNumericMatrix(1, 1, mxSINGLE_CLASS, mxREAL);
  *(real32_T *)emlrtMxGetData(m) = u;
  emlrtAssign(&y, m);
  return y;
}

/*
 * Arguments    : const real_T u
 * Return Type  : const mxArray *
 */
static const mxArray *emlrt_marshallOut(const real_T u)
{
  const mxArray *m;
  const mxArray *y;
  y = NULL;
  m = emlrtCreateDoubleScalar(u);
  emlrtAssign(&y, m);
  return y;
}

/*
 * Arguments    : int32_T nlhs
 *                const mxArray *plhs[5]
 * Return Type  : void
 */
void detectFunction_api(int32_T nlhs, const mxArray *plhs[5])
{
  real_T height_;
  real_T width_;
  real_T x_;
  real_T y_;
  real32_T score_;

  /* Invoke the target function */
  detectFunction(&x_, &y_, &width_, &height_, &score_);

  /* Marshall function outputs */
  plhs[0] = emlrt_marshallOut(x_);
  if (nlhs > 1) {
    plhs[1] = emlrt_marshallOut(y_);
  }

  if (nlhs > 2) {
    plhs[2] = emlrt_marshallOut(width_);
  }

  if (nlhs > 3) {
    plhs[3] = emlrt_marshallOut(height_);
  }

  if (nlhs > 4) {
    plhs[4] = b_emlrt_marshallOut(score_);
  }
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void detectFunction_atexit(void)
{
  emlrtStack st = { NULL,              /* site */
    NULL,                              /* tls */
    NULL                               /* prev */
  };

  mexFunctionCreateRootTLS();
  st.tls = emlrtRootTLSGlobal;
  emlrtEnterRtStackR2012b(&st);
  emlrtLeaveRtStackR2012b(&st);
  emlrtDestroyRootTLS(&emlrtRootTLSGlobal);
  detectFunction_xil_terminate();
  detectFunction_xil_shutdown();
  emlrtExitTimeCleanup(&emlrtContextGlobal);
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void detectFunction_initialize(void)
{
  emlrtStack st = { NULL,              /* site */
    NULL,                              /* tls */
    NULL                               /* prev */
  };

  mexFunctionCreateRootTLS();
  st.tls = emlrtRootTLSGlobal;
  emlrtClearAllocCountR2012b(&st, false, 0U, 0);
  emlrtEnterRtStackR2012b(&st);
  emlrtFirstTimeR2012b(emlrtRootTLSGlobal);
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void detectFunction_terminate(void)
{
  emlrtStack st = { NULL,              /* site */
    NULL,                              /* tls */
    NULL                               /* prev */
  };

  st.tls = emlrtRootTLSGlobal;
  emlrtLeaveRtStackR2012b(&st);
  emlrtDestroyRootTLS(&emlrtRootTLSGlobal);
}

/*
 * File trailer for _coder_detectFunction_api.c
 *
 * [EOF]
 */
