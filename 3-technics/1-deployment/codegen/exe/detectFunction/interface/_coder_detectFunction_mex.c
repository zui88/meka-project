/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: _coder_detectFunction_mex.c
 *
 * GPU Coder version                    : 2.0
 * CUDA/C/C++ source code generated on  : 08-Mar-2021 19:56:17
 */

/* Include Files */
#include "_coder_detectFunction_mex.h"
#include "_coder_detectFunction_api.h"

/* Function Definitions */
/*
 * Arguments    : int32_T nlhs
 *                int32_T nrhs
 * Return Type  : void
 */
void detectFunction_mexFunction(int32_T nlhs, int32_T nrhs)
{
  emlrtStack st = { NULL,              /* site */
    NULL,                              /* tls */
    NULL                               /* prev */
  };

  st.tls = emlrtRootTLSGlobal;

  /* Check for proper number of arguments. */
  if (nrhs != 0) {
    emlrtErrMsgIdAndTxt(&st, "EMLRT:runTime:WrongNumberOfInputs", 5, 12, 0, 4,
                        14, "detectFunction");
  }

  if (nlhs > 0) {
    emlrtErrMsgIdAndTxt(&st, "EMLRT:runTime:TooManyOutputArguments", 3, 4, 14,
                        "detectFunction");
  }

  /* Call the function. */
  detectFunction_api();
}

/*
 * Arguments    : int32_T nlhs
 *                mxArray *plhs[]
 *                int32_T nrhs
 *                const mxArray *prhs[]
 * Return Type  : void
 */
void mexFunction(int32_T nlhs, mxArray *plhs[], int32_T nrhs, const mxArray
                 *prhs[])
{
  (void)plhs;
  (void)prhs;
  mexAtExit(&detectFunction_atexit);

  /* Module initialization. */
  detectFunction_initialize();

  /* Dispatch the entry-point. */
  detectFunction_mexFunction(nlhs, nrhs);

  /* Module termination. */
  detectFunction_terminate();
}

/*
 * Arguments    : void
 * Return Type  : emlrtCTX
 */
emlrtCTX mexFunctionCreateRootTLS(void)
{
  emlrtCreateRootTLS(&emlrtRootTLSGlobal, &emlrtContextGlobal, NULL, 1);
  return emlrtRootTLSGlobal;
}

/*
 * File trailer for _coder_detectFunction_mex.c
 *
 * [EOF]
 */
