//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: detectFunction_terminate.cu
//
// GPU Coder version                    : 2.0
// CUDA/C/C++ source code generated on  : 12-Mar-2021 11:18:37
//

// Include Files
#include "detectFunction_terminate.h"
#include "detectFunction.h"
#include "detectFunction_data.h"
#include "rt_nonfinite.h"

// Custom Source Code
#include "main.h"

// Function Definitions
//
// Arguments    : void
// Return Type  : void
//
void detectFunction_terminate()
{
  detectFunction_free();
  isInitialized_detectFunction = false;
}

//
// File trailer for detectFunction_terminate.cu
//
// [EOF]
//
