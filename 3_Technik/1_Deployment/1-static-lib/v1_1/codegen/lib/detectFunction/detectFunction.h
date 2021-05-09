//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: detectFunction.h
//
// GPU Coder version                    : 2.0
// CUDA/C/C++ source code generated on  : 12-Mar-2021 17:27:33
//
#ifndef DETECTFUNCTION_H
#define DETECTFUNCTION_H

// Include Files
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

// Function Declarations
extern void detectFunction(double *x_, double *y_, double *width_, double
  *height_, float *score_);
void detectFunction_free();
void detectFunction_init();

#endif

//
// File trailer for detectFunction.h
//
// [EOF]
//
