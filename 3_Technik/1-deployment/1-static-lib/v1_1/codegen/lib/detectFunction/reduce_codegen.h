//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: reduce_codegen.h
//
// GPU Coder version                    : 2.0
// CUDA/C/C++ source code generated on  : 12-Mar-2021 17:27:33
//
#ifndef REDUCE_CODEGEN_H
#define REDUCE_CODEGEN_H

// Include Files
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

// Function Declarations
namespace coder
{
  namespace gpucoder
  {
    namespace internal
    {
      __device__ unsigned int maxFunc_device(unsigned int input1, unsigned int
        input2);
      __device__ unsigned int minFunc_device(unsigned int input1, unsigned int
        input2);
    }
  }
}

#endif

//
// File trailer for reduce_codegen.h
//
// [EOF]
//
