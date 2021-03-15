//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: reduce_codegen.cu
//
// GPU Coder version                    : 2.0
// CUDA/C/C++ source code generated on  : 12-Mar-2021 11:28:36
//

// Include Files
#include "reduce_codegen.h"
#include "rt_nonfinite.h"

// Custom Source Code
#include "main.h"

// Function Definitions
//
// Arguments    : unsigned int input1
//                unsigned int input2
// Return Type  : unsigned int
//
namespace coder
{
  namespace gpucoder
  {
    namespace internal
    {
      __device__ unsigned int maxFunc_device(unsigned int input1, unsigned int
        input2)
      {
        unsigned int red;
        if (input1 > input2) {
          red = input1;
        } else {
          red = input2;
        }

        return red;
      }

      //
      // Arguments    : unsigned int input1
      //                unsigned int input2
      // Return Type  : unsigned int
      //
      __device__ unsigned int minFunc_device(unsigned int input1, unsigned int
        input2)
      {
        unsigned int red;
        if (input1 < input2) {
          red = input1;
        } else {
          red = input2;
        }

        return red;
      }
    }
  }
}

//
// File trailer for reduce_codegen.cu
//
// [EOF]
//
