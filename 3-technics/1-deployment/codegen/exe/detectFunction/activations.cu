//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: activations.cu
//
// GPU Coder version                    : 2.0
// CUDA/C/C++ source code generated on  : 08-Mar-2021 19:56:17
//

// Include Files
#include "activations.h"
#include "DeepLearningNetwork.h"
#include "detectFunction_internal_types.h"
#include "rt_nonfinite.h"
#include "MWCudaDimUtility.hpp"
#include "cnn_api.hpp"

// Type Definitions
struct cell_wrap_26
{
  float f1[49152];
};

// Function Declarations
static __global__ void DeepLearningNetwork_activations_kernel110(const float
  varargin_1[49152], cell_wrap_26 miniBatchT[1]);
static __global__ void DeepLearningNetwork_activations_kernel111(const float
  outMiniBatch[3072], float out[3072]);

// Function Definitions
//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const float varargin_1[49152]
//                cell_wrap_26 miniBatchT[1]
// Return Type  : void
//
static __global__ __launch_bounds__(512, 1) void
  DeepLearningNetwork_activations_kernel110(const float varargin_1[49152],
  cell_wrap_26 miniBatchT[1])
{
  unsigned long threadId;
  int i;
  int i1;
  int p;
  threadId = mwGetGlobalThreadIndex();
  i = static_cast<int>(threadId % 128UL);
  threadId = (threadId - static_cast<unsigned long>(i)) / 128UL;
  i1 = static_cast<int>(threadId % 128UL);
  threadId = (threadId - static_cast<unsigned long>(i1)) / 128UL;
  p = static_cast<int>(threadId);
  if ((static_cast<int>((static_cast<int>(p < 3)) && (static_cast<int>(i1 < 128))))
      && (static_cast<int>(i < 128))) {
    miniBatchT[0].f1[(i + (i1 << 7)) + (p << 14)] = varargin_1[(i1 + (i << 7)) +
      (p << 14)];
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const float outMiniBatch[3072]
//                float out[3072]
// Return Type  : void
//
static __global__ __launch_bounds__(512, 1) void
  DeepLearningNetwork_activations_kernel111(const float outMiniBatch[3072],
  float out[3072])
{
  unsigned long threadId;
  int i;
  int i1;
  int p;
  threadId = mwGetGlobalThreadIndex();
  i = static_cast<int>(threadId % 8UL);
  threadId = (threadId - static_cast<unsigned long>(i)) / 8UL;
  i1 = static_cast<int>(threadId % 8UL);
  threadId = (threadId - static_cast<unsigned long>(i1)) / 8UL;
  p = static_cast<int>(threadId);
  if ((static_cast<int>((static_cast<int>(p < 48)) && (static_cast<int>(i1 < 8))))
      && (static_cast<int>(i < 8))) {
    out[(i + (i1 << 3)) + (p << 6)] = outMiniBatch[(i1 + (i << 3)) + (p << 6)];
  }
}

//
// Arguments    : yoloNetwork0_0 *obj
//                const float varargin_1[49152]
//                float out[3072]
// Return Type  : void
//
namespace coder
{
  void DeepLearningNetwork_activations(yoloNetwork0_0 *obj, const float
    varargin_1[49152], float out[3072])
  {
    cell_wrap_26 (*gpu_miniBatchT)[1];
    float (*gpu_varargin_1)[49152];
    float (*gpu_out)[3072];
    float (*gpu_outMiniBatch)[3072];
    cudaMalloc(&gpu_out, 12288UL);
    cudaMalloc(&gpu_outMiniBatch, 12288UL);
    cudaMalloc(&gpu_miniBatchT, 196608UL);
    cudaMalloc(&gpu_varargin_1, 196608UL);
    cudaMemcpy(gpu_varargin_1, (void *)&varargin_1[0], 196608UL,
               cudaMemcpyHostToDevice);
    DeepLearningNetwork_activations_kernel110<<<dim3(96U, 1U, 1U), dim3(512U, 1U,
      1U)>>>(*gpu_varargin_1, *gpu_miniBatchT);
    cudaMemcpy(obj->getInputDataPointer(0), (*gpu_miniBatchT)[0].f1, obj->
               layers[0]->getOutputTensor(0)->getNumElements() * sizeof(float),
               cudaMemcpyDeviceToDevice);
    obj->activations(56);
    cudaMemcpy(*gpu_outMiniBatch, obj->getLayerOutput(56, 0), obj->layers[56]
               ->getOutputTensor(0)->getNumElements() * sizeof(float),
               cudaMemcpyDeviceToDevice);
    DeepLearningNetwork_activations_kernel111<<<dim3(6U, 1U, 1U), dim3(512U, 1U,
      1U)>>>(*gpu_outMiniBatch, *gpu_out);
    cudaMemcpy(&out[0], gpu_out, 12288UL, cudaMemcpyDeviceToHost);
    cudaFree(*gpu_varargin_1);
    cudaFree(*gpu_miniBatchT);
    cudaFree(*gpu_outMiniBatch);
    cudaFree(*gpu_out);
  }
}

//
// File trailer for activations.cu
//
// [EOF]
//
