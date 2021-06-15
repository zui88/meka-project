//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: detectFunction.cu
//
// GPU Coder version                    : 2.0
// CUDA/C/C++ source code generated on  : 12-Mar-2021 17:27:33
//

// Include Files
#include "detectFunction.h"
#include "DeepLearningNetwork.h"
#include "activations.h"
#include "detectFunction_data.h"
#include "detectFunction_initialize.h"
#include "detectFunction_internal_types.h"
#include "reduce_codegen.h"
#include "rt_nonfinite.h"
#include "MWCudaDimUtility.hpp"
#include "MWLaunchParametersUtilities.hpp"
#include "MWShuffleUtility.h"
#include "MWSortFunctors.h"
#include "MWSortWithIndexUtility.h"
#include "frameReader.h"
#include "getCameraProps.h"
#include "math_constants.h"
#include "rt_nonfinite.h"

class yoloNetwork0_0;

// Type Definitions
namespace coder
{
  namespace nvidiacoder
  {
    namespace common
    {
      struct camera
      {
        bool matlabCodegenIsDeleted;
        int isInitialized;
        bool isSetupComplete;
        bool Initialized;
        char VideoDevice[33];
        MW_customData StructCustomData;
        double CamWidth;
        double CamHeight;
        double FrameRate;
        double Duration;
        unsigned int SearchMode;
      };
    }
  }

  struct YOLOv2Network
  {
    yoloNetwork0_0 *Network;
  };
}

// Variable Definitions
static yoloNetwork0_0 gobj_2;
static coder::YOLOv2Network mynet;
static bool mynet_not_empty;
static bool hwobj_not_empty;
static coder::nvidiacoder::common::camera cam;
static bool cam_not_empty;
static __device__ coder::nvidiacoder::common::camera gpu_cam;

// Function Declarations
static __device__ unsigned int atomicOpuint32_T(unsigned int *address, unsigned
  int value);
static __device__ unsigned int b_atomicOpuint32_T(unsigned int *address,
  unsigned int value);
static __device__ unsigned int b_threadGroupReduction(unsigned int val, unsigned
  int lane, unsigned int mask);
static __device__ unsigned int b_workGroupReduction(unsigned int val, unsigned
  int mask, unsigned int numActiveWarps);
static __global__ void coder_reduce0(const unsigned char inputVar[49152],
  unsigned int *outputVar);
static __global__ void detectFunction_kernel1(const char cv[33]);
static __global__ void detectFunction_kernel10(const double rowWeights[2912],
  const int initStatus, double rowWeightsTotal[224]);
static __global__ void detectFunction_kernel11(const double colWeights[5152],
  double colWeightsTotal[224]);
static __global__ void detectFunction_kernel12(const double colWeights[5152],
  const int initStatus, double colWeightsTotal[224]);
static __global__ void detectFunction_kernel13(const double colWeightsTotal[224],
  const double colWeights[5152], const unsigned char img[2764800], const short
  ipColIndices[5152], unsigned char partialResize[483840]);
static __global__ void detectFunction_kernel14(const double rowWeightsTotal[224],
  const double rowWeights[2912], const unsigned char partialResize[483840],
  const short ipRowIndices[2912], unsigned char out[150528]);
static __global__ void detectFunction_kernel15(short aux2[448], short aux1[448]);
static __global__ void detectFunction_kernel16(const short aux1[448], short
  ipRowIndices[896], double rowWeights[896]);
static __global__ void detectFunction_kernel17(const short aux2[448], short
  ipColIndices[896], double colWeights[896]);
static __global__ void detectFunction_kernel18(const double rowWeights[896],
  double rowWeightsTotal[128]);
static __global__ void detectFunction_kernel19(const double rowWeights[896],
  const int initStatus, double rowWeightsTotal[128]);
static __global__ void detectFunction_kernel2(const char cv2[26], char cv1[26]);
static __global__ void detectFunction_kernel20(const double colWeights[896],
  double colWeightsTotal[128]);
static __global__ void detectFunction_kernel21(const double colWeights[896],
  const int initStatus, double colWeightsTotal[128]);
static __global__ void detectFunction_kernel22(const double rowWeightsTotal[128],
  const double rowWeights[896], const unsigned char out[150528], const short
  ipRowIndices[896], unsigned char partialResize[86016]);
static __global__ void detectFunction_kernel23(const double colWeightsTotal[128],
  const double colWeights[896], const unsigned char partialResize[86016], const
  short ipColIndices[896], unsigned char out[49152]);
static __global__ void detectFunction_kernel24(unsigned char out[49152],
  unsigned int castRed[2]);
static __global__ void detectFunction_kernel25(unsigned char outVal[2], unsigned
  int castRed[2]);
static __global__ void detectFunction_kernel26(const int initStatus, const short
  outVal, unsigned char out[49152], float b_out[49152]);
static __global__ void detectFunction_kernel27(const signed char dv[8], double
  anchors[8]);
static __global__ void detectFunction_kernel28(const double dv1[4], double
  anchors[8]);
static __global__ void detectFunction_kernel29(const double anchors[8], const
  float tmpFeatureMap[6144], float boxOut[6144]);
static __global__ void detectFunction_kernel3(const unsigned char pln1[921600],
  const unsigned char pln0[921600], unsigned char img[2764800]);
static __global__ void detectFunction_kernel30(const float boxOut[6144], bool
  b_data[1024]);
static __global__ void detectFunction_kernel31(const float boxOut[6144], const
  short iv_data[1024], const int thresholdedPrediction_size[2], const int
  iv_size[1], float thresholdedPrediction_data[6144]);
static __global__ void detectFunction_kernel32(const float
  thresholdedPrediction_data[6144], const int thresholdedPrediction_size[2],
  const int bboxesX1Y1X2Y2_size[2], const int k, double bboxesX1Y1X2Y2_data[4096]);
static __global__ void detectFunction_kernel33(const double bboxesX1Y1X2Y2_data
  [4096], const int k, double x1_data[1024]);
static __global__ void detectFunction_kernel34(const double bboxesX1Y1X2Y2_data
  [4096], const int bboxesX1Y1X2Y2_size[2], const int k, double y1_data[1024]);
static __global__ void detectFunction_kernel35(const double bboxesX1Y1X2Y2_data
  [4096], const int bboxesX1Y1X2Y2_size[2], const int k, double x2_data[1024]);
static __global__ void detectFunction_kernel36(const double bboxesX1Y1X2Y2_data
  [4096], const int bboxesX1Y1X2Y2_size[2], const int k, double y2_data[1024]);
static __global__ void detectFunction_kernel37(const int cameraDevice, double
  x1_data[1024]);
static __global__ void detectFunction_kernel38(const int cameraDevice, double
  y1_data[1024]);
static __global__ void detectFunction_kernel39(const int cameraDevice, double
  x2_data[1024]);
static __global__ void detectFunction_kernel4(const unsigned char pln2[921600],
  unsigned char img[2764800]);
static __global__ void detectFunction_kernel40(const int cameraDevice, double
  y2_data[1024]);
static __global__ void detectFunction_kernel41(const double x1_data[1024], const
  int initStatus, double bboxesX1Y1X2Y2_data[4096]);
static __global__ void detectFunction_kernel42(const double y1_data[1024], const
  int bboxesX1Y1X2Y2_size[2], const int initStatus, double bboxesX1Y1X2Y2_data
  [4096]);
static __global__ void detectFunction_kernel43(const double x2_data[1024], const
  int bboxesX1Y1X2Y2_size[2], const int initStatus, double bboxesX1Y1X2Y2_data
  [4096]);
static __global__ void detectFunction_kernel44(const double y2_data[1024], const
  int bboxesX1Y1X2Y2_size[2], const int initStatus, double bboxesX1Y1X2Y2_data
  [4096]);
static __global__ void detectFunction_kernel45(const double bboxesX1Y1X2Y2_data
  [4096], const int initStatus, double bboxPred_data[4096]);
static __global__ void detectFunction_kernel46(const double bboxesX1Y1X2Y2_data
  [4096], const int bboxesX1Y1X2Y2_size[2], const int bboxPred_size[2], const
  int initStatus, double bboxPred_data[4096]);
static __global__ void detectFunction_kernel47(const double bboxesX1Y1X2Y2_data
  [4096], const int bboxesX1Y1X2Y2_size[2], const int bboxPred_size[2], const
  int initStatus, double bboxPred_data[4096]);
static __global__ void detectFunction_kernel48(const double bboxesX1Y1X2Y2_data
  [4096], const int bboxesX1Y1X2Y2_size[2], const int bboxPred_size[2], const
  int initStatus, double bboxPred_data[4096]);
static __global__ void detectFunction_kernel49(const int initStatus, double
  bboxPred_data[4096]);
static __global__ void detectFunction_kernel5(short aux1[1440]);
static __global__ void detectFunction_kernel50(const double bboxPred_data[4096],
  const int bboxPred_size[2], const int k, double x1_data[1024]);
static __global__ void detectFunction_kernel51(const double x1_data[1024], const
  int bboxPred_size[2], const int iv_size[1], double bboxPred_data[4096]);
static __global__ void detectFunction_kernel52(const double bboxPred_data[4096],
  const int bboxPred_size[2], const int k, double x1_data[1024]);
static __global__ void detectFunction_kernel53(const double x1_data[1024], const
  int bboxPred_size[2], const int iv_size[1], double bboxPred_data[4096]);
static __global__ void detectFunction_kernel54(const double bboxPred_data[4096],
  const int bboxPred_size[2], const int initStatus, const int b_bboxPred_size[2],
  const int *camIndex, double b_bboxPred_data[4096]);
static __global__ void detectFunction_kernel55(const float
  thresholdedPrediction_data[6144], const int thresholdedPrediction_size[2],
  const int initStatus, const int count, float classPred_data[1024], float
  scorePred_data[1024]);
static __global__ void detectFunction_kernel56(const short outVal, const short i,
  short iv_data[1024]);
static __global__ void detectFunction_kernel57(const int bboxPred_size[2], bool
  b_data[1024]);
static __global__ void detectFunction_kernel58(const bool b_data[1024], int
  initStatus, int *camIndex);
static __global__ void detectFunction_kernel59(const int bboxPred_size[2], const
  int initStatus, const short iv_data[1024], double bboxPred_data[4096]);
static __global__ void detectFunction_kernel6(short aux2[2560]);
static __global__ void detectFunction_kernel60(const double bboxPred_data[4096],
  const int bboxPred_size[2], const int b_bboxPred_size[2], const int initStatus,
  double b_bboxPred_data[4096]);
static __global__ void detectFunction_kernel61(const double bboxPred_data[4096],
  const int bboxPred_size[2], double b_bboxPred_data[4096]);
static __global__ void detectFunction_kernel62(const short outVal, const short i,
  short iv_data[1024]);
static __global__ void detectFunction_kernel63(const int scorePred_size[1], bool
  b_data[1024]);
static __global__ void detectFunction_kernel64(const bool b_data[1024], int
  initStatus, int *camIndex);
static __global__ void detectFunction_kernel65(const short outVal, const short i,
  short iv_data[1024]);
static __global__ void detectFunction_kernel66(const int iv_size[1], bool
  b_data[1024]);
static __global__ void detectFunction_kernel67(const bool b_data[1024], int
  initStatus, int *camIndex);
static __global__ void detectFunction_kernel68(const float classPred_data[1024],
  const int iv_size[1], double y1_data[1024]);
static __global__ void detectFunction_kernel69(const float scorePred_data[1024],
  const int scorePred_size[1], float classPred_data[1024]);
static __global__ void detectFunction_kernel7(const short aux1[1440], short
  ipRowIndices[2912], double rowWeights[2912]);
static __global__ void detectFunction_kernel70(const short dv2[2], double
  idx_data[1024]);
static __global__ void detectFunction_kernel71(const double bboxPred_data[4096],
  const int bboxPred_size[2], const double idx_data[1024], const int
  inputBbox_size[2], const int idx_size[1], double bboxesX1Y1X2Y2_data[4096]);
static __global__ void detectFunction_kernel72(const double y1_data[1024], const
  double idx_data[1024], const int idx_size[1], double x1_data[1024]);
static __global__ void detectFunction_kernel73(const double x1_data[1024], const
  int iv_size[1], double y1_data[1024]);
static __global__ void detectFunction_kernel74(const int idx_size[1], bool
  b_data[1024]);
static __global__ void detectFunction_kernel75(const double bboxesX1Y1X2Y2_data
  [4096], const int inputBbox_size[2], const int k, double x1_data[1024]);
static __global__ void detectFunction_kernel76(const int inputBbox_size[2],
  const double bboxesX1Y1X2Y2_data[4096], const int k, double x2_data[1024]);
static __global__ void detectFunction_kernel77(const double bboxesX1Y1X2Y2_data
  [4096], const int inputBbox_size[2], const int k, double y2_data[1024]);
static __global__ void detectFunction_kernel78(const int *camIndex, const int
  thresholdedPrediction_size[2], bool b_data[1024]);
static __global__ void detectFunction_kernel79(const bool b_data[1024], const
  double idx_data[1024], const int selectedIndex_size[1], bool index_data[1024]);
static __global__ void detectFunction_kernel8(const short aux2[2560], short
  ipColIndices[5152], double colWeights[5152]);
static __global__ void detectFunction_kernel80(const double bboxPred_data[4096],
  const int bboxPred_size[2], const short iv2_data[1024], const int
  b_bboxPred_size[2], const int iv_size[1], double b_bboxPred_data[4096]);
static __global__ void detectFunction_kernel81(const double bboxPred_data[4096],
  const int bboxPred_size[2], double b_bboxPred_data[4096]);
static __global__ void detectFunction_kernel82(const float scorePred_data[1024],
  const int scorePred_size[1], float scores_data[1024]);
static __global__ void detectFunction_kernel9(const double rowWeights[2912],
  double rowWeightsTotal[224]);
static __device__ double rt_powd_snf_device(double u0, double u1);
static __device__ double rt_roundd_snf_device(double u);
static __device__ unsigned int shflDown1(unsigned int in1, unsigned int offset,
  unsigned int mask);
static __device__ int shflDown1(int in1, unsigned int offset, unsigned int mask);
static __device__ unsigned int threadGroupReduction(unsigned int val, unsigned
  int lane, unsigned int mask);
static __device__ int threadGroupReduction(int val, unsigned int lane, unsigned
  int mask);
static __device__ unsigned int workGroupReduction(unsigned int val, unsigned int
  mask, unsigned int numActiveWarps);
static __device__ int workGroupReduction(int val, unsigned int mask, unsigned
  int numActiveWarps);

// Function Definitions
//
// Arguments    : unsigned int *address
//                unsigned int value
// Return Type  : unsigned int
//
static __device__ unsigned int atomicOpuint32_T(unsigned int *address, unsigned
  int value)
{
  unsigned int output;
  output = *address;
  unsigned int assumed;
  do {
    assumed = output;
    output = atomicCAS(address, output, coder::gpucoder::internal::
                       minFunc_device(value, output));
  } while (assumed != output);

  return output;
}

//
// Arguments    : unsigned int *address
//                unsigned int value
// Return Type  : unsigned int
//
static __device__ unsigned int b_atomicOpuint32_T(unsigned int *address,
  unsigned int value)
{
  unsigned int output;
  output = *address;
  unsigned int assumed;
  do {
    assumed = output;
    output = atomicCAS(address, output, coder::gpucoder::internal::
                       maxFunc_device(value, output));
  } while (assumed != output);

  return output;
}

//
// Arguments    : unsigned int val
//                unsigned int lane
//                unsigned int mask
// Return Type  : unsigned int
//
static __device__ unsigned int b_threadGroupReduction(unsigned int val, unsigned
  int lane, unsigned int mask)
{
  unsigned int activeSize;
  unsigned int offset;
  activeSize = __popc(mask);
  offset = (activeSize + 1U) / 2U;
  while (activeSize > 1U) {
    unsigned int other;
    other = shflDown1(val, offset, mask);
    if (lane + offset < activeSize) {
      val = coder::gpucoder::internal::maxFunc_device(val, other);
    }

    activeSize = offset;
    offset = (offset + 1U) / 2U;
  }

  return val;
}

//
// Arguments    : unsigned int val
//                unsigned int mask
//                unsigned int numActiveWarps
// Return Type  : unsigned int
//
static __device__ unsigned int b_workGroupReduction(unsigned int val, unsigned
  int mask, unsigned int numActiveWarps)
{
  __shared__ unsigned int shared[32];
  unsigned int lane;
  unsigned int thBlkId;
  thBlkId = static_cast<unsigned int>(mwGetThreadIndexWithinBlock());
  lane = thBlkId % warpSize;
  thBlkId /= warpSize;
  val = b_threadGroupReduction(val, lane, mask);
  if (lane == 0U) {
    shared[thBlkId] = val;
  }

  __syncthreads();
  mask = __ballot_sync(MAX_uint32_T, lane < numActiveWarps);
  val = shared[lane];
  if (thBlkId == 0U) {
    val = b_threadGroupReduction(val, lane, mask);
  }

  return val;
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const unsigned char inputVar[49152]
//                unsigned int *outputVar
// Return Type  : void
//
static __global__ __launch_bounds__(1024, 1) void coder_reduce0(const unsigned
  char inputVar[49152], unsigned int *outputVar)
{
  unsigned int blockStride;
  unsigned int idx;
  unsigned int mask;
  unsigned int thBlkId;
  unsigned int threadId;
  unsigned int threadStride;
  unsigned int tmpRed0;
  unsigned int tmpRed1;
  threadStride = static_cast<unsigned int>(mwGetTotalThreadsLaunched());
  threadId = static_cast<unsigned int>(mwGetGlobalThreadIndex());
  thBlkId = static_cast<unsigned int>(mwGetThreadIndexWithinBlock());
  blockStride = static_cast<unsigned int>(mwGetThreadsPerBlock());
  tmpRed0 = 0U;
  tmpRed1 = 0U;
  if (mwIsLastBlock()) {
    int m;
    m = static_cast<int>(49151U % blockStride);
    if (static_cast<unsigned int>(m) > 0U) {
      blockStride = static_cast<unsigned int>(m);
    }
  }

  blockStride = ((blockStride + warpSize) - 1U) / warpSize;
  if (threadId <= 49150U) {
    tmpRed0 = static_cast<unsigned int>(inputVar[threadId]);
    tmpRed1 = tmpRed0;
  }

  mask = __ballot_sync(MAX_uint32_T, threadId <= 49150U);
  for (idx = threadId + threadStride; idx <= 49150U; idx += threadStride) {
    tmpRed0 = coder::gpucoder::internal::minFunc_device(tmpRed0, static_cast<
      unsigned int>(inputVar[idx]));
    tmpRed1 = coder::gpucoder::internal::maxFunc_device(tmpRed1, static_cast<
      unsigned int>(inputVar[idx]));
  }

  tmpRed0 = workGroupReduction(tmpRed0, mask, blockStride);
  tmpRed1 = b_workGroupReduction(tmpRed1, mask, blockStride);
  if (thBlkId == 0U) {
    atomicOpuint32_T(&outputVar[0], tmpRed0);
    b_atomicOpuint32_T(&outputVar[1], tmpRed1);
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const char cv[33]
// Return Type  : void
//
static __global__ __launch_bounds__(64, 1) void detectFunction_kernel1(const
  char cv[33])
{
  int oldIdx;
  oldIdx = static_cast<int>(mwGetGlobalThreadIndex());
  if (oldIdx < 33) {
    gpu_cam.VideoDevice[oldIdx] = cv[oldIdx];
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double rowWeights[2912]
//                const int initStatus
//                double rowWeightsTotal[224]
// Return Type  : void
//
static __global__ __launch_bounds__(224, 1) void detectFunction_kernel10(const
  double rowWeights[2912], const int initStatus, double rowWeightsTotal[224])
{
  int rowIdx;
  rowIdx = static_cast<int>(mwGetGlobalThreadIndex());
  if (rowIdx < 224) {
    rowWeightsTotal[rowIdx] += rowWeights[initStatus + rowIdx];
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double colWeights[5152]
//                double colWeightsTotal[224]
// Return Type  : void
//
static __global__ __launch_bounds__(224, 1) void detectFunction_kernel11(const
  double colWeights[5152], double colWeightsTotal[224])
{
  int rowIdx;
  rowIdx = static_cast<int>(mwGetGlobalThreadIndex());
  if (rowIdx < 224) {
    colWeightsTotal[rowIdx] = colWeights[rowIdx];
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double colWeights[5152]
//                const int initStatus
//                double colWeightsTotal[224]
// Return Type  : void
//
static __global__ __launch_bounds__(224, 1) void detectFunction_kernel12(const
  double colWeights[5152], const int initStatus, double colWeightsTotal[224])
{
  int rowIdx;
  rowIdx = static_cast<int>(mwGetGlobalThreadIndex());
  if (rowIdx < 224) {
    colWeightsTotal[rowIdx] += colWeights[initStatus + rowIdx];
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double colWeightsTotal[224]
//                const double colWeights[5152]
//                const unsigned char img[2764800]
//                const short ipColIndices[5152]
//                unsigned char partialResize[483840]
// Return Type  : void
//
static __global__ __launch_bounds__(512, 1) void detectFunction_kernel13(const
  double colWeightsTotal[224], const double colWeights[5152], const unsigned
  char img[2764800], const short ipColIndices[5152], unsigned char
  partialResize[483840])
{
  double sumVal;
  unsigned long threadId;
  int colIdx;
  int oldIdx;
  int rowIdx;
  threadId = mwGetGlobalThreadIndex();
  oldIdx = static_cast<int>(threadId % 3UL);
  threadId = (threadId - static_cast<unsigned long>(oldIdx)) / 3UL;
  colIdx = static_cast<int>(threadId % 224UL);
  threadId = (threadId - static_cast<unsigned long>(colIdx)) / 224UL;
  rowIdx = static_cast<int>(threadId);
  if ((static_cast<int>((static_cast<int>(rowIdx < 720)) && (static_cast<int>
         (colIdx < 224)))) && (static_cast<int>(oldIdx < 3))) {
    unsigned char u1;
    sumVal = 0.0;
    for (int ind = 0; ind < 23; ind++) {
      sumVal += static_cast<double>(img[(rowIdx + 720 * (static_cast<int>
        (ipColIndices[colIdx + 224 * ind]) - 1)) + 921600 * oldIdx]) *
        (colWeights[colIdx + 224 * ind] / colWeightsTotal[colIdx]);
    }

    sumVal = rt_roundd_snf_device(sumVal);
    if (sumVal < 256.0) {
      if (sumVal >= 0.0) {
        u1 = static_cast<unsigned char>(sumVal);
      } else {
        u1 = static_cast<unsigned char>(0U);
      }
    } else if (sumVal >= 256.0) {
      u1 = MAX_uint8_T;
    } else {
      u1 = static_cast<unsigned char>(0U);
    }

    partialResize[(rowIdx + 720 * colIdx) + 161280 * oldIdx] = u1;
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double rowWeightsTotal[224]
//                const double rowWeights[2912]
//                const unsigned char partialResize[483840]
//                const short ipRowIndices[2912]
//                unsigned char out[150528]
// Return Type  : void
//
static __global__ __launch_bounds__(512, 1) void detectFunction_kernel14(const
  double rowWeightsTotal[224], const double rowWeights[2912], const unsigned
  char partialResize[483840], const short ipRowIndices[2912], unsigned char out
  [150528])
{
  double sumVal;
  unsigned long threadId;
  int colIdx;
  int oldIdx;
  int rowIdx;
  threadId = mwGetGlobalThreadIndex();
  oldIdx = static_cast<int>(threadId % 3UL);
  threadId = (threadId - static_cast<unsigned long>(oldIdx)) / 3UL;
  rowIdx = static_cast<int>(threadId % 224UL);
  threadId = (threadId - static_cast<unsigned long>(rowIdx)) / 224UL;
  colIdx = static_cast<int>(threadId);
  if ((static_cast<int>((static_cast<int>(colIdx < 224)) && (static_cast<int>
         (rowIdx < 224)))) && (static_cast<int>(oldIdx < 3))) {
    unsigned char u1;
    sumVal = 0.0;
    for (int ind = 0; ind < 13; ind++) {
      sumVal += static_cast<double>(partialResize[((static_cast<int>
        (ipRowIndices[rowIdx + 224 * ind]) + 720 * colIdx) + 161280 * oldIdx) -
        1]) * (rowWeights[rowIdx + 224 * ind] / rowWeightsTotal[rowIdx]);
    }

    sumVal = rt_roundd_snf_device(sumVal);
    if (sumVal < 256.0) {
      if (sumVal >= 0.0) {
        u1 = static_cast<unsigned char>(sumVal);
      } else {
        u1 = static_cast<unsigned char>(0U);
      }
    } else if (sumVal >= 256.0) {
      u1 = MAX_uint8_T;
    } else {
      u1 = static_cast<unsigned char>(0U);
    }

    out[(rowIdx + 224 * colIdx) + 50176 * oldIdx] = u1;
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                short aux2[448]
//                short aux1[448]
// Return Type  : void
//
static __global__ __launch_bounds__(448, 1) void detectFunction_kernel15(short
  aux2[448], short aux1[448])
{
  int i;
  i = static_cast<int>(mwGetGlobalThreadIndex());
  if (i < 448) {
    if (i + 1 <= 224) {
      aux1[i] = static_cast<short>(i + 1);
      aux2[i] = static_cast<short>(i + 1);
    } else {
      aux1[i] = static_cast<short>(448 - i);
      aux2[i] = static_cast<short>(448 - i);
    }
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const short aux1[448]
//                short ipRowIndices[896]
//                double rowWeights[896]
// Return Type  : void
//
static __global__ __launch_bounds__(512, 1) void detectFunction_kernel16(const
  short aux1[448], short ipRowIndices[896], double rowWeights[896])
{
  unsigned long threadId;
  int i;
  int ind;
  int rowIdx;
  threadId = mwGetGlobalThreadIndex();
  i = static_cast<int>(threadId % 7UL);
  rowIdx = static_cast<int>((threadId - static_cast<unsigned long>(i)) / 7UL);
  if ((static_cast<int>(rowIdx < 128)) && (static_cast<int>(i < 7))) {
    double absx;
    double absx2;
    double sumVal;
    int oldIdx;
    sumVal = (static_cast<double>(rowIdx) + 1.0) / 0.5714285714285714 + -0.375;
    oldIdx = static_cast<int>(floor(sumVal - 3.5));
    absx = fabs(0.5714285714285714 * (sumVal - (static_cast<double>(oldIdx + i)
      + 1.0)));
    absx2 = absx * absx;
    sumVal = rt_powd_snf_device(absx, 3.0);
    rowWeights[rowIdx + (i << 7)] = 0.5714285714285714 * (((1.5 * sumVal - 2.5 *
      absx2) + 1.0) * static_cast<double>(absx <= 1.0) + (((-0.5 * sumVal + 2.5 *
      absx2) - 4.0 * absx) + 2.0) * static_cast<double>((static_cast<int>(1.0 <
      absx)) && (static_cast<int>(absx <= 2.0))));
    oldIdx = (oldIdx + i) + 1;
    if (oldIdx - 1 == 0) {
      ind = 0;
    } else {
      ind = static_cast<int>(fmod(static_cast<double>(oldIdx) - 1.0, 448.0));
      if ((static_cast<int>(ind != 0)) && (static_cast<int>(oldIdx - 1 < 0))) {
        ind += 448;
      }
    }

    ipRowIndices[rowIdx + (i << 7)] = aux1[ind];
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const short aux2[448]
//                short ipColIndices[896]
//                double colWeights[896]
// Return Type  : void
//
static __global__ __launch_bounds__(512, 1) void detectFunction_kernel17(const
  short aux2[448], short ipColIndices[896], double colWeights[896])
{
  unsigned long threadId;
  int colIdx;
  int i;
  int ind;
  threadId = mwGetGlobalThreadIndex();
  i = static_cast<int>(threadId % 7UL);
  colIdx = static_cast<int>((threadId - static_cast<unsigned long>(i)) / 7UL);
  if ((static_cast<int>(colIdx < 128)) && (static_cast<int>(i < 7))) {
    double absx;
    double absx2;
    double sumVal;
    int oldIdx;
    sumVal = (static_cast<double>(colIdx) + 1.0) / 0.5714285714285714 + -0.375;
    oldIdx = static_cast<int>(floor(sumVal - 3.5));
    absx = fabs(0.5714285714285714 * (sumVal - (static_cast<double>(oldIdx + i)
      + 1.0)));
    absx2 = absx * absx;
    sumVal = rt_powd_snf_device(absx, 3.0);
    colWeights[colIdx + (i << 7)] = 0.5714285714285714 * (((1.5 * sumVal - 2.5 *
      absx2) + 1.0) * static_cast<double>(absx <= 1.0) + (((-0.5 * sumVal + 2.5 *
      absx2) - 4.0 * absx) + 2.0) * static_cast<double>((static_cast<int>(1.0 <
      absx)) && (static_cast<int>(absx <= 2.0))));
    oldIdx = (oldIdx + i) + 1;
    if (oldIdx - 1 == 0) {
      ind = 0;
    } else {
      ind = static_cast<int>(fmod(static_cast<double>(oldIdx) - 1.0, 448.0));
      if ((static_cast<int>(ind != 0)) && (static_cast<int>(oldIdx - 1 < 0))) {
        ind += 448;
      }
    }

    ipColIndices[colIdx + (i << 7)] = aux2[ind];
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double rowWeights[896]
//                double rowWeightsTotal[128]
// Return Type  : void
//
static __global__ __launch_bounds__(128, 1) void detectFunction_kernel18(const
  double rowWeights[896], double rowWeightsTotal[128])
{
  int rowIdx;
  rowIdx = static_cast<int>(mwGetGlobalThreadIndex());
  if (rowIdx < 128) {
    rowWeightsTotal[rowIdx] = rowWeights[rowIdx];
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double rowWeights[896]
//                const int initStatus
//                double rowWeightsTotal[128]
// Return Type  : void
//
static __global__ __launch_bounds__(128, 1) void detectFunction_kernel19(const
  double rowWeights[896], const int initStatus, double rowWeightsTotal[128])
{
  int rowIdx;
  rowIdx = static_cast<int>(mwGetGlobalThreadIndex());
  if (rowIdx < 128) {
    rowWeightsTotal[rowIdx] += rowWeights[initStatus + rowIdx];
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const char cv2[26]
//                char cv1[26]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void detectFunction_kernel2(const
  char cv2[26], char cv1[26])
{
  int oldIdx;
  oldIdx = static_cast<int>(mwGetGlobalThreadIndex());
  if (oldIdx < 26) {
    cv1[oldIdx] = cv2[oldIdx];
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double colWeights[896]
//                double colWeightsTotal[128]
// Return Type  : void
//
static __global__ __launch_bounds__(128, 1) void detectFunction_kernel20(const
  double colWeights[896], double colWeightsTotal[128])
{
  int rowIdx;
  rowIdx = static_cast<int>(mwGetGlobalThreadIndex());
  if (rowIdx < 128) {
    colWeightsTotal[rowIdx] = colWeights[rowIdx];
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double colWeights[896]
//                const int initStatus
//                double colWeightsTotal[128]
// Return Type  : void
//
static __global__ __launch_bounds__(128, 1) void detectFunction_kernel21(const
  double colWeights[896], const int initStatus, double colWeightsTotal[128])
{
  int rowIdx;
  rowIdx = static_cast<int>(mwGetGlobalThreadIndex());
  if (rowIdx < 128) {
    colWeightsTotal[rowIdx] += colWeights[initStatus + rowIdx];
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double rowWeightsTotal[128]
//                const double rowWeights[896]
//                const unsigned char out[150528]
//                const short ipRowIndices[896]
//                unsigned char partialResize[86016]
// Return Type  : void
//
static __global__ __launch_bounds__(512, 1) void detectFunction_kernel22(const
  double rowWeightsTotal[128], const double rowWeights[896], const unsigned char
  out[150528], const short ipRowIndices[896], unsigned char partialResize[86016])
{
  double sumVal;
  unsigned long threadId;
  int colIdx;
  int oldIdx;
  int rowIdx;
  threadId = mwGetGlobalThreadIndex();
  oldIdx = static_cast<int>(threadId % 3UL);
  threadId = (threadId - static_cast<unsigned long>(oldIdx)) / 3UL;
  rowIdx = static_cast<int>(threadId % 128UL);
  threadId = (threadId - static_cast<unsigned long>(rowIdx)) / 128UL;
  colIdx = static_cast<int>(threadId);
  if ((static_cast<int>((static_cast<int>(colIdx < 224)) && (static_cast<int>
         (rowIdx < 128)))) && (static_cast<int>(oldIdx < 3))) {
    unsigned char u1;
    sumVal = 0.0;
    for (int ind = 0; ind < 7; ind++) {
      sumVal += static_cast<double>(out[((static_cast<int>(ipRowIndices[rowIdx +
        (ind << 7)]) + 224 * colIdx) + 50176 * oldIdx) - 1]) *
        (rowWeights[rowIdx + (ind << 7)] / rowWeightsTotal[rowIdx]);
    }

    sumVal = rt_roundd_snf_device(sumVal);
    if (sumVal < 256.0) {
      if (sumVal >= 0.0) {
        u1 = static_cast<unsigned char>(sumVal);
      } else {
        u1 = static_cast<unsigned char>(0U);
      }
    } else if (sumVal >= 256.0) {
      u1 = MAX_uint8_T;
    } else {
      u1 = static_cast<unsigned char>(0U);
    }

    partialResize[(rowIdx + (colIdx << 7)) + 28672 * oldIdx] = u1;
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double colWeightsTotal[128]
//                const double colWeights[896]
//                const unsigned char partialResize[86016]
//                const short ipColIndices[896]
//                unsigned char out[49152]
// Return Type  : void
//
static __global__ __launch_bounds__(512, 1) void detectFunction_kernel23(const
  double colWeightsTotal[128], const double colWeights[896], const unsigned char
  partialResize[86016], const short ipColIndices[896], unsigned char out[49152])
{
  double sumVal;
  unsigned long threadId;
  int colIdx;
  int oldIdx;
  int rowIdx;
  threadId = mwGetGlobalThreadIndex();
  oldIdx = static_cast<int>(threadId % 3UL);
  threadId = (threadId - static_cast<unsigned long>(oldIdx)) / 3UL;
  rowIdx = static_cast<int>(threadId % 128UL);
  threadId = (threadId - static_cast<unsigned long>(rowIdx)) / 128UL;
  colIdx = static_cast<int>(threadId);
  if ((static_cast<int>((static_cast<int>(colIdx < 128)) && (static_cast<int>
         (rowIdx < 128)))) && (static_cast<int>(oldIdx < 3))) {
    unsigned char u1;
    sumVal = 0.0;
    for (int ind = 0; ind < 7; ind++) {
      sumVal += static_cast<double>(partialResize[(rowIdx + ((static_cast<int>
        (ipColIndices[colIdx + (ind << 7)]) - 1) << 7)) + 28672 * oldIdx]) *
        (colWeights[colIdx + (ind << 7)] / colWeightsTotal[colIdx]);
    }

    sumVal = rt_roundd_snf_device(sumVal);
    if (sumVal < 256.0) {
      if (sumVal >= 0.0) {
        u1 = static_cast<unsigned char>(sumVal);
      } else {
        u1 = static_cast<unsigned char>(0U);
      }
    } else if (sumVal >= 256.0) {
      u1 = MAX_uint8_T;
    } else {
      u1 = static_cast<unsigned char>(0U);
    }

    out[(rowIdx + (colIdx << 7)) + (oldIdx << 14)] = u1;
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                unsigned char out[49152]
//                unsigned int castRed[2]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void detectFunction_kernel24(unsigned
  char out[49152], unsigned int castRed[2])
{
  int indV;
  indV = static_cast<int>(mwGetGlobalThreadIndex());
  if (indV < 2) {
    castRed[indV] = static_cast<unsigned int>(out[49151]);
    castRed[indV] = static_cast<unsigned int>(out[49151]);
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                unsigned char outVal[2]
//                unsigned int castRed[2]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void detectFunction_kernel25(unsigned
  char outVal[2], unsigned int castRed[2])
{
  int oldIdx;
  oldIdx = static_cast<int>(mwGetGlobalThreadIndex());
  if (oldIdx < 2) {
    unsigned int u;
    u = castRed[oldIdx];
    if (u > 255U) {
      u = 255U;
    }

    outVal[oldIdx] = static_cast<unsigned char>(u);
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const int initStatus
//                const short outVal
//                unsigned char out[49152]
//                float b_out[49152]
// Return Type  : void
//
static __global__ __launch_bounds__(512, 1) void detectFunction_kernel26(const
  int initStatus, const short outVal, unsigned char out[49152], float b_out
  [49152])
{
  int oldIdx;
  oldIdx = static_cast<int>(mwGetGlobalThreadIndex());
  if (oldIdx < 49152) {
    b_out[oldIdx] = static_cast<float>(static_cast<int>(out[oldIdx]) -
      static_cast<int>(outVal)) / static_cast<float>(initStatus);
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const signed char dv[8]
//                double anchors[8]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void detectFunction_kernel27(const
  signed char dv[8], double anchors[8])
{
  int oldIdx;
  oldIdx = static_cast<int>(mwGetGlobalThreadIndex());
  if (oldIdx < 8) {
    anchors[oldIdx] = static_cast<double>(dv[oldIdx]);
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double dv1[4]
//                double anchors[8]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void detectFunction_kernel28(const
  double dv1[4], double anchors[8])
{
  int oldIdx;
  oldIdx = static_cast<int>(mwGetGlobalThreadIndex());
  if (oldIdx < 4) {
    anchors[oldIdx] = dv1[oldIdx];
    anchors[oldIdx + 4] /= 8.0;
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double anchors[8]
//                const float tmpFeatureMap[6144]
//                float boxOut[6144]
// Return Type  : void
//
static __global__ __launch_bounds__(512, 1) void detectFunction_kernel29(const
  double anchors[8], const float tmpFeatureMap[6144], float boxOut[6144])
{
  unsigned long threadId;
  int colIdx;
  int oldIdx;
  int rowIdx;
  threadId = mwGetGlobalThreadIndex();
  rowIdx = static_cast<int>(threadId % 16UL);
  threadId = (threadId - static_cast<unsigned long>(rowIdx)) / 16UL;
  colIdx = static_cast<int>(threadId % 16UL);
  threadId = (threadId - static_cast<unsigned long>(colIdx)) / 16UL;
  oldIdx = static_cast<int>(threadId);
  if ((static_cast<int>((static_cast<int>(oldIdx < 4)) && (static_cast<int>
         (colIdx < 16)))) && (static_cast<int>(rowIdx < 16))) {
    float bh;
    float bw;
    float cx;
    float cy;
    int ind;
    ind = (((rowIdx << 6) + (colIdx << 2)) + oldIdx) + 1;
    cx = (tmpFeatureMap[((rowIdx + (colIdx << 4)) + (oldIdx << 8)) + 1024] +
          static_cast<float>(colIdx)) * 8.0F;
    cy = (tmpFeatureMap[((rowIdx + (colIdx << 4)) + (oldIdx << 8)) + 2048] +
          static_cast<float>(rowIdx)) * 8.0F;
    bw = tmpFeatureMap[((rowIdx + (colIdx << 4)) + (oldIdx << 8)) + 3072] *
      static_cast<float>(anchors[oldIdx + 4]) * 8.0F;
    bh = tmpFeatureMap[((rowIdx + (colIdx << 4)) + (oldIdx << 8)) + 4096] *
      static_cast<float>(anchors[oldIdx]) * 8.0F;
    boxOut[ind - 1] = cx - bw / 2.0F;
    boxOut[ind + 1023] = cy - bh / 2.0F;
    boxOut[ind + 2047] = cx + bw / 2.0F;
    boxOut[ind + 3071] = cy + bh / 2.0F;
    boxOut[ind + 4095] = tmpFeatureMap[(rowIdx + (colIdx << 4)) + (oldIdx << 8)]
      * tmpFeatureMap[((rowIdx + (colIdx << 4)) + (oldIdx << 8)) + 5120];
    boxOut[ind + 5119] = 1.0F;
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const unsigned char pln1[921600]
//                const unsigned char pln0[921600]
//                unsigned char img[2764800]
// Return Type  : void
//
static __global__ __launch_bounds__(512, 1) void detectFunction_kernel3(const
  unsigned char pln1[921600], const unsigned char pln0[921600], unsigned char
  img[2764800])
{
  unsigned long threadId;
  int ind;
  int oldIdx;
  threadId = mwGetGlobalThreadIndex();
  ind = static_cast<int>(threadId % 720UL);
  oldIdx = static_cast<int>((threadId - static_cast<unsigned long>(ind)) / 720UL);
  if ((static_cast<int>(oldIdx < 1280)) && (static_cast<int>(ind < 720))) {
    img[ind + 720 * oldIdx] = pln0[oldIdx + 1280 * ind];
    img[(ind + 720 * oldIdx) + 921600] = pln1[oldIdx + 1280 * ind];
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const float boxOut[6144]
//                bool b_data[1024]
// Return Type  : void
//
static __global__ __launch_bounds__(512, 1) void detectFunction_kernel30(const
  float boxOut[6144], bool b_data[1024])
{
  int oldIdx;
  oldIdx = static_cast<int>(mwGetGlobalThreadIndex());
  if (oldIdx < 1024) {
    b_data[oldIdx] = (boxOut[oldIdx + 4096] >= 0.5F);
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const float boxOut[6144]
//                const short iv_data[1024]
//                const int thresholdedPrediction_size[2]
//                const int iv_size[1]
//                float thresholdedPrediction_data[6144]
// Return Type  : void
//
static __global__ __launch_bounds__(1024, 1) void detectFunction_kernel31(const
  float boxOut[6144], const short iv_data[1024], const int
  thresholdedPrediction_size[2], const int iv_size[1], float
  thresholdedPrediction_data[6144])
{
  unsigned long idx;
  long loopEnd;
  unsigned long threadId;
  unsigned long threadStride;
  threadId = mwGetGlobalThreadIndex();
  threadStride = mwGetTotalThreadsLaunched();
  loopEnd = (static_cast<long>(iv_size[0] - 1) + 1L) * 6L - 1L;
  for (idx = threadId; idx <= static_cast<unsigned long>(loopEnd); idx +=
       threadStride) {
    int ind;
    int oldIdx;
    ind = static_cast<int>(idx % (static_cast<unsigned long>(iv_size[0] - 1) +
      1UL));
    oldIdx = static_cast<int>((idx - static_cast<unsigned long>(ind)) / (
      static_cast<unsigned long>(iv_size[0] - 1) + 1UL));
    thresholdedPrediction_data[ind + thresholdedPrediction_size[0] * oldIdx] =
      boxOut[(static_cast<int>(iv_data[ind]) + (oldIdx << 10)) - 1];
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const float thresholdedPrediction_data[6144]
//                const int thresholdedPrediction_size[2]
//                const int bboxesX1Y1X2Y2_size[2]
//                const int k
//                double bboxesX1Y1X2Y2_data[4096]
// Return Type  : void
//
static __global__ __launch_bounds__(1024, 1) void detectFunction_kernel32(const
  float thresholdedPrediction_data[6144], const int thresholdedPrediction_size[2],
  const int bboxesX1Y1X2Y2_size[2], const int k, double bboxesX1Y1X2Y2_data[4096])
{
  unsigned long idx;
  long loopEnd;
  unsigned long threadId;
  unsigned long threadStride;
  threadId = mwGetGlobalThreadIndex();
  threadStride = mwGetTotalThreadsLaunched();
  loopEnd = (static_cast<long>(k) + 1L) * 4L - 1L;
  for (idx = threadId; idx <= static_cast<unsigned long>(loopEnd); idx +=
       threadStride) {
    int ind;
    int oldIdx;
    ind = static_cast<int>(idx % (static_cast<unsigned long>(k) + 1UL));
    oldIdx = static_cast<int>((idx - static_cast<unsigned long>(ind)) / (
      static_cast<unsigned long>(k) + 1UL));
    bboxesX1Y1X2Y2_data[ind + bboxesX1Y1X2Y2_size[0] * oldIdx] = static_cast<
      double>(thresholdedPrediction_data[ind + thresholdedPrediction_size[0] *
              oldIdx]);
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double bboxesX1Y1X2Y2_data[4096]
//                const int k
//                double x1_data[1024]
// Return Type  : void
//
static __global__ __launch_bounds__(1024, 1) void detectFunction_kernel33(const
  double bboxesX1Y1X2Y2_data[4096], const int k, double x1_data[1024])
{
  unsigned long idx;
  long loopEnd;
  unsigned long threadId;
  unsigned long threadStride;
  threadId = mwGetGlobalThreadIndex();
  threadStride = mwGetTotalThreadsLaunched();
  loopEnd = static_cast<long>(k);
  for (idx = threadId; idx <= static_cast<unsigned long>(loopEnd); idx +=
       threadStride) {
    int oldIdx;
    oldIdx = static_cast<int>(idx);
    x1_data[oldIdx] = bboxesX1Y1X2Y2_data[oldIdx];
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double bboxesX1Y1X2Y2_data[4096]
//                const int bboxesX1Y1X2Y2_size[2]
//                const int k
//                double y1_data[1024]
// Return Type  : void
//
static __global__ __launch_bounds__(1024, 1) void detectFunction_kernel34(const
  double bboxesX1Y1X2Y2_data[4096], const int bboxesX1Y1X2Y2_size[2], const int
  k, double y1_data[1024])
{
  unsigned long idx;
  long loopEnd;
  unsigned long threadId;
  unsigned long threadStride;
  threadId = mwGetGlobalThreadIndex();
  threadStride = mwGetTotalThreadsLaunched();
  loopEnd = static_cast<long>(k);
  for (idx = threadId; idx <= static_cast<unsigned long>(loopEnd); idx +=
       threadStride) {
    int oldIdx;
    oldIdx = static_cast<int>(idx);
    y1_data[oldIdx] = bboxesX1Y1X2Y2_data[oldIdx + bboxesX1Y1X2Y2_size[0]];
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double bboxesX1Y1X2Y2_data[4096]
//                const int bboxesX1Y1X2Y2_size[2]
//                const int k
//                double x2_data[1024]
// Return Type  : void
//
static __global__ __launch_bounds__(1024, 1) void detectFunction_kernel35(const
  double bboxesX1Y1X2Y2_data[4096], const int bboxesX1Y1X2Y2_size[2], const int
  k, double x2_data[1024])
{
  unsigned long idx;
  long loopEnd;
  unsigned long threadId;
  unsigned long threadStride;
  threadId = mwGetGlobalThreadIndex();
  threadStride = mwGetTotalThreadsLaunched();
  loopEnd = static_cast<long>(k);
  for (idx = threadId; idx <= static_cast<unsigned long>(loopEnd); idx +=
       threadStride) {
    int oldIdx;
    oldIdx = static_cast<int>(idx);
    x2_data[oldIdx] = bboxesX1Y1X2Y2_data[oldIdx + (bboxesX1Y1X2Y2_size[0] << 1)];
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double bboxesX1Y1X2Y2_data[4096]
//                const int bboxesX1Y1X2Y2_size[2]
//                const int k
//                double y2_data[1024]
// Return Type  : void
//
static __global__ __launch_bounds__(1024, 1) void detectFunction_kernel36(const
  double bboxesX1Y1X2Y2_data[4096], const int bboxesX1Y1X2Y2_size[2], const int
  k, double y2_data[1024])
{
  unsigned long idx;
  long loopEnd;
  unsigned long threadId;
  unsigned long threadStride;
  threadId = mwGetGlobalThreadIndex();
  threadStride = mwGetTotalThreadsLaunched();
  loopEnd = static_cast<long>(k);
  for (idx = threadId; idx <= static_cast<unsigned long>(loopEnd); idx +=
       threadStride) {
    int oldIdx;
    oldIdx = static_cast<int>(idx);
    y2_data[oldIdx] = bboxesX1Y1X2Y2_data[oldIdx + bboxesX1Y1X2Y2_size[0] * 3];
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const int cameraDevice
//                double x1_data[1024]
// Return Type  : void
//
static __global__ __launch_bounds__(1024, 1) void detectFunction_kernel37(const
  int cameraDevice, double x1_data[1024])
{
  unsigned long idx;
  long loopEnd;
  unsigned long threadId;
  unsigned long threadStride;
  threadId = mwGetGlobalThreadIndex();
  threadStride = mwGetTotalThreadsLaunched();
  loopEnd = static_cast<long>(cameraDevice - 1);
  for (idx = threadId; idx <= static_cast<unsigned long>(loopEnd); idx +=
       threadStride) {
    int i;
    i = static_cast<int>(idx);
    if (x1_data[i] < 1.0) {
      x1_data[i] = 1.0;
    }
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const int cameraDevice
//                double y1_data[1024]
// Return Type  : void
//
static __global__ __launch_bounds__(1024, 1) void detectFunction_kernel38(const
  int cameraDevice, double y1_data[1024])
{
  unsigned long idx;
  long loopEnd;
  unsigned long threadId;
  unsigned long threadStride;
  threadId = mwGetGlobalThreadIndex();
  threadStride = mwGetTotalThreadsLaunched();
  loopEnd = static_cast<long>(cameraDevice - 1);
  for (idx = threadId; idx <= static_cast<unsigned long>(loopEnd); idx +=
       threadStride) {
    int i;
    i = static_cast<int>(idx);
    if (y1_data[i] < 1.0) {
      y1_data[i] = 1.0;
    }
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const int cameraDevice
//                double x2_data[1024]
// Return Type  : void
//
static __global__ __launch_bounds__(1024, 1) void detectFunction_kernel39(const
  int cameraDevice, double x2_data[1024])
{
  unsigned long idx;
  long loopEnd;
  unsigned long threadId;
  unsigned long threadStride;
  threadId = mwGetGlobalThreadIndex();
  threadStride = mwGetTotalThreadsLaunched();
  loopEnd = static_cast<long>(cameraDevice - 1);
  for (idx = threadId; idx <= static_cast<unsigned long>(loopEnd); idx +=
       threadStride) {
    int i;
    i = static_cast<int>(idx);
    if (x2_data[i] > 128.0) {
      x2_data[i] = 128.0;
    }
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const unsigned char pln2[921600]
//                unsigned char img[2764800]
// Return Type  : void
//
static __global__ __launch_bounds__(512, 1) void detectFunction_kernel4(const
  unsigned char pln2[921600], unsigned char img[2764800])
{
  unsigned long threadId;
  int ind;
  int oldIdx;
  threadId = mwGetGlobalThreadIndex();
  ind = static_cast<int>(threadId % 720UL);
  oldIdx = static_cast<int>((threadId - static_cast<unsigned long>(ind)) / 720UL);
  if ((static_cast<int>(oldIdx < 1280)) && (static_cast<int>(ind < 720))) {
    img[(ind + 720 * oldIdx) + 1843200] = pln2[oldIdx + 1280 * ind];
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const int cameraDevice
//                double y2_data[1024]
// Return Type  : void
//
static __global__ __launch_bounds__(1024, 1) void detectFunction_kernel40(const
  int cameraDevice, double y2_data[1024])
{
  unsigned long idx;
  long loopEnd;
  unsigned long threadId;
  unsigned long threadStride;
  threadId = mwGetGlobalThreadIndex();
  threadStride = mwGetTotalThreadsLaunched();
  loopEnd = static_cast<long>(cameraDevice - 1);
  for (idx = threadId; idx <= static_cast<unsigned long>(loopEnd); idx +=
       threadStride) {
    int i;
    i = static_cast<int>(idx);
    if (y2_data[i] > 128.0) {
      y2_data[i] = 128.0;
    }
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double x1_data[1024]
//                const int initStatus
//                double bboxesX1Y1X2Y2_data[4096]
// Return Type  : void
//
static __global__ __launch_bounds__(1024, 1) void detectFunction_kernel41(const
  double x1_data[1024], const int initStatus, double bboxesX1Y1X2Y2_data[4096])
{
  unsigned long idx;
  long loopEnd;
  unsigned long threadId;
  unsigned long threadStride;
  threadId = mwGetGlobalThreadIndex();
  threadStride = mwGetTotalThreadsLaunched();
  loopEnd = static_cast<long>(initStatus);
  for (idx = threadId; idx <= static_cast<unsigned long>(loopEnd); idx +=
       threadStride) {
    int oldIdx;
    oldIdx = static_cast<int>(idx);
    bboxesX1Y1X2Y2_data[oldIdx] = x1_data[oldIdx];
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double y1_data[1024]
//                const int bboxesX1Y1X2Y2_size[2]
//                const int initStatus
//                double bboxesX1Y1X2Y2_data[4096]
// Return Type  : void
//
static __global__ __launch_bounds__(1024, 1) void detectFunction_kernel42(const
  double y1_data[1024], const int bboxesX1Y1X2Y2_size[2], const int initStatus,
  double bboxesX1Y1X2Y2_data[4096])
{
  unsigned long idx;
  long loopEnd;
  unsigned long threadId;
  unsigned long threadStride;
  threadId = mwGetGlobalThreadIndex();
  threadStride = mwGetTotalThreadsLaunched();
  loopEnd = static_cast<long>(initStatus);
  for (idx = threadId; idx <= static_cast<unsigned long>(loopEnd); idx +=
       threadStride) {
    int oldIdx;
    oldIdx = static_cast<int>(idx);
    bboxesX1Y1X2Y2_data[oldIdx + bboxesX1Y1X2Y2_size[0]] = y1_data[oldIdx];
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double x2_data[1024]
//                const int bboxesX1Y1X2Y2_size[2]
//                const int initStatus
//                double bboxesX1Y1X2Y2_data[4096]
// Return Type  : void
//
static __global__ __launch_bounds__(1024, 1) void detectFunction_kernel43(const
  double x2_data[1024], const int bboxesX1Y1X2Y2_size[2], const int initStatus,
  double bboxesX1Y1X2Y2_data[4096])
{
  unsigned long idx;
  long loopEnd;
  unsigned long threadId;
  unsigned long threadStride;
  threadId = mwGetGlobalThreadIndex();
  threadStride = mwGetTotalThreadsLaunched();
  loopEnd = static_cast<long>(initStatus);
  for (idx = threadId; idx <= static_cast<unsigned long>(loopEnd); idx +=
       threadStride) {
    int oldIdx;
    oldIdx = static_cast<int>(idx);
    bboxesX1Y1X2Y2_data[oldIdx + (bboxesX1Y1X2Y2_size[0] << 1)] = x2_data[oldIdx];
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double y2_data[1024]
//                const int bboxesX1Y1X2Y2_size[2]
//                const int initStatus
//                double bboxesX1Y1X2Y2_data[4096]
// Return Type  : void
//
static __global__ __launch_bounds__(1024, 1) void detectFunction_kernel44(const
  double y2_data[1024], const int bboxesX1Y1X2Y2_size[2], const int initStatus,
  double bboxesX1Y1X2Y2_data[4096])
{
  unsigned long idx;
  long loopEnd;
  unsigned long threadId;
  unsigned long threadStride;
  threadId = mwGetGlobalThreadIndex();
  threadStride = mwGetTotalThreadsLaunched();
  loopEnd = static_cast<long>(initStatus);
  for (idx = threadId; idx <= static_cast<unsigned long>(loopEnd); idx +=
       threadStride) {
    int oldIdx;
    oldIdx = static_cast<int>(idx);
    bboxesX1Y1X2Y2_data[oldIdx + bboxesX1Y1X2Y2_size[0] * 3] = y2_data[oldIdx];
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double bboxesX1Y1X2Y2_data[4096]
//                const int initStatus
//                double bboxPred_data[4096]
// Return Type  : void
//
static __global__ __launch_bounds__(1024, 1) void detectFunction_kernel45(const
  double bboxesX1Y1X2Y2_data[4096], const int initStatus, double bboxPred_data
  [4096])
{
  unsigned long idx;
  long loopEnd;
  unsigned long threadId;
  unsigned long threadStride;
  threadId = mwGetGlobalThreadIndex();
  threadStride = mwGetTotalThreadsLaunched();
  loopEnd = static_cast<long>(initStatus);
  for (idx = threadId; idx <= static_cast<unsigned long>(loopEnd); idx +=
       threadStride) {
    int oldIdx;
    oldIdx = static_cast<int>(idx);
    bboxPred_data[oldIdx] = ((bboxesX1Y1X2Y2_data[oldIdx] - 0.5) * 1.75 + -0.375)
      + 0.5;
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double bboxesX1Y1X2Y2_data[4096]
//                const int bboxesX1Y1X2Y2_size[2]
//                const int bboxPred_size[2]
//                const int initStatus
//                double bboxPred_data[4096]
// Return Type  : void
//
static __global__ __launch_bounds__(1024, 1) void detectFunction_kernel46(const
  double bboxesX1Y1X2Y2_data[4096], const int bboxesX1Y1X2Y2_size[2], const int
  bboxPred_size[2], const int initStatus, double bboxPred_data[4096])
{
  unsigned long idx;
  long loopEnd;
  unsigned long threadId;
  unsigned long threadStride;
  threadId = mwGetGlobalThreadIndex();
  threadStride = mwGetTotalThreadsLaunched();
  loopEnd = static_cast<long>(initStatus);
  for (idx = threadId; idx <= static_cast<unsigned long>(loopEnd); idx +=
       threadStride) {
    int oldIdx;
    oldIdx = static_cast<int>(idx);
    bboxPred_data[oldIdx + bboxPred_size[0]] = ((bboxesX1Y1X2Y2_data[oldIdx +
      bboxesX1Y1X2Y2_size[0]] - 0.5) * 1.75 + -0.375) + 0.5;
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double bboxesX1Y1X2Y2_data[4096]
//                const int bboxesX1Y1X2Y2_size[2]
//                const int bboxPred_size[2]
//                const int initStatus
//                double bboxPred_data[4096]
// Return Type  : void
//
static __global__ __launch_bounds__(1024, 1) void detectFunction_kernel47(const
  double bboxesX1Y1X2Y2_data[4096], const int bboxesX1Y1X2Y2_size[2], const int
  bboxPred_size[2], const int initStatus, double bboxPred_data[4096])
{
  unsigned long idx;
  long loopEnd;
  unsigned long threadId;
  unsigned long threadStride;
  threadId = mwGetGlobalThreadIndex();
  threadStride = mwGetTotalThreadsLaunched();
  loopEnd = static_cast<long>(initStatus);
  for (idx = threadId; idx <= static_cast<unsigned long>(loopEnd); idx +=
       threadStride) {
    int oldIdx;
    oldIdx = static_cast<int>(idx);
    bboxPred_data[oldIdx + (bboxPred_size[0] << 1)] =
      ((bboxesX1Y1X2Y2_data[oldIdx + (bboxesX1Y1X2Y2_size[0] << 1)] + 0.5) *
       1.75 + -0.375) - 0.5;
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double bboxesX1Y1X2Y2_data[4096]
//                const int bboxesX1Y1X2Y2_size[2]
//                const int bboxPred_size[2]
//                const int initStatus
//                double bboxPred_data[4096]
// Return Type  : void
//
static __global__ __launch_bounds__(1024, 1) void detectFunction_kernel48(const
  double bboxesX1Y1X2Y2_data[4096], const int bboxesX1Y1X2Y2_size[2], const int
  bboxPred_size[2], const int initStatus, double bboxPred_data[4096])
{
  unsigned long idx;
  long loopEnd;
  unsigned long threadId;
  unsigned long threadStride;
  threadId = mwGetGlobalThreadIndex();
  threadStride = mwGetTotalThreadsLaunched();
  loopEnd = static_cast<long>(initStatus);
  for (idx = threadId; idx <= static_cast<unsigned long>(loopEnd); idx +=
       threadStride) {
    int oldIdx;
    oldIdx = static_cast<int>(idx);
    bboxPred_data[oldIdx + bboxPred_size[0] * 3] = ((bboxesX1Y1X2Y2_data[oldIdx
      + bboxesX1Y1X2Y2_size[0] * 3] + 0.5) * 1.75 + -0.375) - 0.5;
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const int initStatus
//                double bboxPred_data[4096]
// Return Type  : void
//
static __global__ __launch_bounds__(1024, 1) void detectFunction_kernel49(const
  int initStatus, double bboxPred_data[4096])
{
  unsigned long idx;
  long loopEnd;
  unsigned long threadId;
  unsigned long threadStride;
  threadId = mwGetGlobalThreadIndex();
  threadStride = mwGetTotalThreadsLaunched();
  loopEnd = static_cast<long>(initStatus - 1);
  for (idx = threadId; idx <= static_cast<unsigned long>(loopEnd); idx +=
       threadStride) {
    int i;
    i = static_cast<int>(idx);
    bboxPred_data[i] = floor(bboxPred_data[i]);
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                short aux1[1440]
// Return Type  : void
//
static __global__ __launch_bounds__(512, 1) void detectFunction_kernel5(short
  aux1[1440])
{
  int i;
  i = static_cast<int>(mwGetGlobalThreadIndex());
  if (i < 1440) {
    if (i + 1 <= 720) {
      aux1[i] = static_cast<short>(i + 1);
    } else {
      aux1[i] = static_cast<short>(1440 - i);
    }
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double bboxPred_data[4096]
//                const int bboxPred_size[2]
//                const int k
//                double x1_data[1024]
// Return Type  : void
//
static __global__ __launch_bounds__(1024, 1) void detectFunction_kernel50(const
  double bboxPred_data[4096], const int bboxPred_size[2], const int k, double
  x1_data[1024])
{
  unsigned long idx;
  long loopEnd;
  unsigned long threadId;
  unsigned long threadStride;
  threadId = mwGetGlobalThreadIndex();
  threadStride = mwGetTotalThreadsLaunched();
  loopEnd = static_cast<long>(k);
  for (idx = threadId; idx <= static_cast<unsigned long>(loopEnd); idx +=
       threadStride) {
    int oldIdx;
    oldIdx = static_cast<int>(idx);
    x1_data[oldIdx] = (bboxPred_data[oldIdx + (bboxPred_size[0] << 1)] -
                       bboxPred_data[oldIdx]) + 1.0;
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double x1_data[1024]
//                const int bboxPred_size[2]
//                const int iv_size[1]
//                double bboxPred_data[4096]
// Return Type  : void
//
static __global__ __launch_bounds__(1024, 1) void detectFunction_kernel51(const
  double x1_data[1024], const int bboxPred_size[2], const int iv_size[1], double
  bboxPred_data[4096])
{
  unsigned long idx;
  long loopEnd;
  unsigned long threadId;
  unsigned long threadStride;
  threadId = mwGetGlobalThreadIndex();
  threadStride = mwGetTotalThreadsLaunched();
  loopEnd = static_cast<long>(iv_size[0] - 1);
  for (idx = threadId; idx <= static_cast<unsigned long>(loopEnd); idx +=
       threadStride) {
    int oldIdx;
    oldIdx = static_cast<int>(idx);
    bboxPred_data[oldIdx + (bboxPred_size[0] << 1)] = x1_data[oldIdx];
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double bboxPred_data[4096]
//                const int bboxPred_size[2]
//                const int k
//                double x1_data[1024]
// Return Type  : void
//
static __global__ __launch_bounds__(1024, 1) void detectFunction_kernel52(const
  double bboxPred_data[4096], const int bboxPred_size[2], const int k, double
  x1_data[1024])
{
  unsigned long idx;
  long loopEnd;
  unsigned long threadId;
  unsigned long threadStride;
  threadId = mwGetGlobalThreadIndex();
  threadStride = mwGetTotalThreadsLaunched();
  loopEnd = static_cast<long>(k);
  for (idx = threadId; idx <= static_cast<unsigned long>(loopEnd); idx +=
       threadStride) {
    int oldIdx;
    oldIdx = static_cast<int>(idx);
    x1_data[oldIdx] = (bboxPred_data[oldIdx + bboxPred_size[0] * 3] -
                       bboxPred_data[oldIdx + bboxPred_size[0]]) + 1.0;
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double x1_data[1024]
//                const int bboxPred_size[2]
//                const int iv_size[1]
//                double bboxPred_data[4096]
// Return Type  : void
//
static __global__ __launch_bounds__(1024, 1) void detectFunction_kernel53(const
  double x1_data[1024], const int bboxPred_size[2], const int iv_size[1], double
  bboxPred_data[4096])
{
  unsigned long idx;
  long loopEnd;
  unsigned long threadId;
  unsigned long threadStride;
  threadId = mwGetGlobalThreadIndex();
  threadStride = mwGetTotalThreadsLaunched();
  loopEnd = static_cast<long>(iv_size[0] - 1);
  for (idx = threadId; idx <= static_cast<unsigned long>(loopEnd); idx +=
       threadStride) {
    int oldIdx;
    oldIdx = static_cast<int>(idx);
    bboxPred_data[oldIdx + bboxPred_size[0] * 3] = x1_data[oldIdx];
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double bboxPred_data[4096]
//                const int bboxPred_size[2]
//                const int initStatus
//                const int b_bboxPred_size[2]
//                const int *camIndex
//                double b_bboxPred_data[4096]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void detectFunction_kernel54(const
  double bboxPred_data[4096], const int bboxPred_size[2], const int initStatus,
  const int b_bboxPred_size[2], const int *camIndex, double b_bboxPred_data[4096])
{
  int oldIdx;
  oldIdx = static_cast<int>(mwGetGlobalThreadIndex());
  if (oldIdx < 4) {
    b_bboxPred_data[*camIndex + b_bboxPred_size[0] * oldIdx] =
      bboxPred_data[initStatus + bboxPred_size[0] * oldIdx];
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const float thresholdedPrediction_data[6144]
//                const int thresholdedPrediction_size[2]
//                const int initStatus
//                const int count
//                float classPred_data[1024]
//                float scorePred_data[1024]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void detectFunction_kernel55(const
  float thresholdedPrediction_data[6144], const int thresholdedPrediction_size[2],
  const int initStatus, const int count, float classPred_data[1024], float
  scorePred_data[1024])
{
  int tmpIdx;
  tmpIdx = static_cast<int>(mwGetGlobalThreadIndex());
  if (tmpIdx < 1) {
    scorePred_data[count - 1] = thresholdedPrediction_data[initStatus +
      (thresholdedPrediction_size[0] << 2)];
    classPred_data[count - 1] = thresholdedPrediction_data[initStatus +
      thresholdedPrediction_size[0] * 5];
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const short outVal
//                const short i
//                short iv_data[1024]
// Return Type  : void
//
static __global__ __launch_bounds__(1024, 1) void detectFunction_kernel56(const
  short outVal, const short i, short iv_data[1024])
{
  unsigned long idx;
  long loopEnd;
  unsigned long threadId;
  unsigned long threadStride;
  threadId = mwGetGlobalThreadIndex();
  threadStride = mwGetTotalThreadsLaunched();
  loopEnd = static_cast<long>(static_cast<int>(i) - static_cast<int>(outVal));
  for (idx = threadId; idx <= static_cast<unsigned long>(loopEnd); idx +=
       threadStride) {
    int oldIdx;
    oldIdx = static_cast<int>(idx);
    iv_data[oldIdx] = static_cast<short>(static_cast<int>(outVal) + static_cast<
      int>(static_cast<short>(oldIdx)));
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const int bboxPred_size[2]
//                bool b_data[1024]
// Return Type  : void
//
static __global__ __launch_bounds__(1024, 1) void detectFunction_kernel57(const
  int bboxPred_size[2], bool b_data[1024])
{
  unsigned long idx;
  long loopEnd;
  unsigned long threadId;
  unsigned long threadStride;
  threadId = mwGetGlobalThreadIndex();
  threadStride = mwGetTotalThreadsLaunched();
  loopEnd = static_cast<long>(bboxPred_size[0] - 1);
  for (idx = threadId; idx <= static_cast<unsigned long>(loopEnd); idx +=
       threadStride) {
    int oldIdx;
    oldIdx = static_cast<int>(idx);
    b_data[oldIdx] = false;
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const bool b_data[1024]
//                int initStatus
//                int *camIndex
// Return Type  : void
//
static __global__ __launch_bounds__(1024, 1) void detectFunction_kernel58(const
  bool b_data[1024], int initStatus, int *camIndex)
{
  long loopEnd;
  unsigned int blockStride;
  unsigned int idx;
  unsigned int m;
  unsigned int thBlkId;
  unsigned int threadId;
  unsigned int threadStride;
  int tmpRed0;
  threadStride = static_cast<unsigned int>(mwGetTotalThreadsLaunched());
  threadId = static_cast<unsigned int>(mwGetGlobalThreadIndex());
  thBlkId = static_cast<unsigned int>(mwGetThreadIndexWithinBlock());
  blockStride = static_cast<unsigned int>(mwGetThreadsPerBlock());
  tmpRed0 = 0;
  loopEnd = static_cast<long>(initStatus - 1);
  if (mwIsLastBlock()) {
    m = (static_cast<long>(initStatus - 1) + 1L) % static_cast<long>(blockStride);
    if (m > 0U) {
      blockStride = m;
    }
  }

  blockStride = (unsigned int)(blockStride + (static_cast<long>(warpSize) - 1L))
    / warpSize;
  if (static_cast<long>(threadId) <= loopEnd) {
    tmpRed0 = static_cast<int>(b_data[threadId]);
  }

  m = __ballot_sync(MAX_uint32_T, static_cast<long>(threadId) <= loopEnd);
  for (idx = threadId + threadStride; idx <= static_cast<unsigned int>(loopEnd);
       idx += threadStride) {
    tmpRed0 += static_cast<int>(b_data[idx]);
  }

  tmpRed0 = workGroupReduction(tmpRed0, m, blockStride);
  if (thBlkId == 0U) {
    atomicAdd(&camIndex[0], tmpRed0);
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const int bboxPred_size[2]
//                const int initStatus
//                const short iv_data[1024]
//                double bboxPred_data[4096]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void detectFunction_kernel59(const
  int bboxPred_size[2], const int initStatus, const short iv_data[1024], double
  bboxPred_data[4096])
{
  int rowIdx;
  rowIdx = static_cast<int>(mwGetGlobalThreadIndex());
  if (rowIdx < 4) {
    int oldIdx;
    oldIdx = static_cast<int>(iv_data[0]);
    for (int ind = 0; ind <= initStatus - oldIdx; ind++) {
      int i;
      i = oldIdx + ind;
      bboxPred_data[(i + bboxPred_size[0] * rowIdx) - 1] = bboxPred_data[i +
        bboxPred_size[0] * rowIdx];
    }
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                short aux2[2560]
// Return Type  : void
//
static __global__ __launch_bounds__(512, 1) void detectFunction_kernel6(short
  aux2[2560])
{
  int i;
  i = static_cast<int>(mwGetGlobalThreadIndex());
  if (i < 2560) {
    if (i + 1 <= 1280) {
      aux2[i] = static_cast<short>(i + 1);
    } else {
      aux2[i] = static_cast<short>(2560 - i);
    }
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double bboxPred_data[4096]
//                const int bboxPred_size[2]
//                const int b_bboxPred_size[2]
//                const int initStatus
//                double b_bboxPred_data[4096]
// Return Type  : void
//
static __global__ __launch_bounds__(1024, 1) void detectFunction_kernel60(const
  double bboxPred_data[4096], const int bboxPred_size[2], const int
  b_bboxPred_size[2], const int initStatus, double b_bboxPred_data[4096])
{
  unsigned long idx;
  long loopEnd;
  unsigned long threadId;
  unsigned long threadStride;
  threadId = mwGetGlobalThreadIndex();
  threadStride = mwGetTotalThreadsLaunched();
  loopEnd = (static_cast<long>(initStatus) + 1L) * 4L - 1L;
  for (idx = threadId; idx <= static_cast<unsigned long>(loopEnd); idx +=
       threadStride) {
    int ind;
    int oldIdx;
    ind = static_cast<int>(idx % (static_cast<unsigned long>(initStatus) + 1UL));
    oldIdx = static_cast<int>((idx - static_cast<unsigned long>(ind)) / (
      static_cast<unsigned long>(initStatus) + 1UL));
    b_bboxPred_data[ind + b_bboxPred_size[0] * oldIdx] = bboxPred_data[ind +
      bboxPred_size[0] * oldIdx];
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double bboxPred_data[4096]
//                const int bboxPred_size[2]
//                double b_bboxPred_data[4096]
// Return Type  : void
//
static __global__ __launch_bounds__(1024, 1) void detectFunction_kernel61(const
  double bboxPred_data[4096], const int bboxPred_size[2], double
  b_bboxPred_data[4096])
{
  unsigned long idx;
  long loopEnd;
  unsigned long threadId;
  unsigned long threadStride;
  threadId = mwGetGlobalThreadIndex();
  threadStride = mwGetTotalThreadsLaunched();
  loopEnd = static_cast<long>(bboxPred_size[0] * 4 - 1);
  for (idx = threadId; idx <= static_cast<unsigned long>(loopEnd); idx +=
       threadStride) {
    int oldIdx;
    oldIdx = static_cast<int>(idx);
    b_bboxPred_data[oldIdx] = bboxPred_data[oldIdx];
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const short outVal
//                const short i
//                short iv_data[1024]
// Return Type  : void
//
static __global__ __launch_bounds__(1024, 1) void detectFunction_kernel62(const
  short outVal, const short i, short iv_data[1024])
{
  unsigned long idx;
  long loopEnd;
  unsigned long threadId;
  unsigned long threadStride;
  threadId = mwGetGlobalThreadIndex();
  threadStride = mwGetTotalThreadsLaunched();
  loopEnd = static_cast<long>(static_cast<int>(i) - static_cast<int>(outVal));
  for (idx = threadId; idx <= static_cast<unsigned long>(loopEnd); idx +=
       threadStride) {
    int oldIdx;
    oldIdx = static_cast<int>(idx);
    iv_data[oldIdx] = static_cast<short>(static_cast<int>(outVal) + static_cast<
      int>(static_cast<short>(oldIdx)));
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const int scorePred_size[1]
//                bool b_data[1024]
// Return Type  : void
//
static __global__ __launch_bounds__(1024, 1) void detectFunction_kernel63(const
  int scorePred_size[1], bool b_data[1024])
{
  unsigned long idx;
  long loopEnd;
  unsigned long threadId;
  unsigned long threadStride;
  threadId = mwGetGlobalThreadIndex();
  threadStride = mwGetTotalThreadsLaunched();
  loopEnd = static_cast<long>(scorePred_size[0] - 1);
  for (idx = threadId; idx <= static_cast<unsigned long>(loopEnd); idx +=
       threadStride) {
    int oldIdx;
    oldIdx = static_cast<int>(idx);
    b_data[oldIdx] = false;
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const bool b_data[1024]
//                int initStatus
//                int *camIndex
// Return Type  : void
//
static __global__ __launch_bounds__(1024, 1) void detectFunction_kernel64(const
  bool b_data[1024], int initStatus, int *camIndex)
{
  long loopEnd;
  unsigned int blockStride;
  unsigned int idx;
  unsigned int m;
  unsigned int thBlkId;
  unsigned int threadId;
  unsigned int threadStride;
  int tmpRed0;
  threadStride = static_cast<unsigned int>(mwGetTotalThreadsLaunched());
  threadId = static_cast<unsigned int>(mwGetGlobalThreadIndex());
  thBlkId = static_cast<unsigned int>(mwGetThreadIndexWithinBlock());
  blockStride = static_cast<unsigned int>(mwGetThreadsPerBlock());
  tmpRed0 = 0;
  loopEnd = static_cast<long>(initStatus - 1);
  if (mwIsLastBlock()) {
    m = (static_cast<long>(initStatus - 1) + 1L) % static_cast<long>(blockStride);
    if (m > 0U) {
      blockStride = m;
    }
  }

  blockStride = (unsigned int)(blockStride + (static_cast<long>(warpSize) - 1L))
    / warpSize;
  if (static_cast<long>(threadId) <= loopEnd) {
    tmpRed0 = static_cast<int>(b_data[threadId]);
  }

  m = __ballot_sync(MAX_uint32_T, static_cast<long>(threadId) <= loopEnd);
  for (idx = threadId + threadStride; idx <= static_cast<unsigned int>(loopEnd);
       idx += threadStride) {
    tmpRed0 += static_cast<int>(b_data[idx]);
  }

  tmpRed0 = workGroupReduction(tmpRed0, m, blockStride);
  if (thBlkId == 0U) {
    atomicAdd(&camIndex[0], tmpRed0);
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const short outVal
//                const short i
//                short iv_data[1024]
// Return Type  : void
//
static __global__ __launch_bounds__(1024, 1) void detectFunction_kernel65(const
  short outVal, const short i, short iv_data[1024])
{
  unsigned long idx;
  long loopEnd;
  unsigned long threadId;
  unsigned long threadStride;
  threadId = mwGetGlobalThreadIndex();
  threadStride = mwGetTotalThreadsLaunched();
  loopEnd = static_cast<long>(static_cast<int>(i) - static_cast<int>(outVal));
  for (idx = threadId; idx <= static_cast<unsigned long>(loopEnd); idx +=
       threadStride) {
    int oldIdx;
    oldIdx = static_cast<int>(idx);
    iv_data[oldIdx] = static_cast<short>(static_cast<int>(outVal) + static_cast<
      int>(static_cast<short>(oldIdx)));
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const int iv_size[1]
//                bool b_data[1024]
// Return Type  : void
//
static __global__ __launch_bounds__(1024, 1) void detectFunction_kernel66(const
  int iv_size[1], bool b_data[1024])
{
  unsigned long idx;
  long loopEnd;
  unsigned long threadId;
  unsigned long threadStride;
  threadId = mwGetGlobalThreadIndex();
  threadStride = mwGetTotalThreadsLaunched();
  loopEnd = static_cast<long>(iv_size[0] - 1);
  for (idx = threadId; idx <= static_cast<unsigned long>(loopEnd); idx +=
       threadStride) {
    int oldIdx;
    oldIdx = static_cast<int>(idx);
    b_data[oldIdx] = false;
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const bool b_data[1024]
//                int initStatus
//                int *camIndex
// Return Type  : void
//
static __global__ __launch_bounds__(1024, 1) void detectFunction_kernel67(const
  bool b_data[1024], int initStatus, int *camIndex)
{
  long loopEnd;
  unsigned int blockStride;
  unsigned int idx;
  unsigned int m;
  unsigned int thBlkId;
  unsigned int threadId;
  unsigned int threadStride;
  int tmpRed0;
  threadStride = static_cast<unsigned int>(mwGetTotalThreadsLaunched());
  threadId = static_cast<unsigned int>(mwGetGlobalThreadIndex());
  thBlkId = static_cast<unsigned int>(mwGetThreadIndexWithinBlock());
  blockStride = static_cast<unsigned int>(mwGetThreadsPerBlock());
  tmpRed0 = 0;
  loopEnd = static_cast<long>(initStatus - 1);
  if (mwIsLastBlock()) {
    m = (static_cast<long>(initStatus - 1) + 1L) % static_cast<long>(blockStride);
    if (m > 0U) {
      blockStride = m;
    }
  }

  blockStride = (unsigned int)(blockStride + (static_cast<long>(warpSize) - 1L))
    / warpSize;
  if (static_cast<long>(threadId) <= loopEnd) {
    tmpRed0 = static_cast<int>(b_data[threadId]);
  }

  m = __ballot_sync(MAX_uint32_T, static_cast<long>(threadId) <= loopEnd);
  for (idx = threadId + threadStride; idx <= static_cast<unsigned int>(loopEnd);
       idx += threadStride) {
    tmpRed0 += static_cast<int>(b_data[idx]);
  }

  tmpRed0 = workGroupReduction(tmpRed0, m, blockStride);
  if (thBlkId == 0U) {
    atomicAdd(&camIndex[0], tmpRed0);
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const float classPred_data[1024]
//                const int iv_size[1]
//                double y1_data[1024]
// Return Type  : void
//
static __global__ __launch_bounds__(1024, 1) void detectFunction_kernel68(const
  float classPred_data[1024], const int iv_size[1], double y1_data[1024])
{
  unsigned long idx;
  long loopEnd;
  unsigned long threadId;
  unsigned long threadStride;
  threadId = mwGetGlobalThreadIndex();
  threadStride = mwGetTotalThreadsLaunched();
  loopEnd = static_cast<long>(iv_size[0] - 1);
  for (idx = threadId; idx <= static_cast<unsigned long>(loopEnd); idx +=
       threadStride) {
    int oldIdx;
    oldIdx = static_cast<int>(idx);
    y1_data[oldIdx] = static_cast<double>(classPred_data[oldIdx]);
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const float scorePred_data[1024]
//                const int scorePred_size[1]
//                float classPred_data[1024]
// Return Type  : void
//
static __global__ __launch_bounds__(1024, 1) void detectFunction_kernel69(const
  float scorePred_data[1024], const int scorePred_size[1], float classPred_data
  [1024])
{
  unsigned long idx;
  long loopEnd;
  unsigned long threadId;
  unsigned long threadStride;
  threadId = mwGetGlobalThreadIndex();
  threadStride = mwGetTotalThreadsLaunched();
  loopEnd = static_cast<long>(scorePred_size[0] - 1);
  for (idx = threadId; idx <= static_cast<unsigned long>(loopEnd); idx +=
       threadStride) {
    int oldIdx;
    oldIdx = static_cast<int>(idx);
    classPred_data[oldIdx] = scorePred_data[oldIdx];
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const short aux1[1440]
//                short ipRowIndices[2912]
//                double rowWeights[2912]
// Return Type  : void
//
static __global__ __launch_bounds__(512, 1) void detectFunction_kernel7(const
  short aux1[1440], short ipRowIndices[2912], double rowWeights[2912])
{
  unsigned long threadId;
  int i;
  int ind;
  int rowIdx;
  threadId = mwGetGlobalThreadIndex();
  i = static_cast<int>(threadId % 13UL);
  rowIdx = static_cast<int>((threadId - static_cast<unsigned long>(i)) / 13UL);
  if ((static_cast<int>(rowIdx < 224)) && (static_cast<int>(i < 13))) {
    double absx;
    double absx2;
    double sumVal;
    int oldIdx;
    sumVal = (static_cast<double>(rowIdx) + 1.0) / 0.31111111111111112 +
      -1.1071428571428572;
    oldIdx = static_cast<int>(floor(sumVal - 6.4285714285714288));
    absx = fabs(0.31111111111111112 * (sumVal - (static_cast<double>(oldIdx + i)
      + 1.0)));
    absx2 = absx * absx;
    sumVal = rt_powd_snf_device(absx, 3.0);
    rowWeights[rowIdx + 224 * i] = 0.31111111111111112 * (((1.5 * sumVal - 2.5 *
      absx2) + 1.0) * static_cast<double>(absx <= 1.0) + (((-0.5 * sumVal + 2.5 *
      absx2) - 4.0 * absx) + 2.0) * static_cast<double>((static_cast<int>(1.0 <
      absx)) && (static_cast<int>(absx <= 2.0))));
    oldIdx = (oldIdx + i) + 1;
    if (oldIdx - 1 == 0) {
      ind = 0;
    } else {
      ind = static_cast<int>(fmod(static_cast<double>(oldIdx) - 1.0, 1440.0));
      if ((static_cast<int>(ind != 0)) && (static_cast<int>(oldIdx - 1 < 0))) {
        ind += 1440;
      }
    }

    ipRowIndices[rowIdx + 224 * i] = aux1[ind];
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const short dv2[2]
//                double idx_data[1024]
// Return Type  : void
//
static __global__ __launch_bounds__(1024, 1) void detectFunction_kernel70(const
  short dv2[2], double idx_data[1024])
{
  unsigned long idx;
  long loopEnd;
  unsigned long threadId;
  unsigned long threadStride;
  threadId = mwGetGlobalThreadIndex();
  threadStride = mwGetTotalThreadsLaunched();
  loopEnd = static_cast<long>(static_cast<int>(dv2[0]) - 1);
  for (idx = threadId; idx <= static_cast<unsigned long>(loopEnd); idx +=
       threadStride) {
    int oldIdx;
    oldIdx = static_cast<int>(idx);
    idx_data[oldIdx] = 0.0;
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double bboxPred_data[4096]
//                const int bboxPred_size[2]
//                const double idx_data[1024]
//                const int inputBbox_size[2]
//                const int idx_size[1]
//                double bboxesX1Y1X2Y2_data[4096]
// Return Type  : void
//
static __global__ __launch_bounds__(1024, 1) void detectFunction_kernel71(const
  double bboxPred_data[4096], const int bboxPred_size[2], const double idx_data
  [1024], const int inputBbox_size[2], const int idx_size[1], double
  bboxesX1Y1X2Y2_data[4096])
{
  unsigned long idx;
  long loopEnd;
  unsigned long threadId;
  unsigned long threadStride;
  threadId = mwGetGlobalThreadIndex();
  threadStride = mwGetTotalThreadsLaunched();
  loopEnd = (static_cast<long>(idx_size[0] - 1) + 1L) * 4L - 1L;
  for (idx = threadId; idx <= static_cast<unsigned long>(loopEnd); idx +=
       threadStride) {
    int ind;
    int oldIdx;
    ind = static_cast<int>(idx % (static_cast<unsigned long>(idx_size[0] - 1) +
      1UL));
    oldIdx = static_cast<int>((idx - static_cast<unsigned long>(ind)) / (
      static_cast<unsigned long>(idx_size[0] - 1) + 1UL));
    bboxesX1Y1X2Y2_data[ind + inputBbox_size[0] * oldIdx] = bboxPred_data[(
      static_cast<int>(idx_data[ind]) + bboxPred_size[0] * oldIdx) - 1];
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double y1_data[1024]
//                const double idx_data[1024]
//                const int idx_size[1]
//                double x1_data[1024]
// Return Type  : void
//
static __global__ __launch_bounds__(1024, 1) void detectFunction_kernel72(const
  double y1_data[1024], const double idx_data[1024], const int idx_size[1],
  double x1_data[1024])
{
  unsigned long idx;
  long loopEnd;
  unsigned long threadId;
  unsigned long threadStride;
  threadId = mwGetGlobalThreadIndex();
  threadStride = mwGetTotalThreadsLaunched();
  loopEnd = static_cast<long>(idx_size[0] - 1);
  for (idx = threadId; idx <= static_cast<unsigned long>(loopEnd); idx +=
       threadStride) {
    int oldIdx;
    oldIdx = static_cast<int>(idx);
    x1_data[oldIdx] = y1_data[static_cast<int>(idx_data[oldIdx]) - 1];
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double x1_data[1024]
//                const int iv_size[1]
//                double y1_data[1024]
// Return Type  : void
//
static __global__ __launch_bounds__(1024, 1) void detectFunction_kernel73(const
  double x1_data[1024], const int iv_size[1], double y1_data[1024])
{
  unsigned long idx;
  long loopEnd;
  unsigned long threadId;
  unsigned long threadStride;
  threadId = mwGetGlobalThreadIndex();
  threadStride = mwGetTotalThreadsLaunched();
  loopEnd = static_cast<long>(iv_size[0] - 1);
  for (idx = threadId; idx <= static_cast<unsigned long>(loopEnd); idx +=
       threadStride) {
    int oldIdx;
    oldIdx = static_cast<int>(idx);
    y1_data[oldIdx] = x1_data[oldIdx];
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const int idx_size[1]
//                bool b_data[1024]
// Return Type  : void
//
static __global__ __launch_bounds__(1024, 1) void detectFunction_kernel74(const
  int idx_size[1], bool b_data[1024])
{
  unsigned long idx;
  long loopEnd;
  unsigned long threadId;
  unsigned long threadStride;
  threadId = mwGetGlobalThreadIndex();
  threadStride = mwGetTotalThreadsLaunched();
  loopEnd = static_cast<long>(idx_size[0] - 1);
  for (idx = threadId; idx <= static_cast<unsigned long>(loopEnd); idx +=
       threadStride) {
    int oldIdx;
    oldIdx = static_cast<int>(idx);
    b_data[oldIdx] = true;
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double bboxesX1Y1X2Y2_data[4096]
//                const int inputBbox_size[2]
//                const int k
//                double x1_data[1024]
// Return Type  : void
//
static __global__ __launch_bounds__(1024, 1) void detectFunction_kernel75(const
  double bboxesX1Y1X2Y2_data[4096], const int inputBbox_size[2], const int k,
  double x1_data[1024])
{
  unsigned long idx;
  long loopEnd;
  unsigned long threadId;
  unsigned long threadStride;
  threadId = mwGetGlobalThreadIndex();
  threadStride = mwGetTotalThreadsLaunched();
  loopEnd = static_cast<long>(k);
  for (idx = threadId; idx <= static_cast<unsigned long>(loopEnd); idx +=
       threadStride) {
    int oldIdx;
    oldIdx = static_cast<int>(idx);
    x1_data[oldIdx] = bboxesX1Y1X2Y2_data[oldIdx + (inputBbox_size[0] << 1)] *
      bboxesX1Y1X2Y2_data[oldIdx + inputBbox_size[0] * 3];
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const int inputBbox_size[2]
//                const double bboxesX1Y1X2Y2_data[4096]
//                const int k
//                double x2_data[1024]
// Return Type  : void
//
static __global__ __launch_bounds__(1024, 1) void detectFunction_kernel76(const
  int inputBbox_size[2], const double bboxesX1Y1X2Y2_data[4096], const int k,
  double x2_data[1024])
{
  unsigned long idx;
  long loopEnd;
  unsigned long threadId;
  unsigned long threadStride;
  threadId = mwGetGlobalThreadIndex();
  threadStride = mwGetTotalThreadsLaunched();
  loopEnd = static_cast<long>(k);
  for (idx = threadId; idx <= static_cast<unsigned long>(loopEnd); idx +=
       threadStride) {
    int oldIdx;
    oldIdx = static_cast<int>(idx);
    x2_data[oldIdx] = bboxesX1Y1X2Y2_data[oldIdx] + bboxesX1Y1X2Y2_data[oldIdx +
      (inputBbox_size[0] << 1)];
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double bboxesX1Y1X2Y2_data[4096]
//                const int inputBbox_size[2]
//                const int k
//                double y2_data[1024]
// Return Type  : void
//
static __global__ __launch_bounds__(1024, 1) void detectFunction_kernel77(const
  double bboxesX1Y1X2Y2_data[4096], const int inputBbox_size[2], const int k,
  double y2_data[1024])
{
  unsigned long idx;
  long loopEnd;
  unsigned long threadId;
  unsigned long threadStride;
  threadId = mwGetGlobalThreadIndex();
  threadStride = mwGetTotalThreadsLaunched();
  loopEnd = static_cast<long>(k);
  for (idx = threadId; idx <= static_cast<unsigned long>(loopEnd); idx +=
       threadStride) {
    int oldIdx;
    oldIdx = static_cast<int>(idx);
    y2_data[oldIdx] = bboxesX1Y1X2Y2_data[oldIdx + inputBbox_size[0]] +
      bboxesX1Y1X2Y2_data[oldIdx + inputBbox_size[0] * 3];
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const int *camIndex
//                const int thresholdedPrediction_size[2]
//                bool b_data[1024]
// Return Type  : void
//
static __global__ __launch_bounds__(1024, 1) void detectFunction_kernel78(const
  int *camIndex, const int thresholdedPrediction_size[2], bool b_data[1024])
{
  unsigned long idx;
  long loopEnd;
  unsigned long threadId;
  unsigned long threadStride;
  threadId = mwGetGlobalThreadIndex();
  threadStride = mwGetTotalThreadsLaunched();
  loopEnd = static_cast<long>(thresholdedPrediction_size[1] - 1);
  for (idx = threadId; idx <= static_cast<unsigned long>(loopEnd); idx +=
       threadStride) {
    int oldIdx;
    oldIdx = static_cast<int>(idx);
    b_data[*camIndex + oldIdx] = false;
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const bool b_data[1024]
//                const double idx_data[1024]
//                const int selectedIndex_size[1]
//                bool index_data[1024]
// Return Type  : void
//
static __global__ __launch_bounds__(1024, 1) void detectFunction_kernel79(const
  bool b_data[1024], const double idx_data[1024], const int selectedIndex_size[1],
  bool index_data[1024])
{
  unsigned long idx;
  long loopEnd;
  unsigned long threadId;
  unsigned long threadStride;
  threadId = mwGetGlobalThreadIndex();
  threadStride = mwGetTotalThreadsLaunched();
  loopEnd = static_cast<long>(selectedIndex_size[0] - 1);
  for (idx = threadId; idx <= static_cast<unsigned long>(loopEnd); idx +=
       threadStride) {
    int oldIdx;
    oldIdx = static_cast<int>(idx);
    index_data[static_cast<int>(idx_data[oldIdx]) - 1] = b_data[oldIdx];
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const short aux2[2560]
//                short ipColIndices[5152]
//                double colWeights[5152]
// Return Type  : void
//
static __global__ __launch_bounds__(512, 1) void detectFunction_kernel8(const
  short aux2[2560], short ipColIndices[5152], double colWeights[5152])
{
  unsigned long threadId;
  int colIdx;
  int i;
  int ind;
  threadId = mwGetGlobalThreadIndex();
  i = static_cast<int>(threadId % 23UL);
  colIdx = static_cast<int>((threadId - static_cast<unsigned long>(i)) / 23UL);
  if ((static_cast<int>(colIdx < 224)) && (static_cast<int>(i < 23))) {
    double absx;
    double absx2;
    double sumVal;
    int oldIdx;
    sumVal = (static_cast<double>(colIdx) + 1.0) / 0.175 + -2.3571428571428572;
    oldIdx = static_cast<int>(floor(sumVal - 11.428571428571429));
    absx = fabs(0.175 * (sumVal - (static_cast<double>(oldIdx + i) + 1.0)));
    absx2 = absx * absx;
    sumVal = rt_powd_snf_device(absx, 3.0);
    colWeights[colIdx + 224 * i] = 0.175 * (((1.5 * sumVal - 2.5 * absx2) + 1.0)
      * static_cast<double>(absx <= 1.0) + (((-0.5 * sumVal + 2.5 * absx2) - 4.0
      * absx) + 2.0) * static_cast<double>((static_cast<int>(1.0 < absx)) && (
      static_cast<int>(absx <= 2.0))));
    oldIdx = (oldIdx + i) + 1;
    if (oldIdx - 1 == 0) {
      ind = 0;
    } else {
      ind = static_cast<int>(fmod(static_cast<double>(oldIdx) - 1.0, 2560.0));
      if ((static_cast<int>(ind != 0)) && (static_cast<int>(oldIdx - 1 < 0))) {
        ind += 2560;
      }
    }

    ipColIndices[colIdx + 224 * i] = aux2[ind];
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double bboxPred_data[4096]
//                const int bboxPred_size[2]
//                const short iv2_data[1024]
//                const int b_bboxPred_size[2]
//                const int iv_size[1]
//                double b_bboxPred_data[4096]
// Return Type  : void
//
static __global__ __launch_bounds__(1024, 1) void detectFunction_kernel80(const
  double bboxPred_data[4096], const int bboxPred_size[2], const short iv2_data
  [1024], const int b_bboxPred_size[2], const int iv_size[1], double
  b_bboxPred_data[4096])
{
  unsigned long idx;
  long loopEnd;
  unsigned long threadId;
  unsigned long threadStride;
  threadId = mwGetGlobalThreadIndex();
  threadStride = mwGetTotalThreadsLaunched();
  loopEnd = (static_cast<long>(iv_size[0] - 1) + 1L) * 4L - 1L;
  for (idx = threadId; idx <= static_cast<unsigned long>(loopEnd); idx +=
       threadStride) {
    int ind;
    int oldIdx;
    ind = static_cast<int>(idx % (static_cast<unsigned long>(iv_size[0] - 1) +
      1UL));
    oldIdx = static_cast<int>((idx - static_cast<unsigned long>(ind)) / (
      static_cast<unsigned long>(iv_size[0] - 1) + 1UL));
    b_bboxPred_data[ind + b_bboxPred_size[0] * oldIdx] = bboxPred_data[(
      static_cast<int>(iv2_data[ind]) + bboxPred_size[0] * oldIdx) - 1];
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double bboxPred_data[4096]
//                const int bboxPred_size[2]
//                double b_bboxPred_data[4096]
// Return Type  : void
//
static __global__ __launch_bounds__(1024, 1) void detectFunction_kernel81(const
  double bboxPred_data[4096], const int bboxPred_size[2], double
  b_bboxPred_data[4096])
{
  unsigned long idx;
  long loopEnd;
  unsigned long threadId;
  unsigned long threadStride;
  threadId = mwGetGlobalThreadIndex();
  threadStride = mwGetTotalThreadsLaunched();
  loopEnd = static_cast<long>(bboxPred_size[0] * 4 - 1);
  for (idx = threadId; idx <= static_cast<unsigned long>(loopEnd); idx +=
       threadStride) {
    int oldIdx;
    oldIdx = static_cast<int>(idx);
    b_bboxPred_data[oldIdx] = bboxPred_data[oldIdx];
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const float scorePred_data[1024]
//                const int scorePred_size[1]
//                float scores_data[1024]
// Return Type  : void
//
static __global__ __launch_bounds__(1024, 1) void detectFunction_kernel82(const
  float scorePred_data[1024], const int scorePred_size[1], float scores_data
  [1024])
{
  unsigned long idx;
  long loopEnd;
  unsigned long threadId;
  unsigned long threadStride;
  threadId = mwGetGlobalThreadIndex();
  threadStride = mwGetTotalThreadsLaunched();
  loopEnd = static_cast<long>(scorePred_size[0] - 1);
  for (idx = threadId; idx <= static_cast<unsigned long>(loopEnd); idx +=
       threadStride) {
    int oldIdx;
    oldIdx = static_cast<int>(idx);
    scores_data[oldIdx] = scorePred_data[oldIdx];
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double rowWeights[2912]
//                double rowWeightsTotal[224]
// Return Type  : void
//
static __global__ __launch_bounds__(224, 1) void detectFunction_kernel9(const
  double rowWeights[2912], double rowWeightsTotal[224])
{
  int rowIdx;
  rowIdx = static_cast<int>(mwGetGlobalThreadIndex());
  if (rowIdx < 224) {
    rowWeightsTotal[rowIdx] = rowWeights[rowIdx];
  }
}

//
// Arguments    : double u0
//                double u1
// Return Type  : double
//
static __device__ double rt_powd_snf_device(double u0, double u1)
{
  double b_y;
  if ((static_cast<int>(isnan(u0))) || (static_cast<int>(isnan(u1)))) {
    b_y = CUDART_NAN;
  } else {
    double d;
    double d1;
    d = fabs(u0);
    d1 = fabs(u1);
    if (isinf(u1)) {
      if (d == 1.0) {
        b_y = 1.0;
      } else if (d > 1.0) {
        if (u1 > 0.0) {
          b_y = CUDART_INF;
        } else {
          b_y = 0.0;
        }
      } else if (u1 > 0.0) {
        b_y = 0.0;
      } else {
        b_y = CUDART_INF;
      }
    } else if (d1 == 0.0) {
      b_y = 1.0;
    } else if (d1 == 1.0) {
      if (u1 > 0.0) {
        b_y = u0;
      } else {
        b_y = 1.0 / u0;
      }
    } else if (u1 == 2.0) {
      b_y = u0 * u0;
    } else if ((static_cast<int>(u1 == 0.5)) && (static_cast<int>(u0 >= 0.0))) {
      b_y = sqrt(u0);
    } else if ((static_cast<int>(u0 < 0.0)) && (static_cast<int>(u1 > floor(u1))))
    {
      b_y = CUDART_NAN;
    } else {
      b_y = pow(u0, u1);
    }
  }

  return b_y;
}

//
// Arguments    : double u
// Return Type  : double
//
static __device__ double rt_roundd_snf_device(double u)
{
  double b_y;
  if (fabs(u) < 4.503599627370496E+15) {
    if (u >= 0.5) {
      b_y = floor(u + 0.5);
    } else if (u > -0.5) {
      b_y = u * 0.0;
    } else {
      b_y = ceil(u - 0.5);
    }
  } else {
    b_y = u;
  }

  return b_y;
}

//
// Arguments    : unsigned int in1
//                unsigned int offset
//                unsigned int mask
// Return Type  : unsigned int
//
static __device__ unsigned int shflDown1(unsigned int in1, unsigned int offset,
  unsigned int mask)
{
  int *tmp;
  tmp = (int *)&in1;
  *tmp = __shfl_down_sync(mask, *tmp, offset);
  return *(unsigned int *)tmp;
}

//
// Arguments    : int in1
//                unsigned int offset
//                unsigned int mask
// Return Type  : int
//
static __device__ int shflDown1(int in1, unsigned int offset, unsigned int mask)
{
  in1 = __shfl_down_sync(mask, in1, offset);
  return in1;
}

//
// Arguments    : unsigned int val
//                unsigned int lane
//                unsigned int mask
// Return Type  : unsigned int
//
static __device__ unsigned int threadGroupReduction(unsigned int val, unsigned
  int lane, unsigned int mask)
{
  unsigned int activeSize;
  unsigned int offset;
  activeSize = __popc(mask);
  offset = (activeSize + 1U) / 2U;
  while (activeSize > 1U) {
    unsigned int other;
    other = shflDown1(val, offset, mask);
    if (lane + offset < activeSize) {
      val = coder::gpucoder::internal::minFunc_device(val, other);
    }

    activeSize = offset;
    offset = (offset + 1U) / 2U;
  }

  return val;
}

//
// Arguments    : int val
//                unsigned int lane
//                unsigned int mask
// Return Type  : int
//
static __device__ int threadGroupReduction(int val, unsigned int lane, unsigned
  int mask)
{
  unsigned int activeSize;
  unsigned int offset;
  activeSize = __popc(mask);
  offset = (activeSize + 1U) / 2U;
  while (activeSize > 1U) {
    int other;
    other = shflDown1(val, offset, mask);
    if (lane + offset < activeSize) {
      val += other;
    }

    activeSize = offset;
    offset = (offset + 1U) / 2U;
  }

  return val;
}

//
// Arguments    : unsigned int val
//                unsigned int mask
//                unsigned int numActiveWarps
// Return Type  : unsigned int
//
static __device__ unsigned int workGroupReduction(unsigned int val, unsigned int
  mask, unsigned int numActiveWarps)
{
  __shared__ unsigned int shared[32];
  unsigned int lane;
  unsigned int thBlkId;
  thBlkId = static_cast<unsigned int>(mwGetThreadIndexWithinBlock());
  lane = thBlkId % warpSize;
  thBlkId /= warpSize;
  val = threadGroupReduction(val, lane, mask);
  if (lane == 0U) {
    shared[thBlkId] = val;
  }

  __syncthreads();
  mask = __ballot_sync(MAX_uint32_T, lane < numActiveWarps);
  val = shared[lane];
  if (thBlkId == 0U) {
    val = threadGroupReduction(val, lane, mask);
  }

  return val;
}

//
// Arguments    : int val
//                unsigned int mask
//                unsigned int numActiveWarps
// Return Type  : int
//
static __device__ int workGroupReduction(int val, unsigned int mask, unsigned
  int numActiveWarps)
{
  __shared__ int shared[32];
  unsigned int lane;
  unsigned int thBlkId;
  thBlkId = static_cast<unsigned int>(mwGetThreadIndexWithinBlock());
  lane = thBlkId % warpSize;
  thBlkId /= warpSize;
  val = threadGroupReduction(val, lane, mask);
  if (lane == 0U) {
    shared[thBlkId] = val;
  }

  __syncthreads();
  mask = __ballot_sync(MAX_uint32_T, lane < numActiveWarps);
  val = shared[lane];
  if (thBlkId == 0U) {
    val = threadGroupReduction(val, lane, mask);
  }

  return val;
}

//
// Arguments    : double *x_
//                double *y_
//                double *width_
//                double *height_
//                float *score_
// Return Type  : void
//
void detectFunction(double *x_, double *y_, double *width_, double *height_,
                    float *score_)
{
  static const double dv1[4] = { 1.25, 6.25, 2.375, 5.0 };

  static float out[49152];
  static const char cv[33] = "                                ";
  static const char cv2[26] = { 'v', 'i', '-', 'o', 'u', 't', 'p', 'u', 't', ',',
    ' ', 'i', 'm', 'x', '2', '1', '9', ' ', '6', '-', '0', '0', '1', '0', '\x00',
    '\x00' };

  static const signed char dv[8] = { 10, 50, 19, 40, 11, 95, 27, 48 };

  static unsigned char pln0[921600];
  static unsigned char pln1[921600];
  static unsigned char pln2[921600];
  dim3 ab_block;
  dim3 ab_grid;
  dim3 b_block;
  dim3 b_grid;
  dim3 bb_block;
  dim3 bb_grid;
  dim3 block;
  dim3 c_block;
  dim3 c_grid;
  dim3 cb_block;
  dim3 cb_grid;
  dim3 d_block;
  dim3 d_grid;
  dim3 db_block;
  dim3 db_grid;
  dim3 e_block;
  dim3 e_grid;
  dim3 eb_block;
  dim3 eb_grid;
  dim3 f_block;
  dim3 f_grid;
  dim3 fb_block;
  dim3 fb_grid;
  dim3 g_block;
  dim3 g_grid;
  dim3 gb_block;
  dim3 gb_grid;
  dim3 grid;
  dim3 h_block;
  dim3 h_grid;
  dim3 hb_block;
  dim3 hb_grid;
  dim3 i_block;
  dim3 i_grid;
  dim3 ib_block;
  dim3 ib_grid;
  dim3 j_block;
  dim3 j_grid;
  dim3 jb_block;
  dim3 jb_grid;
  dim3 k_block;
  dim3 k_grid;
  dim3 kb_block;
  dim3 kb_grid;
  dim3 l_block;
  dim3 l_grid;
  dim3 lb_block;
  dim3 lb_grid;
  dim3 m_block;
  dim3 m_grid;
  dim3 mb_block;
  dim3 mb_grid;
  dim3 n_block;
  dim3 n_grid;
  dim3 nb_block;
  dim3 nb_grid;
  dim3 o_block;
  dim3 o_grid;
  dim3 ob_block;
  dim3 ob_grid;
  dim3 p_block;
  dim3 p_grid;
  dim3 pb_block;
  dim3 pb_grid;
  dim3 q_block;
  dim3 q_grid;
  dim3 qb_block;
  dim3 qb_grid;
  dim3 r_block;
  dim3 r_grid;
  dim3 rb_block;
  dim3 rb_grid;
  dim3 s_block;
  dim3 s_grid;
  dim3 sb_block;
  dim3 sb_grid;
  dim3 t_block;
  dim3 t_grid;
  dim3 tb_block;
  dim3 tb_grid;
  dim3 u_block;
  dim3 u_grid;
  dim3 ub_block;
  dim3 ub_grid;
  dim3 v_block;
  dim3 v_grid;
  dim3 vb_block;
  dim3 vb_grid;
  dim3 w_block;
  dim3 w_grid;
  dim3 wb_block;
  dim3 wb_grid;
  dim3 x_block;
  dim3 x_grid;
  dim3 y_block;
  dim3 y_grid;
  double (*gpu_colWeights)[5152];
  double b_bboxPred_data[4096];
  double bboxPred_data[4096];
  double bboxesX1Y1X2Y2_data[4096];
  double (*b_gpu_bboxPred_data)[4096];
  double (*gpu_bboxPred_data)[4096];
  double (*gpu_bboxesX1Y1X2Y2_data)[4096];
  double (*gpu_rowWeights)[2912];
  double x1_data[1024];
  double x2_data[1024];
  double y1_data[1024];
  double y2_data[1024];
  double (*gpu_idx_data)[1024];
  double (*gpu_x1_data)[1024];
  double (*gpu_x2_data)[1024];
  double (*gpu_y1_data)[1024];
  double (*gpu_y2_data)[1024];
  double (*b_gpu_colWeights)[896];
  double (*b_gpu_rowWeights)[896];
  double (*gpu_colWeightsTotal)[224];
  double (*gpu_rowWeightsTotal)[224];
  double (*b_gpu_colWeightsTotal)[128];
  double (*b_gpu_rowWeightsTotal)[128];
  double (*gpu_anchors)[8];
  double (*gpu_dv1)[4];
  double validSampleTime;
  float (*c_gpu_out)[49152];
  float tmpFeatureMap[6144];
  float (*gpu_boxOut)[6144];
  float (*gpu_thresholdedPrediction_data)[6144];
  float (*gpu_tmpFeatureMap)[6144];
  float classPred_data[1024];
  float scorePred_data[1024];
  float scores_data[1024];
  float (*gpu_classPred_data)[1024];
  float (*gpu_scorePred_data)[1024];
  float (*gpu_scores_data)[1024];
  int b_b_size[2];
  int b_bboxPred_size[2];
  int b_size[2];
  int bboxPred_size[2];
  int bboxesX1Y1X2Y2_size[2];
  int c_b_size[2];
  int c_bboxPred_size[2];
  int d_bboxPred_size[2];
  int inDims[2];
  int inputBbox_size[2];
  int thresholdedPrediction_size[2];
  int (*b_gpu_b_size)[2];
  int (*b_gpu_bboxPred_size)[2];
  int (*c_gpu_b_size)[2];
  int (*c_gpu_bboxPred_size)[2];
  int (*d_gpu_bboxPred_size)[2];
  int (*gpu_b_size)[2];
  int (*gpu_bboxPred_size)[2];
  int (*gpu_bboxesX1Y1X2Y2_size)[2];
  unsigned int (*gpu_castRed)[2];
  int (*gpu_inputBbox_size)[2];
  int (*gpu_thresholdedPrediction_size)[2];
  int idx_size[1];
  int iv_size[1];
  int scorePred_size[1];
  int selectedIndex_size[1];
  int x2_size[1];
  int y1_size[1];
  int y2_size[1];
  int (*gpu_idx_size)[1];
  int (*gpu_iv_size)[1];
  int (*gpu_scorePred_size)[1];
  int (*gpu_selectedIndex_size)[1];
  int camIndex;
  int cameraDevice;
  int i;
  int initStatus;
  int k;
  int *gpu_camIndex;
  short (*gpu_ipColIndices)[5152];
  short (*gpu_ipRowIndices)[2912];
  short (*gpu_aux2)[2560];
  short (*gpu_aux1)[1440];
  short iv2_data[1024];
  short iv_data[1024];
  short (*gpu_iv2_data)[1024];
  short (*gpu_iv_data)[1024];
  short (*b_gpu_ipColIndices)[896];
  short (*b_gpu_ipRowIndices)[896];
  short (*b_gpu_aux1)[448];
  short (*b_gpu_aux2)[448];
  short dv2[2];
  short (*gpu_dv2)[2];
  unsigned char (*gpu_img)[2764800];
  unsigned char (*gpu_pln0)[921600];
  unsigned char (*gpu_pln1)[921600];
  unsigned char (*gpu_pln2)[921600];
  unsigned char (*gpu_partialResize)[483840];
  unsigned char (*b_gpu_out)[150528];
  unsigned char (*b_gpu_partialResize)[86016];
  unsigned char (*gpu_out)[49152];
  char (*gpu_cv)[33];
  char cv1[26];
  char (*gpu_cv1)[26];
  char (*gpu_cv2)[26];
  signed char (*gpu_dv)[8];
  unsigned char outVal[2];
  unsigned char (*gpu_outVal)[2];
  bool b_data[1024];
  bool index_data[1024];
  bool (*gpu_b_data)[1024];
  bool (*gpu_index_data)[1024];
  bool b_bboxPred_data_dirtyOnGpu;
  bool b_data_dirtyOnCpu;
  bool b_data_dirtyOnGpu;
  bool bboxPred_data_dirtyOnCpu;
  bool bboxPred_data_dirtyOnGpu;
  bool bboxesX1Y1X2Y2_data_dirtyOnGpu;
  bool bboxesX1Y1X2Y2_size_dirtyOnCpu;
  bool camIndex_dirtyOnGpu;
  bool classPred_data_dirtyOnCpu;
  bool classPred_data_dirtyOnGpu;
  bool index_data_dirtyOnGpu;
  bool iv2_data_dirtyOnCpu;
  bool iv_data_dirtyOnCpu;
  bool iv_data_dirtyOnGpu;
  bool scorePred_data_dirtyOnCpu;
  bool scorePred_data_dirtyOnGpu;
  bool scores_data_dirtyOnGpu;
  bool thresholdedPrediction_size_dirtyOnCpu;
  bool validLaunchParams;
  bool x1_data_dirtyOnGpu;
  bool x2_data_dirtyOnGpu;
  bool y1_data_dirtyOnGpu;
  bool y2_data_dirtyOnGpu;
  if (!isInitialized_detectFunction) {
    detectFunction_initialize();
  }

  cudaMalloc(&gpu_scores_data, 4096UL);
  cudaMalloc(&d_gpu_bboxPred_size, 8UL);
  cudaMalloc(&gpu_iv2_data, 2048UL);
  cudaMalloc(&gpu_index_data, 1024UL);
  cudaMalloc(&gpu_selectedIndex_size, 4UL);
  cudaMalloc(&gpu_inputBbox_size, 8UL);
  cudaMalloc(&gpu_idx_data, 8192UL);
  cudaMalloc(&gpu_dv2, 4UL);
  cudaMalloc(&gpu_idx_size, 4UL);
  cudaMalloc(&c_gpu_b_size, 8UL);
  cudaMalloc(&b_gpu_b_size, 8UL);
  cudaMalloc(&gpu_scorePred_size, 4UL);
  cudaMalloc(&c_gpu_bboxPred_size, 8UL);
  cudaMalloc(&gpu_b_size, 8UL);
  cudaMalloc(&gpu_scorePred_data, 4096UL);
  cudaMalloc(&gpu_classPred_data, 4096UL);
  cudaMalloc(&b_gpu_bboxPred_data, 32768UL);
  cudaMalloc(&gpu_bboxPred_data, 32768UL);
  cudaMalloc(&gpu_bboxPred_size, 8UL);
  cudaMalloc(&gpu_y2_data, 8192UL);
  cudaMalloc(&gpu_x2_data, 8192UL);
  cudaMalloc(&gpu_y1_data, 8192UL);
  cudaMalloc(&gpu_x1_data, 8192UL);
  cudaMalloc(&gpu_bboxesX1Y1X2Y2_data, 32768UL);
  cudaMalloc(&gpu_thresholdedPrediction_data, 24576UL);
  cudaMalloc(&gpu_bboxesX1Y1X2Y2_size, 8UL);
  cudaMalloc(&b_gpu_bboxPred_size, 8UL);
  cudaMalloc(&gpu_iv_size, 4UL);
  cudaMalloc(&gpu_thresholdedPrediction_size, 8UL);
  cudaMalloc(&gpu_iv_data, 2048UL);
  cudaMalloc(&gpu_b_data, 1024UL);
  cudaMalloc(&gpu_boxOut, 24576UL);
  cudaMalloc(&gpu_tmpFeatureMap, 24576UL);
  cudaMalloc(&gpu_dv1, 32UL);
  cudaMalloc(&gpu_anchors, 64UL);
  cudaMalloc(&gpu_dv, 8UL);
  cudaMalloc(&c_gpu_out, 196608UL);
  cudaMalloc(&gpu_outVal, 2UL);
  cudaMalloc(&gpu_castRed, 8UL);
  cudaMalloc(&gpu_out, 49152UL);
  cudaMalloc(&b_gpu_partialResize, 86016UL);
  cudaMalloc(&b_gpu_ipColIndices, 1792UL);
  cudaMalloc(&b_gpu_ipRowIndices, 1792UL);
  cudaMalloc(&b_gpu_out, 150528UL);
  cudaMalloc(&b_gpu_colWeightsTotal, 1024UL);
  cudaMalloc(&b_gpu_colWeights, 7168UL);
  cudaMalloc(&b_gpu_rowWeightsTotal, 1024UL);
  cudaMalloc(&b_gpu_rowWeights, 7168UL);
  cudaMalloc(&b_gpu_aux2, 896UL);
  cudaMalloc(&b_gpu_aux1, 896UL);
  cudaMalloc(&gpu_partialResize, 483840UL);
  cudaMalloc(&gpu_ipRowIndices, 5824UL);
  cudaMalloc(&gpu_ipColIndices, 10304UL);
  cudaMalloc(&gpu_colWeightsTotal, 1792UL);
  cudaMalloc(&gpu_colWeights, 41216UL);
  cudaMalloc(&gpu_rowWeightsTotal, 1792UL);
  cudaMalloc(&gpu_rowWeights, 23296UL);
  cudaMalloc(&gpu_aux2, 5120UL);
  cudaMalloc(&gpu_aux1, 2880UL);
  cudaMalloc(&gpu_pln2, 921600UL);
  cudaMalloc(&gpu_img, 2764800UL);
  cudaMalloc(&gpu_pln0, 921600UL);
  cudaMalloc(&gpu_pln1, 921600UL);
  cudaMalloc(&gpu_camIndex, 4UL);
  cudaMalloc(&gpu_cv1, 26UL);
  cudaMalloc(&gpu_cv2, 26UL);
  cudaMalloc(&gpu_cv, 33UL);
  iv2_data_dirtyOnCpu = false;
  scorePred_data_dirtyOnCpu = false;
  bboxPred_data_dirtyOnCpu = false;
  iv_data_dirtyOnCpu = false;
  classPred_data_dirtyOnCpu = false;
  scores_data_dirtyOnGpu = false;
  index_data_dirtyOnGpu = false;
  scorePred_data_dirtyOnGpu = false;
  b_bboxPred_data_dirtyOnGpu = false;
  bboxPred_data_dirtyOnGpu = false;
  y2_data_dirtyOnGpu = false;
  x2_data_dirtyOnGpu = false;
  y1_data_dirtyOnGpu = false;
  x1_data_dirtyOnGpu = false;
  bboxesX1Y1X2Y2_data_dirtyOnGpu = false;
  iv_data_dirtyOnGpu = false;
  classPred_data_dirtyOnGpu = false;
  if ((!mynet_not_empty) || (!hwobj_not_empty) || (!cam_not_empty)) {
    char nullChar;
    hwobj_not_empty = true;
    cam.Initialized = false;
    cam.CamWidth = 1.0;
    cam.CamHeight = 1.0;
    cam.Duration = 0.0;
    cam.SearchMode = 0U;
    cam.isInitialized = 0;
    cudaMemcpy(gpu_cv, (void *)&cv[0], 33UL, cudaMemcpyHostToDevice);
    cudaMemcpyToSymbol(gpu_cam, &cam, 88UL, 0UL, cudaMemcpyHostToDevice);
    detectFunction_kernel1<<<dim3(1U, 1U, 1U), dim3(64U, 1U, 1U)>>>(*gpu_cv);
    cudaMemcpyFromSymbol(&cam, gpu_cam, 88UL, 0UL, cudaMemcpyDeviceToHost);
    nullChar = '\x00';
    camIndex = 0;
    cameraDevice = 0;
    validSampleTime = 0.0;
    cudaMemcpy(gpu_cv2, (void *)&cv2[0], 26UL, cudaMemcpyHostToDevice);
    detectFunction_kernel2<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>(*gpu_cv2,
      *gpu_cv1);
    cudaMemcpy(&cv1[0], gpu_cv1, 26UL, cudaMemcpyDeviceToHost);
    initStatus = validateArgsAndGetDetails(&cv1[0], 1280.0, 720.0, &camIndex,
      &cameraDevice, cam.SearchMode, &cam.VideoDevice[0], 0.0, &validSampleTime);
    if (initStatus != 0) {
      mw_terminate();
    }

    initStatus = EXT_gstreamerMediaInit(&cam.StructCustomData, &nullChar,
      &cam.CamWidth, &cam.CamHeight, 1280U, 720U, &cam.Duration, 0.0,
      &cam.FrameRate, camIndex, true, cameraDevice, true, validSampleTime);
    if (initStatus == 0) {
      cam.Initialized = true;
    } else {
      mw_terminate();
    }

    cam.matlabCodegenIsDeleted = false;
    cam.isSetupComplete = false;
    cam.isInitialized = 1;
    cam.isSetupComplete = true;
    cam_not_empty = true;
    coder::DeepLearningNetwork_setup(&gobj_2);
    mynet.Network = &gobj_2;
    mynet_not_empty = true;
  }

  validSampleTime = 0.0;
  if (cam.Initialized) {
    initStatus = EXT_gstreamerFrameRead(&cam.StructCustomData, 1280.0, 720.0,
      0.0, &validSampleTime, &pln0[0], &pln1[0], &pln2[0]);
    cudaMemcpy(gpu_pln1, &pln1[0], 921600UL, cudaMemcpyHostToDevice);
    cudaMemcpy(gpu_pln0, &pln0[0], 921600UL, cudaMemcpyHostToDevice);
    detectFunction_kernel3<<<dim3(1800U, 1U, 1U), dim3(512U, 1U, 1U)>>>
      (*gpu_pln1, *gpu_pln0, *gpu_img);
    cudaMemcpy(gpu_pln2, &pln2[0], 921600UL, cudaMemcpyHostToDevice);
    detectFunction_kernel4<<<dim3(1800U, 1U, 1U), dim3(512U, 1U, 1U)>>>
      (*gpu_pln2, *gpu_img);
    if (initStatus != 0) {
      cam.Initialized = false;
    }
  }

  detectFunction_kernel5<<<dim3(3U, 1U, 1U), dim3(512U, 1U, 1U)>>>(*gpu_aux1);
  detectFunction_kernel6<<<dim3(5U, 1U, 1U), dim3(512U, 1U, 1U)>>>(*gpu_aux2);
  detectFunction_kernel7<<<dim3(6U, 1U, 1U), dim3(512U, 1U, 1U)>>>(*gpu_aux1,
    *gpu_ipRowIndices, *gpu_rowWeights);
  detectFunction_kernel8<<<dim3(11U, 1U, 1U), dim3(512U, 1U, 1U)>>>(*gpu_aux2,
    *gpu_ipColIndices, *gpu_colWeights);
  detectFunction_kernel9<<<dim3(1U, 1U, 1U), dim3(224U, 1U, 1U)>>>
    (*gpu_rowWeights, *gpu_rowWeightsTotal);
  for (k = 0; k < 12; k++) {
    detectFunction_kernel10<<<dim3(1U, 1U, 1U), dim3(224U, 1U, 1U)>>>
      (*gpu_rowWeights, (k + 1) * 224, *gpu_rowWeightsTotal);
  }

  detectFunction_kernel11<<<dim3(1U, 1U, 1U), dim3(224U, 1U, 1U)>>>
    (*gpu_colWeights, *gpu_colWeightsTotal);
  for (k = 0; k < 22; k++) {
    detectFunction_kernel12<<<dim3(1U, 1U, 1U), dim3(224U, 1U, 1U)>>>
      (*gpu_colWeights, (k + 1) * 224, *gpu_colWeightsTotal);
  }

  detectFunction_kernel13<<<dim3(945U, 1U, 1U), dim3(512U, 1U, 1U)>>>
    (*gpu_colWeightsTotal, *gpu_colWeights, *gpu_img, *gpu_ipColIndices,
     *gpu_partialResize);
  detectFunction_kernel14<<<dim3(294U, 1U, 1U), dim3(512U, 1U, 1U)>>>
    (*gpu_rowWeightsTotal, *gpu_rowWeights, *gpu_partialResize,
     *gpu_ipRowIndices, *b_gpu_out);
  detectFunction_kernel15<<<dim3(1U, 1U, 1U), dim3(448U, 1U, 1U)>>>(*b_gpu_aux2,
    *b_gpu_aux1);
  detectFunction_kernel16<<<dim3(2U, 1U, 1U), dim3(512U, 1U, 1U)>>>(*b_gpu_aux1,
    *b_gpu_ipRowIndices, *b_gpu_rowWeights);
  detectFunction_kernel17<<<dim3(2U, 1U, 1U), dim3(512U, 1U, 1U)>>>(*b_gpu_aux2,
    *b_gpu_ipColIndices, *b_gpu_colWeights);
  detectFunction_kernel18<<<dim3(1U, 1U, 1U), dim3(128U, 1U, 1U)>>>
    (*b_gpu_rowWeights, *b_gpu_rowWeightsTotal);
  for (k = 0; k < 6; k++) {
    detectFunction_kernel19<<<dim3(1U, 1U, 1U), dim3(128U, 1U, 1U)>>>
      (*b_gpu_rowWeights, (k + 1) << 7, *b_gpu_rowWeightsTotal);
  }

  detectFunction_kernel20<<<dim3(1U, 1U, 1U), dim3(128U, 1U, 1U)>>>
    (*b_gpu_colWeights, *b_gpu_colWeightsTotal);
  for (k = 0; k < 6; k++) {
    detectFunction_kernel21<<<dim3(1U, 1U, 1U), dim3(128U, 1U, 1U)>>>
      (*b_gpu_colWeights, (k + 1) << 7, *b_gpu_colWeightsTotal);
  }

  detectFunction_kernel22<<<dim3(168U, 1U, 1U), dim3(512U, 1U, 1U)>>>
    (*b_gpu_rowWeightsTotal, *b_gpu_rowWeights, *b_gpu_out, *b_gpu_ipRowIndices,
     *b_gpu_partialResize);
  detectFunction_kernel23<<<dim3(96U, 1U, 1U), dim3(512U, 1U, 1U)>>>
    (*b_gpu_colWeightsTotal, *b_gpu_colWeights, *b_gpu_partialResize,
     *b_gpu_ipColIndices, *gpu_out);
  detectFunction_kernel24<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>(*gpu_out,
    *gpu_castRed);
  coder_reduce0<<<dim3(96U, 1U, 1U), dim3(512U, 1U, 1U)>>>(*gpu_out,
    *gpu_castRed);
  detectFunction_kernel25<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>(*gpu_outVal, *
    gpu_castRed);
  cudaMemcpy(&outVal[0], gpu_outVal, 2UL, cudaMemcpyDeviceToHost);
  initStatus = outVal[1] - outVal[0];
  detectFunction_kernel26<<<dim3(96U, 1U, 1U), dim3(512U, 1U, 1U)>>>(initStatus,
    static_cast<short>(outVal[0]), *gpu_out, *c_gpu_out);
  cudaMemcpy(&out[0], c_gpu_out, 196608UL, cudaMemcpyDeviceToHost);
  coder::DeepLearningNetwork_activations(mynet.Network, out, tmpFeatureMap);
  cudaMemcpy(gpu_dv, (void *)&dv[0], 8UL, cudaMemcpyHostToDevice);
  detectFunction_kernel27<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>(*gpu_dv,
    *gpu_anchors);
  cudaMemcpy(gpu_dv1, (void *)&dv1[0], 32UL, cudaMemcpyHostToDevice);
  detectFunction_kernel28<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>(*gpu_dv1,
    *gpu_anchors);
  cudaMemcpy(gpu_tmpFeatureMap, &tmpFeatureMap[0], 24576UL,
             cudaMemcpyHostToDevice);
  detectFunction_kernel29<<<dim3(2U, 1U, 1U), dim3(512U, 1U, 1U)>>>(*gpu_anchors,
    *gpu_tmpFeatureMap, *gpu_boxOut);
  detectFunction_kernel30<<<dim3(2U, 1U, 1U), dim3(512U, 1U, 1U)>>>(*gpu_boxOut,
    *gpu_b_data);
  b_data_dirtyOnCpu = false;
  b_data_dirtyOnGpu = true;
  initStatus = 0;
  for (i = 0; i < 1024; i++) {
    if (b_data_dirtyOnGpu) {
      cudaMemcpy(&b_data[0], gpu_b_data, 1024UL, cudaMemcpyDeviceToHost);
      b_data_dirtyOnGpu = false;
    }

    if (b_data[i]) {
      initStatus++;
    }
  }

  iv_size[0] = initStatus;
  camIndex = 0;
  camIndex_dirtyOnGpu = false;
  for (i = 0; i < 1024; i++) {
    if (b_data_dirtyOnGpu) {
      cudaMemcpy(&b_data[0], gpu_b_data, 1024UL, cudaMemcpyDeviceToHost);
      b_data_dirtyOnGpu = false;
    }

    if (b_data[i]) {
      iv_data[camIndex] = static_cast<short>(i + 1);
      iv_data_dirtyOnCpu = true;
      camIndex++;
    }
  }

  thresholdedPrediction_size[0] = initStatus;
  thresholdedPrediction_size[1] = 6;
  thresholdedPrediction_size_dirtyOnCpu = true;
  validLaunchParams = mwGetLaunchParameters(static_cast<double>(((iv_size[0] - 1)
    + 1L) * 6L), &grid, &block, 1024U, 65535U);
  if (validLaunchParams) {
    if (iv_data_dirtyOnCpu) {
      cudaMemcpy(gpu_iv_data, &iv_data[0], 2048UL, cudaMemcpyHostToDevice);
      iv_data_dirtyOnCpu = false;
    }

    cudaMemcpy(gpu_thresholdedPrediction_size, &thresholdedPrediction_size[0],
               8UL, cudaMemcpyHostToDevice);
    thresholdedPrediction_size_dirtyOnCpu = false;
    cudaMemcpy(gpu_iv_size, &iv_size[0], 4UL, cudaMemcpyHostToDevice);
    detectFunction_kernel31<<<grid, block>>>(*gpu_boxOut, *gpu_iv_data,
      *gpu_thresholdedPrediction_size, *gpu_iv_size,
      *gpu_thresholdedPrediction_data);
  }

  if (iv_size[0] != 0) {
    int count;
    int j;
    short b_i;
    bool b_bboxPred_size_dirtyOnCpu;
    bool bboxPred_size_dirtyOnCpu;
    bool guard1 = false;
    k = iv_size[0] - 1;
    bboxesX1Y1X2Y2_size[0] = iv_size[0];
    bboxesX1Y1X2Y2_size[1] = 4;
    bboxesX1Y1X2Y2_size_dirtyOnCpu = true;
    validLaunchParams = mwGetLaunchParameters(static_cast<double>((k + 1L) * 4L),
      &b_grid, &b_block, 1024U, 65535U);
    if (validLaunchParams) {
      if (thresholdedPrediction_size_dirtyOnCpu) {
        cudaMemcpy(gpu_thresholdedPrediction_size, &thresholdedPrediction_size[0],
                   8UL, cudaMemcpyHostToDevice);
        thresholdedPrediction_size_dirtyOnCpu = false;
      }

      cudaMemcpy(gpu_bboxesX1Y1X2Y2_size, &bboxesX1Y1X2Y2_size[0], 8UL,
                 cudaMemcpyHostToDevice);
      bboxesX1Y1X2Y2_size_dirtyOnCpu = false;
      detectFunction_kernel32<<<b_grid, b_block>>>
        (*gpu_thresholdedPrediction_data, *gpu_thresholdedPrediction_size,
         *gpu_bboxesX1Y1X2Y2_size, k, *gpu_bboxesX1Y1X2Y2_data);
      bboxesX1Y1X2Y2_data_dirtyOnGpu = true;
    }

    k = bboxesX1Y1X2Y2_size[0] - 1;
    iv_size[0] = bboxesX1Y1X2Y2_size[0];
    validLaunchParams = mwGetLaunchParameters(static_cast<double>(k + 1L),
      &c_grid, &c_block, 1024U, 65535U);
    if (validLaunchParams) {
      detectFunction_kernel33<<<c_grid, c_block>>>(*gpu_bboxesX1Y1X2Y2_data, k, *
        gpu_x1_data);
      x1_data_dirtyOnGpu = true;
    }

    k = bboxesX1Y1X2Y2_size[0] - 1;
    y1_size[0] = bboxesX1Y1X2Y2_size[0];
    validLaunchParams = mwGetLaunchParameters(static_cast<double>(k + 1L),
      &d_grid, &d_block, 1024U, 65535U);
    if (validLaunchParams) {
      if (bboxesX1Y1X2Y2_size_dirtyOnCpu) {
        cudaMemcpy(gpu_bboxesX1Y1X2Y2_size, &bboxesX1Y1X2Y2_size[0], 8UL,
                   cudaMemcpyHostToDevice);
        bboxesX1Y1X2Y2_size_dirtyOnCpu = false;
      }

      detectFunction_kernel34<<<d_grid, d_block>>>(*gpu_bboxesX1Y1X2Y2_data,
        *gpu_bboxesX1Y1X2Y2_size, k, *gpu_y1_data);
      y1_data_dirtyOnGpu = true;
    }

    k = bboxesX1Y1X2Y2_size[0] - 1;
    x2_size[0] = bboxesX1Y1X2Y2_size[0];
    validLaunchParams = mwGetLaunchParameters(static_cast<double>(k + 1L),
      &e_grid, &e_block, 1024U, 65535U);
    if (validLaunchParams) {
      if (bboxesX1Y1X2Y2_size_dirtyOnCpu) {
        cudaMemcpy(gpu_bboxesX1Y1X2Y2_size, &bboxesX1Y1X2Y2_size[0], 8UL,
                   cudaMemcpyHostToDevice);
        bboxesX1Y1X2Y2_size_dirtyOnCpu = false;
      }

      detectFunction_kernel35<<<e_grid, e_block>>>(*gpu_bboxesX1Y1X2Y2_data,
        *gpu_bboxesX1Y1X2Y2_size, k, *gpu_x2_data);
      x2_data_dirtyOnGpu = true;
    }

    k = bboxesX1Y1X2Y2_size[0] - 1;
    y2_size[0] = bboxesX1Y1X2Y2_size[0];
    validLaunchParams = mwGetLaunchParameters(static_cast<double>(k + 1L),
      &f_grid, &f_block, 1024U, 65535U);
    if (validLaunchParams) {
      if (bboxesX1Y1X2Y2_size_dirtyOnCpu) {
        cudaMemcpy(gpu_bboxesX1Y1X2Y2_size, &bboxesX1Y1X2Y2_size[0], 8UL,
                   cudaMemcpyHostToDevice);
      }

      detectFunction_kernel36<<<f_grid, f_block>>>(*gpu_bboxesX1Y1X2Y2_data,
        *gpu_bboxesX1Y1X2Y2_size, k, *gpu_y2_data);
      y2_data_dirtyOnGpu = true;
    }

    validLaunchParams = mwGetLaunchParameters(static_cast<double>
      ((bboxesX1Y1X2Y2_size[0] - 1) + 1L), &g_grid, &g_block, 1024U, 65535U);
    if (validLaunchParams) {
      detectFunction_kernel37<<<g_grid, g_block>>>(bboxesX1Y1X2Y2_size[0],
        *gpu_x1_data);
      x1_data_dirtyOnGpu = true;
    }

    validLaunchParams = mwGetLaunchParameters(static_cast<double>
      ((bboxesX1Y1X2Y2_size[0] - 1) + 1L), &h_grid, &h_block, 1024U, 65535U);
    if (validLaunchParams) {
      detectFunction_kernel38<<<h_grid, h_block>>>(bboxesX1Y1X2Y2_size[0],
        *gpu_y1_data);
      y1_data_dirtyOnGpu = true;
    }

    validLaunchParams = mwGetLaunchParameters(static_cast<double>
      ((bboxesX1Y1X2Y2_size[0] - 1) + 1L), &i_grid, &i_block, 1024U, 65535U);
    if (validLaunchParams) {
      detectFunction_kernel39<<<i_grid, i_block>>>(bboxesX1Y1X2Y2_size[0],
        *gpu_x2_data);
      x2_data_dirtyOnGpu = true;
    }

    validLaunchParams = mwGetLaunchParameters(static_cast<double>
      ((bboxesX1Y1X2Y2_size[0] - 1) + 1L), &j_grid, &j_block, 1024U, 65535U);
    if (validLaunchParams) {
      detectFunction_kernel40<<<j_grid, j_block>>>(bboxesX1Y1X2Y2_size[0],
        *gpu_y2_data);
      y2_data_dirtyOnGpu = true;
    }

    bboxesX1Y1X2Y2_size[0] = iv_size[0];
    bboxesX1Y1X2Y2_size[1] = 4;
    bboxesX1Y1X2Y2_size_dirtyOnCpu = true;
    validLaunchParams = mwGetLaunchParameters(static_cast<double>((iv_size[0] -
      1) + 1L), &k_grid, &k_block, 1024U, 65535U);
    if (validLaunchParams) {
      detectFunction_kernel41<<<k_grid, k_block>>>(*gpu_x1_data, iv_size[0] - 1,
        *gpu_bboxesX1Y1X2Y2_data);
      bboxesX1Y1X2Y2_data_dirtyOnGpu = true;
    }

    validLaunchParams = mwGetLaunchParameters(static_cast<double>((y1_size[0] -
      1) + 1L), &l_grid, &l_block, 1024U, 65535U);
    if (validLaunchParams) {
      cudaMemcpy(gpu_bboxesX1Y1X2Y2_size, &bboxesX1Y1X2Y2_size[0], 8UL,
                 cudaMemcpyHostToDevice);
      bboxesX1Y1X2Y2_size_dirtyOnCpu = false;
      detectFunction_kernel42<<<l_grid, l_block>>>(*gpu_y1_data,
        *gpu_bboxesX1Y1X2Y2_size, y1_size[0] - 1, *gpu_bboxesX1Y1X2Y2_data);
      bboxesX1Y1X2Y2_data_dirtyOnGpu = true;
    }

    validLaunchParams = mwGetLaunchParameters(static_cast<double>((x2_size[0] -
      1) + 1L), &m_grid, &m_block, 1024U, 65535U);
    if (validLaunchParams) {
      if (bboxesX1Y1X2Y2_size_dirtyOnCpu) {
        cudaMemcpy(gpu_bboxesX1Y1X2Y2_size, &bboxesX1Y1X2Y2_size[0], 8UL,
                   cudaMemcpyHostToDevice);
        bboxesX1Y1X2Y2_size_dirtyOnCpu = false;
      }

      detectFunction_kernel43<<<m_grid, m_block>>>(*gpu_x2_data,
        *gpu_bboxesX1Y1X2Y2_size, x2_size[0] - 1, *gpu_bboxesX1Y1X2Y2_data);
      bboxesX1Y1X2Y2_data_dirtyOnGpu = true;
    }

    validLaunchParams = mwGetLaunchParameters(static_cast<double>((y2_size[0] -
      1) + 1L), &n_grid, &n_block, 1024U, 65535U);
    if (validLaunchParams) {
      if (bboxesX1Y1X2Y2_size_dirtyOnCpu) {
        cudaMemcpy(gpu_bboxesX1Y1X2Y2_size, &bboxesX1Y1X2Y2_size[0], 8UL,
                   cudaMemcpyHostToDevice);
        bboxesX1Y1X2Y2_size_dirtyOnCpu = false;
      }

      detectFunction_kernel44<<<n_grid, n_block>>>(*gpu_y2_data,
        *gpu_bboxesX1Y1X2Y2_size, y2_size[0] - 1, *gpu_bboxesX1Y1X2Y2_data);
      bboxesX1Y1X2Y2_data_dirtyOnGpu = true;
    }

    k = bboxesX1Y1X2Y2_size[0];
    camIndex = bboxesX1Y1X2Y2_size[0];
    cameraDevice = bboxesX1Y1X2Y2_size[0];
    b_bboxPred_size[0] = bboxesX1Y1X2Y2_size[0];
    b_bboxPred_size[1] = 4;
    bboxPred_size_dirtyOnCpu = true;
    validLaunchParams = mwGetLaunchParameters(static_cast<double>
      ((bboxesX1Y1X2Y2_size[0] - 1) + 1L), &o_grid, &o_block, 1024U, 65535U);
    if (validLaunchParams) {
      detectFunction_kernel45<<<o_grid, o_block>>>(*gpu_bboxesX1Y1X2Y2_data,
        bboxesX1Y1X2Y2_size[0] - 1, *gpu_bboxPred_data);
      bboxPred_data_dirtyOnGpu = true;
    }

    validLaunchParams = mwGetLaunchParameters(static_cast<double>((k - 1) + 1L),
      &p_grid, &p_block, 1024U, 65535U);
    if (validLaunchParams) {
      if (bboxesX1Y1X2Y2_size_dirtyOnCpu) {
        cudaMemcpy(gpu_bboxesX1Y1X2Y2_size, &bboxesX1Y1X2Y2_size[0], 8UL,
                   cudaMemcpyHostToDevice);
        bboxesX1Y1X2Y2_size_dirtyOnCpu = false;
      }

      cudaMemcpy(gpu_bboxPred_size, &b_bboxPred_size[0], 8UL,
                 cudaMemcpyHostToDevice);
      bboxPred_size_dirtyOnCpu = false;
      detectFunction_kernel46<<<p_grid, p_block>>>(*gpu_bboxesX1Y1X2Y2_data,
        *gpu_bboxesX1Y1X2Y2_size, *gpu_bboxPred_size, k - 1, *gpu_bboxPred_data);
      bboxPred_data_dirtyOnGpu = true;
    }

    validLaunchParams = mwGetLaunchParameters(static_cast<double>((camIndex - 1)
      + 1L), &q_grid, &q_block, 1024U, 65535U);
    if (validLaunchParams) {
      if (bboxesX1Y1X2Y2_size_dirtyOnCpu) {
        cudaMemcpy(gpu_bboxesX1Y1X2Y2_size, &bboxesX1Y1X2Y2_size[0], 8UL,
                   cudaMemcpyHostToDevice);
        bboxesX1Y1X2Y2_size_dirtyOnCpu = false;
      }

      if (bboxPred_size_dirtyOnCpu) {
        cudaMemcpy(gpu_bboxPred_size, &b_bboxPred_size[0], 8UL,
                   cudaMemcpyHostToDevice);
        bboxPred_size_dirtyOnCpu = false;
      }

      detectFunction_kernel47<<<q_grid, q_block>>>(*gpu_bboxesX1Y1X2Y2_data,
        *gpu_bboxesX1Y1X2Y2_size, *gpu_bboxPred_size, camIndex - 1,
        *gpu_bboxPred_data);
      bboxPred_data_dirtyOnGpu = true;
    }

    validLaunchParams = mwGetLaunchParameters(static_cast<double>((cameraDevice
      - 1) + 1L), &r_grid, &r_block, 1024U, 65535U);
    if (validLaunchParams) {
      if (bboxesX1Y1X2Y2_size_dirtyOnCpu) {
        cudaMemcpy(gpu_bboxesX1Y1X2Y2_size, &bboxesX1Y1X2Y2_size[0], 8UL,
                   cudaMemcpyHostToDevice);
      }

      if (bboxPred_size_dirtyOnCpu) {
        cudaMemcpy(gpu_bboxPred_size, &b_bboxPred_size[0], 8UL,
                   cudaMemcpyHostToDevice);
        bboxPred_size_dirtyOnCpu = false;
      }

      detectFunction_kernel48<<<r_grid, r_block>>>(*gpu_bboxesX1Y1X2Y2_data,
        *gpu_bboxesX1Y1X2Y2_size, *gpu_bboxPred_size, cameraDevice - 1,
        *gpu_bboxPred_data);
      bboxPred_data_dirtyOnGpu = true;
    }

    initStatus = b_bboxPred_size[0] << 2;
    validLaunchParams = mwGetLaunchParameters(static_cast<double>((initStatus -
      1) + 1L), &s_grid, &s_block, 1024U, 65535U);
    if (validLaunchParams) {
      detectFunction_kernel49<<<s_grid, s_block>>>(initStatus,
        *gpu_bboxPred_data);
      bboxPred_data_dirtyOnGpu = true;
    }

    k = b_bboxPred_size[0] - 1;
    iv_size[0] = b_bboxPred_size[0];
    validLaunchParams = mwGetLaunchParameters(static_cast<double>(k + 1L),
      &t_grid, &t_block, 1024U, 65535U);
    if (validLaunchParams) {
      if (bboxPred_size_dirtyOnCpu) {
        cudaMemcpy(gpu_bboxPred_size, &b_bboxPred_size[0], 8UL,
                   cudaMemcpyHostToDevice);
        bboxPred_size_dirtyOnCpu = false;
      }

      detectFunction_kernel50<<<t_grid, t_block>>>(*gpu_bboxPred_data,
        *gpu_bboxPred_size, k, *gpu_x1_data);
      x1_data_dirtyOnGpu = true;
    }

    validLaunchParams = mwGetLaunchParameters(static_cast<double>((iv_size[0] -
      1) + 1L), &u_grid, &u_block, 1024U, 65535U);
    if (validLaunchParams) {
      cudaMemcpy(gpu_iv_size, &iv_size[0], 4UL, cudaMemcpyHostToDevice);
      if (bboxPred_size_dirtyOnCpu) {
        cudaMemcpy(gpu_bboxPred_size, &b_bboxPred_size[0], 8UL,
                   cudaMemcpyHostToDevice);
        bboxPred_size_dirtyOnCpu = false;
      }

      detectFunction_kernel51<<<u_grid, u_block>>>(*gpu_x1_data,
        *gpu_bboxPred_size, *gpu_iv_size, *gpu_bboxPred_data);
      bboxPred_data_dirtyOnGpu = true;
    }

    k = b_bboxPred_size[0] - 1;
    iv_size[0] = b_bboxPred_size[0];
    validLaunchParams = mwGetLaunchParameters(static_cast<double>(k + 1L),
      &v_grid, &v_block, 1024U, 65535U);
    if (validLaunchParams) {
      if (bboxPred_size_dirtyOnCpu) {
        cudaMemcpy(gpu_bboxPred_size, &b_bboxPred_size[0], 8UL,
                   cudaMemcpyHostToDevice);
        bboxPred_size_dirtyOnCpu = false;
      }

      detectFunction_kernel52<<<v_grid, v_block>>>(*gpu_bboxPred_data,
        *gpu_bboxPred_size, k, *gpu_x1_data);
      x1_data_dirtyOnGpu = true;
    }

    validLaunchParams = mwGetLaunchParameters(static_cast<double>((iv_size[0] -
      1) + 1L), &w_grid, &w_block, 1024U, 65535U);
    if (validLaunchParams) {
      cudaMemcpy(gpu_iv_size, &iv_size[0], 4UL, cudaMemcpyHostToDevice);
      if (bboxPred_size_dirtyOnCpu) {
        cudaMemcpy(gpu_bboxPred_size, &b_bboxPred_size[0], 8UL,
                   cudaMemcpyHostToDevice);
        bboxPred_size_dirtyOnCpu = false;
      }

      detectFunction_kernel53<<<w_grid, w_block>>>(*gpu_x1_data,
        *gpu_bboxPred_size, *gpu_iv_size, *gpu_bboxPred_data);
      bboxPred_data_dirtyOnGpu = true;
    }

    count = 0;
    bboxPred_size[0] = b_bboxPred_size[0];
    bboxPred_size[1] = 4;
    b_bboxPred_size_dirtyOnCpu = true;
    scorePred_size[0] = b_bboxPred_size[0];
    iv_size[0] = b_bboxPred_size[0];
    i = b_bboxPred_size[0];
    for (initStatus = 0; initStatus < i; initStatus++) {
      if (bboxPred_data_dirtyOnGpu) {
        cudaMemcpy(&b_bboxPred_data[0], gpu_bboxPred_data, 32768UL,
                   cudaMemcpyDeviceToHost);
        bboxPred_data_dirtyOnGpu = false;
      }

      if ((b_bboxPred_data[initStatus + b_bboxPred_size[0] * 3] >= 1.0) &&
          (b_bboxPred_data[initStatus + (b_bboxPred_size[0] << 1)] >= 1.0) &&
          (b_bboxPred_data[initStatus + b_bboxPred_size[0] * 3] <= 224.0) &&
          (b_bboxPred_data[initStatus + (b_bboxPred_size[0] << 1)] <= 224.0)) {
        count++;
        camIndex = count - 1;
        if (bboxPred_size_dirtyOnCpu) {
          cudaMemcpy(gpu_bboxPred_size, &b_bboxPred_size[0], 8UL,
                     cudaMemcpyHostToDevice);
          bboxPred_size_dirtyOnCpu = false;
        }

        if (b_bboxPred_size_dirtyOnCpu) {
          cudaMemcpy(b_gpu_bboxPred_size, &bboxPred_size[0], 8UL,
                     cudaMemcpyHostToDevice);
          b_bboxPred_size_dirtyOnCpu = false;
        }

        cudaMemcpy(gpu_camIndex, &camIndex, 4UL, cudaMemcpyHostToDevice);
        detectFunction_kernel54<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
          (*gpu_bboxPred_data, *gpu_bboxPred_size, initStatus,
           *b_gpu_bboxPred_size, gpu_camIndex, *b_gpu_bboxPred_data);
        b_bboxPred_data_dirtyOnGpu = true;
        if (thresholdedPrediction_size_dirtyOnCpu) {
          cudaMemcpy(gpu_thresholdedPrediction_size,
                     &thresholdedPrediction_size[0], 8UL, cudaMemcpyHostToDevice);
          thresholdedPrediction_size_dirtyOnCpu = false;
        }

        detectFunction_kernel55<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
          (*gpu_thresholdedPrediction_data, *gpu_thresholdedPrediction_size,
           initStatus, count, *gpu_classPred_data, *gpu_scorePred_data);
        scorePred_data_dirtyOnGpu = true;
        classPred_data_dirtyOnGpu = true;
      }
    }

    b_i = static_cast<short>(bboxPred_size[0]);
    validLaunchParams = mwGetLaunchParameters(static_cast<double>((b_i -
      static_cast<short>(count + 1)) + 1L), &x_grid, &x_block, 1024U, 65535U);
    if (validLaunchParams) {
      if (iv_data_dirtyOnCpu) {
        cudaMemcpy(gpu_iv_data, &iv_data[0], 2048UL, cudaMemcpyHostToDevice);
      }

      detectFunction_kernel56<<<x_grid, x_block>>>(static_cast<short>(count + 1),
        b_i, *gpu_iv_data);
      iv_data_dirtyOnCpu = false;
      iv_data_dirtyOnGpu = true;
    }

    cameraDevice = bboxPred_size[0];
    if (static_cast<short>(bboxPred_size[0] - count) == 1) {
      initStatus = bboxPred_size[0] - 1;
      if (iv_data_dirtyOnCpu) {
        cudaMemcpy(gpu_iv_data, &iv_data[0], 2048UL, cudaMemcpyHostToDevice);
        iv_data_dirtyOnCpu = false;
      }

      if (b_bboxPred_size_dirtyOnCpu) {
        cudaMemcpy(b_gpu_bboxPred_size, &bboxPred_size[0], 8UL,
                   cudaMemcpyHostToDevice);
        b_bboxPred_size_dirtyOnCpu = false;
      }

      detectFunction_kernel59<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
        (*b_gpu_bboxPred_size, initStatus, *gpu_iv_data, *b_gpu_bboxPred_data);
      b_bboxPred_data_dirtyOnGpu = true;
    } else {
      b_size[0] = 1;
      b_size[1] = bboxPred_size[0];
      validLaunchParams = mwGetLaunchParameters(static_cast<double>
        ((bboxPred_size[0] - 1) + 1L), &y_grid, &y_block, 1024U, 65535U);
      if (validLaunchParams) {
        if (b_bboxPred_size_dirtyOnCpu) {
          cudaMemcpy(b_gpu_bboxPred_size, &bboxPred_size[0], 8UL,
                     cudaMemcpyHostToDevice);
          b_bboxPred_size_dirtyOnCpu = false;
        }

        detectFunction_kernel57<<<y_grid, y_block>>>(*b_gpu_bboxPred_size,
          *gpu_b_data);
        b_data_dirtyOnGpu = true;
      }

      initStatus = static_cast<short>(bboxPred_size[0] - count);
      for (k = 0; k < initStatus; k++) {
        if (b_data_dirtyOnGpu) {
          cudaMemcpy(&b_data[0], gpu_b_data, 1024UL, cudaMemcpyDeviceToHost);
          b_data_dirtyOnGpu = false;
        }

        if (iv_data_dirtyOnGpu) {
          cudaMemcpy(&iv_data[0], gpu_iv_data, 2048UL, cudaMemcpyDeviceToHost);
          iv_data_dirtyOnGpu = false;
        }

        b_data[iv_data[k] - 1] = true;
        b_data_dirtyOnCpu = true;
      }

      camIndex = 0;
      initStatus = b_size[1];
      validLaunchParams = mwGetLaunchParameters(static_cast<double>((b_size[1] -
        1) + 1L), &ab_grid, &ab_block, 1024U, 65535U);
      if (validLaunchParams) {
        if (b_data_dirtyOnCpu) {
          cudaMemcpy(gpu_b_data, &b_data[0], 1024UL, cudaMemcpyHostToDevice);
          b_data_dirtyOnCpu = false;
        }

        cudaMemcpy(gpu_camIndex, &camIndex, 4UL, cudaMemcpyHostToDevice);
        cudaMemcpy(gpu_b_size, &b_size[0], 8UL, cudaMemcpyHostToDevice);
        detectFunction_kernel58<<<ab_grid, ab_block>>>(*gpu_b_data, initStatus,
          gpu_camIndex);
        camIndex_dirtyOnGpu = true;
      }

      if (camIndex_dirtyOnGpu) {
        cudaMemcpy(&camIndex, gpu_camIndex, 4UL, cudaMemcpyDeviceToHost);
        camIndex_dirtyOnGpu = false;
      }

      initStatus = bboxPred_size[0] - camIndex;
      i = 0;
      for (k = 0; k < cameraDevice; k++) {
        guard1 = false;
        if (k + 1 > b_size[1]) {
          guard1 = true;
        } else {
          if (b_data_dirtyOnGpu) {
            cudaMemcpy(&b_data[0], gpu_b_data, 1024UL, cudaMemcpyDeviceToHost);
            b_data_dirtyOnGpu = false;
          }

          if (!b_data[k]) {
            guard1 = true;
          }
        }

        if (guard1) {
          for (j = 0; j < 4; j++) {
            if (b_bboxPred_data_dirtyOnGpu) {
              cudaMemcpy(&bboxPred_data[0], b_gpu_bboxPred_data, 32768UL,
                         cudaMemcpyDeviceToHost);
              b_bboxPred_data_dirtyOnGpu = false;
            }

            bboxPred_data[i + bboxPred_size[0] * j] = bboxPred_data[k +
              bboxPred_size[0] * j];
            bboxPred_data_dirtyOnCpu = true;
          }

          i++;
        }
      }
    }

    if (1 > initStatus) {
      initStatus = -1;
    } else {
      initStatus--;
    }

    c_bboxPred_size[0] = initStatus + 1;
    c_bboxPred_size[1] = 4;
    bboxPred_size_dirtyOnCpu = true;
    validLaunchParams = mwGetLaunchParameters(static_cast<double>((initStatus +
      1L) * 4L), &bb_grid, &bb_block, 1024U, 65535U);
    if (validLaunchParams) {
      if (b_bboxPred_size_dirtyOnCpu) {
        cudaMemcpy(b_gpu_bboxPred_size, &bboxPred_size[0], 8UL,
                   cudaMemcpyHostToDevice);
      }

      if (bboxPred_data_dirtyOnCpu) {
        cudaMemcpy(b_gpu_bboxPred_data, &bboxPred_data[0], 32768UL,
                   cudaMemcpyHostToDevice);
        bboxPred_data_dirtyOnCpu = false;
      }

      cudaMemcpy(c_gpu_bboxPred_size, &c_bboxPred_size[0], 8UL,
                 cudaMemcpyHostToDevice);
      bboxPred_size_dirtyOnCpu = false;
      detectFunction_kernel60<<<bb_grid, bb_block>>>(*b_gpu_bboxPred_data,
        *b_gpu_bboxPred_size, *c_gpu_bboxPred_size, initStatus,
        *gpu_bboxPred_data);
    }

    bboxPred_size[0] = c_bboxPred_size[0];
    bboxPred_size[1] = 4;
    b_bboxPred_size_dirtyOnCpu = true;
    validLaunchParams = mwGetLaunchParameters(static_cast<double>
      ((c_bboxPred_size[0] * 4 - 1) + 1L), &cb_grid, &cb_block, 1024U, 65535U);
    if (validLaunchParams) {
      if (bboxPred_data_dirtyOnCpu) {
        cudaMemcpy(b_gpu_bboxPred_data, &bboxPred_data[0], 32768UL,
                   cudaMemcpyHostToDevice);
        bboxPred_data_dirtyOnCpu = false;
      }

      if (bboxPred_size_dirtyOnCpu) {
        cudaMemcpy(c_gpu_bboxPred_size, &c_bboxPred_size[0], 8UL,
                   cudaMemcpyHostToDevice);
      }

      detectFunction_kernel61<<<cb_grid, cb_block>>>(*gpu_bboxPred_data,
        *c_gpu_bboxPred_size, *b_gpu_bboxPred_data);
      b_bboxPred_data_dirtyOnGpu = true;
    }

    b_i = static_cast<short>(scorePred_size[0]);
    validLaunchParams = mwGetLaunchParameters(static_cast<double>((b_i -
      static_cast<short>(count + 1)) + 1L), &db_grid, &db_block, 1024U, 65535U);
    if (validLaunchParams) {
      if (iv_data_dirtyOnCpu) {
        cudaMemcpy(gpu_iv_data, &iv_data[0], 2048UL, cudaMemcpyHostToDevice);
        iv_data_dirtyOnCpu = false;
      }

      detectFunction_kernel62<<<db_grid, db_block>>>(static_cast<short>(count +
        1), b_i, *gpu_iv_data);
      iv_data_dirtyOnGpu = true;
    }

    cameraDevice = scorePred_size[0];
    b_b_size[0] = 1;
    b_b_size[1] = scorePred_size[0];
    validLaunchParams = mwGetLaunchParameters(static_cast<double>
      ((scorePred_size[0] - 1) + 1L), &eb_grid, &eb_block, 1024U, 65535U);
    if (validLaunchParams) {
      if (b_data_dirtyOnCpu) {
        cudaMemcpy(gpu_b_data, &b_data[0], 1024UL, cudaMemcpyHostToDevice);
        b_data_dirtyOnCpu = false;
      }

      cudaMemcpy(gpu_scorePred_size, &scorePred_size[0], 4UL,
                 cudaMemcpyHostToDevice);
      detectFunction_kernel63<<<eb_grid, eb_block>>>(*gpu_scorePred_size,
        *gpu_b_data);
      b_data_dirtyOnGpu = true;
    }

    initStatus = static_cast<short>(scorePred_size[0] - count);
    for (k = 0; k < initStatus; k++) {
      if (b_data_dirtyOnGpu) {
        cudaMemcpy(&b_data[0], gpu_b_data, 1024UL, cudaMemcpyDeviceToHost);
        b_data_dirtyOnGpu = false;
      }

      if (iv_data_dirtyOnGpu) {
        cudaMemcpy(&iv_data[0], gpu_iv_data, 2048UL, cudaMemcpyDeviceToHost);
        iv_data_dirtyOnGpu = false;
      }

      b_data[iv_data[k] - 1] = true;
      b_data_dirtyOnCpu = true;
    }

    camIndex = 0;
    initStatus = b_b_size[1];
    validLaunchParams = mwGetLaunchParameters(static_cast<double>((b_b_size[1] -
      1) + 1L), &fb_grid, &fb_block, 1024U, 65535U);
    if (validLaunchParams) {
      if (b_data_dirtyOnCpu) {
        cudaMemcpy(gpu_b_data, &b_data[0], 1024UL, cudaMemcpyHostToDevice);
        b_data_dirtyOnCpu = false;
      }

      cudaMemcpy(gpu_camIndex, &camIndex, 4UL, cudaMemcpyHostToDevice);
      cudaMemcpy(b_gpu_b_size, &b_b_size[0], 8UL, cudaMemcpyHostToDevice);
      detectFunction_kernel64<<<fb_grid, fb_block>>>(*gpu_b_data, initStatus,
        gpu_camIndex);
      cudaMemcpy(&camIndex, gpu_camIndex, 4UL, cudaMemcpyDeviceToHost);
      camIndex_dirtyOnGpu = false;
    }

    initStatus = scorePred_size[0] - camIndex;
    camIndex = -1;
    for (k = 0; k < cameraDevice; k++) {
      guard1 = false;
      if (k + 1 > b_b_size[1]) {
        guard1 = true;
      } else {
        if (b_data_dirtyOnGpu) {
          cudaMemcpy(&b_data[0], gpu_b_data, 1024UL, cudaMemcpyDeviceToHost);
          b_data_dirtyOnGpu = false;
        }

        if (!b_data[k]) {
          guard1 = true;
        }
      }

      if (guard1) {
        camIndex++;
        if (scorePred_data_dirtyOnGpu) {
          cudaMemcpy(&scorePred_data[0], gpu_scorePred_data, 4096UL,
                     cudaMemcpyDeviceToHost);
          scorePred_data_dirtyOnGpu = false;
        }

        scorePred_data[camIndex] = scorePred_data[k];
        scorePred_data_dirtyOnCpu = true;
      }
    }

    if (1 > initStatus) {
      scorePred_size[0] = 0;
    } else {
      scorePred_size[0] = initStatus;
    }

    b_i = static_cast<short>(iv_size[0]);
    validLaunchParams = mwGetLaunchParameters(static_cast<double>((b_i -
      static_cast<short>(count + 1)) + 1L), &gb_grid, &gb_block, 1024U, 65535U);
    if (validLaunchParams) {
      if (iv_data_dirtyOnCpu) {
        cudaMemcpy(gpu_iv_data, &iv_data[0], 2048UL, cudaMemcpyHostToDevice);
      }

      detectFunction_kernel65<<<gb_grid, gb_block>>>(static_cast<short>(count +
        1), b_i, *gpu_iv_data);
      iv_data_dirtyOnGpu = true;
    }

    cameraDevice = iv_size[0];
    c_b_size[0] = 1;
    c_b_size[1] = iv_size[0];
    validLaunchParams = mwGetLaunchParameters(static_cast<double>((iv_size[0] -
      1) + 1L), &hb_grid, &hb_block, 1024U, 65535U);
    if (validLaunchParams) {
      if (b_data_dirtyOnCpu) {
        cudaMemcpy(gpu_b_data, &b_data[0], 1024UL, cudaMemcpyHostToDevice);
        b_data_dirtyOnCpu = false;
      }

      cudaMemcpy(gpu_iv_size, &iv_size[0], 4UL, cudaMemcpyHostToDevice);
      detectFunction_kernel66<<<hb_grid, hb_block>>>(*gpu_iv_size, *gpu_b_data);
      b_data_dirtyOnGpu = true;
    }

    initStatus = static_cast<short>(iv_size[0] - count);
    for (k = 0; k < initStatus; k++) {
      if (b_data_dirtyOnGpu) {
        cudaMemcpy(&b_data[0], gpu_b_data, 1024UL, cudaMemcpyDeviceToHost);
        b_data_dirtyOnGpu = false;
      }

      if (iv_data_dirtyOnGpu) {
        cudaMemcpy(&iv_data[0], gpu_iv_data, 2048UL, cudaMemcpyDeviceToHost);
        iv_data_dirtyOnGpu = false;
      }

      b_data[iv_data[k] - 1] = true;
      b_data_dirtyOnCpu = true;
    }

    camIndex = 0;
    initStatus = c_b_size[1];
    validLaunchParams = mwGetLaunchParameters(static_cast<double>((c_b_size[1] -
      1) + 1L), &ib_grid, &ib_block, 1024U, 65535U);
    if (validLaunchParams) {
      if (b_data_dirtyOnCpu) {
        cudaMemcpy(gpu_b_data, &b_data[0], 1024UL, cudaMemcpyHostToDevice);
        b_data_dirtyOnCpu = false;
      }

      cudaMemcpy(gpu_camIndex, &camIndex, 4UL, cudaMemcpyHostToDevice);
      cudaMemcpy(c_gpu_b_size, &c_b_size[0], 8UL, cudaMemcpyHostToDevice);
      detectFunction_kernel67<<<ib_grid, ib_block>>>(*gpu_b_data, initStatus,
        gpu_camIndex);
      camIndex_dirtyOnGpu = true;
    }

    if (camIndex_dirtyOnGpu) {
      cudaMemcpy(&camIndex, gpu_camIndex, 4UL, cudaMemcpyDeviceToHost);
    }

    initStatus = iv_size[0] - camIndex;
    camIndex = -1;
    for (k = 0; k < cameraDevice; k++) {
      guard1 = false;
      if (k + 1 > c_b_size[1]) {
        guard1 = true;
      } else {
        if (b_data_dirtyOnGpu) {
          cudaMemcpy(&b_data[0], gpu_b_data, 1024UL, cudaMemcpyDeviceToHost);
          b_data_dirtyOnGpu = false;
        }

        if (!b_data[k]) {
          guard1 = true;
        }
      }

      if (guard1) {
        camIndex++;
        if (classPred_data_dirtyOnGpu) {
          cudaMemcpy(&classPred_data[0], gpu_classPred_data, 4096UL,
                     cudaMemcpyDeviceToHost);
          classPred_data_dirtyOnGpu = false;
        }

        classPred_data[camIndex] = classPred_data[k];
        classPred_data_dirtyOnCpu = true;
      }
    }

    if (1 > initStatus) {
      iv_size[0] = 0;
    } else {
      iv_size[0] = initStatus;
    }

    if (bboxPred_size[0] == 0) {
      iv_size[0] = scorePred_size[0];
      validLaunchParams = mwGetLaunchParameters(static_cast<double>
        ((scorePred_size[0] - 1) + 1L), &jb_grid, &jb_block, 1024U, 65535U);
      if (validLaunchParams) {
        if (scorePred_data_dirtyOnCpu) {
          cudaMemcpy(gpu_scorePred_data, &scorePred_data[0], 4096UL,
                     cudaMemcpyHostToDevice);
        }

        cudaMemcpy(gpu_scorePred_size, &scorePred_size[0], 4UL,
                   cudaMemcpyHostToDevice);
        detectFunction_kernel82<<<jb_grid, jb_block>>>(*gpu_scorePred_data,
          *gpu_scorePred_size, *gpu_scores_data);
        scores_data_dirtyOnGpu = true;
      }
    } else {
      validLaunchParams = mwGetLaunchParameters(static_cast<double>((iv_size[0]
        - 1) + 1L), &jb_grid, &jb_block, 1024U, 65535U);
      if (validLaunchParams) {
        if (classPred_data_dirtyOnCpu) {
          cudaMemcpy(gpu_classPred_data, &classPred_data[0], 4096UL,
                     cudaMemcpyHostToDevice);
          classPred_data_dirtyOnCpu = false;
        }

        cudaMemcpy(gpu_iv_size, &iv_size[0], 4UL, cudaMemcpyHostToDevice);
        detectFunction_kernel68<<<jb_grid, jb_block>>>(*gpu_classPred_data,
          *gpu_iv_size, *gpu_y1_data);
        y1_data_dirtyOnGpu = true;
      }

      idx_size[0] = scorePred_size[0];
      iv_data_dirtyOnCpu = true;
      if (scorePred_size[0] != 0) {
        initStatus = 2;
        if (scorePred_size[0] != 1) {
          initStatus = 1;
        }

        inDims[0] = scorePred_size[0];
        inDims[1] = 1;
        validLaunchParams = mwGetLaunchParameters(static_cast<double>
          ((scorePred_size[0] - 1) + 1L), &kb_grid, &kb_block, 1024U, 65535U);
        if (validLaunchParams) {
          if (classPred_data_dirtyOnCpu) {
            cudaMemcpy(gpu_classPred_data, &classPred_data[0], 4096UL,
                       cudaMemcpyHostToDevice);
            classPred_data_dirtyOnCpu = false;
          }

          if (scorePred_data_dirtyOnCpu) {
            cudaMemcpy(gpu_scorePred_data, &scorePred_data[0], 4096UL,
                       cudaMemcpyHostToDevice);
          }

          cudaMemcpy(gpu_scorePred_size, &scorePred_size[0], 4UL,
                     cudaMemcpyHostToDevice);
          detectFunction_kernel69<<<kb_grid, kb_block>>>(*gpu_scorePred_data,
            *gpu_scorePred_size, *gpu_classPred_data);
        }

        dv2[0] = static_cast<short>(scorePred_size[0]);
        idx_size[0] = static_cast<short>(scorePred_size[0]);
        validLaunchParams = mwGetLaunchParameters(static_cast<double>((dv2[0] -
          1) + 1L), &lb_grid, &lb_block, 1024U, 65535U);
        if (validLaunchParams) {
          cudaMemcpy(gpu_dv2, &dv2[0], 4UL, cudaMemcpyHostToDevice);
          detectFunction_kernel70<<<lb_grid, lb_block>>>(*gpu_dv2, *gpu_idx_data);
        }

        if (classPred_data_dirtyOnCpu) {
          cudaMemcpy(gpu_classPred_data, &classPred_data[0], 4096UL,
                     cudaMemcpyHostToDevice);
        }

        thrustSortImplWithIndex(&(*gpu_classPred_data)[0], &(*gpu_idx_data)[0],
          2, &inDims[0], initStatus, 'd', false);
      }

      inputBbox_size[0] = idx_size[0];
      inputBbox_size[1] = 4;
      bboxesX1Y1X2Y2_size_dirtyOnCpu = true;
      validLaunchParams = mwGetLaunchParameters(static_cast<double>(((idx_size[0]
        - 1) + 1L) * 4L), &mb_grid, &mb_block, 1024U, 65535U);
      if (validLaunchParams) {
        cudaMemcpy(b_gpu_bboxPred_size, &bboxPred_size[0], 8UL,
                   cudaMemcpyHostToDevice);
        b_bboxPred_size_dirtyOnCpu = false;
        if (bboxPred_data_dirtyOnCpu) {
          cudaMemcpy(b_gpu_bboxPred_data, &bboxPred_data[0], 32768UL,
                     cudaMemcpyHostToDevice);
          bboxPred_data_dirtyOnCpu = false;
        }

        cudaMemcpy(gpu_inputBbox_size, &inputBbox_size[0], 8UL,
                   cudaMemcpyHostToDevice);
        bboxesX1Y1X2Y2_size_dirtyOnCpu = false;
        cudaMemcpy(gpu_idx_size, &idx_size[0], 4UL, cudaMemcpyHostToDevice);
        iv_data_dirtyOnCpu = false;
        detectFunction_kernel71<<<mb_grid, mb_block>>>(*b_gpu_bboxPred_data,
          *b_gpu_bboxPred_size, *gpu_idx_data, *gpu_inputBbox_size,
          *gpu_idx_size, *gpu_bboxesX1Y1X2Y2_data);
        bboxesX1Y1X2Y2_data_dirtyOnGpu = true;
      }

      iv_size[0] = idx_size[0];
      validLaunchParams = mwGetLaunchParameters(static_cast<double>((idx_size[0]
        - 1) + 1L), &nb_grid, &nb_block, 1024U, 65535U);
      if (validLaunchParams) {
        if (iv_data_dirtyOnCpu) {
          cudaMemcpy(gpu_idx_size, &idx_size[0], 4UL, cudaMemcpyHostToDevice);
          iv_data_dirtyOnCpu = false;
        }

        detectFunction_kernel72<<<nb_grid, nb_block>>>(*gpu_y1_data,
          *gpu_idx_data, *gpu_idx_size, *gpu_x1_data);
        x1_data_dirtyOnGpu = true;
      }

      validLaunchParams = mwGetLaunchParameters(static_cast<double>((iv_size[0]
        - 1) + 1L), &ob_grid, &ob_block, 1024U, 65535U);
      if (validLaunchParams) {
        cudaMemcpy(gpu_iv_size, &iv_size[0], 4UL, cudaMemcpyHostToDevice);
        detectFunction_kernel73<<<ob_grid, ob_block>>>(*gpu_x1_data,
          *gpu_iv_size, *gpu_y1_data);
        y1_data_dirtyOnGpu = true;
      }

      selectedIndex_size[0] = idx_size[0];
      validLaunchParams = mwGetLaunchParameters(static_cast<double>((idx_size[0]
        - 1) + 1L), &pb_grid, &pb_block, 1024U, 65535U);
      if (validLaunchParams) {
        if (b_data_dirtyOnCpu) {
          cudaMemcpy(gpu_b_data, &b_data[0], 1024UL, cudaMemcpyHostToDevice);
          b_data_dirtyOnCpu = false;
        }

        if (iv_data_dirtyOnCpu) {
          cudaMemcpy(gpu_idx_size, &idx_size[0], 4UL, cudaMemcpyHostToDevice);
        }

        detectFunction_kernel74<<<pb_grid, pb_block>>>(*gpu_idx_size,
          *gpu_b_data);
        b_data_dirtyOnGpu = true;
      }

      k = idx_size[0] - 1;
      validLaunchParams = mwGetLaunchParameters(static_cast<double>(k + 1L),
        &qb_grid, &qb_block, 1024U, 65535U);
      if (validLaunchParams) {
        if (bboxesX1Y1X2Y2_size_dirtyOnCpu) {
          cudaMemcpy(gpu_inputBbox_size, &inputBbox_size[0], 8UL,
                     cudaMemcpyHostToDevice);
          bboxesX1Y1X2Y2_size_dirtyOnCpu = false;
        }

        detectFunction_kernel75<<<qb_grid, qb_block>>>(*gpu_bboxesX1Y1X2Y2_data,
          *gpu_inputBbox_size, k, *gpu_x1_data);
        x1_data_dirtyOnGpu = true;
      }

      k = idx_size[0] - 1;
      validLaunchParams = mwGetLaunchParameters(static_cast<double>(k + 1L),
        &rb_grid, &rb_block, 1024U, 65535U);
      if (validLaunchParams) {
        if (bboxesX1Y1X2Y2_size_dirtyOnCpu) {
          cudaMemcpy(gpu_inputBbox_size, &inputBbox_size[0], 8UL,
                     cudaMemcpyHostToDevice);
          bboxesX1Y1X2Y2_size_dirtyOnCpu = false;
        }

        detectFunction_kernel76<<<rb_grid, rb_block>>>(*gpu_inputBbox_size,
          *gpu_bboxesX1Y1X2Y2_data, k, *gpu_x2_data);
        x2_data_dirtyOnGpu = true;
      }

      k = idx_size[0] - 1;
      validLaunchParams = mwGetLaunchParameters(static_cast<double>(k + 1L),
        &sb_grid, &sb_block, 1024U, 65535U);
      if (validLaunchParams) {
        if (bboxesX1Y1X2Y2_size_dirtyOnCpu) {
          cudaMemcpy(gpu_inputBbox_size, &inputBbox_size[0], 8UL,
                     cudaMemcpyHostToDevice);
        }

        detectFunction_kernel77<<<sb_grid, sb_block>>>(*gpu_bboxesX1Y1X2Y2_data,
          *gpu_inputBbox_size, k, *gpu_y2_data);
        y2_data_dirtyOnGpu = true;
      }

      initStatus = -1;
      camIndex = idx_size[0];
      for (i = 0; i < camIndex; i++) {
        initStatus = i;
        if (y1_data_dirtyOnGpu) {
          cudaMemcpy(&y1_data[0], gpu_y1_data, 8192UL, cudaMemcpyDeviceToHost);
          y1_data_dirtyOnGpu = false;
        }

        if (rtIsNaN(y1_data[i])) {
          if (b_data_dirtyOnGpu) {
            cudaMemcpy(&b_data[0], gpu_b_data, 1024UL, cudaMemcpyDeviceToHost);
            b_data_dirtyOnGpu = false;
          }

          b_data[i] = false;
          b_data_dirtyOnCpu = true;
        } else {
          if (b_data_dirtyOnGpu) {
            cudaMemcpy(&b_data[0], gpu_b_data, 1024UL, cudaMemcpyDeviceToHost);
            b_data_dirtyOnGpu = false;
          }

          if (b_data[i]) {
            cameraDevice = (idx_size[0] - i) - 2;
            for (j = 0; j <= cameraDevice; j++) {
              k = (i + j) + 1;
              if (b_data[k] && (!(y1_data[k] != y1_data[i]))) {
                double maxval;
                double width;
                if (x2_data_dirtyOnGpu) {
                  cudaMemcpy(&x2_data[0], gpu_x2_data, 8192UL,
                             cudaMemcpyDeviceToHost);
                  x2_data_dirtyOnGpu = false;
                }

                if ((x2_data[i] < x2_data[k]) || rtIsNaN(x2_data[k])) {
                  validSampleTime = x2_data[i];
                } else {
                  validSampleTime = x2_data[k];
                }

                if (bboxesX1Y1X2Y2_data_dirtyOnGpu) {
                  cudaMemcpy(&bboxesX1Y1X2Y2_data[0], gpu_bboxesX1Y1X2Y2_data,
                             32768UL, cudaMemcpyDeviceToHost);
                  bboxesX1Y1X2Y2_data_dirtyOnGpu = false;
                }

                if ((bboxesX1Y1X2Y2_data[i] > bboxesX1Y1X2Y2_data[k]) || rtIsNaN
                    (bboxesX1Y1X2Y2_data[k])) {
                  maxval = bboxesX1Y1X2Y2_data[i];
                } else {
                  maxval = bboxesX1Y1X2Y2_data[k];
                }

                width = validSampleTime - maxval;
                if (!(width <= 0.0)) {
                  if (y2_data_dirtyOnGpu) {
                    cudaMemcpy(&y2_data[0], gpu_y2_data, 8192UL,
                               cudaMemcpyDeviceToHost);
                    y2_data_dirtyOnGpu = false;
                  }

                  if ((y2_data[i] < y2_data[k]) || rtIsNaN(y2_data[k])) {
                    validSampleTime = y2_data[i];
                  } else {
                    validSampleTime = y2_data[k];
                  }

                  if ((bboxesX1Y1X2Y2_data[i + inputBbox_size[0]] >
                       bboxesX1Y1X2Y2_data[k + inputBbox_size[0]]) || rtIsNaN
                      (bboxesX1Y1X2Y2_data[k + inputBbox_size[0]])) {
                    maxval = bboxesX1Y1X2Y2_data[i + inputBbox_size[0]];
                  } else {
                    maxval = bboxesX1Y1X2Y2_data[k + inputBbox_size[0]];
                  }

                  validSampleTime -= maxval;
                  if (!(validSampleTime <= 0.0)) {
                    validSampleTime *= width;
                    if (x1_data_dirtyOnGpu) {
                      cudaMemcpy(&x1_data[0], gpu_x1_data, 8192UL,
                                 cudaMemcpyDeviceToHost);
                      x1_data_dirtyOnGpu = false;
                    }

                    if (validSampleTime / ((x1_data[i] + x1_data[k]) -
                                           validSampleTime) > 0.5) {
                      b_data[k] = false;
                      b_data_dirtyOnCpu = true;
                    }
                  }
                }
              }
            }
          }
        }
      }

      if (initStatus + 2 > selectedIndex_size[0]) {
        camIndex = 0;
        initStatus = 0;
      } else {
        camIndex = initStatus + 1;
        initStatus = selectedIndex_size[0];
      }

      thresholdedPrediction_size[1] = initStatus - camIndex;
      validLaunchParams = mwGetLaunchParameters(static_cast<double>
        ((thresholdedPrediction_size[1] - 1) + 1L), &tb_grid, &tb_block, 1024U,
        65535U);
      if (validLaunchParams) {
        if (b_data_dirtyOnCpu) {
          cudaMemcpy(gpu_b_data, &b_data[0], 1024UL, cudaMemcpyHostToDevice);
          b_data_dirtyOnCpu = false;
        }

        cudaMemcpy(gpu_thresholdedPrediction_size, &thresholdedPrediction_size[0],
                   8UL, cudaMemcpyHostToDevice);
        cudaMemcpy(gpu_camIndex, &camIndex, 4UL, cudaMemcpyHostToDevice);
        detectFunction_kernel78<<<tb_grid, tb_block>>>(gpu_camIndex,
          *gpu_thresholdedPrediction_size, *gpu_b_data);
      }

      y1_size[0] = selectedIndex_size[0];
      validLaunchParams = mwGetLaunchParameters(static_cast<double>
        ((selectedIndex_size[0] - 1) + 1L), &ub_grid, &ub_block, 1024U, 65535U);
      if (validLaunchParams) {
        if (b_data_dirtyOnCpu) {
          cudaMemcpy(gpu_b_data, &b_data[0], 1024UL, cudaMemcpyHostToDevice);
        }

        cudaMemcpy(gpu_selectedIndex_size, &selectedIndex_size[0], 4UL,
                   cudaMemcpyHostToDevice);
        detectFunction_kernel79<<<ub_grid, ub_block>>>(*gpu_b_data,
          *gpu_idx_data, *gpu_selectedIndex_size, *gpu_index_data);
        index_data_dirtyOnGpu = true;
      }

      cameraDevice = y1_size[0] - 1;
      initStatus = 0;
      for (i = 0; i <= cameraDevice; i++) {
        if (index_data_dirtyOnGpu) {
          cudaMemcpy(&index_data[0], gpu_index_data, 1024UL,
                     cudaMemcpyDeviceToHost);
          index_data_dirtyOnGpu = false;
        }

        if (index_data[i]) {
          initStatus++;
        }
      }

      iv_size[0] = initStatus;
      camIndex = 0;
      for (i = 0; i <= cameraDevice; i++) {
        if (index_data_dirtyOnGpu) {
          cudaMemcpy(&index_data[0], gpu_index_data, 1024UL,
                     cudaMemcpyDeviceToHost);
          index_data_dirtyOnGpu = false;
        }

        if (index_data[i]) {
          iv2_data[camIndex] = static_cast<short>(i + 1);
          iv2_data_dirtyOnCpu = true;
          camIndex++;
        }
      }

      d_bboxPred_size[0] = initStatus;
      d_bboxPred_size[1] = 4;
      bboxPred_size_dirtyOnCpu = true;
      validLaunchParams = mwGetLaunchParameters(static_cast<double>(((iv_size[0]
        - 1) + 1L) * 4L), &vb_grid, &vb_block, 1024U, 65535U);
      if (validLaunchParams) {
        cudaMemcpy(gpu_iv_size, &iv_size[0], 4UL, cudaMemcpyHostToDevice);
        if (b_bboxPred_size_dirtyOnCpu) {
          cudaMemcpy(b_gpu_bboxPred_size, &bboxPred_size[0], 8UL,
                     cudaMemcpyHostToDevice);
        }

        if (bboxPred_data_dirtyOnCpu) {
          cudaMemcpy(b_gpu_bboxPred_data, &bboxPred_data[0], 32768UL,
                     cudaMemcpyHostToDevice);
          bboxPred_data_dirtyOnCpu = false;
        }

        if (iv2_data_dirtyOnCpu) {
          cudaMemcpy(gpu_iv2_data, &iv2_data[0], 2048UL, cudaMemcpyHostToDevice);
        }

        cudaMemcpy(d_gpu_bboxPred_size, &d_bboxPred_size[0], 8UL,
                   cudaMemcpyHostToDevice);
        bboxPred_size_dirtyOnCpu = false;
        detectFunction_kernel80<<<vb_grid, vb_block>>>(*b_gpu_bboxPred_data,
          *b_gpu_bboxPred_size, *gpu_iv2_data, *d_gpu_bboxPred_size,
          *gpu_iv_size, *gpu_bboxPred_data);
      }

      bboxPred_size[0] = d_bboxPred_size[0];
      validLaunchParams = mwGetLaunchParameters(static_cast<double>
        ((d_bboxPred_size[0] * 4 - 1) + 1L), &wb_grid, &wb_block, 1024U, 65535U);
      if (validLaunchParams) {
        if (bboxPred_data_dirtyOnCpu) {
          cudaMemcpy(b_gpu_bboxPred_data, &bboxPred_data[0], 32768UL,
                     cudaMemcpyHostToDevice);
        }

        if (bboxPred_size_dirtyOnCpu) {
          cudaMemcpy(d_gpu_bboxPred_size, &d_bboxPred_size[0], 8UL,
                     cudaMemcpyHostToDevice);
        }

        detectFunction_kernel81<<<wb_grid, wb_block>>>(*gpu_bboxPred_data,
          *d_gpu_bboxPred_size, *b_gpu_bboxPred_data);
        b_bboxPred_data_dirtyOnGpu = true;
      }

      cameraDevice = y1_size[0] - 1;
      initStatus = 0;
      for (i = 0; i <= cameraDevice; i++) {
        if (index_data_dirtyOnGpu) {
          cudaMemcpy(&index_data[0], gpu_index_data, 1024UL,
                     cudaMemcpyDeviceToHost);
          index_data_dirtyOnGpu = false;
        }

        if (index_data[i]) {
          initStatus++;
        }
      }

      iv_size[0] = initStatus;
      camIndex = 0;
      for (i = 0; i <= cameraDevice; i++) {
        if (index_data_dirtyOnGpu) {
          cudaMemcpy(&index_data[0], gpu_index_data, 1024UL,
                     cudaMemcpyDeviceToHost);
          index_data_dirtyOnGpu = false;
        }

        if (index_data[i]) {
          if (scorePred_data_dirtyOnGpu) {
            cudaMemcpy(&scorePred_data[0], gpu_scorePred_data, 4096UL,
                       cudaMemcpyDeviceToHost);
            scorePred_data_dirtyOnGpu = false;
          }

          scores_data[camIndex] = scorePred_data[i];
          camIndex++;
        }
      }
    }
  } else {
    bboxPred_size[0] = 0;
    iv_size[0] = 0;
  }

  //  only the one with the highest score
  initStatus = 0;
  camIndex = iv_size[0];
  if (scores_data_dirtyOnGpu) {
    cudaMemcpy(&scores_data[0], gpu_scores_data, 4096UL, cudaMemcpyDeviceToHost);
  }

  *score_ = scores_data[0];
  for (i = 0; i <= camIndex - 2; i++) {
    if (rtIsNaNF(scores_data[i + 1])) {
      bboxesX1Y1X2Y2_size_dirtyOnCpu = false;
    } else if (rtIsNaNF(*score_)) {
      bboxesX1Y1X2Y2_size_dirtyOnCpu = true;
    } else {
      bboxesX1Y1X2Y2_size_dirtyOnCpu = (*score_ < scores_data[i + 1]);
    }

    if (bboxesX1Y1X2Y2_size_dirtyOnCpu) {
      *score_ = scores_data[i + 1];
      initStatus = i + 1;
    }
  }

  if (bboxPred_size[0] != 0) {
    if (b_bboxPred_data_dirtyOnGpu) {
      cudaMemcpy(&bboxPred_data[0], b_gpu_bboxPred_data, 32768UL,
                 cudaMemcpyDeviceToHost);
    }

    *x_ = bboxPred_data[initStatus];
    *y_ = bboxPred_data[initStatus + bboxPred_size[0]];
    *width_ = bboxPred_data[initStatus + (bboxPred_size[0] << 1)];
    *height_ = bboxPred_data[initStatus + bboxPred_size[0] * 3];
  } else {
    //  nothing detected
    *x_ = -1.0;
    *y_ = -1.0;
    *width_ = -1.0;
    *height_ = -1.0;
    *score_ = -1.0F;
  }

  cudaFree(*gpu_cv);
  cudaFree(*gpu_cv2);
  cudaFree(*gpu_cv1);
  cudaFree(gpu_camIndex);
  cudaFree(*gpu_pln1);
  cudaFree(*gpu_pln0);
  cudaFree(*gpu_img);
  cudaFree(*gpu_pln2);
  cudaFree(*gpu_aux1);
  cudaFree(*gpu_aux2);
  cudaFree(*gpu_rowWeights);
  cudaFree(*gpu_rowWeightsTotal);
  cudaFree(*gpu_colWeights);
  cudaFree(*gpu_colWeightsTotal);
  cudaFree(*gpu_ipColIndices);
  cudaFree(*gpu_ipRowIndices);
  cudaFree(*gpu_partialResize);
  cudaFree(*b_gpu_aux1);
  cudaFree(*b_gpu_aux2);
  cudaFree(*b_gpu_rowWeights);
  cudaFree(*b_gpu_rowWeightsTotal);
  cudaFree(*b_gpu_colWeights);
  cudaFree(*b_gpu_colWeightsTotal);
  cudaFree(*b_gpu_out);
  cudaFree(*b_gpu_ipRowIndices);
  cudaFree(*b_gpu_ipColIndices);
  cudaFree(*b_gpu_partialResize);
  cudaFree(*gpu_out);
  cudaFree(*gpu_castRed);
  cudaFree(*gpu_outVal);
  cudaFree(*c_gpu_out);
  cudaFree(*gpu_dv);
  cudaFree(*gpu_anchors);
  cudaFree(*gpu_dv1);
  cudaFree(*gpu_tmpFeatureMap);
  cudaFree(*gpu_boxOut);
  cudaFree(*gpu_b_data);
  cudaFree(*gpu_iv_data);
  cudaFree(*gpu_thresholdedPrediction_size);
  cudaFree(*gpu_iv_size);
  cudaFree(*b_gpu_bboxPred_size);
  cudaFree(*gpu_bboxesX1Y1X2Y2_size);
  cudaFree(*gpu_thresholdedPrediction_data);
  cudaFree(*gpu_bboxesX1Y1X2Y2_data);
  cudaFree(*gpu_x1_data);
  cudaFree(*gpu_y1_data);
  cudaFree(*gpu_x2_data);
  cudaFree(*gpu_y2_data);
  cudaFree(*gpu_bboxPred_size);
  cudaFree(*gpu_bboxPred_data);
  cudaFree(*b_gpu_bboxPred_data);
  cudaFree(*gpu_classPred_data);
  cudaFree(*gpu_scorePred_data);
  cudaFree(*gpu_b_size);
  cudaFree(*c_gpu_bboxPred_size);
  cudaFree(*gpu_scorePred_size);
  cudaFree(*b_gpu_b_size);
  cudaFree(*c_gpu_b_size);
  cudaFree(*gpu_idx_size);
  cudaFree(*gpu_dv2);
  cudaFree(*gpu_idx_data);
  cudaFree(*gpu_inputBbox_size);
  cudaFree(*gpu_selectedIndex_size);
  cudaFree(*gpu_index_data);
  cudaFree(*gpu_iv2_data);
  cudaFree(*d_gpu_bboxPred_size);
  cudaFree(*gpu_scores_data);
}

//
// Arguments    : void
// Return Type  : void
//
void detectFunction_free()
{
  if (!cam.matlabCodegenIsDeleted) {
    cam.matlabCodegenIsDeleted = true;
    if (cam.isInitialized == 1) {
      cam.isInitialized = 2;
      if (cam.isSetupComplete && cam.Initialized) {
        EXT_gstreamerMediaTerminate(&cam.StructCustomData);
      }
    }
  }
}

//
// Arguments    : void
// Return Type  : void
//
void detectFunction_init()
{
  cam_not_empty = false;
  hwobj_not_empty = false;
  mynet_not_empty = false;
  cam.matlabCodegenIsDeleted = true;
}

//
// File trailer for detectFunction.cu
//
// [EOF]
//
