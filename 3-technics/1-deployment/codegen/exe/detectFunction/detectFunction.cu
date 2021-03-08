//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: detectFunction.cu
//
// GPU Coder version                    : 2.0
// CUDA/C/C++ source code generated on  : 08-Mar-2021 19:56:17
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
#include "MW_SDL_video_display.h"
#include "insertShapeUtilsCore_api.hpp"
#include "math_constants.h"
#include "rt_nonfinite.h"
#include "stdio.h"
#include "v4l2_cam.h"

class yoloNetwork0_0;

// Type Definitions
struct emxArray_char_T_1x3
{
  char data[3];
  int size[2];
};

struct cell_wrap_7
{
  emxArray_char_T_1x3 f1;
};

struct emxArray_cell_wrap_7_512
{
  cell_wrap_7 data[512];
  int size[1];
};

namespace coder
{
  struct YOLOv2Network
  {
    yoloNetwork0_0 *Network;
  };
}

// Variable Definitions
static yoloNetwork0_0 gobj_0;
static coder::YOLOv2Network mynet;
static bool mynet_not_empty;

// Function Declarations
static __device__ double atomicOpreal_T(double *address, double value);
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
static __global__ void detectFunction_kernel1(const char cv2[22], char cv1[22]);
static __global__ void detectFunction_kernel10(const double colWeights[1344],
  double colWeightsTotal[224]);
static __global__ void detectFunction_kernel100(const signed char x_data[3], int
  *status, double *height);
static __global__ void detectFunction_kernel101(const unsigned short uv1[256],
  const signed char thisCharcodes_1b_data[3], const int thisCharcodes_1b_size[2],
  bool x_data[3]);
static __global__ void detectFunction_kernel102(const bool x_data[3], int *nrows);
static __global__ void detectFunction_kernel103(const bool x_data[3], int
  *status, int *nrows);
static __global__ void detectFunction_kernel104(const unsigned char uv5[10664],
  const int *nrows, const int *status, unsigned char uv5_data[10664]);
static __global__ void detectFunction_kernel105(const unsigned char uv5_data
  [10664], const int uv5_size[2], const signed char num[2], unsigned char
  b_uv5_data[144]);
static __global__ void detectFunction_kernel106(const unsigned char uv5_data[144],
  const signed char num[2], unsigned char thisGlyphBitmap_data[144]);
static __global__ void detectFunction_kernel107(const signed char iv2[261],
  const unsigned short uv1[256], const int i, const signed char
  thisTextU16_data[3], int *nrows);
static __global__ void detectFunction_kernel108(const unsigned char out[150528],
  unsigned char varargin_2[50176], unsigned char varargin_1[50176]);
static __global__ void detectFunction_kernel109(const unsigned char out[150528],
  unsigned char varargin_3[50176]);
static __global__ void detectFunction_kernel11(const double colWeights[1344],
  const int *status, double colWeightsTotal[224]);
static __global__ void detectFunction_kernel12(const double colWeightsTotal[224],
  const double colWeights[1344], const unsigned char img[230400], const short
  ipColIndices[1344], unsigned char partialResize[161280]);
static __global__ void detectFunction_kernel13(const double rowWeightsTotal[224],
  const double rowWeights[1120], const unsigned char partialResize[161280],
  const short ipRowIndices[1120], unsigned char out[150528]);
static __global__ void detectFunction_kernel14(short aux2[448], short aux1[448]);
static __global__ void detectFunction_kernel15(const short aux1[448], short
  ipRowIndices[896], double rowWeights[896]);
static __global__ void detectFunction_kernel16(const short aux2[448], short
  ipColIndices[896], double colWeights[896]);
static __global__ void detectFunction_kernel17(const double rowWeights[896],
  double rowWeightsTotal[128]);
static __global__ void detectFunction_kernel18(const double rowWeights[896],
  const int *status, double rowWeightsTotal[128]);
static __global__ void detectFunction_kernel19(const double colWeights[896],
  double colWeightsTotal[128]);
static __global__ void detectFunction_kernel2(const unsigned char pln1[76800],
  const unsigned char pln0[76800], unsigned char img[230400]);
static __global__ void detectFunction_kernel20(const double colWeights[896],
  const int *status, double colWeightsTotal[128]);
static __global__ void detectFunction_kernel21(const double rowWeightsTotal[128],
  const double rowWeights[896], const unsigned char out[150528], const short
  ipRowIndices[896], unsigned char partialResize[86016]);
static __global__ void detectFunction_kernel22(const double colWeightsTotal[128],
  const double colWeights[896], const unsigned char partialResize[86016], const
  short ipColIndices[896], unsigned char out[49152]);
static __global__ void detectFunction_kernel23(unsigned char out[49152],
  unsigned int castRed[2]);
static __global__ void detectFunction_kernel24(unsigned char outVal[2], unsigned
  int castRed[2]);
static __global__ void detectFunction_kernel25(const int *status, const short
  outVal, unsigned char out[49152], float b_out[49152]);
static __global__ void detectFunction_kernel26(const signed char dv[16], double
  anchors[16]);
static __global__ void detectFunction_kernel27(const double dv1[8], double
  anchors[16]);
static __global__ void detectFunction_kernel28(const double anchors[16], const
  float tmpFeatureMap[3072], float boxOut[3072]);
static __global__ void detectFunction_kernel29(const float boxOut[3072], bool
  b_data[512]);
static __global__ void detectFunction_kernel3(const unsigned char pln2[76800],
  unsigned char img[230400]);
static __global__ void detectFunction_kernel30(const float boxOut[3072], const
  short iv_data[512], const int thresholdedPrediction_size[2], const int
  iv_size[1], float thresholdedPrediction_data[3072]);
static __global__ void detectFunction_kernel31(emxArray_cell_wrap_7_512
  *labelCells);
static __global__ void detectFunction_kernel32(const float
  thresholdedPrediction_data[3072], const int thresholdedPrediction_size[2],
  const int bboxesX1Y1X2Y2_size[2], const int k, double bboxesX1Y1X2Y2_data[2048]);
static __global__ void detectFunction_kernel33(const double bboxesX1Y1X2Y2_data
  [2048], const int k, double x1_data[512]);
static __global__ void detectFunction_kernel34(const double bboxesX1Y1X2Y2_data
  [2048], const int bboxesX1Y1X2Y2_size[2], const int k, double y1_data[512]);
static __global__ void detectFunction_kernel35(const double bboxesX1Y1X2Y2_data
  [2048], const int bboxesX1Y1X2Y2_size[2], const int k, double x2_data[512]);
static __global__ void detectFunction_kernel36(const double bboxesX1Y1X2Y2_data
  [2048], const int bboxesX1Y1X2Y2_size[2], const int k, double y2_data[512]);
static __global__ void detectFunction_kernel37(const int *tbWidth, double
  x1_data[512]);
static __global__ void detectFunction_kernel38(const int *tbWidth, double
  y1_data[512]);
static __global__ void detectFunction_kernel39(const int *tbWidth, double
  x2_data[512]);
static __global__ void detectFunction_kernel4(short aux1[480]);
static __global__ void detectFunction_kernel40(const int *tbWidth, double
  y2_data[512]);
static __global__ void detectFunction_kernel41(const double x1_data[512], const
  int *status, double bboxesX1Y1X2Y2_data[2048]);
static __global__ void detectFunction_kernel42(const double y1_data[512], const
  int bboxesX1Y1X2Y2_size[2], const int *status, double bboxesX1Y1X2Y2_data[2048]);
static __global__ void detectFunction_kernel43(const double x2_data[512], const
  int bboxesX1Y1X2Y2_size[2], const int *status, double bboxesX1Y1X2Y2_data[2048]);
static __global__ void detectFunction_kernel44(const double y2_data[512], const
  int bboxesX1Y1X2Y2_size[2], const int *status, double bboxesX1Y1X2Y2_data[2048]);
static __global__ void detectFunction_kernel45(const double bboxesX1Y1X2Y2_data
  [2048], const int *status, double bboxPred_data[2048]);
static __global__ void detectFunction_kernel46(const double bboxesX1Y1X2Y2_data
  [2048], const int bboxesX1Y1X2Y2_size[2], const int bboxPred_size[2], const
  int *status, double bboxPred_data[2048]);
static __global__ void detectFunction_kernel47(const double bboxesX1Y1X2Y2_data
  [2048], const int bboxesX1Y1X2Y2_size[2], const int bboxPred_size[2], const
  int *status, double bboxPred_data[2048]);
static __global__ void detectFunction_kernel48(const double bboxesX1Y1X2Y2_data
  [2048], const int bboxesX1Y1X2Y2_size[2], const int bboxPred_size[2], const
  int *status, double bboxPred_data[2048]);
static __global__ void detectFunction_kernel49(const int *status, double
  bboxPred_data[2048]);
static __global__ void detectFunction_kernel5(short aux2[640]);
static __global__ void detectFunction_kernel50(const double bboxPred_data[2048],
  const int bboxPred_size[2], const int k, double x1_data[512]);
static __global__ void detectFunction_kernel51(const double x1_data[512], const
  int bboxPred_size[2], const int iv_size[1], double bboxPred_data[2048]);
static __global__ void detectFunction_kernel52(const double bboxPred_data[2048],
  const int bboxPred_size[2], const int k, double x1_data[512]);
static __global__ void detectFunction_kernel53(const double x1_data[512], const
  int bboxPred_size[2], const int iv_size[1], double bboxPred_data[2048]);
static __global__ void detectFunction_kernel54(const double bboxPred_data[2048],
  const int bboxPred_size[2], const int i, const int b_bboxPred_size[2], const
  int *nrows, double b_bboxPred_data[2048]);
static __global__ void detectFunction_kernel55(const float
  thresholdedPrediction_data[3072], const int thresholdedPrediction_size[2],
  const int i, const int count, float scores_data[512], float scorePred_data[512]);
static __global__ void detectFunction_kernel56(const short outVal, const short i,
  short idx_data[512]);
static __global__ void detectFunction_kernel57(const int bboxPred_size[2], bool
  b_data[512]);
static __global__ void detectFunction_kernel58(const bool b_data[512], int
  *status, int *tbWidth);
static __global__ void detectFunction_kernel59(const short idx_data[512], int
  *status);
static __global__ void detectFunction_kernel6(const short aux1[480], short
  ipRowIndices[1120], double rowWeights[1120]);
static __global__ void detectFunction_kernel60(const double bboxPred_data[2048],
  const int bboxPred_size[2], const int b_bboxPred_size[2], const int *status,
  double b_bboxPred_data[2048]);
static __global__ void detectFunction_kernel61(const double bboxPred_data[2048],
  const int bboxPred_size[2], double b_bboxPred_data[2048]);
static __global__ void detectFunction_kernel62(const short outVal, const short i,
  short idx_data[512]);
static __global__ void detectFunction_kernel63(const int scorePred_size[1], bool
  b_data[512]);
static __global__ void detectFunction_kernel64(const bool b_data[512], int
  *status, int *tbWidth);
static __global__ void detectFunction_kernel65(const short outVal, const short i,
  short idx_data[512]);
static __global__ void detectFunction_kernel66(const int iv_size[1], bool
  b_data[512]);
static __global__ void detectFunction_kernel67(const bool b_data[512], int
  *status, int *tbWidth);
static __global__ void detectFunction_kernel68(const float scores_data[512],
  const int iv_size[1], double y1_data[512]);
static __global__ void detectFunction_kernel69(const float scorePred_data[512],
  const int scorePred_size[1], float scores_data[512]);
static __global__ void detectFunction_kernel7(const short aux2[640], short
  ipColIndices[1344], double colWeights[1344]);
static __global__ void detectFunction_kernel70(const short dv2[2], double
  idx_data[512]);
static __global__ void detectFunction_kernel71(const double bboxPred_data[2048],
  const int bboxPred_size[2], const double idx_data[512], const int
  inputBbox_size[2], const int idx_size[1], double bboxesX1Y1X2Y2_data[2048]);
static __global__ void detectFunction_kernel72(const double y1_data[512], const
  double idx_data[512], const int idx_size[1], double x1_data[512]);
static __global__ void detectFunction_kernel73(const double x1_data[512], const
  int iv_size[1], double y1_data[512]);
static __global__ void detectFunction_kernel74(const int idx_size[1], bool
  b_data[512]);
static __global__ void detectFunction_kernel75(const double bboxesX1Y1X2Y2_data
  [2048], const int inputBbox_size[2], const int k, double x1_data[512]);
static __global__ void detectFunction_kernel76(const int inputBbox_size[2],
  const double bboxesX1Y1X2Y2_data[2048], const int k, double x2_data[512]);
static __global__ void detectFunction_kernel77(const double bboxesX1Y1X2Y2_data
  [2048], const int inputBbox_size[2], const int k, double y2_data[512]);
static __global__ void detectFunction_kernel78(const int *nrows, const int iv[2],
  bool b_data[512]);
static __global__ void detectFunction_kernel79(const bool b_data[512], const
  double idx_data[512], const int y1_size[1], bool index_data[512]);
static __global__ void detectFunction_kernel8(const double rowWeights[1120],
  double rowWeightsTotal[224]);
static __global__ void detectFunction_kernel80(const double bboxPred_data[2048],
  const int bboxPred_size[2], const int iv6_data[512], const int
  b_bboxPred_size[2], const int iv_size[1], double b_bboxPred_data[2048]);
static __global__ void detectFunction_kernel81(const double bboxPred_data[2048],
  const int bboxPred_size[2], double b_bboxPred_data[2048]);
static __global__ void detectFunction_kernel82(const float scorePred_data[512],
  const int scorePred_size[1], float scores_data[512]);
static __global__ void detectFunction_kernel83(const float scores_data[512],
  const int y1_size[1], float b_scores_data[512]);
static __global__ void detectFunction_kernel84(const int *ind, int v_size[1]);
static __global__ void detectFunction_kernel85(const char cv3[3], const int
  initAuxVar, char v_data[3]);
static __global__ void detectFunction_kernel86(const char v_data[3], const int
  *i, const int v_size[1], emxArray_cell_wrap_7_512 *labelCells);
static __global__ void detectFunction_kernel87(const double bboxPred_data[2048],
  const int bboxPred_size[2], const int *status, int position[4]);
static __global__ void detectFunction_kernel88(const unsigned char out[150528],
  unsigned char tmpRGB[150528]);
static __global__ void detectFunction_kernel89(const unsigned char uv[3],
  unsigned char color[3]);
static __global__ void detectFunction_kernel9(const double rowWeights[1120],
  const int *status, double rowWeightsTotal[224]);
static __global__ void detectFunction_kernel90(const int position[4], int
  positionOut[4]);
static __global__ void detectFunction_kernel91(unsigned char out[150528]);
static __global__ void detectFunction_kernel92(unsigned char pixCount[224]);
static __global__ void detectFunction_kernel93(const int position[4], int
  positionOut[4]);
static __global__ void detectFunction_kernel94(const int qY, int positionOut[4]);
static __global__ void detectFunction_kernel95(const unsigned char uv[3],
  unsigned char color[3]);
static __global__ void detectFunction_kernel96(const emxArray_cell_wrap_7_512
  *labelCells, const int *tbWidth, signed char thisTextU16_data[3]);
static __global__ void detectFunction_kernel97(const signed char
  thisTextU16_data[3], const int thisTextU16_size[2], signed char
  thisCharcodes_1b_data[3]);
static __global__ void detectFunction_kernel98(const signed char iv2[261], const
  unsigned short uv1[256], const signed char thisCharcodes_1b_data[3], const int
  thisCharcodes_1b_size[2], signed char x_data[3]);
static __global__ void detectFunction_kernel99(const signed char iv2[261], const
  unsigned short uv1[256], const signed char thisCharcodes_1b_data[3], double
  *height);
static __device__ double rt_powd_snf_device(double u0, double u1);
static __device__ double rt_roundd_snf_device(double u);
static __device__ unsigned int shflDown1(unsigned int in1, unsigned int offset,
  unsigned int mask);
static __device__ int shflDown1(int in1, unsigned int offset, unsigned int mask);
static __device__ double shflDown2(double in1, unsigned int offset, unsigned int
  mask);
static __device__ unsigned int threadGroupReduction(unsigned int val, unsigned
  int lane, unsigned int mask);
static __device__ double threadGroupReduction(double val, unsigned int lane,
  unsigned int mask);
static __device__ int threadGroupReduction(int val, unsigned int lane, unsigned
  int mask);
static __device__ unsigned int workGroupReduction(unsigned int val, unsigned int
  mask, unsigned int numActiveWarps);
static __device__ double workGroupReduction(double val, unsigned int mask,
  unsigned int numActiveWarps);
static __device__ int workGroupReduction(int val, unsigned int mask, unsigned
  int numActiveWarps);

// Function Definitions
//
// Arguments    : double *address
//                double value
// Return Type  : double
//
static __device__ double atomicOpreal_T(double *address, double value)
{
  unsigned long long int old;
  unsigned long long int *address_as_up;
  address_as_up = (unsigned long long int *)address;
  old = *address_as_up;
  unsigned long long int assumed;
  do {
    assumed = old;
    old = atomicCAS(address_as_up, old, __double_as_longlong(value +
      __longlong_as_double(old)));
  } while (assumed != old);

  return __longlong_as_double(old);
}

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
//                const char cv2[22]
//                char cv1[22]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void detectFunction_kernel1(const
  char cv2[22], char cv1[22])
{
  int i;
  i = static_cast<int>(mwGetGlobalThreadIndex());
  if (i < 22) {
    cv1[i] = cv2[i];
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double colWeights[1344]
//                double colWeightsTotal[224]
// Return Type  : void
//
static __global__ __launch_bounds__(224, 1) void detectFunction_kernel10(const
  double colWeights[1344], double colWeightsTotal[224])
{
  int i;
  i = static_cast<int>(mwGetGlobalThreadIndex());
  if (i < 224) {
    colWeightsTotal[i] = colWeights[i];
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const signed char x_data[3]
//                int *status
//                double *height
// Return Type  : void
//
static __global__ __launch_bounds__(1024, 1) void detectFunction_kernel100(const
  signed char x_data[3], int *status, double *height)
{
  double tmpRed0;
  long loopEnd;
  unsigned int blockStride;
  unsigned int idx;
  unsigned int m;
  unsigned int thBlkId;
  unsigned int threadId;
  unsigned int threadStride;
  threadStride = static_cast<unsigned int>(mwGetTotalThreadsLaunched());
  threadId = static_cast<unsigned int>(mwGetGlobalThreadIndex());
  thBlkId = static_cast<unsigned int>(mwGetThreadIndexWithinBlock());
  blockStride = static_cast<unsigned int>(mwGetThreadsPerBlock());
  tmpRed0 = 0.0;
  loopEnd = static_cast<long>(*status - 2);
  if (mwIsLastBlock()) {
    m = (static_cast<long>(*status - 2) + 1L) % static_cast<long>(blockStride);
    if (m > 0U) {
      blockStride = m;
    }
  }

  blockStride = (unsigned int)(blockStride + (static_cast<long>(warpSize) - 1L))
    / warpSize;
  if (static_cast<long>(threadId) <= loopEnd) {
    tmpRed0 = static_cast<double>(x_data[static_cast<int>(threadId) + 1]);
  }

  m = __ballot_sync(MAX_uint32_T, static_cast<long>(threadId) <= loopEnd);
  for (idx = threadId + threadStride; idx <= static_cast<unsigned int>(loopEnd);
       idx += threadStride) {
    tmpRed0 += static_cast<double>(x_data[static_cast<int>(idx) + 1]);
  }

  tmpRed0 = workGroupReduction(tmpRed0, m, blockStride);
  if (thBlkId == 0U) {
    atomicOpreal_T(&height[0], tmpRed0);
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const unsigned short uv1[256]
//                const signed char thisCharcodes_1b_data[3]
//                const int thisCharcodes_1b_size[2]
//                bool x_data[3]
// Return Type  : void
//
static __global__ __launch_bounds__(1024, 1) void detectFunction_kernel101(const
  unsigned short uv1[256], const signed char thisCharcodes_1b_data[3], const int
  thisCharcodes_1b_size[2], bool x_data[3])
{
  unsigned long idx;
  long loopEnd;
  unsigned long threadId;
  unsigned long threadStride;
  threadId = mwGetGlobalThreadIndex();
  threadStride = mwGetTotalThreadsLaunched();
  loopEnd = static_cast<long>(thisCharcodes_1b_size[1] - 1);
  for (idx = threadId; idx <= static_cast<unsigned long>(loopEnd); idx +=
       threadStride) {
    int i;
    i = static_cast<int>(idx);
    x_data[i] = (static_cast<int>(uv1[static_cast<int>(thisCharcodes_1b_data[i])
      - 1]) == 0);
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const bool x_data[3]
//                int *nrows
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void detectFunction_kernel102(const
  bool x_data[3], int *nrows)
{
  int tmpIdx;
  tmpIdx = static_cast<int>(mwGetGlobalThreadIndex());
  if (tmpIdx < 1) {
    *nrows = static_cast<int>(x_data[0]);
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const bool x_data[3]
//                int *status
//                int *nrows
// Return Type  : void
//
static __global__ __launch_bounds__(1024, 1) void detectFunction_kernel103(const
  bool x_data[3], int *status, int *nrows)
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
  loopEnd = static_cast<long>(*status - 2);
  if (mwIsLastBlock()) {
    m = (static_cast<long>(*status - 2) + 1L) % static_cast<long>(blockStride);
    if (m > 0U) {
      blockStride = m;
    }
  }

  blockStride = (unsigned int)(blockStride + (static_cast<long>(warpSize) - 1L))
    / warpSize;
  if (static_cast<long>(threadId) <= loopEnd) {
    tmpRed0 = static_cast<int>(x_data[static_cast<int>(threadId) + 1]);
  }

  m = __ballot_sync(MAX_uint32_T, static_cast<long>(threadId) <= loopEnd);
  for (idx = threadId + threadStride; idx <= static_cast<unsigned int>(loopEnd);
       idx += threadStride) {
    tmpRed0 += static_cast<int>(x_data[static_cast<int>(idx) + 1]);
  }

  tmpRed0 = workGroupReduction(tmpRed0, m, blockStride);
  if (thBlkId == 0U) {
    atomicAdd(&nrows[0], tmpRed0);
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const unsigned char uv5[10664]
//                const int *nrows
//                const int *status
//                unsigned char uv5_data[10664]
// Return Type  : void
//
static __global__ __launch_bounds__(1024, 1) void detectFunction_kernel104(const
  unsigned char uv5[10664], const int *nrows, const int *status, unsigned char
  uv5_data[10664])
{
  unsigned long idx;
  long loopEnd;
  unsigned long threadId;
  unsigned long threadStride;
  threadId = mwGetGlobalThreadIndex();
  threadStride = mwGetTotalThreadsLaunched();
  loopEnd = static_cast<long>(*status - *nrows);
  for (idx = threadId; idx <= static_cast<unsigned long>(loopEnd); idx +=
       threadStride) {
    int i;
    i = static_cast<int>(idx);
    uv5_data[i] = uv5[*nrows + i];
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const unsigned char uv5_data[10664]
//                const int uv5_size[2]
//                const signed char num[2]
//                unsigned char b_uv5_data[144]
// Return Type  : void
//
static __global__ __launch_bounds__(1024, 1) void detectFunction_kernel105(const
  unsigned char uv5_data[10664], const int uv5_size[2], const signed char num[2],
  unsigned char b_uv5_data[144])
{
  unsigned long idx;
  long loopEnd;
  unsigned long threadId;
  unsigned long threadStride;
  threadId = mwGetGlobalThreadIndex();
  threadStride = mwGetTotalThreadsLaunched();
  loopEnd = (static_cast<long>(static_cast<int>(num[1]) - 1) + 1L) * (
    static_cast<long>(static_cast<int>(num[0]) - 1) + 1L) - 1L;
  for (idx = threadId; idx <= static_cast<unsigned long>(loopEnd); idx +=
       threadStride) {
    int i;
    int ind;
    ind = static_cast<int>(idx % (static_cast<unsigned long>(static_cast<int>
      (num[1]) - 1) + 1UL));
    i = static_cast<int>((idx - static_cast<unsigned long>(ind)) / (static_cast<
      unsigned long>(static_cast<int>(num[1]) - 1) + 1UL));
    b_uv5_data[ind + uv5_size[0] * i] = uv5_data[i + static_cast<int>(num[0]) *
      ind];
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const unsigned char uv5_data[144]
//                const signed char num[2]
//                unsigned char thisGlyphBitmap_data[144]
// Return Type  : void
//
static __global__ __launch_bounds__(1024, 1) void detectFunction_kernel106(const
  unsigned char uv5_data[144], const signed char num[2], unsigned char
  thisGlyphBitmap_data[144])
{
  unsigned long idx;
  long loopEnd;
  unsigned long threadId;
  unsigned long threadStride;
  threadId = mwGetGlobalThreadIndex();
  threadStride = mwGetTotalThreadsLaunched();
  loopEnd = static_cast<long>(static_cast<int>(num[0]) * static_cast<int>(num[1])
    - 1);
  for (idx = threadId; idx <= static_cast<unsigned long>(loopEnd); idx +=
       threadStride) {
    int i;
    i = static_cast<int>(idx);
    thisGlyphBitmap_data[i] = uv5_data[i];
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const signed char iv2[261]
//                const unsigned short uv1[256]
//                const int i
//                const signed char thisTextU16_data[3]
//                int *nrows
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void detectFunction_kernel107(const
  signed char iv2[261], const unsigned short uv1[256], const int i, const signed
  char thisTextU16_data[3], int *nrows)
{
  int tmpIdx;
  tmpIdx = static_cast<int>(mwGetGlobalThreadIndex());
  if (tmpIdx < 1) {
    *nrows = static_cast<int>(iv2[uv1[thisTextU16_data[i]]]);
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const unsigned char out[150528]
//                unsigned char varargin_2[50176]
//                unsigned char varargin_1[50176]
// Return Type  : void
//
static __global__ __launch_bounds__(512, 1) void detectFunction_kernel108(const
  unsigned char out[150528], unsigned char varargin_2[50176], unsigned char
  varargin_1[50176])
{
  unsigned long threadId;
  int i;
  int ind;
  threadId = mwGetGlobalThreadIndex();
  ind = static_cast<int>(threadId % 224UL);
  i = static_cast<int>((threadId - static_cast<unsigned long>(ind)) / 224UL);
  if ((static_cast<int>(i < 224)) && (static_cast<int>(ind < 224))) {
    varargin_1[ind + 224 * i] = out[i + 224 * ind];
    varargin_2[ind + 224 * i] = out[(i + 224 * ind) + 50176];
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const unsigned char out[150528]
//                unsigned char varargin_3[50176]
// Return Type  : void
//
static __global__ __launch_bounds__(512, 1) void detectFunction_kernel109(const
  unsigned char out[150528], unsigned char varargin_3[50176])
{
  unsigned long threadId;
  int i;
  int ind;
  threadId = mwGetGlobalThreadIndex();
  ind = static_cast<int>(threadId % 224UL);
  i = static_cast<int>((threadId - static_cast<unsigned long>(ind)) / 224UL);
  if ((static_cast<int>(i < 224)) && (static_cast<int>(ind < 224))) {
    varargin_3[ind + 224 * i] = out[(i + 224 * ind) + 100352];
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double colWeights[1344]
//                const int *status
//                double colWeightsTotal[224]
// Return Type  : void
//
static __global__ __launch_bounds__(224, 1) void detectFunction_kernel11(const
  double colWeights[1344], const int *status, double colWeightsTotal[224])
{
  int i;
  i = static_cast<int>(mwGetGlobalThreadIndex());
  if (i < 224) {
    colWeightsTotal[i] += colWeights[*status + i];
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double colWeightsTotal[224]
//                const double colWeights[1344]
//                const unsigned char img[230400]
//                const short ipColIndices[1344]
//                unsigned char partialResize[161280]
// Return Type  : void
//
static __global__ __launch_bounds__(512, 1) void detectFunction_kernel12(const
  double colWeightsTotal[224], const double colWeights[1344], const unsigned
  char img[230400], const short ipColIndices[1344], unsigned char partialResize
  [161280])
{
  double sumVal;
  unsigned long threadId;
  int b_i;
  int colIdx;
  int rowIdx;
  threadId = mwGetGlobalThreadIndex();
  b_i = static_cast<int>(threadId % 3UL);
  threadId = (threadId - static_cast<unsigned long>(b_i)) / 3UL;
  colIdx = static_cast<int>(threadId % 224UL);
  threadId = (threadId - static_cast<unsigned long>(colIdx)) / 224UL;
  rowIdx = static_cast<int>(threadId);
  if ((static_cast<int>((static_cast<int>(rowIdx < 240)) && (static_cast<int>
         (colIdx < 224)))) && (static_cast<int>(b_i < 3))) {
    unsigned char u1;
    sumVal = 0.0;
    for (int i = 0; i < 6; i++) {
      sumVal += static_cast<double>(img[(rowIdx + 240 * (static_cast<int>
        (ipColIndices[colIdx + 224 * i]) - 1)) + 76800 * b_i]) *
        (colWeights[colIdx + 224 * i] / colWeightsTotal[colIdx]);
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

    partialResize[(rowIdx + 240 * colIdx) + 53760 * b_i] = u1;
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double rowWeightsTotal[224]
//                const double rowWeights[1120]
//                const unsigned char partialResize[161280]
//                const short ipRowIndices[1120]
//                unsigned char out[150528]
// Return Type  : void
//
static __global__ __launch_bounds__(512, 1) void detectFunction_kernel13(const
  double rowWeightsTotal[224], const double rowWeights[1120], const unsigned
  char partialResize[161280], const short ipRowIndices[1120], unsigned char out
  [150528])
{
  double sumVal;
  unsigned long threadId;
  int b_i;
  int colIdx;
  int rowIdx;
  threadId = mwGetGlobalThreadIndex();
  b_i = static_cast<int>(threadId % 3UL);
  threadId = (threadId - static_cast<unsigned long>(b_i)) / 3UL;
  rowIdx = static_cast<int>(threadId % 224UL);
  threadId = (threadId - static_cast<unsigned long>(rowIdx)) / 224UL;
  colIdx = static_cast<int>(threadId);
  if ((static_cast<int>((static_cast<int>(colIdx < 224)) && (static_cast<int>
         (rowIdx < 224)))) && (static_cast<int>(b_i < 3))) {
    unsigned char u1;
    sumVal = 0.0;
    for (int i = 0; i < 5; i++) {
      sumVal += static_cast<double>(partialResize[((static_cast<int>
        (ipRowIndices[rowIdx + 224 * i]) + 240 * colIdx) + 53760 * b_i) - 1]) *
        (rowWeights[rowIdx + 224 * i] / rowWeightsTotal[rowIdx]);
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

    out[(rowIdx + 224 * colIdx) + 50176 * b_i] = u1;
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                short aux2[448]
//                short aux1[448]
// Return Type  : void
//
static __global__ __launch_bounds__(448, 1) void detectFunction_kernel14(short
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
static __global__ __launch_bounds__(512, 1) void detectFunction_kernel15(const
  short aux1[448], short ipRowIndices[896], double rowWeights[896])
{
  unsigned long threadId;
  int i;
  int k;
  int rowIdx;
  threadId = mwGetGlobalThreadIndex();
  k = static_cast<int>(threadId % 7UL);
  rowIdx = static_cast<int>((threadId - static_cast<unsigned long>(k)) / 7UL);
  if ((static_cast<int>(rowIdx < 128)) && (static_cast<int>(k < 7))) {
    double absx;
    double absx2;
    double sumVal;
    int ind;
    sumVal = (static_cast<double>(rowIdx) + 1.0) / 0.5714285714285714 + -0.375;
    i = static_cast<int>(floor(sumVal - 3.5));
    absx = fabs(0.5714285714285714 * (sumVal - (static_cast<double>(i + k) + 1.0)));
    absx2 = absx * absx;
    sumVal = rt_powd_snf_device(absx, 3.0);
    rowWeights[rowIdx + (k << 7)] = 0.5714285714285714 * (((1.5 * sumVal - 2.5 *
      absx2) + 1.0) * static_cast<double>(absx <= 1.0) + (((-0.5 * sumVal + 2.5 *
      absx2) - 4.0 * absx) + 2.0) * static_cast<double>((static_cast<int>(1.0 <
      absx)) && (static_cast<int>(absx <= 2.0))));
    ind = (i + k) + 1;
    if (ind - 1 == 0) {
      i = 0;
    } else {
      i = static_cast<int>(fmod(static_cast<double>(ind) - 1.0, 448.0));
      if ((static_cast<int>(i != 0)) && (static_cast<int>(ind - 1 < 0))) {
        i += 448;
      }
    }

    ipRowIndices[rowIdx + (k << 7)] = aux1[i];
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
static __global__ __launch_bounds__(512, 1) void detectFunction_kernel16(const
  short aux2[448], short ipColIndices[896], double colWeights[896])
{
  unsigned long threadId;
  int colIdx;
  int i;
  int k;
  threadId = mwGetGlobalThreadIndex();
  k = static_cast<int>(threadId % 7UL);
  colIdx = static_cast<int>((threadId - static_cast<unsigned long>(k)) / 7UL);
  if ((static_cast<int>(colIdx < 128)) && (static_cast<int>(k < 7))) {
    double absx;
    double absx2;
    double sumVal;
    int ind;
    sumVal = (static_cast<double>(colIdx) + 1.0) / 0.5714285714285714 + -0.375;
    i = static_cast<int>(floor(sumVal - 3.5));
    absx = fabs(0.5714285714285714 * (sumVal - (static_cast<double>(i + k) + 1.0)));
    absx2 = absx * absx;
    sumVal = rt_powd_snf_device(absx, 3.0);
    colWeights[colIdx + (k << 7)] = 0.5714285714285714 * (((1.5 * sumVal - 2.5 *
      absx2) + 1.0) * static_cast<double>(absx <= 1.0) + (((-0.5 * sumVal + 2.5 *
      absx2) - 4.0 * absx) + 2.0) * static_cast<double>((static_cast<int>(1.0 <
      absx)) && (static_cast<int>(absx <= 2.0))));
    ind = (i + k) + 1;
    if (ind - 1 == 0) {
      i = 0;
    } else {
      i = static_cast<int>(fmod(static_cast<double>(ind) - 1.0, 448.0));
      if ((static_cast<int>(i != 0)) && (static_cast<int>(ind - 1 < 0))) {
        i += 448;
      }
    }

    ipColIndices[colIdx + (k << 7)] = aux2[i];
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double rowWeights[896]
//                double rowWeightsTotal[128]
// Return Type  : void
//
static __global__ __launch_bounds__(128, 1) void detectFunction_kernel17(const
  double rowWeights[896], double rowWeightsTotal[128])
{
  int i;
  i = static_cast<int>(mwGetGlobalThreadIndex());
  if (i < 128) {
    rowWeightsTotal[i] = rowWeights[i];
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double rowWeights[896]
//                const int *status
//                double rowWeightsTotal[128]
// Return Type  : void
//
static __global__ __launch_bounds__(128, 1) void detectFunction_kernel18(const
  double rowWeights[896], const int *status, double rowWeightsTotal[128])
{
  int i;
  i = static_cast<int>(mwGetGlobalThreadIndex());
  if (i < 128) {
    rowWeightsTotal[i] += rowWeights[*status + i];
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double colWeights[896]
//                double colWeightsTotal[128]
// Return Type  : void
//
static __global__ __launch_bounds__(128, 1) void detectFunction_kernel19(const
  double colWeights[896], double colWeightsTotal[128])
{
  int i;
  i = static_cast<int>(mwGetGlobalThreadIndex());
  if (i < 128) {
    colWeightsTotal[i] = colWeights[i];
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const unsigned char pln1[76800]
//                const unsigned char pln0[76800]
//                unsigned char img[230400]
// Return Type  : void
//
static __global__ __launch_bounds__(512, 1) void detectFunction_kernel2(const
  unsigned char pln1[76800], const unsigned char pln0[76800], unsigned char img
  [230400])
{
  unsigned long threadId;
  int i;
  int ind;
  threadId = mwGetGlobalThreadIndex();
  ind = static_cast<int>(threadId % 240UL);
  i = static_cast<int>((threadId - static_cast<unsigned long>(ind)) / 240UL);
  if ((static_cast<int>(i < 320)) && (static_cast<int>(ind < 240))) {
    img[ind + 240 * i] = pln0[i + 320 * ind];
    img[(ind + 240 * i) + 76800] = pln1[i + 320 * ind];
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double colWeights[896]
//                const int *status
//                double colWeightsTotal[128]
// Return Type  : void
//
static __global__ __launch_bounds__(128, 1) void detectFunction_kernel20(const
  double colWeights[896], const int *status, double colWeightsTotal[128])
{
  int i;
  i = static_cast<int>(mwGetGlobalThreadIndex());
  if (i < 128) {
    colWeightsTotal[i] += colWeights[*status + i];
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
static __global__ __launch_bounds__(512, 1) void detectFunction_kernel21(const
  double rowWeightsTotal[128], const double rowWeights[896], const unsigned char
  out[150528], const short ipRowIndices[896], unsigned char partialResize[86016])
{
  double sumVal;
  unsigned long threadId;
  int b_i;
  int colIdx;
  int rowIdx;
  threadId = mwGetGlobalThreadIndex();
  b_i = static_cast<int>(threadId % 3UL);
  threadId = (threadId - static_cast<unsigned long>(b_i)) / 3UL;
  rowIdx = static_cast<int>(threadId % 128UL);
  threadId = (threadId - static_cast<unsigned long>(rowIdx)) / 128UL;
  colIdx = static_cast<int>(threadId);
  if ((static_cast<int>((static_cast<int>(colIdx < 224)) && (static_cast<int>
         (rowIdx < 128)))) && (static_cast<int>(b_i < 3))) {
    unsigned char u1;
    sumVal = 0.0;
    for (int i = 0; i < 7; i++) {
      sumVal += static_cast<double>(out[((static_cast<int>(ipRowIndices[rowIdx +
        (i << 7)]) + 224 * colIdx) + 50176 * b_i) - 1]) * (rowWeights[rowIdx +
        (i << 7)] / rowWeightsTotal[rowIdx]);
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

    partialResize[(rowIdx + (colIdx << 7)) + 28672 * b_i] = u1;
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
static __global__ __launch_bounds__(512, 1) void detectFunction_kernel22(const
  double colWeightsTotal[128], const double colWeights[896], const unsigned char
  partialResize[86016], const short ipColIndices[896], unsigned char out[49152])
{
  double sumVal;
  unsigned long threadId;
  int b_i;
  int colIdx;
  int rowIdx;
  threadId = mwGetGlobalThreadIndex();
  b_i = static_cast<int>(threadId % 3UL);
  threadId = (threadId - static_cast<unsigned long>(b_i)) / 3UL;
  rowIdx = static_cast<int>(threadId % 128UL);
  threadId = (threadId - static_cast<unsigned long>(rowIdx)) / 128UL;
  colIdx = static_cast<int>(threadId);
  if ((static_cast<int>((static_cast<int>(colIdx < 128)) && (static_cast<int>
         (rowIdx < 128)))) && (static_cast<int>(b_i < 3))) {
    unsigned char u1;
    sumVal = 0.0;
    for (int i = 0; i < 7; i++) {
      sumVal += static_cast<double>(partialResize[(rowIdx + ((static_cast<int>
        (ipColIndices[colIdx + (i << 7)]) - 1) << 7)) + 28672 * b_i]) *
        (colWeights[colIdx + (i << 7)] / colWeightsTotal[colIdx]);
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

    out[(rowIdx + (colIdx << 7)) + (b_i << 14)] = u1;
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                unsigned char out[49152]
//                unsigned int castRed[2]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void detectFunction_kernel23(unsigned
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
static __global__ __launch_bounds__(32, 1) void detectFunction_kernel24(unsigned
  char outVal[2], unsigned int castRed[2])
{
  int i;
  i = static_cast<int>(mwGetGlobalThreadIndex());
  if (i < 2) {
    unsigned int u;
    u = castRed[i];
    if (u > 255U) {
      u = 255U;
    }

    outVal[i] = static_cast<unsigned char>(u);
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const int *status
//                const short outVal
//                unsigned char out[49152]
//                float b_out[49152]
// Return Type  : void
//
static __global__ __launch_bounds__(512, 1) void detectFunction_kernel25(const
  int *status, const short outVal, unsigned char out[49152], float b_out[49152])
{
  int i;
  i = static_cast<int>(mwGetGlobalThreadIndex());
  if (i < 49152) {
    b_out[i] = static_cast<float>(static_cast<int>(out[i]) - static_cast<int>
      (outVal)) / static_cast<float>(*status);
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const signed char dv[16]
//                double anchors[16]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void detectFunction_kernel26(const
  signed char dv[16], double anchors[16])
{
  int i;
  i = static_cast<int>(mwGetGlobalThreadIndex());
  if (i < 16) {
    anchors[i] = static_cast<double>(dv[i]);
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double dv1[8]
//                double anchors[16]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void detectFunction_kernel27(const
  double dv1[8], double anchors[16])
{
  int i;
  i = static_cast<int>(mwGetGlobalThreadIndex());
  if (i < 8) {
    anchors[i] = dv1[i];
    anchors[i + 8] /= 16.0;
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double anchors[16]
//                const float tmpFeatureMap[3072]
//                float boxOut[3072]
// Return Type  : void
//
static __global__ __launch_bounds__(512, 1) void detectFunction_kernel28(const
  double anchors[16], const float tmpFeatureMap[3072], float boxOut[3072])
{
  unsigned long threadId;
  int colIdx;
  int i;
  int rowIdx;
  threadId = mwGetGlobalThreadIndex();
  rowIdx = static_cast<int>(threadId % 8UL);
  threadId = (threadId - static_cast<unsigned long>(rowIdx)) / 8UL;
  colIdx = static_cast<int>(threadId % 8UL);
  threadId = (threadId - static_cast<unsigned long>(colIdx)) / 8UL;
  i = static_cast<int>(threadId);
  if ((static_cast<int>((static_cast<int>(i < 8)) && (static_cast<int>(colIdx <
          8)))) && (static_cast<int>(rowIdx < 8))) {
    float bh;
    float bw;
    float cx;
    float cy;
    int ind;
    ind = (((rowIdx << 6) + (colIdx << 3)) + i) + 1;
    cx = (tmpFeatureMap[((rowIdx + (colIdx << 3)) + (i << 6)) + 512] +
          static_cast<float>(colIdx)) * 16.0F;
    cy = (tmpFeatureMap[((rowIdx + (colIdx << 3)) + (i << 6)) + 1024] +
          static_cast<float>(rowIdx)) * 16.0F;
    bw = tmpFeatureMap[((rowIdx + (colIdx << 3)) + (i << 6)) + 1536] *
      static_cast<float>(anchors[i + 8]) * 16.0F;
    bh = tmpFeatureMap[((rowIdx + (colIdx << 3)) + (i << 6)) + 2048] *
      static_cast<float>(anchors[i]) * 16.0F;
    boxOut[ind - 1] = cx - bw / 2.0F;
    boxOut[ind + 511] = cy - bh / 2.0F;
    boxOut[ind + 1023] = cx + bw / 2.0F;
    boxOut[ind + 1535] = cy + bh / 2.0F;
    boxOut[ind + 2047] = tmpFeatureMap[(rowIdx + (colIdx << 3)) + (i << 6)] *
      tmpFeatureMap[((rowIdx + (colIdx << 3)) + (i << 6)) + 2560];
    boxOut[ind + 2559] = 1.0F;
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const float boxOut[3072]
//                bool b_data[512]
// Return Type  : void
//
static __global__ __launch_bounds__(512, 1) void detectFunction_kernel29(const
  float boxOut[3072], bool b_data[512])
{
  int i;
  i = static_cast<int>(mwGetGlobalThreadIndex());
  if (i < 512) {
    b_data[i] = (static_cast<double>(boxOut[i + 2048]) >= 0.6);
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const unsigned char pln2[76800]
//                unsigned char img[230400]
// Return Type  : void
//
static __global__ __launch_bounds__(512, 1) void detectFunction_kernel3(const
  unsigned char pln2[76800], unsigned char img[230400])
{
  unsigned long threadId;
  int i;
  int ind;
  threadId = mwGetGlobalThreadIndex();
  ind = static_cast<int>(threadId % 240UL);
  i = static_cast<int>((threadId - static_cast<unsigned long>(ind)) / 240UL);
  if ((static_cast<int>(i < 320)) && (static_cast<int>(ind < 240))) {
    img[(ind + 240 * i) + 153600] = pln2[i + 320 * ind];
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const float boxOut[3072]
//                const short iv_data[512]
//                const int thresholdedPrediction_size[2]
//                const int iv_size[1]
//                float thresholdedPrediction_data[3072]
// Return Type  : void
//
static __global__ __launch_bounds__(1024, 1) void detectFunction_kernel30(const
  float boxOut[3072], const short iv_data[512], const int
  thresholdedPrediction_size[2], const int iv_size[1], float
  thresholdedPrediction_data[3072])
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
    int i;
    int ind;
    ind = static_cast<int>(idx % (static_cast<unsigned long>(iv_size[0] - 1) +
      1UL));
    i = static_cast<int>((idx - static_cast<unsigned long>(ind)) / (static_cast<
      unsigned long>(iv_size[0] - 1) + 1UL));
    thresholdedPrediction_data[ind + thresholdedPrediction_size[0] * i] =
      boxOut[(static_cast<int>(iv_data[ind]) + (i << 9)) - 1];
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                emxArray_cell_wrap_7_512 *labelCells
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void detectFunction_kernel31
  (emxArray_cell_wrap_7_512 *labelCells)
{
  int tmpIdx;
  tmpIdx = static_cast<int>(mwGetGlobalThreadIndex());
  if (tmpIdx < 1) {
    labelCells->size[0] = 0;
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const float thresholdedPrediction_data[3072]
//                const int thresholdedPrediction_size[2]
//                const int bboxesX1Y1X2Y2_size[2]
//                const int k
//                double bboxesX1Y1X2Y2_data[2048]
// Return Type  : void
//
static __global__ __launch_bounds__(1024, 1) void detectFunction_kernel32(const
  float thresholdedPrediction_data[3072], const int thresholdedPrediction_size[2],
  const int bboxesX1Y1X2Y2_size[2], const int k, double bboxesX1Y1X2Y2_data[2048])
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
    int i;
    int ind;
    ind = static_cast<int>(idx % (static_cast<unsigned long>(k) + 1UL));
    i = static_cast<int>((idx - static_cast<unsigned long>(ind)) / (static_cast<
      unsigned long>(k) + 1UL));
    bboxesX1Y1X2Y2_data[ind + bboxesX1Y1X2Y2_size[0] * i] = static_cast<double>
      (thresholdedPrediction_data[ind + thresholdedPrediction_size[0] * i]);
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double bboxesX1Y1X2Y2_data[2048]
//                const int k
//                double x1_data[512]
// Return Type  : void
//
static __global__ __launch_bounds__(1024, 1) void detectFunction_kernel33(const
  double bboxesX1Y1X2Y2_data[2048], const int k, double x1_data[512])
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
    int i;
    i = static_cast<int>(idx);
    x1_data[i] = bboxesX1Y1X2Y2_data[i];
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double bboxesX1Y1X2Y2_data[2048]
//                const int bboxesX1Y1X2Y2_size[2]
//                const int k
//                double y1_data[512]
// Return Type  : void
//
static __global__ __launch_bounds__(1024, 1) void detectFunction_kernel34(const
  double bboxesX1Y1X2Y2_data[2048], const int bboxesX1Y1X2Y2_size[2], const int
  k, double y1_data[512])
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
    int i;
    i = static_cast<int>(idx);
    y1_data[i] = bboxesX1Y1X2Y2_data[i + bboxesX1Y1X2Y2_size[0]];
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double bboxesX1Y1X2Y2_data[2048]
//                const int bboxesX1Y1X2Y2_size[2]
//                const int k
//                double x2_data[512]
// Return Type  : void
//
static __global__ __launch_bounds__(1024, 1) void detectFunction_kernel35(const
  double bboxesX1Y1X2Y2_data[2048], const int bboxesX1Y1X2Y2_size[2], const int
  k, double x2_data[512])
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
    int i;
    i = static_cast<int>(idx);
    x2_data[i] = bboxesX1Y1X2Y2_data[i + (bboxesX1Y1X2Y2_size[0] << 1)];
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double bboxesX1Y1X2Y2_data[2048]
//                const int bboxesX1Y1X2Y2_size[2]
//                const int k
//                double y2_data[512]
// Return Type  : void
//
static __global__ __launch_bounds__(1024, 1) void detectFunction_kernel36(const
  double bboxesX1Y1X2Y2_data[2048], const int bboxesX1Y1X2Y2_size[2], const int
  k, double y2_data[512])
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
    int i;
    i = static_cast<int>(idx);
    y2_data[i] = bboxesX1Y1X2Y2_data[i + bboxesX1Y1X2Y2_size[0] * 3];
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const int *tbWidth
//                double x1_data[512]
// Return Type  : void
//
static __global__ __launch_bounds__(1024, 1) void detectFunction_kernel37(const
  int *tbWidth, double x1_data[512])
{
  unsigned long idx;
  long loopEnd;
  unsigned long threadId;
  unsigned long threadStride;
  threadId = mwGetGlobalThreadIndex();
  threadStride = mwGetTotalThreadsLaunched();
  loopEnd = static_cast<long>(*tbWidth - 1);
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
//                const int *tbWidth
//                double y1_data[512]
// Return Type  : void
//
static __global__ __launch_bounds__(1024, 1) void detectFunction_kernel38(const
  int *tbWidth, double y1_data[512])
{
  unsigned long idx;
  long loopEnd;
  unsigned long threadId;
  unsigned long threadStride;
  threadId = mwGetGlobalThreadIndex();
  threadStride = mwGetTotalThreadsLaunched();
  loopEnd = static_cast<long>(*tbWidth - 1);
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
//                const int *tbWidth
//                double x2_data[512]
// Return Type  : void
//
static __global__ __launch_bounds__(1024, 1) void detectFunction_kernel39(const
  int *tbWidth, double x2_data[512])
{
  unsigned long idx;
  long loopEnd;
  unsigned long threadId;
  unsigned long threadStride;
  threadId = mwGetGlobalThreadIndex();
  threadStride = mwGetTotalThreadsLaunched();
  loopEnd = static_cast<long>(*tbWidth - 1);
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
//                short aux1[480]
// Return Type  : void
//
static __global__ __launch_bounds__(480, 1) void detectFunction_kernel4(short
  aux1[480])
{
  int i;
  i = static_cast<int>(mwGetGlobalThreadIndex());
  if (i < 480) {
    if (i + 1 <= 240) {
      aux1[i] = static_cast<short>(i + 1);
    } else {
      aux1[i] = static_cast<short>(480 - i);
    }
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const int *tbWidth
//                double y2_data[512]
// Return Type  : void
//
static __global__ __launch_bounds__(1024, 1) void detectFunction_kernel40(const
  int *tbWidth, double y2_data[512])
{
  unsigned long idx;
  long loopEnd;
  unsigned long threadId;
  unsigned long threadStride;
  threadId = mwGetGlobalThreadIndex();
  threadStride = mwGetTotalThreadsLaunched();
  loopEnd = static_cast<long>(*tbWidth - 1);
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
//                const double x1_data[512]
//                const int *status
//                double bboxesX1Y1X2Y2_data[2048]
// Return Type  : void
//
static __global__ __launch_bounds__(1024, 1) void detectFunction_kernel41(const
  double x1_data[512], const int *status, double bboxesX1Y1X2Y2_data[2048])
{
  unsigned long idx;
  long loopEnd;
  unsigned long threadId;
  unsigned long threadStride;
  threadId = mwGetGlobalThreadIndex();
  threadStride = mwGetTotalThreadsLaunched();
  loopEnd = static_cast<long>(*status);
  for (idx = threadId; idx <= static_cast<unsigned long>(loopEnd); idx +=
       threadStride) {
    int i;
    i = static_cast<int>(idx);
    bboxesX1Y1X2Y2_data[i] = x1_data[i];
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double y1_data[512]
//                const int bboxesX1Y1X2Y2_size[2]
//                const int *status
//                double bboxesX1Y1X2Y2_data[2048]
// Return Type  : void
//
static __global__ __launch_bounds__(1024, 1) void detectFunction_kernel42(const
  double y1_data[512], const int bboxesX1Y1X2Y2_size[2], const int *status,
  double bboxesX1Y1X2Y2_data[2048])
{
  unsigned long idx;
  long loopEnd;
  unsigned long threadId;
  unsigned long threadStride;
  threadId = mwGetGlobalThreadIndex();
  threadStride = mwGetTotalThreadsLaunched();
  loopEnd = static_cast<long>(*status);
  for (idx = threadId; idx <= static_cast<unsigned long>(loopEnd); idx +=
       threadStride) {
    int i;
    i = static_cast<int>(idx);
    bboxesX1Y1X2Y2_data[i + bboxesX1Y1X2Y2_size[0]] = y1_data[i];
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double x2_data[512]
//                const int bboxesX1Y1X2Y2_size[2]
//                const int *status
//                double bboxesX1Y1X2Y2_data[2048]
// Return Type  : void
//
static __global__ __launch_bounds__(1024, 1) void detectFunction_kernel43(const
  double x2_data[512], const int bboxesX1Y1X2Y2_size[2], const int *status,
  double bboxesX1Y1X2Y2_data[2048])
{
  unsigned long idx;
  long loopEnd;
  unsigned long threadId;
  unsigned long threadStride;
  threadId = mwGetGlobalThreadIndex();
  threadStride = mwGetTotalThreadsLaunched();
  loopEnd = static_cast<long>(*status);
  for (idx = threadId; idx <= static_cast<unsigned long>(loopEnd); idx +=
       threadStride) {
    int i;
    i = static_cast<int>(idx);
    bboxesX1Y1X2Y2_data[i + (bboxesX1Y1X2Y2_size[0] << 1)] = x2_data[i];
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double y2_data[512]
//                const int bboxesX1Y1X2Y2_size[2]
//                const int *status
//                double bboxesX1Y1X2Y2_data[2048]
// Return Type  : void
//
static __global__ __launch_bounds__(1024, 1) void detectFunction_kernel44(const
  double y2_data[512], const int bboxesX1Y1X2Y2_size[2], const int *status,
  double bboxesX1Y1X2Y2_data[2048])
{
  unsigned long idx;
  long loopEnd;
  unsigned long threadId;
  unsigned long threadStride;
  threadId = mwGetGlobalThreadIndex();
  threadStride = mwGetTotalThreadsLaunched();
  loopEnd = static_cast<long>(*status);
  for (idx = threadId; idx <= static_cast<unsigned long>(loopEnd); idx +=
       threadStride) {
    int i;
    i = static_cast<int>(idx);
    bboxesX1Y1X2Y2_data[i + bboxesX1Y1X2Y2_size[0] * 3] = y2_data[i];
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double bboxesX1Y1X2Y2_data[2048]
//                const int *status
//                double bboxPred_data[2048]
// Return Type  : void
//
static __global__ __launch_bounds__(1024, 1) void detectFunction_kernel45(const
  double bboxesX1Y1X2Y2_data[2048], const int *status, double bboxPred_data[2048])
{
  unsigned long idx;
  long loopEnd;
  unsigned long threadId;
  unsigned long threadStride;
  threadId = mwGetGlobalThreadIndex();
  threadStride = mwGetTotalThreadsLaunched();
  loopEnd = static_cast<long>(*status);
  for (idx = threadId; idx <= static_cast<unsigned long>(loopEnd); idx +=
       threadStride) {
    int i;
    i = static_cast<int>(idx);
    bboxPred_data[i] = ((bboxesX1Y1X2Y2_data[i] - 0.5) * 1.75 + -0.375) + 0.5;
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double bboxesX1Y1X2Y2_data[2048]
//                const int bboxesX1Y1X2Y2_size[2]
//                const int bboxPred_size[2]
//                const int *status
//                double bboxPred_data[2048]
// Return Type  : void
//
static __global__ __launch_bounds__(1024, 1) void detectFunction_kernel46(const
  double bboxesX1Y1X2Y2_data[2048], const int bboxesX1Y1X2Y2_size[2], const int
  bboxPred_size[2], const int *status, double bboxPred_data[2048])
{
  unsigned long idx;
  long loopEnd;
  unsigned long threadId;
  unsigned long threadStride;
  threadId = mwGetGlobalThreadIndex();
  threadStride = mwGetTotalThreadsLaunched();
  loopEnd = static_cast<long>(*status);
  for (idx = threadId; idx <= static_cast<unsigned long>(loopEnd); idx +=
       threadStride) {
    int i;
    i = static_cast<int>(idx);
    bboxPred_data[i + bboxPred_size[0]] = ((bboxesX1Y1X2Y2_data[i +
      bboxesX1Y1X2Y2_size[0]] - 0.5) * 1.75 + -0.375) + 0.5;
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double bboxesX1Y1X2Y2_data[2048]
//                const int bboxesX1Y1X2Y2_size[2]
//                const int bboxPred_size[2]
//                const int *status
//                double bboxPred_data[2048]
// Return Type  : void
//
static __global__ __launch_bounds__(1024, 1) void detectFunction_kernel47(const
  double bboxesX1Y1X2Y2_data[2048], const int bboxesX1Y1X2Y2_size[2], const int
  bboxPred_size[2], const int *status, double bboxPred_data[2048])
{
  unsigned long idx;
  long loopEnd;
  unsigned long threadId;
  unsigned long threadStride;
  threadId = mwGetGlobalThreadIndex();
  threadStride = mwGetTotalThreadsLaunched();
  loopEnd = static_cast<long>(*status);
  for (idx = threadId; idx <= static_cast<unsigned long>(loopEnd); idx +=
       threadStride) {
    int i;
    i = static_cast<int>(idx);
    bboxPred_data[i + (bboxPred_size[0] << 1)] = ((bboxesX1Y1X2Y2_data[i +
      (bboxesX1Y1X2Y2_size[0] << 1)] + 0.5) * 1.75 + -0.375) - 0.5;
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double bboxesX1Y1X2Y2_data[2048]
//                const int bboxesX1Y1X2Y2_size[2]
//                const int bboxPred_size[2]
//                const int *status
//                double bboxPred_data[2048]
// Return Type  : void
//
static __global__ __launch_bounds__(1024, 1) void detectFunction_kernel48(const
  double bboxesX1Y1X2Y2_data[2048], const int bboxesX1Y1X2Y2_size[2], const int
  bboxPred_size[2], const int *status, double bboxPred_data[2048])
{
  unsigned long idx;
  long loopEnd;
  unsigned long threadId;
  unsigned long threadStride;
  threadId = mwGetGlobalThreadIndex();
  threadStride = mwGetTotalThreadsLaunched();
  loopEnd = static_cast<long>(*status);
  for (idx = threadId; idx <= static_cast<unsigned long>(loopEnd); idx +=
       threadStride) {
    int i;
    i = static_cast<int>(idx);
    bboxPred_data[i + bboxPred_size[0] * 3] = ((bboxesX1Y1X2Y2_data[i +
      bboxesX1Y1X2Y2_size[0] * 3] + 0.5) * 1.75 + -0.375) - 0.5;
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const int *status
//                double bboxPred_data[2048]
// Return Type  : void
//
static __global__ __launch_bounds__(1024, 1) void detectFunction_kernel49(const
  int *status, double bboxPred_data[2048])
{
  unsigned long idx;
  long loopEnd;
  unsigned long threadId;
  unsigned long threadStride;
  threadId = mwGetGlobalThreadIndex();
  threadStride = mwGetTotalThreadsLaunched();
  loopEnd = static_cast<long>(*status - 1);
  for (idx = threadId; idx <= static_cast<unsigned long>(loopEnd); idx +=
       threadStride) {
    int k;
    k = static_cast<int>(idx);
    bboxPred_data[k] = floor(bboxPred_data[k]);
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                short aux2[640]
// Return Type  : void
//
static __global__ __launch_bounds__(512, 1) void detectFunction_kernel5(short
  aux2[640])
{
  int i;
  i = static_cast<int>(mwGetGlobalThreadIndex());
  if (i < 640) {
    if (i + 1 <= 320) {
      aux2[i] = static_cast<short>(i + 1);
    } else {
      aux2[i] = static_cast<short>(640 - i);
    }
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double bboxPred_data[2048]
//                const int bboxPred_size[2]
//                const int k
//                double x1_data[512]
// Return Type  : void
//
static __global__ __launch_bounds__(1024, 1) void detectFunction_kernel50(const
  double bboxPred_data[2048], const int bboxPred_size[2], const int k, double
  x1_data[512])
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
    int i;
    i = static_cast<int>(idx);
    x1_data[i] = (bboxPred_data[i + (bboxPred_size[0] << 1)] - bboxPred_data[i])
      + 1.0;
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double x1_data[512]
//                const int bboxPred_size[2]
//                const int iv_size[1]
//                double bboxPred_data[2048]
// Return Type  : void
//
static __global__ __launch_bounds__(1024, 1) void detectFunction_kernel51(const
  double x1_data[512], const int bboxPred_size[2], const int iv_size[1], double
  bboxPred_data[2048])
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
    int i;
    i = static_cast<int>(idx);
    bboxPred_data[i + (bboxPred_size[0] << 1)] = x1_data[i];
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double bboxPred_data[2048]
//                const int bboxPred_size[2]
//                const int k
//                double x1_data[512]
// Return Type  : void
//
static __global__ __launch_bounds__(1024, 1) void detectFunction_kernel52(const
  double bboxPred_data[2048], const int bboxPred_size[2], const int k, double
  x1_data[512])
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
    int i;
    i = static_cast<int>(idx);
    x1_data[i] = (bboxPred_data[i + bboxPred_size[0] * 3] - bboxPred_data[i +
                  bboxPred_size[0]]) + 1.0;
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double x1_data[512]
//                const int bboxPred_size[2]
//                const int iv_size[1]
//                double bboxPred_data[2048]
// Return Type  : void
//
static __global__ __launch_bounds__(1024, 1) void detectFunction_kernel53(const
  double x1_data[512], const int bboxPred_size[2], const int iv_size[1], double
  bboxPred_data[2048])
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
    int i;
    i = static_cast<int>(idx);
    bboxPred_data[i + bboxPred_size[0] * 3] = x1_data[i];
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double bboxPred_data[2048]
//                const int bboxPred_size[2]
//                const int i
//                const int b_bboxPred_size[2]
//                const int *nrows
//                double b_bboxPred_data[2048]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void detectFunction_kernel54(const
  double bboxPred_data[2048], const int bboxPred_size[2], const int i, const int
  b_bboxPred_size[2], const int *nrows, double b_bboxPred_data[2048])
{
  int b_i;
  b_i = static_cast<int>(mwGetGlobalThreadIndex());
  if (b_i < 4) {
    b_bboxPred_data[*nrows + b_bboxPred_size[0] * b_i] = bboxPred_data[i +
      bboxPred_size[0] * b_i];
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const float thresholdedPrediction_data[3072]
//                const int thresholdedPrediction_size[2]
//                const int i
//                const int count
//                float scores_data[512]
//                float scorePred_data[512]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void detectFunction_kernel55(const
  float thresholdedPrediction_data[3072], const int thresholdedPrediction_size[2],
  const int i, const int count, float scores_data[512], float scorePred_data[512])
{
  int tmpIdx;
  tmpIdx = static_cast<int>(mwGetGlobalThreadIndex());
  if (tmpIdx < 1) {
    scorePred_data[count - 1] = thresholdedPrediction_data[i +
      (thresholdedPrediction_size[0] << 2)];
    scores_data[count - 1] = thresholdedPrediction_data[i +
      thresholdedPrediction_size[0] * 5];
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const short outVal
//                const short i
//                short idx_data[512]
// Return Type  : void
//
static __global__ __launch_bounds__(1024, 1) void detectFunction_kernel56(const
  short outVal, const short i, short idx_data[512])
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
    int b_i;
    b_i = static_cast<int>(idx);
    idx_data[b_i] = static_cast<short>(static_cast<int>(outVal) + static_cast<
      int>(static_cast<short>(b_i)));
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const int bboxPred_size[2]
//                bool b_data[512]
// Return Type  : void
//
static __global__ __launch_bounds__(1024, 1) void detectFunction_kernel57(const
  int bboxPred_size[2], bool b_data[512])
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
    int i;
    i = static_cast<int>(idx);
    b_data[i] = false;
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const bool b_data[512]
//                int *status
//                int *tbWidth
// Return Type  : void
//
static __global__ __launch_bounds__(1024, 1) void detectFunction_kernel58(const
  bool b_data[512], int *status, int *tbWidth)
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
  loopEnd = static_cast<long>(*status - 1);
  if (mwIsLastBlock()) {
    m = (static_cast<long>(*status - 1) + 1L) % static_cast<long>(blockStride);
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
    atomicAdd(&tbWidth[0], tmpRed0);
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const short idx_data[512]
//                int *status
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void detectFunction_kernel59(const
  short idx_data[512], int *status)
{
  int tmpIdx;
  tmpIdx = static_cast<int>(mwGetGlobalThreadIndex());
  if (tmpIdx < 1) {
    *status = static_cast<int>(idx_data[0]);
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const short aux1[480]
//                short ipRowIndices[1120]
//                double rowWeights[1120]
// Return Type  : void
//
static __global__ __launch_bounds__(512, 1) void detectFunction_kernel6(const
  short aux1[480], short ipRowIndices[1120], double rowWeights[1120])
{
  unsigned long threadId;
  int i;
  int k;
  int rowIdx;
  threadId = mwGetGlobalThreadIndex();
  k = static_cast<int>(threadId % 5UL);
  rowIdx = static_cast<int>((threadId - static_cast<unsigned long>(k)) / 5UL);
  if ((static_cast<int>(rowIdx < 224)) && (static_cast<int>(k < 5))) {
    double absx;
    double absx2;
    double sumVal;
    int ind;
    sumVal = (static_cast<double>(rowIdx) + 1.0) / 0.93333333333333335 +
      -0.0357142857142857;
    i = static_cast<int>(floor(sumVal - 2.1428571428571428));
    absx = fabs(0.93333333333333335 * (sumVal - (static_cast<double>(i + k) +
      1.0)));
    absx2 = absx * absx;
    sumVal = rt_powd_snf_device(absx, 3.0);
    rowWeights[rowIdx + 224 * k] = 0.93333333333333335 * (((1.5 * sumVal - 2.5 *
      absx2) + 1.0) * static_cast<double>(absx <= 1.0) + (((-0.5 * sumVal + 2.5 *
      absx2) - 4.0 * absx) + 2.0) * static_cast<double>((static_cast<int>(1.0 <
      absx)) && (static_cast<int>(absx <= 2.0))));
    ind = (i + k) + 1;
    if (ind - 1 == 0) {
      i = 0;
    } else {
      i = static_cast<int>(fmod(static_cast<double>(ind) - 1.0, 480.0));
      if ((static_cast<int>(i != 0)) && (static_cast<int>(ind - 1 < 0))) {
        i += 480;
      }
    }

    ipRowIndices[rowIdx + 224 * k] = aux1[i];
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double bboxPred_data[2048]
//                const int bboxPred_size[2]
//                const int b_bboxPred_size[2]
//                const int *status
//                double b_bboxPred_data[2048]
// Return Type  : void
//
static __global__ __launch_bounds__(1024, 1) void detectFunction_kernel60(const
  double bboxPred_data[2048], const int bboxPred_size[2], const int
  b_bboxPred_size[2], const int *status, double b_bboxPred_data[2048])
{
  unsigned long idx;
  long loopEnd;
  unsigned long threadId;
  unsigned long threadStride;
  threadId = mwGetGlobalThreadIndex();
  threadStride = mwGetTotalThreadsLaunched();
  loopEnd = (static_cast<long>(*status) + 1L) * 4L - 1L;
  for (idx = threadId; idx <= static_cast<unsigned long>(loopEnd); idx +=
       threadStride) {
    int i;
    int ind;
    ind = static_cast<int>(idx % (static_cast<unsigned long>(*status) + 1UL));
    i = static_cast<int>((idx - static_cast<unsigned long>(ind)) / (static_cast<
      unsigned long>(*status) + 1UL));
    b_bboxPred_data[ind + b_bboxPred_size[0] * i] = bboxPred_data[ind +
      bboxPred_size[0] * i];
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double bboxPred_data[2048]
//                const int bboxPred_size[2]
//                double b_bboxPred_data[2048]
// Return Type  : void
//
static __global__ __launch_bounds__(1024, 1) void detectFunction_kernel61(const
  double bboxPred_data[2048], const int bboxPred_size[2], double
  b_bboxPred_data[2048])
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
    int i;
    i = static_cast<int>(idx);
    b_bboxPred_data[i] = bboxPred_data[i];
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const short outVal
//                const short i
//                short idx_data[512]
// Return Type  : void
//
static __global__ __launch_bounds__(1024, 1) void detectFunction_kernel62(const
  short outVal, const short i, short idx_data[512])
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
    int b_i;
    b_i = static_cast<int>(idx);
    idx_data[b_i] = static_cast<short>(static_cast<int>(outVal) + static_cast<
      int>(static_cast<short>(b_i)));
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const int scorePred_size[1]
//                bool b_data[512]
// Return Type  : void
//
static __global__ __launch_bounds__(1024, 1) void detectFunction_kernel63(const
  int scorePred_size[1], bool b_data[512])
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
    int i;
    i = static_cast<int>(idx);
    b_data[i] = false;
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const bool b_data[512]
//                int *status
//                int *tbWidth
// Return Type  : void
//
static __global__ __launch_bounds__(1024, 1) void detectFunction_kernel64(const
  bool b_data[512], int *status, int *tbWidth)
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
  loopEnd = static_cast<long>(*status - 1);
  if (mwIsLastBlock()) {
    m = (static_cast<long>(*status - 1) + 1L) % static_cast<long>(blockStride);
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
    atomicAdd(&tbWidth[0], tmpRed0);
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const short outVal
//                const short i
//                short idx_data[512]
// Return Type  : void
//
static __global__ __launch_bounds__(1024, 1) void detectFunction_kernel65(const
  short outVal, const short i, short idx_data[512])
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
    int b_i;
    b_i = static_cast<int>(idx);
    idx_data[b_i] = static_cast<short>(static_cast<int>(outVal) + static_cast<
      int>(static_cast<short>(b_i)));
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const int iv_size[1]
//                bool b_data[512]
// Return Type  : void
//
static __global__ __launch_bounds__(1024, 1) void detectFunction_kernel66(const
  int iv_size[1], bool b_data[512])
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
    int i;
    i = static_cast<int>(idx);
    b_data[i] = false;
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const bool b_data[512]
//                int *status
//                int *tbWidth
// Return Type  : void
//
static __global__ __launch_bounds__(1024, 1) void detectFunction_kernel67(const
  bool b_data[512], int *status, int *tbWidth)
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
  loopEnd = static_cast<long>(*status - 1);
  if (mwIsLastBlock()) {
    m = (static_cast<long>(*status - 1) + 1L) % static_cast<long>(blockStride);
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
    atomicAdd(&tbWidth[0], tmpRed0);
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const float scores_data[512]
//                const int iv_size[1]
//                double y1_data[512]
// Return Type  : void
//
static __global__ __launch_bounds__(1024, 1) void detectFunction_kernel68(const
  float scores_data[512], const int iv_size[1], double y1_data[512])
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
    int i;
    i = static_cast<int>(idx);
    y1_data[i] = static_cast<double>(scores_data[i]);
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const float scorePred_data[512]
//                const int scorePred_size[1]
//                float scores_data[512]
// Return Type  : void
//
static __global__ __launch_bounds__(1024, 1) void detectFunction_kernel69(const
  float scorePred_data[512], const int scorePred_size[1], float scores_data[512])
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
    int i;
    i = static_cast<int>(idx);
    scores_data[i] = scorePred_data[i];
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const short aux2[640]
//                short ipColIndices[1344]
//                double colWeights[1344]
// Return Type  : void
//
static __global__ __launch_bounds__(512, 1) void detectFunction_kernel7(const
  short aux2[640], short ipColIndices[1344], double colWeights[1344])
{
  unsigned long threadId;
  int colIdx;
  int i;
  int k;
  threadId = mwGetGlobalThreadIndex();
  k = static_cast<int>(threadId % 6UL);
  colIdx = static_cast<int>((threadId - static_cast<unsigned long>(k)) / 6UL);
  if ((static_cast<int>(colIdx < 224)) && (static_cast<int>(k < 6))) {
    double absx;
    double absx2;
    double sumVal;
    int ind;
    sumVal = (static_cast<double>(colIdx) + 1.0) / 0.7 + -0.2142857142857143;
    i = static_cast<int>(floor(sumVal - 2.8571428571428572));
    absx = fabs(0.7 * (sumVal - (static_cast<double>(i + k) + 1.0)));
    absx2 = absx * absx;
    sumVal = rt_powd_snf_device(absx, 3.0);
    colWeights[colIdx + 224 * k] = 0.7 * (((1.5 * sumVal - 2.5 * absx2) + 1.0) *
      static_cast<double>(absx <= 1.0) + (((-0.5 * sumVal + 2.5 * absx2) - 4.0 *
      absx) + 2.0) * static_cast<double>((static_cast<int>(1.0 < absx)) && (
      static_cast<int>(absx <= 2.0))));
    ind = (i + k) + 1;
    if (ind - 1 == 0) {
      i = 0;
    } else {
      i = static_cast<int>(fmod(static_cast<double>(ind) - 1.0, 640.0));
      if ((static_cast<int>(i != 0)) && (static_cast<int>(ind - 1 < 0))) {
        i += 640;
      }
    }

    ipColIndices[colIdx + 224 * k] = aux2[i];
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const short dv2[2]
//                double idx_data[512]
// Return Type  : void
//
static __global__ __launch_bounds__(1024, 1) void detectFunction_kernel70(const
  short dv2[2], double idx_data[512])
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
    int i;
    i = static_cast<int>(idx);
    idx_data[i] = 0.0;
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double bboxPred_data[2048]
//                const int bboxPred_size[2]
//                const double idx_data[512]
//                const int inputBbox_size[2]
//                const int idx_size[1]
//                double bboxesX1Y1X2Y2_data[2048]
// Return Type  : void
//
static __global__ __launch_bounds__(1024, 1) void detectFunction_kernel71(const
  double bboxPred_data[2048], const int bboxPred_size[2], const double idx_data
  [512], const int inputBbox_size[2], const int idx_size[1], double
  bboxesX1Y1X2Y2_data[2048])
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
    int i;
    int ind;
    ind = static_cast<int>(idx % (static_cast<unsigned long>(idx_size[0] - 1) +
      1UL));
    i = static_cast<int>((idx - static_cast<unsigned long>(ind)) / (static_cast<
      unsigned long>(idx_size[0] - 1) + 1UL));
    bboxesX1Y1X2Y2_data[ind + inputBbox_size[0] * i] = bboxPred_data[(
      static_cast<int>(idx_data[ind]) + bboxPred_size[0] * i) - 1];
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double y1_data[512]
//                const double idx_data[512]
//                const int idx_size[1]
//                double x1_data[512]
// Return Type  : void
//
static __global__ __launch_bounds__(1024, 1) void detectFunction_kernel72(const
  double y1_data[512], const double idx_data[512], const int idx_size[1], double
  x1_data[512])
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
    int i;
    i = static_cast<int>(idx);
    x1_data[i] = y1_data[static_cast<int>(idx_data[i]) - 1];
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double x1_data[512]
//                const int iv_size[1]
//                double y1_data[512]
// Return Type  : void
//
static __global__ __launch_bounds__(1024, 1) void detectFunction_kernel73(const
  double x1_data[512], const int iv_size[1], double y1_data[512])
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
    int i;
    i = static_cast<int>(idx);
    y1_data[i] = x1_data[i];
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const int idx_size[1]
//                bool b_data[512]
// Return Type  : void
//
static __global__ __launch_bounds__(1024, 1) void detectFunction_kernel74(const
  int idx_size[1], bool b_data[512])
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
    int i;
    i = static_cast<int>(idx);
    b_data[i] = true;
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double bboxesX1Y1X2Y2_data[2048]
//                const int inputBbox_size[2]
//                const int k
//                double x1_data[512]
// Return Type  : void
//
static __global__ __launch_bounds__(1024, 1) void detectFunction_kernel75(const
  double bboxesX1Y1X2Y2_data[2048], const int inputBbox_size[2], const int k,
  double x1_data[512])
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
    int i;
    i = static_cast<int>(idx);
    x1_data[i] = bboxesX1Y1X2Y2_data[i + (inputBbox_size[0] << 1)] *
      bboxesX1Y1X2Y2_data[i + inputBbox_size[0] * 3];
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const int inputBbox_size[2]
//                const double bboxesX1Y1X2Y2_data[2048]
//                const int k
//                double x2_data[512]
// Return Type  : void
//
static __global__ __launch_bounds__(1024, 1) void detectFunction_kernel76(const
  int inputBbox_size[2], const double bboxesX1Y1X2Y2_data[2048], const int k,
  double x2_data[512])
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
    int i;
    i = static_cast<int>(idx);
    x2_data[i] = bboxesX1Y1X2Y2_data[i] + bboxesX1Y1X2Y2_data[i +
      (inputBbox_size[0] << 1)];
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double bboxesX1Y1X2Y2_data[2048]
//                const int inputBbox_size[2]
//                const int k
//                double y2_data[512]
// Return Type  : void
//
static __global__ __launch_bounds__(1024, 1) void detectFunction_kernel77(const
  double bboxesX1Y1X2Y2_data[2048], const int inputBbox_size[2], const int k,
  double y2_data[512])
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
    int i;
    i = static_cast<int>(idx);
    y2_data[i] = bboxesX1Y1X2Y2_data[i + inputBbox_size[0]] +
      bboxesX1Y1X2Y2_data[i + inputBbox_size[0] * 3];
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const int *nrows
//                const int iv[2]
//                bool b_data[512]
// Return Type  : void
//
static __global__ __launch_bounds__(1024, 1) void detectFunction_kernel78(const
  int *nrows, const int iv[2], bool b_data[512])
{
  unsigned long idx;
  long loopEnd;
  unsigned long threadId;
  unsigned long threadStride;
  threadId = mwGetGlobalThreadIndex();
  threadStride = mwGetTotalThreadsLaunched();
  loopEnd = static_cast<long>(iv[1] - 1);
  for (idx = threadId; idx <= static_cast<unsigned long>(loopEnd); idx +=
       threadStride) {
    int i;
    i = static_cast<int>(idx);
    b_data[*nrows + i] = false;
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const bool b_data[512]
//                const double idx_data[512]
//                const int y1_size[1]
//                bool index_data[512]
// Return Type  : void
//
static __global__ __launch_bounds__(1024, 1) void detectFunction_kernel79(const
  bool b_data[512], const double idx_data[512], const int y1_size[1], bool
  index_data[512])
{
  unsigned long idx;
  long loopEnd;
  unsigned long threadId;
  unsigned long threadStride;
  threadId = mwGetGlobalThreadIndex();
  threadStride = mwGetTotalThreadsLaunched();
  loopEnd = static_cast<long>(y1_size[0] - 1);
  for (idx = threadId; idx <= static_cast<unsigned long>(loopEnd); idx +=
       threadStride) {
    int i;
    i = static_cast<int>(idx);
    index_data[static_cast<int>(idx_data[i]) - 1] = b_data[i];
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double rowWeights[1120]
//                double rowWeightsTotal[224]
// Return Type  : void
//
static __global__ __launch_bounds__(224, 1) void detectFunction_kernel8(const
  double rowWeights[1120], double rowWeightsTotal[224])
{
  int i;
  i = static_cast<int>(mwGetGlobalThreadIndex());
  if (i < 224) {
    rowWeightsTotal[i] = rowWeights[i];
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double bboxPred_data[2048]
//                const int bboxPred_size[2]
//                const int iv6_data[512]
//                const int b_bboxPred_size[2]
//                const int iv_size[1]
//                double b_bboxPred_data[2048]
// Return Type  : void
//
static __global__ __launch_bounds__(1024, 1) void detectFunction_kernel80(const
  double bboxPred_data[2048], const int bboxPred_size[2], const int iv6_data[512],
  const int b_bboxPred_size[2], const int iv_size[1], double b_bboxPred_data
  [2048])
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
    int i;
    int ind;
    ind = static_cast<int>(idx % (static_cast<unsigned long>(iv_size[0] - 1) +
      1UL));
    i = static_cast<int>((idx - static_cast<unsigned long>(ind)) / (static_cast<
      unsigned long>(iv_size[0] - 1) + 1UL));
    b_bboxPred_data[ind + b_bboxPred_size[0] * i] = bboxPred_data[(iv6_data[ind]
      + bboxPred_size[0] * i) - 1];
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double bboxPred_data[2048]
//                const int bboxPred_size[2]
//                double b_bboxPred_data[2048]
// Return Type  : void
//
static __global__ __launch_bounds__(1024, 1) void detectFunction_kernel81(const
  double bboxPred_data[2048], const int bboxPred_size[2], double
  b_bboxPred_data[2048])
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
    int i;
    i = static_cast<int>(idx);
    b_bboxPred_data[i] = bboxPred_data[i];
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const float scorePred_data[512]
//                const int scorePred_size[1]
//                float scores_data[512]
// Return Type  : void
//
static __global__ __launch_bounds__(1024, 1) void detectFunction_kernel82(const
  float scorePred_data[512], const int scorePred_size[1], float scores_data[512])
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
    int i;
    i = static_cast<int>(idx);
    scores_data[i] = scorePred_data[i];
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const float scores_data[512]
//                const int y1_size[1]
//                float b_scores_data[512]
// Return Type  : void
//
static __global__ __launch_bounds__(1024, 1) void detectFunction_kernel83(const
  float scores_data[512], const int y1_size[1], float b_scores_data[512])
{
  unsigned long idx;
  long loopEnd;
  unsigned long threadId;
  unsigned long threadStride;
  threadId = mwGetGlobalThreadIndex();
  threadStride = mwGetTotalThreadsLaunched();
  loopEnd = static_cast<long>(y1_size[0] - 1);
  for (idx = threadId; idx <= static_cast<unsigned long>(loopEnd); idx +=
       threadStride) {
    int i;
    i = static_cast<int>(idx);
    b_scores_data[i] = scores_data[i];
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const int *ind
//                int v_size[1]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void detectFunction_kernel84(const
  int *ind, int v_size[1])
{
  int tmpIdx;
  tmpIdx = static_cast<int>(mwGetGlobalThreadIndex());
  if (tmpIdx < 1) {
    v_size[0] = *ind;
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const char cv3[3]
//                const int initAuxVar
//                char v_data[3]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void detectFunction_kernel85(const
  char cv3[3], const int initAuxVar, char v_data[3])
{
  int k;
  k = static_cast<int>(mwGetGlobalThreadIndex());
  if (k < 3) {
    v_data[initAuxVar + k] = cv3[k];
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const char v_data[3]
//                const int *i
//                const int v_size[1]
//                emxArray_cell_wrap_7_512 *labelCells
// Return Type  : void
//
static __global__ __launch_bounds__(1024, 1) void detectFunction_kernel86(const
  char v_data[3], const int *i, const int v_size[1], emxArray_cell_wrap_7_512
  *labelCells)
{
  unsigned long idx;
  long loopEnd;
  unsigned long threadId;
  unsigned long threadStride;
  threadId = mwGetGlobalThreadIndex();
  threadStride = mwGetTotalThreadsLaunched();
  loopEnd = static_cast<long>(v_size[0] - 1);
  for (idx = threadId; idx <= static_cast<unsigned long>(loopEnd); idx +=
       threadStride) {
    int b_i;
    b_i = static_cast<int>(idx);
    labelCells->data[*i].f1.data[b_i] = v_data[b_i];
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double bboxPred_data[2048]
//                const int bboxPred_size[2]
//                const int *status
//                int position[4]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void detectFunction_kernel87(const
  double bboxPred_data[2048], const int bboxPred_size[2], const int *status, int
  position[4])
{
  int i;
  i = static_cast<int>(mwGetGlobalThreadIndex());
  if (i < 4) {
    double sumVal;
    int ind;
    sumVal = rt_roundd_snf_device(bboxPred_data[*status + bboxPred_size[0] * i]);
    if (sumVal < 2.147483648E+9) {
      if (sumVal >= -2.147483648E+9) {
        ind = static_cast<int>(sumVal);
      } else {
        ind = MIN_int32_T;
      }
    } else if (sumVal >= 2.147483648E+9) {
      ind = MAX_int32_T;
    } else {
      ind = 0;
    }

    position[i] = ind;
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const unsigned char out[150528]
//                unsigned char tmpRGB[150528]
// Return Type  : void
//
static __global__ __launch_bounds__(512, 1) void detectFunction_kernel88(const
  unsigned char out[150528], unsigned char tmpRGB[150528])
{
  int i;
  i = static_cast<int>(mwGetGlobalThreadIndex());
  if (i < 150528) {
    tmpRGB[i] = out[i];
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const unsigned char uv[3]
//                unsigned char color[3]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void detectFunction_kernel89(const
  unsigned char uv[3], unsigned char color[3])
{
  int i;
  i = static_cast<int>(mwGetGlobalThreadIndex());
  if (i < 3) {
    color[i] = uv[i];
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double rowWeights[1120]
//                const int *status
//                double rowWeightsTotal[224]
// Return Type  : void
//
static __global__ __launch_bounds__(224, 1) void detectFunction_kernel9(const
  double rowWeights[1120], const int *status, double rowWeightsTotal[224])
{
  int i;
  i = static_cast<int>(mwGetGlobalThreadIndex());
  if (i < 224) {
    rowWeightsTotal[i] += rowWeights[*status + i];
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const int position[4]
//                int positionOut[4]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void detectFunction_kernel90(const
  int position[4], int positionOut[4])
{
  int i;
  i = static_cast<int>(mwGetGlobalThreadIndex());
  if (i < 4) {
    positionOut[i] = position[i];
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                unsigned char out[150528]
// Return Type  : void
//
static __global__ __launch_bounds__(512, 1) void detectFunction_kernel91
  (unsigned char out[150528])
{
  int i;
  i = static_cast<int>(mwGetGlobalThreadIndex());
  if (i < 150528) {
    out[i] = static_cast<unsigned char>(0U);
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                unsigned char pixCount[224]
// Return Type  : void
//
static __global__ __launch_bounds__(224, 1) void detectFunction_kernel92
  (unsigned char pixCount[224])
{
  int i;
  i = static_cast<int>(mwGetGlobalThreadIndex());
  if (i < 224) {
    pixCount[i] = static_cast<unsigned char>(0U);
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const int position[4]
//                int positionOut[4]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void detectFunction_kernel93(const
  int position[4], int positionOut[4])
{
  int i;
  i = static_cast<int>(mwGetGlobalThreadIndex());
  if (i < 4) {
    positionOut[i] = position[i];
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const int qY
//                int positionOut[4]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void detectFunction_kernel94(const
  int qY, int positionOut[4])
{
  int tmpIdx;
  tmpIdx = static_cast<int>(mwGetGlobalThreadIndex());
  if (tmpIdx < 1) {
    positionOut[1] = qY;
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const unsigned char uv[3]
//                unsigned char color[3]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void detectFunction_kernel95(const
  unsigned char uv[3], unsigned char color[3])
{
  int i;
  i = static_cast<int>(mwGetGlobalThreadIndex());
  if (i < 3) {
    color[i] = uv[i];
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const emxArray_cell_wrap_7_512 *labelCells
//                const int *tbWidth
//                signed char thisTextU16_data[3]
// Return Type  : void
//
static __global__ __launch_bounds__(1024, 1) void detectFunction_kernel96(const
  emxArray_cell_wrap_7_512 *labelCells, const int *tbWidth, signed char
  thisTextU16_data[3])
{
  unsigned long idx;
  long loopEnd;
  unsigned long threadId;
  unsigned long threadStride;
  threadId = mwGetGlobalThreadIndex();
  threadStride = mwGetTotalThreadsLaunched();
  loopEnd = static_cast<long>(labelCells->data[*tbWidth - 1].f1.size[1] - 1);
  for (idx = threadId; idx <= static_cast<unsigned long>(loopEnd); idx +=
       threadStride) {
    int i;
    i = static_cast<int>(idx);
    thisTextU16_data[i] = static_cast<signed char>(labelCells->data[*tbWidth - 1]
      .f1.data[i]);
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const signed char thisTextU16_data[3]
//                const int thisTextU16_size[2]
//                signed char thisCharcodes_1b_data[3]
// Return Type  : void
//
static __global__ __launch_bounds__(1024, 1) void detectFunction_kernel97(const
  signed char thisTextU16_data[3], const int thisTextU16_size[2], signed char
  thisCharcodes_1b_data[3])
{
  unsigned long idx;
  long loopEnd;
  unsigned long threadId;
  unsigned long threadStride;
  threadId = mwGetGlobalThreadIndex();
  threadStride = mwGetTotalThreadsLaunched();
  loopEnd = static_cast<long>(thisTextU16_size[1] - 1);
  for (idx = threadId; idx <= static_cast<unsigned long>(loopEnd); idx +=
       threadStride) {
    int i;
    i = static_cast<int>(idx);
    thisCharcodes_1b_data[i] = static_cast<signed char>(static_cast<int>
      (thisTextU16_data[i]) + 1);
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const signed char iv2[261]
//                const unsigned short uv1[256]
//                const signed char thisCharcodes_1b_data[3]
//                const int thisCharcodes_1b_size[2]
//                signed char x_data[3]
// Return Type  : void
//
static __global__ __launch_bounds__(1024, 1) void detectFunction_kernel98(const
  signed char iv2[261], const unsigned short uv1[256], const signed char
  thisCharcodes_1b_data[3], const int thisCharcodes_1b_size[2], signed char
  x_data[3])
{
  unsigned long idx;
  long loopEnd;
  unsigned long threadId;
  unsigned long threadStride;
  threadId = mwGetGlobalThreadIndex();
  threadStride = mwGetTotalThreadsLaunched();
  loopEnd = static_cast<long>(thisCharcodes_1b_size[1] - 1);
  for (idx = threadId; idx <= static_cast<unsigned long>(loopEnd); idx +=
       threadStride) {
    int i;
    i = static_cast<int>(idx);
    x_data[i] = iv2[uv1[static_cast<int>(thisCharcodes_1b_data[i]) - 1]];
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const signed char iv2[261]
//                const unsigned short uv1[256]
//                const signed char thisCharcodes_1b_data[3]
//                double *height
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void detectFunction_kernel99(const
  signed char iv2[261], const unsigned short uv1[256], const signed char
  thisCharcodes_1b_data[3], double *height)
{
  int tmpIdx;
  tmpIdx = static_cast<int>(mwGetGlobalThreadIndex());
  if (tmpIdx < 1) {
    *height = static_cast<double>(iv2[uv1[static_cast<int>
      (thisCharcodes_1b_data[0]) - 1]]);
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
// Arguments    : double in1
//                unsigned int offset
//                unsigned int mask
// Return Type  : double
//
static __device__ double shflDown2(double in1, unsigned int offset, unsigned int
  mask)
{
  int2 tmp;
  tmp = *(int2 *)&in1;
  tmp.x = __shfl_down_sync(mask, tmp.x, offset);
  tmp.y = __shfl_down_sync(mask, tmp.y, offset);
  return *(double *)&tmp;
}

//
// Arguments    : double val
//                unsigned int lane
//                unsigned int mask
// Return Type  : double
//
static __device__ double threadGroupReduction(double val, unsigned int lane,
  unsigned int mask)
{
  unsigned int activeSize;
  unsigned int offset;
  activeSize = __popc(mask);
  offset = (activeSize + 1U) / 2U;
  while (activeSize > 1U) {
    double other;
    other = shflDown2(val, offset, mask);
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
// Arguments    : double val
//                unsigned int mask
//                unsigned int numActiveWarps
// Return Type  : double
//
static __device__ double workGroupReduction(double val, unsigned int mask,
  unsigned int numActiveWarps)
{
  __shared__ double shared[32];
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
// %% standart input values if no are given
//  % nargin returns the number of function input arguments given int the
//  % call to the currently executing function.
//  switch nargin
//    case 0
//      inputImgSize = [];
//      imageDisplay = [];
//    case 1
//      imageDisplay = [];
//    case 2
//    otherwise
//      error('2 inputs are accepted.')
//  end
//  if isempty(inputImgSize)
//    inputImgSize = [416 416];
//  end
//  if isempty(lwRadiation)
//    imageDisplay = true;
//  end
// Arguments    : void
// Return Type  : void
//
void detectFunction()
{
  static const double dv1[8] = { 1.1875, 2.5, 0.9375, 1.6875, 4.25, 2.5, 0.5625,
    2.9375 };

  static float b_out[49152];
  static const short uv4[261] = { 0, 0, 0, 56, 56, 74, 86, 158, 224, 296, 368,
    377, 421, 465, 489, 545, 551, 558, 560, 615, 678, 732, 786, 840, 903, 957,
    1020, 1074, 1128, 1191, 1205, 1223, 1279, 1303, 1359, 1404, 1494, 1575, 1629,
    1701, 1773, 1827, 1872, 1944, 2007, 2025, 2080, 2143, 2197, 2278, 2341, 2422,
    2476, 2586, 2649, 2703, 2775, 2838, 2910, 3009, 3081, 3153, 3216, 3249, 3304,
    3337, 3379, 3385, 3393, 3442, 3496, 3538, 3601, 3643, 3688, 3751, 3805, 3823,
    3867, 3921, 3939, 4009, 4051, 4100, 4154, 4217, 4245, 4287, 4327, 4369, 4418,
    4488, 4537, 4600, 4649, 4693, 4715, 4759, 6416, 6515, 6722, 6890, 7390, 7906,
    8394, 8745, 8675, 8815, 8955, 8885, 9018, 9165, 9285, 9225, 9345, 9405, 9499,
    9459, 9539, 9589, 9695, 9825, 9755, 9895, 10035, 9965, 10263, 10203, 10323,
    10383, 0, 5322, 4791, 4845, 5049, 0, 5515, 8621, 5286, 5119, 0, 5453, 5115,
    0, 6623, 8061, 0, 5337, 0, 0, 4955, 5461, 0, 0, 0, 0, 0, 5209, 5619, 0, 9095,
    10154, 5939, 4773, 5255, 0, 0, 0, 0, 5225, 5639, 0, 4773, 5984, 6308, 7798,
    0, 0, 0, 0, 0, 0, 0, 0, 10098, 0, 10587, 0, 0, 4899, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 6200, 6962, 6092, 7034, 6818, 7148, 7196, 7256, 7100, 7582, 7690, 0, 7474,
    8226, 8310, 8142, 0, 0, 0, 0, 0, 0, 0, 5583, 0, 0, 0, 0, 0, 0, 0, 0, 0, 5027,
    7300, 9625, 8471, 10437, 8567, 10521, 0, 8005, 5595, 5393, 5423, 5759, 5669,
    5849, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 5279, 5316, 5581 };

  static const unsigned short uv1[256] = { 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
    0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
    0U, 0U, 0U, 0U, 3U, 4U, 5U, 6U, 7U, 8U, 9U, 10U, 11U, 12U, 13U, 14U, 15U,
    16U, 17U, 18U, 19U, 20U, 21U, 22U, 23U, 24U, 25U, 26U, 27U, 28U, 29U, 30U,
    31U, 32U, 33U, 34U, 35U, 36U, 37U, 38U, 39U, 40U, 41U, 42U, 43U, 44U, 45U,
    46U, 47U, 48U, 49U, 50U, 51U, 52U, 53U, 54U, 55U, 56U, 57U, 58U, 59U, 60U,
    61U, 62U, 63U, 64U, 65U, 66U, 67U, 68U, 69U, 70U, 71U, 72U, 73U, 74U, 75U,
    76U, 77U, 78U, 79U, 80U, 81U, 82U, 83U, 84U, 85U, 86U, 87U, 88U, 89U, 90U,
    91U, 92U, 93U, 94U, 95U, 96U, 97U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
    0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
    0U, 0U, 0U, 0U, 172U, 163U, 132U, 133U, 189U, 150U, 232U, 134U, 142U, 139U,
    157U, 169U, 164U, 258U, 138U, 259U, 131U, 147U, 242U, 243U, 141U, 151U, 136U,
    260U, 222U, 241U, 158U, 170U, 245U, 244U, 246U, 162U, 173U, 201U, 199U, 174U,
    98U, 99U, 144U, 100U, 203U, 101U, 200U, 202U, 207U, 204U, 205U, 206U, 233U,
    102U, 211U, 208U, 209U, 175U, 103U, 240U, 145U, 214U, 212U, 213U, 104U, 235U,
    237U, 137U, 106U, 105U, 107U, 109U, 108U, 110U, 160U, 111U, 113U, 112U, 114U,
    115U, 117U, 116U, 118U, 119U, 234U, 120U, 122U, 121U, 123U, 125U, 124U, 184U,
    161U, 127U, 126U, 128U, 129U, 236U, 238U, 186U };

  static const unsigned char uv5[10664] = { 60U, 96U, 96U, 96U, 96U, 96U, 60U,
    96U, 0U, 0U, 0U, 0U, 0U, 96U, 96U, 0U, 0U, 0U, 0U, 0U, 96U, 96U, 0U, 0U, 0U,
    0U, 0U, 96U, 96U, 0U, 0U, 0U, 0U, 0U, 96U, 96U, 0U, 0U, 0U, 0U, 0U, 96U, 96U,
    0U, 0U, 0U, 0U, 0U, 96U, 108U, 96U, 96U, 96U, 96U, 96U, 108U, 176U, 120U,
    176U, 119U, 172U, 115U, 165U, 108U, 158U, 101U, 151U, 94U, 144U, 87U, 0U, 0U,
    176U, 120U, 83U, 201U, 79U, 205U, 71U, 189U, 67U, 193U, 58U, 177U, 54U, 181U,
    0U, 0U, 0U, 185U, 6U, 117U, 75U, 0U, 0U, 0U, 3U, 187U, 0U, 172U, 20U, 0U, 0U,
    0U, 48U, 143U, 0U, 192U, 0U, 0U, 74U, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T,
    MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, 109U, 0U, 0U, 172U, 19U, 110U, 79U,
    0U, 0U, 214U, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T,
    MAX_uint8_T, 225U, 0U, 0U, 69U, 123U, 3U, 187U, 0U, 0U, 0U, 0U, 125U, 67U,
    49U, 143U, 0U, 0U, 0U, 0U, 178U, 13U, 104U, 87U, 0U, 0U, 0U, 0U, 0U, 108U,
    40U, 0U, 0U, 3U, 155U, 245U, 230U, 104U, 0U, 89U, 202U, 130U, 73U, 164U, 7U,
    109U, 164U, 108U, 40U, 0U, 0U, 26U, 231U, 195U, 40U, 0U, 0U, 0U, 38U, 207U,
    190U, 29U, 0U, 0U, 0U, 108U, 163U, 223U, 7U, 0U, 0U, 108U, 40U, 230U, 38U,
    145U, 76U, 117U, 107U, 230U, 7U, 42U, 181U, 246U, 221U, 67U, 0U, 0U, 0U,
    108U, 40U, 0U, 0U, 80U, 234U, 221U, 50U, 0U, 0U, 106U, 126U, 211U, 40U, 81U,
    171U, 0U, 50U, 179U, 2U, 212U, 39U, 90U, 170U, 15U, 194U, 21U, 0U, 83U, 235U,
    222U, 52U, 170U, 60U, 0U, 0U, 0U, 0U, 0U, 114U, 117U, 0U, 0U, 0U, 0U, 0U,
    57U, 173U, 48U, 219U, 236U, 86U, 0U, 19U, 194U, 17U, 166U, 89U, 38U, 218U,
    1U, 176U, 53U, 0U, 167U, 92U, 45U, 220U, 123U, 109U, 0U, 0U, 49U, 221U, 236U,
    88U, 0U, 0U, 83U, 227U, 235U, 93U, 0U, 0U, 0U, 1U, 240U, 84U, 72U, 238U, 0U,
    0U, 0U, 0U, 230U, 69U, 73U, 208U, 0U, 0U, 0U, 3U, 171U, 218U, 208U, 42U, 0U,
    0U, 12U, 199U, 146U, 230U, 87U, 0U, 91U, 198U, 107U, 179U, 0U, 86U, 229U,
    17U, 111U, 162U, 129U, 183U, 0U, 0U, 172U, 181U, 177U, 80U, 55U, 252U, 110U,
    13U, 41U, 240U, 215U, 1U, 0U, 81U, 209U, 247U, 228U, 160U, 247U, 101U, 77U,
    MAX_uint8_T, 10U, 53U, 241U, 0U, 28U, 216U, 0U, 0U, 0U, 48U, 94U, 0U, 19U,
    216U, 35U, 0U, 150U, 115U, 0U, 6U, 239U, 26U, 0U, 43U, 240U, 0U, 0U, 65U,
    228U, 0U, 0U, 43U, 240U, 0U, 0U, 6U, 239U, 25U, 0U, 0U, 150U, 113U, 0U, 0U,
    19U, 216U, 35U, 0U, 0U, 48U, 94U, 107U, 34U, 0U, 0U, 51U, 209U, 8U, 0U, 0U,
    142U, 122U, 0U, 0U, 53U, 219U, 0U, 0U, 9U, MAX_uint8_T, 17U, 0U, 0U, 252U,
    40U, 0U, 9U, MAX_uint8_T, 17U, 0U, 52U, 219U, 0U, 0U, 140U, 122U, 0U, 50U,
    209U, 8U, 0U, 107U, 34U, 0U, 0U, 0U, 0U, 124U, 69U, 0U, 0U, 58U, 208U, 141U,
    142U, 217U, 9U, 0U, 19U, 131U, 142U, 2U, 0U, 0U, 123U, 57U, 109U, 70U, 0U,
    0U, 0U, 0U, 168U, 52U, 0U, 0U, 0U, 0U, 0U, 0U, 168U, 52U, 0U, 0U, 0U, 0U, 0U,
    0U, 168U, 52U, 0U, 0U, 0U, 180U, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T,
    MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, 60U, 0U, 0U, 0U, 168U, 52U, 0U, 0U,
    0U, 0U, 0U, 0U, 168U, 52U, 0U, 0U, 0U, 0U, 0U, 0U, 168U, 52U, 0U, 0U, 0U,
    212U, 160U, 105U, 147U, 186U, 51U, 32U, MAX_uint8_T, MAX_uint8_T,
    MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, 20U, 212U, 160U, 0U, 0U, 0U, 220U,
    10U, 0U, 0U, 45U, 186U, 0U, 0U, 0U, 118U, 113U, 0U, 0U, 0U, 191U, 41U, 0U,
    0U, 13U, 218U, 0U, 0U, 0U, 80U, 151U, 0U, 0U, 0U, 153U, 78U, 0U, 0U, 0U,
    219U, 12U, 0U, 0U, 42U, 189U, 0U, 0U, 0U, 115U, 116U, 0U, 0U, 0U, 188U, 44U,
    0U, 0U, 0U, 0U, 6U, 155U, 244U, 222U, 79U, 0U, 0U, 130U, 192U, 23U, 68U,
    239U, 29U, 4U, 235U, 65U, 0U, 0U, 165U, 134U, 31U, MAX_uint8_T, 18U, 0U, 0U,
    119U, 182U, 53U, MAX_uint8_T, 4U, 0U, 0U, 105U, 204U, 31U, MAX_uint8_T, 18U,
    0U, 0U, 120U, 181U, 3U, 234U, 64U, 0U, 0U, 168U, 133U, 0U, 128U, 190U, 23U,
    70U, 240U, 29U, 0U, 6U, 156U, 245U, 223U, 79U, 0U, 1U, 60U, 149U, 157U, 0U,
    0U, 40U, 178U, 186U, 180U, 0U, 0U, 0U, 0U, 116U, 180U, 0U, 0U, 0U, 0U, 116U,
    180U, 0U, 0U, 0U, 0U, 116U, 180U, 0U, 0U, 0U, 0U, 116U, 180U, 0U, 0U, 0U, 0U,
    116U, 180U, 0U, 0U, 0U, 0U, 116U, 180U, 0U, 0U, 48U, MAX_uint8_T,
    MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, 112U, 200U, MAX_uint8_T, 251U, 208U,
    65U, 0U, 0U, 0U, 8U, 120U, 240U, 10U, 0U, 0U, 0U, 18U, MAX_uint8_T, 42U, 0U,
    0U, 0U, 81U, 237U, 9U, 0U, 0U, 35U, 224U, 76U, 0U, 0U, 43U, 220U, 73U, 0U,
    0U, 30U, 226U, 64U, 0U, 0U, 0U, 187U, 148U, 0U, 0U, 0U, 0U, MAX_uint8_T,
    MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, 40U, 168U, MAX_uint8_T,
    249U, 209U, 67U, 0U, 0U, 0U, 8U, 133U, 221U, 0U, 0U, 0U, 0U, 59U, 234U, 0U,
    0U, 4U, 38U, 183U, 111U, 0U, 0U, 252U, MAX_uint8_T, 183U, 26U, 0U, 0U, 3U,
    28U, 139U, 231U, 14U, 0U, 0U, 0U, 5U, 253U, 60U, 0U, 0U, 11U, 122U, 244U,
    20U, 200U, MAX_uint8_T, 246U, 202U, 67U, 0U, 0U, 0U, 0U, 9U, 211U, 156U, 0U,
    0U, 0U, 0U, 158U, 188U, 156U, 0U, 0U, 0U, 96U, 153U, 104U, 156U, 0U, 0U, 44U,
    200U, 9U, 104U, 156U, 0U, 12U, 205U, 41U, 0U, 104U, 156U, 0U, 97U,
    MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, 196U, 0U,
    0U, 0U, 0U, 124U, 156U, 0U, 0U, 0U, 0U, 0U, 124U, 156U, 0U, 0U, 0U, 0U, 0U,
    124U, 156U, 0U, 100U, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, 0U,
    100U, 160U, 0U, 0U, 0U, 0U, 100U, 160U, 0U, 0U, 0U, 0U, 100U, 253U, 224U,
    147U, 16U, 0U, 0U, 8U, 49U, 194U, 180U, 0U, 0U, 0U, 0U, 43U, MAX_uint8_T,
    16U, 0U, 0U, 0U, 35U, MAX_uint8_T, 25U, 0U, 0U, 18U, 167U, 204U, 0U, 140U,
    MAX_uint8_T, 236U, 173U, 30U, 0U, 0U, 0U, 117U, 225U, 254U, MAX_uint8_T, 72U,
    0U, 100U, 215U, 48U, 2U, 0U, 0U, 0U, 219U, 80U, 0U, 0U, 0U, 0U, 17U,
    MAX_uint8_T, 117U, 226U, 237U, 144U, 3U, 43U, MAX_uint8_T, 169U, 20U, 45U,
    229U, 107U, 29U, MAX_uint8_T, 40U, 0U, 0U, 133U, 173U, 3U, 241U, 63U, 0U, 0U,
    129U, 158U, 0U, 140U, 195U, 27U, 39U, 224U, 69U, 0U, 10U, 162U, 244U, 224U,
    102U, 0U, 180U, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, 224U, 0U,
    0U, 0U, 0U, 136U, 145U, 0U, 0U, 0U, 36U, 225U, 15U, 0U, 0U, 0U, 180U, 94U,
    0U, 0U, 0U, 74U, 213U, 2U, 0U, 0U, 4U, 216U, 85U, 0U, 0U, 0U, 101U, 228U, 3U,
    0U, 0U, 0U, 208U, 134U, 0U, 0U, 0U, 21U, MAX_uint8_T, 67U, 0U, 0U, 0U, 1U,
    137U, 233U, 241U, 163U, 8U, 89U, 197U, 22U, 27U, 206U, 105U, 114U, 176U, 0U,
    0U, 171U, 86U, 18U, 216U, 174U, 121U, 159U, 2U, 12U, 179U, 191U, 252U, 125U,
    1U, 161U, 128U, 0U, 58U, 227U, 127U, 236U, 62U, 0U, 0U, 103U, 203U, 194U,
    171U, 20U, 26U, 187U, 149U, 32U, 180U, 242U, 234U, 154U, 12U, 0U, 13U, 160U,
    239U, 226U, 94U, 0U, 0U, 159U, 154U, 14U, 71U, 245U, 45U, 2U, 246U, 31U, 0U,
    0U, 160U, 148U, 7U, 252U, 37U, 0U, 0U, 137U, 187U, 0U, 185U, 162U, 15U, 53U,
    229U, 196U, 0U, 26U, 184U, 245U, 188U, 164U, 165U, 0U, 0U, 0U, 0U, 0U, 196U,
    101U, 0U, 0U, 0U, 15U, 121U, 221U, 6U, 0U, 164U, MAX_uint8_T, 246U, 181U,
    34U, 0U, 176U, 120U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 176U, 120U,
    176U, 120U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 176U, 119U, 96U, 104U,
    145U, 26U, 0U, 0U, 0U, 0U, 0U, 36U, 160U, 56U, 0U, 0U, 0U, 34U, 158U, 210U,
    87U, 1U, 0U, 32U, 156U, 212U, 91U, 2U, 0U, 0U, 63U, 245U, 180U, 6U, 0U, 0U,
    0U, 0U, 0U, 32U, 157U, 212U, 91U, 2U, 0U, 0U, 0U, 0U, 0U, 34U, 159U, 211U,
    90U, 2U, 0U, 0U, 0U, 0U, 0U, 36U, 161U, 56U, 180U, MAX_uint8_T, MAX_uint8_T,
    MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, 60U, 0U, 0U, 0U, 0U, 0U,
    0U, 0U, 0U, 180U, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T,
    MAX_uint8_T, MAX_uint8_T, 60U, 144U, 102U, 5U, 0U, 0U, 0U, 0U, 0U, 25U, 146U,
    215U, 99U, 4U, 0U, 0U, 0U, 0U, 0U, 27U, 149U, 215U, 97U, 4U, 0U, 0U, 0U, 0U,
    0U, 59U, 243U, 185U, 8U, 0U, 0U, 28U, 149U, 215U, 98U, 4U, 0U, 27U, 148U,
    216U, 100U, 4U, 0U, 0U, 0U, 145U, 103U, 5U, 0U, 0U, 0U, 0U, 0U, 200U,
    MAX_uint8_T, 245U, 188U, 34U, 0U, 0U, 16U, 174U, 179U, 0U, 0U, 0U, 128U,
    193U, 0U, 0U, 21U, 232U, 83U, 0U, 0U, 179U, 107U, 0U, 0U, 67U, 201U, 0U, 0U,
    0U, 125U, 162U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 136U, 160U, 0U, 0U, 0U, 0U,
    0U, 77U, 186U, 240U, 244U, 185U, 50U, 0U, 0U, 2U, 155U, 178U, 61U, 10U, 22U,
    89U, 217U, 48U, 0U, 125U, 124U, 3U, 143U, 240U, MAX_uint8_T, 111U, 43U, 173U,
    19U, 171U, 0U, 131U, 117U, 13U, 165U, 55U, 0U, 175U, 87U, 81U, 10U, 197U, 0U,
    43U, 238U, 6U, 9U, 167U, 110U, 65U, 54U, 189U, 44U, 185U, 201U, 20U, 164U,
    75U, 73U, 143U, 22U, 230U, 208U, 51U, 233U, 220U, 97U, 0U, 2U, 197U, 138U,
    35U, 3U, 0U, 0U, 0U, 0U, 0U, 0U, 12U, 138U, 221U, 253U, MAX_uint8_T, 77U, 0U,
    0U, 0U, 0U, 0U, 0U, 166U, 236U, 6U, 0U, 0U, 0U, 0U, 0U, 16U, 243U, 246U, 83U,
    0U, 0U, 0U, 0U, 0U, 106U, 187U, 171U, 179U, 0U, 0U, 0U, 0U, 0U, 204U, 101U,
    85U, 251U, 23U, 0U, 0U, 0U, 46U, 251U, 20U, 10U, 244U, 115U, 0U, 0U, 0U,
    145U, 186U, 0U, 0U, 167U, 211U, 0U, 0U, 6U, 236U, MAX_uint8_T, MAX_uint8_T,
    MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, 51U, 0U, 85U, 235U, 10U, 0U, 0U, 4U,
    224U, 147U, 0U, 183U, 124U, 0U, 0U, 0U, 0U, 108U, 236U, 6U, 224U,
    MAX_uint8_T, 250U, 211U, 74U, 0U, 224U, 88U, 21U, 154U, 239U, 1U, 224U, 88U,
    0U, 66U, 249U, 4U, 224U, 89U, 32U, 184U, 137U, 0U, 224U, MAX_uint8_T,
    MAX_uint8_T, 192U, 16U, 0U, 224U, 89U, 32U, 157U, 220U, 14U, 224U, 88U, 0U,
    2U, 244U, 88U, 224U, 88U, 9U, 81U, MAX_uint8_T, 65U, 224U, MAX_uint8_T, 254U,
    229U, 128U, 0U, 0U, 0U, 98U, 207U, 248U, MAX_uint8_T, MAX_uint8_T, 160U, 0U,
    125U, 240U, 93U, 18U, 0U, 0U, 0U, 19U, 248U, 99U, 0U, 0U, 0U, 0U, 0U, 77U,
    MAX_uint8_T, 13U, 0U, 0U, 0U, 0U, 0U, 96U, 244U, 0U, 0U, 0U, 0U, 0U, 0U, 78U,
    MAX_uint8_T, 15U, 0U, 0U, 0U, 0U, 0U, 21U, 250U, 108U, 0U, 0U, 0U, 0U, 0U,
    0U, 135U, 244U, 104U, 23U, 0U, 0U, 0U, 0U, 1U, 107U, 211U, 249U, MAX_uint8_T,
    MAX_uint8_T, 164U, 224U, MAX_uint8_T, MAX_uint8_T, 247U, 215U, 119U, 2U, 0U,
    224U, 88U, 2U, 19U, 87U, 239U, 135U, 0U, 224U, 88U, 0U, 0U, 0U, 98U, 247U,
    21U, 224U, 88U, 0U, 0U, 0U, 11U, MAX_uint8_T, 68U, 224U, 88U, 0U, 0U, 0U, 0U,
    244U, 89U, 224U, 88U, 0U, 0U, 0U, 15U, MAX_uint8_T, 65U, 224U, 88U, 0U, 0U,
    0U, 102U, 240U, 10U, 224U, 88U, 0U, 23U, 94U, 240U, 106U, 0U, 224U,
    MAX_uint8_T, MAX_uint8_T, 246U, 201U, 86U, 0U, 0U, 224U, MAX_uint8_T,
    MAX_uint8_T, MAX_uint8_T, 248U, 0U, 224U, 88U, 0U, 0U, 0U, 0U, 224U, 88U, 0U,
    0U, 0U, 0U, 224U, 88U, 0U, 0U, 0U, 0U, 224U, MAX_uint8_T, MAX_uint8_T,
    MAX_uint8_T, 96U, 0U, 224U, 88U, 0U, 0U, 0U, 0U, 224U, 88U, 0U, 0U, 0U, 0U,
    224U, 88U, 0U, 0U, 0U, 0U, 224U, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T,
    MAX_uint8_T, 56U, 224U, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, 248U, 224U,
    88U, 0U, 0U, 0U, 224U, 88U, 0U, 0U, 0U, 224U, 88U, 0U, 0U, 0U, 224U,
    MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, 100U, 224U, 88U, 0U, 0U, 0U, 224U,
    88U, 0U, 0U, 0U, 224U, 88U, 0U, 0U, 0U, 224U, 88U, 0U, 0U, 0U, 0U, 0U, 100U,
    208U, 248U, MAX_uint8_T, MAX_uint8_T, 160U, 0U, 128U, 240U, 93U, 18U, 0U, 0U,
    0U, 19U, 248U, 99U, 0U, 0U, 0U, 0U, 0U, 77U, MAX_uint8_T, 13U, 0U, 0U, 0U,
    0U, 0U, 96U, 244U, 0U, 0U, 0U, 0U, 0U, 0U, 78U, MAX_uint8_T, 15U, 0U, 0U, 0U,
    152U, 164U, 21U, 250U, 106U, 0U, 0U, 0U, 152U, 164U, 0U, 138U, 243U, 104U,
    24U, 17U, 171U, 164U, 0U, 1U, 108U, 210U, 248U, 242U, 206U, 101U, 224U, 88U,
    0U, 0U, 0U, 136U, 180U, 224U, 88U, 0U, 0U, 0U, 136U, 180U, 224U, 88U, 0U, 0U,
    0U, 136U, 180U, 224U, 88U, 0U, 0U, 0U, 136U, 180U, 224U, MAX_uint8_T,
    MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, 180U, 224U, 88U, 0U, 0U,
    0U, 136U, 180U, 224U, 88U, 0U, 0U, 0U, 136U, 180U, 224U, 88U, 0U, 0U, 0U,
    136U, 180U, 224U, 88U, 0U, 0U, 0U, 136U, 180U, 228U, 88U, 228U, 88U, 228U,
    88U, 228U, 88U, 228U, 88U, 228U, 88U, 228U, 88U, 228U, 88U, 228U, 88U, 0U,
    0U, 0U, 148U, 168U, 0U, 0U, 0U, 148U, 168U, 0U, 0U, 0U, 148U, 168U, 0U, 0U,
    0U, 148U, 168U, 0U, 0U, 0U, 148U, 168U, 0U, 0U, 0U, 148U, 168U, 0U, 0U, 0U,
    148U, 168U, 0U, 0U, 0U, 148U, 168U, 0U, 0U, 0U, 156U, 151U, 0U, 0U, 20U,
    218U, 94U, 20U, MAX_uint8_T, 236U, 147U, 3U, 224U, 72U, 0U, 0U, 157U, 158U,
    0U, 224U, 72U, 0U, 104U, 204U, 8U, 0U, 224U, 72U, 56U, 229U, 30U, 0U, 0U,
    224U, 95U, 225U, 67U, 0U, 0U, 0U, 224U, 204U, 215U, 7U, 0U, 0U, 0U, 224U,
    82U, 205U, 165U, 0U, 0U, 0U, 224U, 72U, 27U, 230U, 127U, 0U, 0U, 224U, 72U,
    0U, 54U, 246U, 90U, 0U, 224U, 72U, 0U, 0U, 91U, 247U, 58U, 224U, 88U, 0U, 0U,
    0U, 0U, 224U, 88U, 0U, 0U, 0U, 0U, 224U, 88U, 0U, 0U, 0U, 0U, 224U, 88U, 0U,
    0U, 0U, 0U, 224U, 88U, 0U, 0U, 0U, 0U, 224U, 88U, 0U, 0U, 0U, 0U, 224U, 88U,
    0U, 0U, 0U, 0U, 224U, 88U, 0U, 0U, 0U, 0U, 224U, MAX_uint8_T, MAX_uint8_T,
    MAX_uint8_T, MAX_uint8_T, 64U, 224U, 245U, 12U, 0U, 0U, 0U, 124U,
    MAX_uint8_T, 56U, 224U, 247U, 94U, 0U, 0U, 0U, 216U, 249U, 56U, 224U, 171U,
    187U, 0U, 0U, 58U, 189U, 236U, 56U, 224U, 78U, 252U, 27U, 0U, 153U, 93U,
    236U, 56U, 224U, 44U, 197U, 117U, 8U, 227U, 11U, 236U, 56U, 224U, 44U, 104U,
    209U, 88U, 158U, 0U, 236U, 56U, 224U, 44U, 18U, 249U, 217U, 63U, 0U, 236U,
    56U, 224U, 44U, 0U, 174U, 222U, 1U, 0U, 236U, 56U, 224U, 44U, 0U, 0U, 0U, 0U,
    0U, 236U, 56U, 224U, 163U, 0U, 0U, 0U, 80U, 192U, 224U, MAX_uint8_T, 68U, 0U,
    0U, 80U, 192U, 224U, 183U, 220U, 8U, 0U, 80U, 192U, 224U, 54U, 223U, 133U,
    0U, 80U, 192U, 224U, 44U, 73U, 250U, 43U, 80U, 192U, 224U, 44U, 0U, 168U,
    196U, 81U, 192U, 224U, 44U, 0U, 23U, 239U, 183U, 192U, 224U, 44U, 0U, 0U,
    102U, MAX_uint8_T, 192U, 224U, 44U, 0U, 0U, 1U, 195U, 192U, 0U, 0U, 101U,
    213U, 249U, 231U, 148U, 14U, 0U, 0U, 122U, 235U, 80U, 14U, 45U, 195U, 200U,
    5U, 17U, 247U, 91U, 0U, 0U, 0U, 21U, 244U, 92U, 76U, MAX_uint8_T, 11U, 0U,
    0U, 0U, 0U, 183U, 159U, 96U, 243U, 0U, 0U, 0U, 0U, 0U, 160U, 179U, 76U,
    MAX_uint8_T, 12U, 0U, 0U, 0U, 0U, 184U, 159U, 17U, 246U, 94U, 0U, 0U, 0U,
    21U, 245U, 91U, 0U, 122U, 236U, 78U, 13U, 46U, 196U, 200U, 5U, 0U, 0U, 103U,
    215U, 250U, 230U, 148U, 14U, 0U, 224U, MAX_uint8_T, 252U, 229U, 141U, 1U,
    224U, 88U, 8U, 73U, 253U, 75U, 224U, 88U, 0U, 0U, 239U, 89U, 224U, 88U, 20U,
    129U, 239U, 26U, 224U, MAX_uint8_T, 236U, 184U, 52U, 0U, 224U, 88U, 0U, 0U,
    0U, 0U, 224U, 88U, 0U, 0U, 0U, 0U, 224U, 88U, 0U, 0U, 0U, 0U, 224U, 88U, 0U,
    0U, 0U, 0U, 0U, 0U, 101U, 213U, 249U, 231U, 148U, 14U, 0U, 0U, 0U, 122U,
    235U, 80U, 14U, 45U, 195U, 197U, 4U, 0U, 17U, 247U, 91U, 0U, 0U, 0U, 21U,
    244U, 90U, 0U, 76U, MAX_uint8_T, 11U, 0U, 0U, 0U, 0U, 183U, 158U, 0U, 96U,
    243U, 0U, 0U, 0U, 0U, 0U, 160U, 178U, 0U, 76U, MAX_uint8_T, 12U, 0U, 0U, 0U,
    0U, 184U, 157U, 0U, 16U, 245U, 94U, 0U, 0U, 0U, 21U, 244U, 89U, 0U, 0U, 118U,
    236U, 78U, 13U, 46U, 196U, 197U, 5U, 0U, 0U, 0U, 103U, 217U, 250U, 250U,
    215U, 15U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 25U, 187U, 212U, 98U, 15U, 0U, 0U, 0U,
    0U, 0U, 0U, 1U, 87U, 208U, 57U, 224U, MAX_uint8_T, 254U, 231U, 123U, 0U, 0U,
    224U, 88U, 9U, 91U, MAX_uint8_T, 49U, 0U, 224U, 88U, 0U, 4U, 251U, 70U, 0U,
    224U, 88U, 22U, 138U, 225U, 10U, 0U, 224U, MAX_uint8_T, MAX_uint8_T, 214U,
    29U, 0U, 0U, 224U, 88U, 55U, 248U, 47U, 0U, 0U, 224U, 88U, 0U, 152U, 205U,
    4U, 0U, 224U, 88U, 0U, 16U, 232U, 122U, 0U, 224U, 88U, 0U, 0U, 91U, 248U,
    42U, 0U, 101U, 225U, MAX_uint8_T, MAX_uint8_T, 84U, 45U, 242U, 53U, 1U, 0U,
    0U, 78U, 233U, 9U, 0U, 0U, 0U, 11U, 216U, 206U, 55U, 0U, 0U, 0U, 18U, 161U,
    253U, 148U, 5U, 0U, 0U, 0U, 62U, 233U, 143U, 0U, 0U, 0U, 0U, 116U, 208U, 0U,
    0U, 0U, 22U, 187U, 151U, 104U, MAX_uint8_T, MAX_uint8_T, 232U, 153U, 12U,
    228U, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T,
    MAX_uint8_T, 124U, 0U, 0U, 0U, 208U, 104U, 0U, 0U, 0U, 0U, 0U, 0U, 208U,
    104U, 0U, 0U, 0U, 0U, 0U, 0U, 208U, 104U, 0U, 0U, 0U, 0U, 0U, 0U, 208U, 104U,
    0U, 0U, 0U, 0U, 0U, 0U, 208U, 104U, 0U, 0U, 0U, 0U, 0U, 0U, 208U, 104U, 0U,
    0U, 0U, 0U, 0U, 0U, 208U, 104U, 0U, 0U, 0U, 0U, 0U, 0U, 208U, 104U, 0U, 0U,
    0U, 244U, 72U, 0U, 0U, 0U, 208U, 68U, 244U, 72U, 0U, 0U, 0U, 208U, 68U, 244U,
    72U, 0U, 0U, 0U, 208U, 68U, 244U, 72U, 0U, 0U, 0U, 208U, 68U, 244U, 72U, 0U,
    0U, 0U, 208U, 68U, 240U, 75U, 0U, 0U, 0U, 210U, 64U, 214U, 105U, 0U, 0U, 1U,
    235U, 39U, 137U, 216U, 43U, 11U, 110U, 221U, 4U, 12U, 150U, 229U, 243U, 197U,
    51U, 0U, 154U, 157U, 0U, 0U, 0U, 0U, 117U, 158U, 61U, 241U, 9U, 0U, 0U, 0U,
    206U, 69U, 1U, 223U, 87U, 0U, 0U, 40U, 231U, 3U, 0U, 131U, 180U, 0U, 0U,
    129U, 145U, 0U, 0U, 38U, 250U, 21U, 0U, 218U, 55U, 0U, 0U, 0U, 201U, 109U,
    52U, 221U, 0U, 0U, 0U, 0U, 108U, 202U, 142U, 132U, 0U, 0U, 0U, 0U, 20U, 250U,
    240U, 42U, 0U, 0U, 0U, 0U, 0U, 178U, 208U, 0U, 0U, 0U, 221U, 83U, 0U, 0U,
    130U, 231U, 0U, 0U, 0U, 226U, 29U, 157U, 146U, 0U, 0U, 188U, MAX_uint8_T,
    31U, 0U, 39U, 215U, 0U, 93U, 209U, 0U, 3U, 227U, 213U, 87U, 0U, 108U, 146U,
    0U, 30U, 254U, 18U, 49U, 183U, 156U, 143U, 0U, 177U, 76U, 0U, 0U, 222U, 79U,
    108U, 125U, 100U, 199U, 4U, 236U, 12U, 0U, 0U, 159U, 142U, 166U, 67U, 44U,
    248U, 66U, 194U, 0U, 0U, 0U, 95U, 205U, 221U, 12U, 2U, 241U, 183U, 124U, 0U,
    0U, 0U, 31U, 254U, 206U, 0U, 0U, 188U, MAX_uint8_T, 55U, 0U, 0U, 0U, 0U,
    224U, 149U, 0U, 0U, 132U, 239U, 3U, 0U, 0U, 118U, 235U, 19U, 0U, 0U, 91U,
    203U, 3U, 5U, 212U, 157U, 0U, 20U, 230U, 48U, 0U, 0U, 62U, 253U, 60U, 165U,
    136U, 0U, 0U, 0U, 0U, 162U, 231U, 220U, 9U, 0U, 0U, 0U, 0U, 48U, MAX_uint8_T,
    135U, 0U, 0U, 0U, 0U, 0U, 174U, 204U, 243U, 28U, 0U, 0U, 0U, 80U, 214U, 9U,
    201U, 172U, 0U, 0U, 14U, 227U, 59U, 0U, 49U, 252U, 73U, 0U, 151U, 149U, 0U,
    0U, 0U, 143U, 220U, 8U, 159U, 199U, 1U, 0U, 0U, 39U, 231U, 22U, 24U, 242U,
    92U, 0U, 0U, 190U, 102U, 0U, 0U, 120U, 228U, 11U, 96U, 198U, 1U, 0U, 0U, 7U,
    221U, 150U, 231U, 46U, 0U, 0U, 0U, 0U, 81U, MAX_uint8_T, 139U, 0U, 0U, 0U,
    0U, 0U, 0U, MAX_uint8_T, 56U, 0U, 0U, 0U, 0U, 0U, 0U, MAX_uint8_T, 56U, 0U,
    0U, 0U, 0U, 0U, 0U, MAX_uint8_T, 56U, 0U, 0U, 0U, 0U, 0U, 0U, MAX_uint8_T,
    56U, 0U, 0U, 0U, 40U, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T,
    MAX_uint8_T, 176U, 0U, 0U, 0U, 0U, 38U, 246U, 90U, 0U, 0U, 0U, 2U, 197U,
    176U, 0U, 0U, 0U, 0U, 114U, 237U, 24U, 0U, 0U, 0U, 37U, 246U, 91U, 0U, 0U,
    0U, 2U, 196U, 177U, 0U, 0U, 0U, 0U, 113U, 238U, 25U, 0U, 0U, 0U, 36U, 246U,
    93U, 0U, 0U, 0U, 0U, 112U, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T,
    MAX_uint8_T, MAX_uint8_T, 176U, 216U, MAX_uint8_T, 120U, 216U, 44U, 0U, 216U,
    44U, 0U, 216U, 44U, 0U, 216U, 44U, 0U, 216U, 44U, 0U, 216U, 44U, 0U, 216U,
    44U, 0U, 216U, 44U, 0U, 216U, 44U, 0U, 216U, MAX_uint8_T, 120U, 184U, 47U,
    0U, 0U, 0U, 111U, 120U, 0U, 0U, 0U, 38U, 193U, 0U, 0U, 0U, 0U, 217U, 14U, 0U,
    0U, 0U, 149U, 82U, 0U, 0U, 0U, 76U, 155U, 0U, 0U, 0U, 10U, 220U, 0U, 0U, 0U,
    0U, 186U, 44U, 0U, 0U, 0U, 114U, 117U, 0U, 0U, 0U, 41U, 190U, 0U, 0U, 0U, 0U,
    218U, 13U, 143U, MAX_uint8_T, 191U, 0U, 67U, 191U, 0U, 67U, 191U, 0U, 67U,
    191U, 0U, 67U, 191U, 0U, 67U, 191U, 0U, 67U, 191U, 0U, 67U, 191U, 0U, 67U,
    191U, 0U, 67U, 191U, 143U, MAX_uint8_T, 191U, 0U, 0U, 0U, 110U, 25U, 0U, 0U,
    0U, 0U, 17U, 235U, 151U, 0U, 0U, 0U, 0U, 137U, 115U, 208U, 36U, 0U, 0U, 26U,
    215U, 7U, 81U, 167U, 0U, 0U, 153U, 96U, 0U, 0U, 198U, 48U, 36U, 208U, 2U, 0U,
    0U, 64U, 183U, 128U, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T,
    124U, 64U, 225U, 23U, 0U, 0U, 77U, 175U, 0U, 0U, 60U, 200U, 244U, 183U, 9U,
    0U, 0U, 149U, 44U, 23U, 229U, 87U, 0U, 0U, 0U, 0U, 0U, 188U, 107U, 0U, 0U,
    91U, 205U, 242U, MAX_uint8_T, 108U, 0U, 52U, 242U, 73U, 6U, 188U, 108U, 0U,
    84U, 226U, 27U, 47U, 221U, 137U, 0U, 8U, 182U, 241U, 166U, 92U, 235U, 60U,
    216U, 80U, 0U, 0U, 0U, 0U, 216U, 80U, 0U, 0U, 0U, 0U, 216U, 113U, 200U, 243U,
    173U, 14U, 216U, 196U, 45U, 21U, 204U, 137U, 216U, 80U, 0U, 0U, 107U, 207U,
    216U, 80U, 0U, 0U, 90U, 222U, 216U, 125U, 0U, 0U, 119U, 191U, 216U, 246U,
    62U, 34U, 220U, 98U, 216U, 115U, 211U, 244U, 147U, 2U, 0U, 33U, 180U, 240U,
    MAX_uint8_T, 128U, 3U, 212U, 180U, 23U, 0U, 0U, 53U, MAX_uint8_T, 32U, 0U,
    0U, 0U, 78U, 251U, 0U, 0U, 0U, 0U, 46U, MAX_uint8_T, 33U, 0U, 0U, 0U, 0U,
    198U, 183U, 26U, 0U, 0U, 0U, 25U, 178U, 245U, MAX_uint8_T, 148U, 0U, 0U, 0U,
    0U, 0U, 196U, 104U, 0U, 0U, 0U, 0U, 0U, 196U, 104U, 0U, 47U, 208U, 241U,
    154U, 202U, 104U, 1U, 212U, 130U, 15U, 82U, 239U, 104U, 49U, 249U, 8U, 0U,
    0U, 196U, 104U, 79U, 229U, 0U, 0U, 0U, 196U, 104U, 64U, 243U, 2U, 0U, 0U,
    196U, 104U, 12U, 238U, 105U, 11U, 107U, 242U, 104U, 0U, 73U, 221U, 241U,
    130U, 196U, 104U, 0U, 35U, 194U, 241U, 167U, 9U, 2U, 211U, 108U, 15U, 183U,
    115U, 52U, 244U, 2U, 0U, 93U, 186U, 78U, MAX_uint8_T, MAX_uint8_T,
    MAX_uint8_T, MAX_uint8_T, 211U, 47U, 241U, 4U, 0U, 0U, 0U, 0U, 196U, 152U,
    21U, 0U, 0U, 0U, 22U, 170U, 239U, MAX_uint8_T, 208U, 0U, 57U, 218U, 253U,
    196U, 0U, 153U, 160U, 2U, 0U, 136U, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T,
    32U, 0U, 160U, 136U, 0U, 0U, 0U, 160U, 136U, 0U, 0U, 0U, 160U, 136U, 0U, 0U,
    0U, 160U, 136U, 0U, 0U, 0U, 160U, 136U, 0U, 0U, 0U, 160U, 136U, 0U, 0U, 0U,
    42U, 204U, 241U, 154U, 202U, 104U, 1U, 208U, 147U, 16U, 76U, 238U, 104U, 49U,
    251U, 14U, 0U, 0U, 196U, 104U, 79U, 230U, 0U, 0U, 0U, 196U, 104U, 61U, 243U,
    2U, 0U, 0U, 196U, 103U, 9U, 234U, 104U, 11U, 105U, 242U, 97U, 0U, 66U, 219U,
    242U, 136U, 203U, 79U, 0U, 0U, 0U, 7U, 77U, 242U, 22U, 0U, 194U, MAX_uint8_T,
    247U, 206U, 71U, 0U, 216U, 80U, 0U, 0U, 0U, 0U, 216U, 80U, 0U, 0U, 0U, 0U,
    216U, 103U, 184U, 245U, 171U, 2U, 216U, 222U, 66U, 21U, 239U, 67U, 216U, 95U,
    0U, 0U, 206U, 91U, 216U, 80U, 0U, 0U, 204U, 92U, 216U, 80U, 0U, 0U, 204U,
    92U, 216U, 80U, 0U, 0U, 204U, 92U, 216U, 80U, 0U, 0U, 204U, 92U, 216U, 80U,
    0U, 0U, 216U, 80U, 216U, 80U, 216U, 80U, 216U, 80U, 216U, 80U, 216U, 80U,
    216U, 80U, 0U, 0U, 168U, 128U, 0U, 0U, 0U, 0U, 0U, 0U, 168U, 128U, 0U, 0U,
    168U, 128U, 0U, 0U, 168U, 128U, 0U, 0U, 168U, 128U, 0U, 0U, 168U, 128U, 0U,
    0U, 168U, 128U, 0U, 0U, 170U, 124U, 0U, 10U, 211U, 88U, 232U, 248U, 176U, 7U,
    216U, 80U, 0U, 0U, 0U, 0U, 216U, 80U, 0U, 0U, 0U, 0U, 216U, 80U, 0U, 98U,
    208U, 10U, 216U, 80U, 53U, 230U, 34U, 0U, 216U, 102U, 225U, 72U, 0U, 0U,
    216U, 203U, 224U, 12U, 0U, 0U, 216U, 85U, 192U, 176U, 1U, 0U, 216U, 80U, 19U,
    221U, 135U, 0U, 216U, 80U, 0U, 42U, 240U, 93U, 216U, 80U, 216U, 80U, 216U,
    80U, 216U, 80U, 216U, 80U, 216U, 80U, 216U, 80U, 216U, 80U, 216U, 80U, 216U,
    117U, 198U, 245U, 128U, 14U, 184U, 243U, 133U, 0U, 216U, 245U, 80U, 36U,
    251U, 163U, 62U, 61U, 254U, 13U, 216U, 124U, 0U, 0U, 240U, 100U, 0U, 12U,
    MAX_uint8_T, 31U, 216U, 80U, 0U, 0U, 240U, 56U, 0U, 12U, MAX_uint8_T, 32U,
    216U, 80U, 0U, 0U, 240U, 56U, 0U, 12U, MAX_uint8_T, 32U, 216U, 80U, 0U, 0U,
    240U, 56U, 0U, 12U, MAX_uint8_T, 32U, 216U, 80U, 0U, 0U, 240U, 56U, 0U, 12U,
    MAX_uint8_T, 32U, 216U, 103U, 184U, 245U, 171U, 2U, 216U, 222U, 66U, 21U,
    239U, 67U, 216U, 95U, 0U, 0U, 206U, 91U, 216U, 80U, 0U, 0U, 204U, 92U, 216U,
    80U, 0U, 0U, 204U, 92U, 216U, 80U, 0U, 0U, 204U, 92U, 216U, 80U, 0U, 0U,
    204U, 92U, 0U, 35U, 186U, 242U, 216U, 91U, 0U, 3U, 211U, 136U, 14U, 62U,
    242U, 55U, 52U, 249U, 8U, 0U, 0U, 161U, 148U, 79U, 230U, 0U, 0U, 0U, 135U,
    174U, 51U, 248U, 7U, 0U, 0U, 163U, 146U, 2U, 210U, 134U, 13U, 64U, 243U, 53U,
    0U, 33U, 187U, 243U, 215U, 88U, 0U, 216U, 113U, 200U, 243U, 173U, 14U, 216U,
    196U, 45U, 21U, 204U, 137U, 216U, 80U, 0U, 0U, 107U, 207U, 216U, 80U, 0U, 0U,
    90U, 222U, 216U, 104U, 0U, 0U, 119U, 191U, 216U, 209U, 53U, 34U, 220U, 98U,
    216U, 99U, 198U, 244U, 147U, 2U, 216U, 80U, 0U, 0U, 0U, 0U, 216U, 80U, 0U,
    0U, 0U, 0U, 0U, 47U, 208U, 241U, 154U, 202U, 104U, 1U, 212U, 130U, 15U, 82U,
    239U, 104U, 49U, 249U, 8U, 0U, 0U, 196U, 104U, 79U, 229U, 0U, 0U, 0U, 196U,
    104U, 64U, 243U, 2U, 0U, 0U, 196U, 104U, 12U, 238U, 105U, 11U, 107U, 242U,
    104U, 0U, 73U, 221U, 241U, 130U, 196U, 104U, 0U, 0U, 0U, 0U, 0U, 196U, 104U,
    0U, 0U, 0U, 0U, 0U, 196U, 104U, 216U, 115U, 206U, 170U, 216U, 209U, 43U, 0U,
    216U, 87U, 0U, 0U, 216U, 80U, 0U, 0U, 216U, 80U, 0U, 0U, 216U, 80U, 0U, 0U,
    216U, 80U, 0U, 0U, 0U, 98U, 230U, MAX_uint8_T, 184U, 0U, 2U, 246U, 59U, 0U,
    0U, 0U, 0U, 226U, 157U, 20U, 0U, 0U, 0U, 46U, 198U, 244U, 107U, 0U, 0U, 0U,
    0U, 88U, 254U, 28U, 0U, 0U, 0U, 56U, 251U, 23U, 20U, MAX_uint8_T,
    MAX_uint8_T, 228U, 98U, 0U, 0U, 191U, 90U, 0U, 0U, 152U, MAX_uint8_T,
    MAX_uint8_T, MAX_uint8_T, 4U, 0U, 204U, 92U, 0U, 0U, 0U, 204U, 92U, 0U, 0U,
    0U, 204U, 92U, 0U, 0U, 0U, 203U, 93U, 0U, 0U, 0U, 183U, 154U, 4U, 0U, 0U,
    61U, 226U, 251U, 0U, 236U, 60U, 0U, 0U, 220U, 72U, 236U, 60U, 0U, 0U, 220U,
    72U, 236U, 60U, 0U, 0U, 220U, 72U, 236U, 60U, 0U, 0U, 220U, 72U, 235U, 61U,
    0U, 4U, 230U, 72U, 212U, 112U, 21U, 162U, 246U, 72U, 80U, 234U, 227U, 88U,
    220U, 72U, 185U, 109U, 0U, 0U, 30U, 237U, 6U, 99U, 194U, 0U, 0U, 118U, 153U,
    0U, 18U, 248U, 25U, 0U, 208U, 61U, 0U, 0U, 182U, 107U, 40U, 224U, 1U, 0U, 0U,
    96U, 191U, 130U, 133U, 0U, 0U, 0U, 16U, 246U, 228U, 41U, 0U, 0U, 0U, 0U,
    179U, 205U, 0U, 0U, 0U, 203U, 83U, 0U, 12U, 250U, 94U, 0U, 5U, 236U, 12U,
    137U, 147U, 0U, 77U, 234U, 155U, 0U, 66U, 185U, 0U, 71U, 211U, 0U, 147U,
    115U, 216U, 0U, 140U, 109U, 0U, 10U, 248U, 20U, 210U, 11U, 233U, 22U, 213U,
    33U, 0U, 0U, 194U, 113U, 189U, 0U, 173U, 112U, 212U, 0U, 0U, 0U, 128U, 234U,
    120U, 0U, 110U, 234U, 137U, 0U, 0U, 0U, 61U, MAX_uint8_T, 50U, 0U, 47U,
    MAX_uint8_T, 61U, 0U, 0U, 30U, 240U, 76U, 0U, 3U, 205U, 79U, 0U, 95U, 232U,
    20U, 113U, 177U, 0U, 0U, 0U, 173U, 188U, 232U, 29U, 0U, 0U, 0U, 61U,
    MAX_uint8_T, 149U, 0U, 0U, 0U, 3U, 202U, 154U, 242U, 33U, 0U, 0U, 123U, 171U,
    0U, 155U, 195U, 2U, 44U, 228U, 22U, 0U, 12U, 220U, 119U, 187U, 131U, 0U, 0U,
    23U, 240U, 13U, 97U, 220U, 0U, 0U, 114U, 158U, 0U, 14U, 247U, 53U, 0U, 208U,
    59U, 0U, 0U, 172U, 142U, 45U, 216U, 0U, 0U, 0U, 82U, 229U, 142U, 117U, 0U,
    0U, 0U, 7U, 240U, 244U, 23U, 0U, 0U, 0U, 0U, 157U, 175U, 0U, 0U, 0U, 0U, 0U,
    179U, 76U, 0U, 0U, 0U, 0U, 54U, 229U, 3U, 0U, 0U, 0U, 24U, MAX_uint8_T,
    MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, 20U, 0U, 0U, 0U, 2U,
    186U, 174U, 0U, 0U, 0U, 0U, 135U, 215U, 13U, 0U, 0U, 0U, 80U, 241U, 41U, 0U,
    0U, 0U, 39U, 241U, 83U, 0U, 0U, 0U, 12U, 214U, 136U, 0U, 0U, 0U, 0U, 72U,
    MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, 40U, 0U,
    84U, 223U, 27U, 0U, 235U, 47U, 0U, 0U, 230U, 35U, 0U, 0U, 189U, 61U, 0U, 14U,
    205U, 23U, 0U, 220U, 155U, 0U, 0U, 14U, 204U, 23U, 0U, 0U, 190U, 61U, 0U, 0U,
    230U, 35U, 0U, 0U, 235U, 47U, 0U, 0U, 86U, 224U, 27U, 48U, 172U, 48U, 172U,
    48U, 172U, 48U, 172U, 48U, 172U, 48U, 172U, 48U, 172U, 48U, 172U, 48U, 172U,
    48U, 172U, 48U, 172U, 50U, 216U, 66U, 0U, 0U, 72U, 210U, 0U, 0U, 59U, 205U,
    0U, 0U, 85U, 164U, 0U, 0U, 43U, 190U, 9U, 0U, 0U, 179U, 195U, 0U, 42U, 189U,
    9U, 0U, 85U, 165U, 0U, 0U, 59U, 205U, 0U, 0U, 71U, 210U, 0U, 51U, 217U, 68U,
    0U, 6U, 189U, 242U, 170U, 66U, 27U, 222U, 79U, 168U, 21U, 111U, 206U, 244U,
    110U, 176U, 120U, 0U, 0U, 144U, 87U, 151U, 94U, 158U, 101U, 165U, 108U, 172U,
    115U, 176U, 119U, 176U, 120U, 0U, 0U, 28U, 120U, 0U, 0U, 0U, 102U, 223U,
    MAX_uint8_T, MAX_uint8_T, 32U, 75U, 236U, 80U, 120U, 0U, 0U, 170U, 150U, 28U,
    120U, 0U, 0U, 195U, 125U, 28U, 120U, 0U, 0U, 165U, 158U, 28U, 120U, 0U, 0U,
    65U, 246U, 102U, 120U, 0U, 0U, 0U, 97U, 224U, MAX_uint8_T, MAX_uint8_T, 32U,
    0U, 0U, 28U, 120U, 0U, 0U, 0U, 0U, 114U, 234U, MAX_uint8_T, 80U, 0U, 33U,
    243U, 34U, 0U, 0U, 0U, 76U, 213U, 0U, 0U, 0U, 0U, 84U, 212U, 0U, 0U, 0U, 96U,
    MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, 24U, 0U, 0U, 84U, 211U, 0U, 0U, 0U,
    0U, 93U, 190U, 0U, 0U, 0U, 18U, 192U, 76U, 0U, 0U, 0U, 140U, MAX_uint8_T,
    MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, 104U, 52U, 117U, 0U, 0U, 0U, 11U,
    160U, 0U, 0U, 165U, 174U, 242U, 220U, 191U, 67U, 0U, 0U, 139U, 147U, 14U,
    44U, 216U, 37U, 0U, 0U, 197U, 29U, 0U, 0U, 134U, 89U, 0U, 0U, 139U, 147U,
    14U, 42U, 215U, 37U, 0U, 0U, 165U, 174U, 242U, 221U, 191U, 67U, 0U, 52U,
    117U, 0U, 0U, 0U, 11U, 160U, 0U, 53U, 246U, 47U, 0U, 0U, 45U, 209U, 12U, 0U,
    132U, 208U, 5U, 10U, 207U, 49U, 0U, 0U, 5U, 207U, 131U, 155U, 111U, 0U, 0U,
    0U, 0U, 46U, 246U, 180U, 0U, 0U, 0U, 0U, 140U, MAX_uint8_T, MAX_uint8_T,
    MAX_uint8_T, MAX_uint8_T, 20U, 0U, 0U, 0U, 0U, 208U, 88U, 0U, 0U, 0U, 0U,
    140U, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, 20U, 0U, 0U, 0U,
    0U, 208U, 88U, 0U, 0U, 0U, 0U, 0U, 0U, 208U, 88U, 0U, 0U, 0U, 48U, 172U, 48U,
    172U, 48U, 172U, 48U, 172U, 0U, 0U, 0U, 0U, 0U, 0U, 48U, 172U, 48U, 172U,
    48U, 172U, 48U, 172U, 9U, 156U, 235U, MAX_uint8_T, MAX_uint8_T, 32U, 125U,
    179U, 23U, 0U, 0U, 0U, 135U, 171U, 6U, 0U, 0U, 0U, 18U, 237U, 227U, 109U, 6U,
    0U, 115U, 119U, 42U, 163U, 206U, 12U, 156U, 117U, 0U, 0U, 187U, 82U, 43U,
    226U, 128U, 17U, 198U, 36U, 0U, 18U, 135U, 232U, 184U, 0U, 0U, 0U, 0U, 20U,
    212U, 76U, 0U, 0U, 1U, 32U, 209U, 84U, 180U, MAX_uint8_T, 253U, 226U, 130U,
    1U, 252U, 8U, 168U, 88U, 0U, 0U, 45U, 174U, 239U, 246U, 199U, 79U, 0U, 0U,
    0U, 65U, 213U, 86U, 16U, 8U, 61U, 189U, 120U, 0U, 9U, 199U, 22U, 73U, 216U,
    253U, 204U, 2U, 176U, 50U, 75U, 106U, 12U, 227U, 60U, 2U, 0U, 0U, 38U, 141U,
    102U, 66U, 45U, 180U, 0U, 0U, 0U, 0U, 3U, 167U, 74U, 106U, 12U, 224U, 59U,
    3U, 0U, 0U, 48U, 141U, 8U, 199U, 22U, 71U, 216U, 254U, 204U, 3U, 186U, 51U,
    0U, 65U, 213U, 84U, 14U, 9U, 59U, 187U, 124U, 0U, 0U, 0U, 46U, 177U, 241U,
    246U, 198U, 81U, 0U, 0U, 156U, MAX_uint8_T, 233U, 56U, 0U, 2U, 159U, 111U,
    145U, 239U, MAX_uint8_T, 121U, 183U, 237U, 165U, 227U, 0U, 0U, 126U, 72U,
    57U, 141U, 0U, 107U, 164U, 45U, 206U, 21U, 32U, 241U, 16U, 204U, 85U, 0U, 0U,
    107U, 164U, 45U, 206U, 21U, 0U, 0U, 126U, 73U, 57U, 142U, 180U, MAX_uint8_T,
    MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, 60U, 0U, 0U,
    0U, 0U, 0U, 0U, 160U, 60U, 0U, 0U, 0U, 0U, 0U, 0U, 160U, 60U, 32U,
    MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, 20U, 8U,
    154U, 239U, 221U, 84U, 0U, 132U, 109U, 12U, 35U, 180U, 39U, 152U, 64U, 254U,
    167U, 61U, 91U, 133U, 170U, MAX_uint8_T, 184U, 178U, 39U, 9U, 157U, 240U,
    221U, 86U, 0U, 128U, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T,
    124U, 0U, 149U, 244U, 186U, 11U, 18U, 201U, 27U, 160U, 73U, 0U, 149U, 245U,
    186U, 11U, 0U, 0U, 0U, 168U, 52U, 0U, 0U, 0U, 0U, 0U, 0U, 168U, 52U, 0U, 0U,
    0U, 180U, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T,
    MAX_uint8_T, 60U, 0U, 0U, 0U, 168U, 52U, 0U, 0U, 0U, 0U, 0U, 0U, 168U, 52U,
    0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 180U, MAX_uint8_T, MAX_uint8_T,
    MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, 60U, 0U, MAX_uint8_T,
    250U, 167U, 2U, 0U, 0U, 16U, 221U, 36U, 0U, 0U, 23U, 212U, 5U, 0U, 9U, 188U,
    60U, 0U, 2U, 173U, 82U, 0U, 0U, 32U, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T,
    40U, 0U, MAX_uint8_T, 249U, 160U, 0U, 0U, 0U, 44U, 215U, 0U, 0U, 152U, 253U,
    94U, 0U, 0U, 1U, 38U, 225U, 12U, 0U, 0U, 29U, 227U, 19U, 20U, MAX_uint8_T,
    244U, 131U, 0U, 0U, 163U, 151U, 0U, 85U, 166U, 2U, 0U, 216U, 80U, 0U, 0U,
    204U, 92U, 216U, 80U, 0U, 0U, 204U, 92U, 216U, 80U, 0U, 0U, 204U, 92U, 216U,
    80U, 0U, 0U, 204U, 92U, 216U, 81U, 0U, 0U, 216U, 92U, 216U, 144U, 13U, 139U,
    245U, 92U, 216U, 231U, 244U, 122U, 204U, 92U, 216U, 80U, 0U, 0U, 0U, 0U,
    216U, 80U, 0U, 0U, 0U, 0U, 62U, 214U, 249U, MAX_uint8_T, MAX_uint8_T, 8U,
    174U, MAX_uint8_T, MAX_uint8_T, 116U, 176U, 8U, 145U, MAX_uint8_T,
    MAX_uint8_T, 116U, 176U, 8U, 18U, 169U, 243U, 116U, 176U, 8U, 0U, 0U, 72U,
    116U, 176U, 8U, 0U, 0U, 72U, 116U, 176U, 8U, 0U, 0U, 72U, 116U, 176U, 8U, 0U,
    0U, 72U, 116U, 176U, 8U, 0U, 0U, 72U, 116U, 176U, 8U, 0U, 0U, 72U, 116U,
    176U, 8U, 0U, 0U, 72U, 116U, 176U, 8U, 212U, 160U, 0U, 156U, 162U, 0U, 0U,
    18U, 226U, 0U, 4U, MAX_uint8_T, 161U, 0U, 45U, 170U, 42U, 0U, 192U, 235U,
    44U, 0U, 0U, 176U, 44U, 0U, 0U, 176U, 44U, 0U, 0U, 176U, 44U, 0U, 216U,
    MAX_uint8_T, MAX_uint8_T, 88U, 0U, 141U, 241U, 222U, 66U, 48U, 222U, 26U,
    76U, 208U, 48U, 221U, 17U, 76U, 208U, 0U, 144U, 243U, 222U, 68U, 81U, 117U,
    17U, 173U, 7U, 0U, 1U, 171U, 99U, 94U, 174U, 3U, 0U, 21U, 242U, 27U, 189U,
    100U, 1U, 171U, 100U, 94U, 174U, 3U, 81U, 117U, 17U, 174U, 7U, 0U, 19U, 133U,
    106U, 0U, 0U, 0U, 38U, 165U, 0U, 0U, 133U, 222U, 120U, 0U, 0U, 5U, 178U, 20U,
    0U, 0U, 0U, 104U, 120U, 0U, 0U, 132U, 71U, 0U, 0U, 0U, 0U, 104U, 120U, 0U,
    61U, 141U, 1U, 201U, 80U, 0U, 0U, 104U, 120U, 15U, 180U, 8U, 97U, 210U, 80U,
    0U, 0U, 104U, 120U, 157U, 46U, 14U, 171U, 124U, 80U, 0U, 0U, 0U, 90U, 112U,
    0U, 138U, 49U, 124U, 80U, 0U, 0U, 31U, 171U, 1U, 0U, 219U, MAX_uint8_T,
    MAX_uint8_T, MAX_uint8_T, 8U, 2U, 175U, 26U, 0U, 0U, 0U, 0U, 124U, 80U, 0U,
    19U, 133U, 106U, 0U, 0U, 0U, 103U, 100U, 0U, 0U, 133U, 222U, 120U, 0U, 0U,
    38U, 165U, 0U, 0U, 0U, 0U, 104U, 120U, 0U, 5U, 178U, 20U, 0U, 0U, 0U, 0U,
    104U, 120U, 0U, 134U, 106U, MAX_uint8_T, 246U, 134U, 0U, 0U, 104U, 120U, 62U,
    140U, 0U, 0U, 28U, 243U, 1U, 0U, 104U, 136U, 180U, 7U, 0U, 0U, 60U, 184U, 0U,
    0U, 0U, 158U, 44U, 0U, 0U, 24U, 205U, 28U, 0U, 0U, 92U, 111U, 0U, 0U, 9U,
    191U, 59U, 0U, 0U, 32U, 170U, 1U, 0U, 0U, 68U, MAX_uint8_T, MAX_uint8_T,
    MAX_uint8_T, 4U, 88U, MAX_uint8_T, 237U, 84U, 0U, 0U, 0U, 134U, 70U, 0U, 0U,
    3U, 124U, 131U, 0U, 0U, 62U, 140U, 0U, 0U, 0U, 240U, 229U, 30U, 0U, 16U,
    180U, 7U, 0U, 0U, 0U, 5U, 101U, 171U, 0U, 158U, 46U, 201U, 80U, 0U, 0U, 1U,
    88U, 186U, 92U, 111U, 97U, 210U, 80U, 0U, 108U, MAX_uint8_T, 226U, 95U, 170U,
    15U, 171U, 124U, 80U, 0U, 0U, 0U, 3U, 175U, 25U, 138U, 49U, 124U, 80U, 0U,
    0U, 0U, 124U, 80U, 0U, 219U, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, 8U, 0U,
    53U, 149U, 0U, 0U, 0U, 0U, 124U, 80U, 0U, 0U, 0U, 140U, 156U, 0U, 0U, 0U, 0U,
    0U, 0U, 0U, 0U, 142U, 145U, 0U, 0U, 0U, 181U, 87U, 0U, 0U, 90U, 195U, 1U, 0U,
    67U, 236U, 29U, 0U, 0U, 177U, 144U, 0U, 0U, 0U, 164U, 185U, 18U, 0U, 0U, 28U,
    184U, 244U, MAX_uint8_T, 220U, 0U, 0U, 74U, 222U, 17U, 0U, 0U, 0U, 0U, 0U,
    0U, 0U, 87U, 166U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
    0U, 0U, 166U, 236U, 6U, 0U, 0U, 0U, 0U, 0U, 16U, 243U, 246U, 83U, 0U, 0U, 0U,
    0U, 0U, 106U, 187U, 171U, 179U, 0U, 0U, 0U, 0U, 0U, 204U, 101U, 85U, 251U,
    23U, 0U, 0U, 0U, 46U, 251U, 20U, 10U, 244U, 115U, 0U, 0U, 0U, 145U, 186U, 0U,
    0U, 167U, 211U, 0U, 0U, 6U, 236U, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T,
    MAX_uint8_T, MAX_uint8_T, 51U, 0U, 85U, 235U, 10U, 0U, 0U, 4U, 224U, 147U,
    0U, 183U, 124U, 0U, 0U, 0U, 0U, 108U, 236U, 6U, 0U, 0U, 0U, 0U, 170U, 143U,
    0U, 0U, 0U, 0U, 0U, 0U, 94U, 158U, 1U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
    0U, 0U, 0U, 0U, 0U, 0U, 166U, 236U, 6U, 0U, 0U, 0U, 0U, 0U, 16U, 243U, 246U,
    83U, 0U, 0U, 0U, 0U, 0U, 106U, 187U, 171U, 179U, 0U, 0U, 0U, 0U, 0U, 204U,
    101U, 85U, 251U, 23U, 0U, 0U, 0U, 46U, 251U, 20U, 10U, 244U, 115U, 0U, 0U,
    0U, 145U, 186U, 0U, 0U, 167U, 211U, 0U, 0U, 6U, 236U, MAX_uint8_T,
    MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, 51U, 0U, 85U, 235U, 10U,
    0U, 0U, 4U, 224U, 147U, 0U, 183U, 124U, 0U, 0U, 0U, 0U, 108U, 236U, 6U, 0U,
    0U, 6U, 206U, 229U, 45U, 0U, 0U, 0U, 0U, 0U, 138U, 109U, 45U, 193U, 7U, 0U,
    0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 166U, 236U, 6U, 0U, 0U,
    0U, 0U, 0U, 16U, 243U, 246U, 83U, 0U, 0U, 0U, 0U, 0U, 106U, 187U, 171U, 179U,
    0U, 0U, 0U, 0U, 0U, 204U, 101U, 85U, 251U, 23U, 0U, 0U, 0U, 46U, 251U, 20U,
    10U, 244U, 115U, 0U, 0U, 0U, 145U, 186U, 0U, 0U, 167U, 211U, 0U, 0U, 6U,
    236U, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, 51U,
    0U, 85U, 235U, 10U, 0U, 0U, 4U, 224U, 147U, 0U, 183U, 124U, 0U, 0U, 0U, 0U,
    108U, 236U, 6U, 0U, 0U, 114U, 232U, 70U, 194U, 6U, 0U, 0U, 0U, 0U, 190U, 44U,
    206U, 173U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 166U,
    236U, 6U, 0U, 0U, 0U, 0U, 0U, 16U, 243U, 246U, 83U, 0U, 0U, 0U, 0U, 0U, 106U,
    187U, 171U, 179U, 0U, 0U, 0U, 0U, 0U, 204U, 101U, 85U, 251U, 23U, 0U, 0U, 0U,
    46U, 251U, 20U, 10U, 244U, 115U, 0U, 0U, 0U, 145U, 186U, 0U, 0U, 167U, 211U,
    0U, 0U, 6U, 236U, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T,
    MAX_uint8_T, 51U, 0U, 85U, 235U, 10U, 0U, 0U, 4U, 224U, 147U, 0U, 183U, 124U,
    0U, 0U, 0U, 0U, 108U, 236U, 6U, 0U, 0U, 140U, 120U, 56U, 200U, 0U, 0U, 0U,
    0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 166U, 236U, 6U, 0U, 0U, 0U,
    0U, 0U, 16U, 243U, 246U, 83U, 0U, 0U, 0U, 0U, 0U, 106U, 187U, 171U, 179U, 0U,
    0U, 0U, 0U, 0U, 204U, 101U, 85U, 251U, 23U, 0U, 0U, 0U, 46U, 251U, 20U, 10U,
    244U, 115U, 0U, 0U, 0U, 145U, 186U, 0U, 0U, 167U, 211U, 0U, 0U, 6U, 236U,
    MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, 51U, 0U,
    85U, 235U, 10U, 0U, 0U, 4U, 224U, 147U, 0U, 183U, 124U, 0U, 0U, 0U, 0U, 108U,
    236U, 6U, 0U, 0U, 3U, 187U, 227U, 38U, 0U, 0U, 0U, 0U, 0U, 34U, 143U, 69U,
    113U, 0U, 0U, 0U, 0U, 0U, 3U, 187U, 228U, 39U, 0U, 0U, 0U, 0U, 0U, 0U, 166U,
    236U, 6U, 0U, 0U, 0U, 0U, 0U, 16U, 243U, 246U, 83U, 0U, 0U, 0U, 0U, 0U, 106U,
    187U, 171U, 179U, 0U, 0U, 0U, 0U, 0U, 204U, 101U, 85U, 251U, 23U, 0U, 0U, 0U,
    46U, 251U, 20U, 10U, 244U, 115U, 0U, 0U, 0U, 145U, 186U, 0U, 0U, 167U, 211U,
    0U, 0U, 6U, 236U, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T,
    MAX_uint8_T, 51U, 0U, 85U, 235U, 10U, 0U, 0U, 4U, 224U, 147U, 0U, 183U, 124U,
    0U, 0U, 0U, 0U, 108U, 236U, 6U, 0U, 0U, 0U, 0U, 37U, 250U, MAX_uint8_T,
    MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, 84U, 0U, 0U, 0U, 0U, 176U, 233U, 180U,
    0U, 0U, 0U, 0U, 0U, 0U, 0U, 63U, 228U, 143U, 180U, 0U, 0U, 0U, 0U, 0U, 0U,
    1U, 204U, 107U, 136U, 180U, 0U, 0U, 0U, 0U, 0U, 0U, 94U, 228U, 7U, 136U,
    MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, 188U, 0U, 0U, 10U, 227U, 107U, 0U,
    136U, 180U, 0U, 0U, 0U, 0U, 0U, 125U, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T,
    MAX_uint8_T, 180U, 0U, 0U, 0U, 0U, 24U, 243U, 74U, 0U, 0U, 136U, 180U, 0U,
    0U, 0U, 0U, 156U, 149U, 0U, 0U, 0U, 136U, MAX_uint8_T, MAX_uint8_T,
    MAX_uint8_T, MAX_uint8_T, 152U, 0U, 0U, 98U, 207U, 248U, MAX_uint8_T,
    MAX_uint8_T, 160U, 0U, 125U, 240U, 93U, 18U, 0U, 0U, 0U, 19U, 248U, 99U, 0U,
    0U, 0U, 0U, 0U, 77U, MAX_uint8_T, 13U, 0U, 0U, 0U, 0U, 0U, 96U, 244U, 0U, 0U,
    0U, 0U, 0U, 0U, 78U, MAX_uint8_T, 15U, 0U, 0U, 0U, 0U, 0U, 21U, 250U, 108U,
    0U, 0U, 0U, 0U, 0U, 0U, 135U, 244U, 104U, 23U, 0U, 0U, 0U, 0U, 1U, 107U,
    211U, 249U, MAX_uint8_T, MAX_uint8_T, 164U, 0U, 0U, 0U, 0U, 193U, 124U, 0U,
    0U, 0U, 0U, 0U, 0U, 36U, 208U, 0U, 0U, 0U, 0U, 0U, 44U, 253U, 123U, 0U, 0U,
    9U, 194U, 109U, 0U, 0U, 0U, 0U, 14U, 196U, 42U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
    0U, 224U, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, 248U, 0U, 224U, 88U, 0U, 0U,
    0U, 0U, 224U, 88U, 0U, 0U, 0U, 0U, 224U, 88U, 0U, 0U, 0U, 0U, 224U,
    MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, 96U, 0U, 224U, 88U, 0U, 0U, 0U, 0U,
    224U, 88U, 0U, 0U, 0U, 0U, 224U, 88U, 0U, 0U, 0U, 0U, 224U, MAX_uint8_T,
    MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, 56U, 0U, 0U, 71U, 218U, 24U, 0U, 0U,
    19U, 202U, 31U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 224U, MAX_uint8_T,
    MAX_uint8_T, MAX_uint8_T, 248U, 0U, 224U, 88U, 0U, 0U, 0U, 0U, 224U, 88U, 0U,
    0U, 0U, 0U, 224U, 88U, 0U, 0U, 0U, 0U, 224U, MAX_uint8_T, MAX_uint8_T,
    MAX_uint8_T, 96U, 0U, 224U, 88U, 0U, 0U, 0U, 0U, 224U, 88U, 0U, 0U, 0U, 0U,
    224U, 88U, 0U, 0U, 0U, 0U, 224U, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T,
    MAX_uint8_T, 56U, 0U, 99U, 235U, 153U, 0U, 0U, 34U, 196U, 18U, 167U, 78U, 0U,
    0U, 0U, 0U, 0U, 0U, 0U, 224U, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, 248U,
    0U, 224U, 88U, 0U, 0U, 0U, 0U, 224U, 88U, 0U, 0U, 0U, 0U, 224U, 88U, 0U, 0U,
    0U, 0U, 224U, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, 96U, 0U, 224U, 88U, 0U,
    0U, 0U, 0U, 224U, 88U, 0U, 0U, 0U, 0U, 224U, 88U, 0U, 0U, 0U, 0U, 224U,
    MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, 56U, 0U, MAX_uint8_T, 4U,
    172U, 84U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 224U, MAX_uint8_T, MAX_uint8_T,
    MAX_uint8_T, 248U, 0U, 224U, 88U, 0U, 0U, 0U, 0U, 224U, 88U, 0U, 0U, 0U, 0U,
    224U, 88U, 0U, 0U, 0U, 0U, 224U, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, 96U,
    0U, 224U, 88U, 0U, 0U, 0U, 0U, 224U, 88U, 0U, 0U, 0U, 0U, 224U, 88U, 0U, 0U,
    0U, 0U, 224U, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, 56U, 3U,
    173U, 137U, 0U, 0U, 6U, 183U, 64U, 0U, 0U, 0U, 0U, 0U, 0U, 228U, 88U, 0U, 0U,
    228U, 88U, 0U, 0U, 228U, 88U, 0U, 0U, 228U, 88U, 0U, 0U, 228U, 88U, 0U, 0U,
    228U, 88U, 0U, 0U, 228U, 88U, 0U, 0U, 228U, 88U, 0U, 0U, 228U, 88U, 0U, 35U,
    228U, 50U, 3U, 189U, 61U, 0U, 0U, 0U, 0U, 0U, 0U, 228U, 88U, 0U, 0U, 228U,
    88U, 0U, 0U, 228U, 88U, 0U, 0U, 228U, 88U, 0U, 0U, 228U, 88U, 0U, 0U, 228U,
    88U, 0U, 0U, 228U, 88U, 0U, 0U, 228U, 88U, 0U, 0U, 228U, 88U, 0U, 0U, 67U,
    233U, 184U, 1U, 17U, 199U, 29U, 137U, 109U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 228U,
    88U, 0U, 0U, 0U, 228U, 88U, 0U, 0U, 0U, 228U, 88U, 0U, 0U, 0U, 228U, 88U, 0U,
    0U, 0U, 228U, 88U, 0U, 0U, 0U, 228U, 88U, 0U, 0U, 0U, 228U, 88U, 0U, 0U, 0U,
    228U, 88U, 0U, 0U, 0U, 228U, 88U, 0U, 240U, 20U, 156U, 100U, 0U, 0U, 0U, 0U,
    0U, 228U, 88U, 0U, 0U, 228U, 88U, 0U, 0U, 228U, 88U, 0U, 0U, 228U, 88U, 0U,
    0U, 228U, 88U, 0U, 0U, 228U, 88U, 0U, 0U, 228U, 88U, 0U, 0U, 228U, 88U, 0U,
    0U, 228U, 88U, 0U, 0U, 0U, 224U, MAX_uint8_T, MAX_uint8_T, 247U, 215U, 119U,
    2U, 0U, 0U, 0U, 224U, 88U, 2U, 19U, 87U, 239U, 135U, 0U, 0U, 0U, 224U, 88U,
    0U, 0U, 0U, 98U, 247U, 21U, 0U, 0U, 224U, 88U, 0U, 0U, 0U, 11U, MAX_uint8_T,
    68U, 48U, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, 76U, 0U, 0U,
    244U, 89U, 0U, 0U, 224U, 88U, 0U, 0U, 0U, 15U, MAX_uint8_T, 65U, 0U, 0U,
    224U, 88U, 0U, 0U, 0U, 102U, 240U, 10U, 0U, 0U, 224U, 88U, 0U, 23U, 93U,
    240U, 106U, 0U, 0U, 0U, 224U, MAX_uint8_T, MAX_uint8_T, 246U, 201U, 86U, 0U,
    0U, 0U, 21U, 228U, 158U, 88U, 121U, 0U, 0U, 82U, 122U, 125U, 238U, 46U, 0U,
    0U, 0U, 0U, 0U, 0U, 0U, 0U, 224U, 163U, 0U, 0U, 0U, 80U, 192U, 224U,
    MAX_uint8_T, 68U, 0U, 0U, 80U, 192U, 224U, 183U, 220U, 8U, 0U, 80U, 192U,
    224U, 54U, 223U, 133U, 0U, 80U, 192U, 224U, 44U, 73U, 250U, 43U, 80U, 192U,
    224U, 44U, 0U, 168U, 196U, 81U, 192U, 224U, 44U, 0U, 23U, 239U, 183U, 192U,
    224U, 44U, 0U, 0U, 102U, MAX_uint8_T, 192U, 224U, 44U, 0U, 0U, 1U, 195U,
    192U, 0U, 0U, 6U, 185U, 121U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 10U, 191U, 51U, 0U,
    0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 101U, 213U, 249U, 231U,
    148U, 14U, 0U, 0U, 122U, 235U, 80U, 14U, 45U, 195U, 200U, 5U, 17U, 247U, 91U,
    0U, 0U, 0U, 21U, 244U, 92U, 76U, MAX_uint8_T, 11U, 0U, 0U, 0U, 0U, 183U,
    159U, 96U, 243U, 0U, 0U, 0U, 0U, 0U, 160U, 179U, 76U, MAX_uint8_T, 12U, 0U,
    0U, 0U, 0U, 184U, 159U, 17U, 246U, 94U, 0U, 0U, 0U, 21U, 245U, 91U, 0U, 122U,
    236U, 78U, 13U, 46U, 196U, 200U, 5U, 0U, 0U, 103U, 215U, 250U, 230U, 148U,
    14U, 0U, 0U, 0U, 0U, 0U, 45U, 227U, 40U, 0U, 0U, 0U, 0U, 0U, 7U, 196U, 50U,
    0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 101U, 213U, 249U,
    231U, 148U, 14U, 0U, 0U, 122U, 235U, 80U, 14U, 45U, 195U, 200U, 5U, 17U,
    247U, 91U, 0U, 0U, 0U, 21U, 244U, 92U, 76U, MAX_uint8_T, 11U, 0U, 0U, 0U, 0U,
    183U, 159U, 96U, 243U, 0U, 0U, 0U, 0U, 0U, 160U, 179U, 76U, MAX_uint8_T, 12U,
    0U, 0U, 0U, 0U, 184U, 159U, 17U, 246U, 94U, 0U, 0U, 0U, 21U, 245U, 91U, 0U,
    122U, 236U, 78U, 13U, 46U, 196U, 200U, 5U, 0U, 0U, 103U, 215U, 250U, 230U,
    148U, 14U, 0U, 0U, 0U, 0U, 82U, 235U, 169U, 0U, 0U, 0U, 0U, 0U, 25U, 199U,
    22U, 152U, 93U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 101U,
    213U, 249U, 231U, 148U, 14U, 0U, 0U, 122U, 235U, 80U, 14U, 45U, 195U, 200U,
    5U, 17U, 247U, 91U, 0U, 0U, 0U, 21U, 244U, 92U, 76U, MAX_uint8_T, 11U, 0U,
    0U, 0U, 0U, 183U, 159U, 96U, 243U, 0U, 0U, 0U, 0U, 0U, 160U, 179U, 76U,
    MAX_uint8_T, 12U, 0U, 0U, 0U, 0U, 184U, 159U, 17U, 246U, 94U, 0U, 0U, 0U,
    21U, 245U, 91U, 0U, 122U, 236U, 78U, 13U, 46U, 196U, 200U, 5U, 0U, 0U, 103U,
    215U, 250U, 230U, 148U, 14U, 0U, 0U, 0U, 8U, 218U, 175U, 69U, 145U, 0U, 0U,
    0U, 0U, 58U, 144U, 106U, 239U, 65U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
    0U, 0U, 0U, 101U, 213U, 249U, 231U, 148U, 14U, 0U, 0U, 122U, 235U, 80U, 14U,
    45U, 195U, 200U, 5U, 17U, 247U, 91U, 0U, 0U, 0U, 21U, 244U, 92U, 76U,
    MAX_uint8_T, 11U, 0U, 0U, 0U, 0U, 183U, 159U, 96U, 243U, 0U, 0U, 0U, 0U, 0U,
    160U, 179U, 76U, MAX_uint8_T, 12U, 0U, 0U, 0U, 0U, 184U, 159U, 17U, 246U,
    94U, 0U, 0U, 0U, 21U, 245U, 91U, 0U, 122U, 236U, 78U, 13U, 46U, 196U, 200U,
    5U, 0U, 0U, 103U, 215U, 250U, 230U, 148U, 14U, 0U, 0U, 0U, 0U, MAX_uint8_T,
    4U, 172U, 84U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 101U,
    213U, 249U, 231U, 148U, 14U, 0U, 0U, 122U, 235U, 80U, 14U, 45U, 195U, 200U,
    5U, 17U, 247U, 91U, 0U, 0U, 0U, 21U, 244U, 92U, 76U, MAX_uint8_T, 11U, 0U,
    0U, 0U, 0U, 183U, 159U, 96U, 243U, 0U, 0U, 0U, 0U, 0U, 160U, 179U, 76U,
    MAX_uint8_T, 12U, 0U, 0U, 0U, 0U, 184U, 159U, 17U, 246U, 94U, 0U, 0U, 0U,
    21U, 245U, 91U, 0U, 122U, 236U, 78U, 13U, 46U, 196U, 200U, 5U, 0U, 0U, 103U,
    215U, 250U, 230U, 148U, 14U, 0U, 109U, 107U, 0U, 0U, 0U, 17U, 183U, 13U, 7U,
    184U, 106U, 0U, 17U, 201U, 78U, 0U, 0U, 10U, 193U, 118U, 203U, 88U, 0U, 0U,
    0U, 0U, 28U, 249U, 160U, 0U, 0U, 0U, 0U, 10U, 193U, 118U, 204U, 88U, 0U, 0U,
    6U, 184U, 106U, 0U, 18U, 201U, 77U, 0U, 108U, 107U, 0U, 0U, 0U, 18U, 183U,
    13U, 0U, 0U, 102U, 213U, 248U, 225U, 140U, 187U, 71U, 0U, 122U, 235U, 79U,
    13U, 43U, 216U, 219U, 4U, 17U, 247U, 91U, 0U, 0U, 69U, 188U, 244U, 91U, 76U,
    MAX_uint8_T, 11U, 0U, 32U, 206U, 20U, 180U, 158U, 95U, 242U, 0U, 9U, 198U,
    51U, 0U, 159U, 179U, 75U, MAX_uint8_T, 10U, 163U, 96U, 0U, 0U, 184U, 159U,
    16U, 246U, 183U, 149U, 0U, 0U, 21U, 245U, 91U, 0U, 137U, 253U, 82U, 12U, 45U,
    195U, 200U, 5U, 17U, 206U, 114U, 208U, 246U, 233U, 150U, 15U, 0U, 0U, 56U,
    227U, 29U, 0U, 0U, 0U, 0U, 0U, 68U, 183U, 2U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
    0U, 244U, 72U, 0U, 0U, 0U, 208U, 68U, 244U, 72U, 0U, 0U, 0U, 208U, 68U, 244U,
    72U, 0U, 0U, 0U, 208U, 68U, 244U, 72U, 0U, 0U, 0U, 208U, 68U, 244U, 72U, 0U,
    0U, 0U, 208U, 68U, 240U, 75U, 0U, 0U, 0U, 210U, 64U, 214U, 105U, 0U, 0U, 1U,
    235U, 39U, 137U, 216U, 43U, 11U, 110U, 221U, 4U, 12U, 150U, 229U, 243U, 197U,
    51U, 0U, 0U, 0U, 0U, 147U, 165U, 2U, 0U, 0U, 0U, 70U, 178U, 4U, 0U, 0U, 0U,
    0U, 0U, 0U, 0U, 0U, 0U, 244U, 72U, 0U, 0U, 0U, 208U, 68U, 244U, 72U, 0U, 0U,
    0U, 208U, 68U, 244U, 72U, 0U, 0U, 0U, 208U, 68U, 244U, 72U, 0U, 0U, 0U, 208U,
    68U, 244U, 72U, 0U, 0U, 0U, 208U, 68U, 240U, 75U, 0U, 0U, 0U, 210U, 64U,
    214U, 105U, 0U, 0U, 1U, 235U, 39U, 137U, 216U, 43U, 11U, 110U, 221U, 4U, 12U,
    150U, 229U, 243U, 197U, 51U, 0U, 0U, 2U, 191U, 232U, 60U, 0U, 0U, 0U, 118U,
    129U, 34U, 198U, 14U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 244U, 72U, 0U, 0U, 0U,
    208U, 68U, 244U, 72U, 0U, 0U, 0U, 208U, 68U, 244U, 72U, 0U, 0U, 0U, 208U,
    68U, 244U, 72U, 0U, 0U, 0U, 208U, 68U, 244U, 72U, 0U, 0U, 0U, 208U, 68U,
    240U, 75U, 0U, 0U, 0U, 210U, 64U, 214U, 105U, 0U, 0U, 1U, 235U, 39U, 137U,
    216U, 43U, 11U, 110U, 221U, 4U, 12U, 150U, 229U, 243U, 197U, 51U, 0U, 0U,
    132U, 128U, 48U, 208U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 244U, 72U, 0U, 0U,
    0U, 208U, 68U, 244U, 72U, 0U, 0U, 0U, 208U, 68U, 244U, 72U, 0U, 0U, 0U, 208U,
    68U, 244U, 72U, 0U, 0U, 0U, 208U, 68U, 244U, 72U, 0U, 0U, 0U, 208U, 68U,
    240U, 75U, 0U, 0U, 0U, 210U, 64U, 214U, 105U, 0U, 0U, 1U, 235U, 39U, 137U,
    216U, 43U, 11U, 110U, 221U, 4U, 12U, 150U, 229U, 243U, 197U, 51U, 0U, 0U, 0U,
    0U, 30U, 227U, 55U, 0U, 0U, 0U, 0U, 2U, 184U, 67U, 0U, 0U, 0U, 0U, 0U, 0U,
    0U, 0U, 0U, 0U, 0U, 159U, 199U, 1U, 0U, 0U, 39U, 231U, 22U, 24U, 242U, 92U,
    0U, 0U, 190U, 102U, 0U, 0U, 120U, 228U, 11U, 96U, 198U, 1U, 0U, 0U, 7U, 221U,
    150U, 231U, 46U, 0U, 0U, 0U, 0U, 81U, MAX_uint8_T, 139U, 0U, 0U, 0U, 0U, 0U,
    0U, MAX_uint8_T, 56U, 0U, 0U, 0U, 0U, 0U, 0U, MAX_uint8_T, 56U, 0U, 0U, 0U,
    0U, 0U, 0U, MAX_uint8_T, 56U, 0U, 0U, 0U, 0U, 0U, 0U, MAX_uint8_T, 56U, 0U,
    0U, 0U, 224U, 88U, 0U, 0U, 0U, 0U, 224U, 88U, 0U, 0U, 0U, 0U, 224U,
    MAX_uint8_T, 252U, 229U, 141U, 1U, 224U, 88U, 8U, 73U, 253U, 75U, 224U, 88U,
    0U, 0U, 238U, 89U, 224U, 88U, 16U, 126U, 239U, 26U, 224U, MAX_uint8_T, 236U,
    184U, 52U, 0U, 224U, 88U, 0U, 0U, 0U, 0U, 224U, 88U, 0U, 0U, 0U, 0U, 60U,
    219U, 246U, 185U, 17U, 0U, 182U, 133U, 20U, 218U, 96U, 0U, 212U, 80U, 12U,
    229U, 39U, 0U, 216U, 80U, 136U, 140U, 0U, 0U, 216U, 80U, 172U, 105U, 0U, 0U,
    216U, 80U, 20U, 165U, 142U, 11U, 216U, 80U, 0U, 0U, 122U, 164U, 216U, 80U,
    0U, 1U, 121U, 193U, 216U, 80U, 224U, MAX_uint8_T, 220U, 62U, 0U, 36U, 226U,
    50U, 0U, 0U, 0U, 0U, 0U, 45U, 197U, 9U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
    0U, 60U, 200U, 244U, 183U, 9U, 0U, 0U, 149U, 44U, 23U, 229U, 87U, 0U, 0U, 0U,
    0U, 0U, 188U, 107U, 0U, 0U, 91U, 205U, 242U, MAX_uint8_T, 108U, 0U, 52U,
    242U, 73U, 6U, 188U, 108U, 0U, 84U, 226U, 27U, 47U, 221U, 137U, 0U, 8U, 182U,
    241U, 166U, 92U, 235U, 60U, 0U, 0U, 0U, 115U, 191U, 8U, 0U, 0U, 0U, 45U,
    195U, 13U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 60U, 200U, 244U, 183U, 9U,
    0U, 0U, 149U, 44U, 23U, 229U, 87U, 0U, 0U, 0U, 0U, 0U, 188U, 107U, 0U, 0U,
    91U, 205U, 242U, MAX_uint8_T, 108U, 0U, 52U, 242U, 73U, 6U, 188U, 108U, 0U,
    84U, 226U, 27U, 47U, 221U, 137U, 0U, 8U, 182U, 241U, 166U, 92U, 235U, 60U,
    0U, 0U, 166U, 235U, 85U, 0U, 0U, 0U, 89U, 156U, 21U, 198U, 27U, 0U, 0U, 0U,
    0U, 0U, 0U, 0U, 0U, 0U, 60U, 200U, 244U, 183U, 9U, 0U, 0U, 149U, 44U, 23U,
    229U, 87U, 0U, 0U, 0U, 0U, 0U, 188U, 107U, 0U, 0U, 91U, 205U, 242U,
    MAX_uint8_T, 108U, 0U, 52U, 242U, 73U, 6U, 188U, 108U, 0U, 84U, 226U, 27U,
    47U, 221U, 137U, 0U, 8U, 182U, 241U, 166U, 92U, 235U, 60U, 0U, 67U, 239U,
    107U, 145U, 57U, 0U, 0U, 146U, 69U, 174U, 217U, 7U, 0U, 0U, 0U, 0U, 0U, 0U,
    0U, 0U, 0U, 60U, 200U, 244U, 183U, 9U, 0U, 0U, 149U, 44U, 23U, 229U, 87U, 0U,
    0U, 0U, 0U, 0U, 188U, 107U, 0U, 0U, 91U, 205U, 242U, MAX_uint8_T, 108U, 0U,
    52U, 242U, 73U, 6U, 188U, 108U, 0U, 84U, 226U, 27U, 47U, 221U, 137U, 0U, 8U,
    182U, 241U, 166U, 92U, 235U, 60U, 0U, 88U, 172U, 4U, 252U, 0U, 0U, 0U, 0U,
    0U, 0U, 0U, 0U, 0U, 0U, 60U, 200U, 244U, 183U, 9U, 0U, 0U, 149U, 44U, 23U,
    229U, 87U, 0U, 0U, 0U, 0U, 0U, 188U, 107U, 0U, 0U, 91U, 205U, 242U,
    MAX_uint8_T, 108U, 0U, 52U, 242U, 73U, 6U, 188U, 108U, 0U, 84U, 226U, 27U,
    47U, 221U, 137U, 0U, 8U, 182U, 241U, 166U, 92U, 235U, 60U, 0U, 0U, 148U,
    238U, 69U, 0U, 0U, 0U, 0U, 172U, 39U, 148U, 0U, 0U, 0U, 0U, 148U, 239U, 71U,
    0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 60U, 200U, 244U, 183U, 9U, 0U, 0U,
    149U, 44U, 23U, 229U, 87U, 0U, 0U, 0U, 0U, 0U, 188U, 107U, 0U, 0U, 91U, 205U,
    242U, MAX_uint8_T, 108U, 0U, 52U, 242U, 73U, 6U, 188U, 108U, 0U, 84U, 226U,
    27U, 47U, 221U, 137U, 0U, 8U, 182U, 241U, 166U, 92U, 235U, 60U, 0U, 60U,
    200U, 240U, 163U, 116U, 232U, 220U, 70U, 0U, 0U, 149U, 44U, 23U, 229U, 226U,
    31U, 71U, 233U, 4U, 0U, 0U, 0U, 0U, 188U, 141U, 0U, 0U, 232U, 56U, 0U, 91U,
    205U, 242U, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T,
    80U, 52U, 242U, 73U, 6U, 188U, 136U, 0U, 0U, 0U, 0U, 84U, 226U, 27U, 54U,
    201U, 238U, 64U, 5U, 0U, 0U, 8U, 182U, 246U, 176U, 21U, 115U, 229U,
    MAX_uint8_T, MAX_uint8_T, 92U, 0U, 33U, 180U, 240U, MAX_uint8_T, 128U, 3U,
    212U, 180U, 23U, 0U, 0U, 53U, MAX_uint8_T, 32U, 0U, 0U, 0U, 78U, 251U, 0U,
    0U, 0U, 0U, 46U, MAX_uint8_T, 33U, 0U, 0U, 0U, 0U, 198U, 183U, 26U, 0U, 0U,
    0U, 25U, 178U, 245U, MAX_uint8_T, 148U, 0U, 0U, 1U, 199U, 117U, 0U, 0U, 0U,
    0U, 43U, 202U, 0U, 0U, 0U, 52U, 252U, 115U, 0U, 0U, 41U, 227U, 45U, 0U, 0U,
    0U, 0U, 51U, 195U, 7U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 35U, 194U, 241U, 167U,
    9U, 2U, 211U, 108U, 15U, 183U, 115U, 52U, 244U, 2U, 0U, 93U, 186U, 78U,
    MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, 211U, 47U, 241U, 4U, 0U,
    0U, 0U, 0U, 196U, 152U, 21U, 0U, 0U, 0U, 22U, 170U, 239U, MAX_uint8_T, 208U,
    0U, 0U, 0U, 123U, 185U, 6U, 0U, 0U, 51U, 191U, 10U, 0U, 0U, 0U, 0U, 0U, 0U,
    0U, 0U, 35U, 194U, 241U, 167U, 9U, 2U, 211U, 108U, 15U, 183U, 115U, 52U,
    244U, 2U, 0U, 93U, 186U, 78U, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T,
    MAX_uint8_T, 211U, 47U, 241U, 4U, 0U, 0U, 0U, 0U, 196U, 152U, 21U, 0U, 0U,
    0U, 22U, 170U, 239U, MAX_uint8_T, 208U, 0U, 0U, 147U, 235U, 105U, 0U, 0U,
    70U, 173U, 16U, 193U, 39U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 35U, 194U, 241U, 167U,
    9U, 2U, 211U, 108U, 15U, 183U, 115U, 52U, 244U, 2U, 0U, 93U, 186U, 78U,
    MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, 211U, 47U, 241U, 4U, 0U,
    0U, 0U, 0U, 196U, 152U, 21U, 0U, 0U, 0U, 22U, 170U, 239U, MAX_uint8_T, 208U,
    0U, 52U, 208U, 0U, 224U, 32U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 35U, 194U, 241U,
    167U, 9U, 2U, 211U, 108U, 15U, 183U, 115U, 52U, 244U, 2U, 0U, 93U, 186U, 78U,
    MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, 211U, 47U, 241U, 4U, 0U,
    0U, 0U, 0U, 196U, 152U, 21U, 0U, 0U, 0U, 22U, 170U, 239U, MAX_uint8_T, 208U,
    3U, 173U, 137U, 0U, 0U, 6U, 183U, 64U, 0U, 0U, 0U, 0U, 0U, 0U, 216U, 80U, 0U,
    0U, 216U, 80U, 0U, 0U, 216U, 80U, 0U, 0U, 216U, 80U, 0U, 0U, 216U, 80U, 0U,
    0U, 216U, 80U, 0U, 0U, 216U, 80U, 0U, 35U, 228U, 50U, 3U, 189U, 61U, 0U, 0U,
    0U, 0U, 0U, 0U, 216U, 80U, 0U, 0U, 216U, 80U, 0U, 0U, 216U, 80U, 0U, 0U,
    216U, 80U, 0U, 0U, 216U, 80U, 0U, 0U, 216U, 80U, 0U, 0U, 216U, 80U, 0U, 0U,
    67U, 233U, 184U, 1U, 17U, 199U, 29U, 137U, 109U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
    216U, 80U, 0U, 0U, 0U, 216U, 80U, 0U, 0U, 0U, 216U, 80U, 0U, 0U, 0U, 216U,
    80U, 0U, 0U, 0U, 216U, 80U, 0U, 0U, 0U, 216U, 80U, 0U, 0U, 0U, 216U, 80U, 0U,
    240U, 20U, 156U, 100U, 0U, 0U, 0U, 0U, 0U, 216U, 80U, 0U, 0U, 216U, 80U, 0U,
    0U, 216U, 80U, 0U, 0U, 216U, 80U, 0U, 0U, 216U, 80U, 0U, 0U, 216U, 80U, 0U,
    0U, 216U, 80U, 0U, 0U, 1U, 5U, 139U, 56U, 0U, 0U, 90U, 227U, 251U, 226U, 21U,
    0U, 0U, 0U, 86U, 114U, 105U, 219U, 25U, 0U, 0U, 36U, 190U, 243U, 248U, 190U,
    0U, 3U, 214U, 144U, 15U, 70U, 250U, 54U, 57U, 250U, 10U, 0U, 0U, 181U, 117U,
    83U, 231U, 0U, 0U, 0U, 159U, 139U, 54U, 251U, 12U, 0U, 0U, 194U, 109U, 2U,
    210U, 147U, 15U, 89U, 243U, 24U, 0U, 33U, 187U, 242U, 208U, 63U, 0U, 0U,
    168U, 211U, 45U, 192U, 0U, 2U, 198U, 64U, 229U, 119U, 0U, 0U, 0U, 0U, 0U, 0U,
    0U, 216U, 103U, 184U, 245U, 171U, 2U, 216U, 222U, 66U, 21U, 239U, 67U, 216U,
    95U, 0U, 0U, 206U, 91U, 216U, 80U, 0U, 0U, 204U, 92U, 216U, 80U, 0U, 0U,
    204U, 92U, 216U, 80U, 0U, 0U, 204U, 92U, 216U, 80U, 0U, 0U, 204U, 92U, 0U,
    5U, 182U, 125U, 0U, 0U, 0U, 0U, 0U, 9U, 189U, 54U, 0U, 0U, 0U, 0U, 0U, 0U,
    0U, 0U, 0U, 0U, 35U, 186U, 242U, 216U, 91U, 0U, 3U, 211U, 136U, 14U, 62U,
    242U, 55U, 52U, 249U, 8U, 0U, 0U, 161U, 148U, 79U, 230U, 0U, 0U, 0U, 135U,
    174U, 51U, 248U, 7U, 0U, 0U, 163U, 146U, 2U, 210U, 134U, 13U, 64U, 243U, 53U,
    0U, 33U, 187U, 243U, 215U, 88U, 0U, 0U, 0U, 0U, 42U, 228U, 43U, 0U, 0U, 0U,
    6U, 194U, 53U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 35U, 186U, 242U, 216U,
    91U, 0U, 3U, 211U, 136U, 14U, 62U, 242U, 55U, 52U, 249U, 8U, 0U, 0U, 161U,
    148U, 79U, 230U, 0U, 0U, 0U, 135U, 174U, 51U, 248U, 7U, 0U, 0U, 163U, 146U,
    2U, 210U, 134U, 13U, 64U, 243U, 53U, 0U, 33U, 187U, 243U, 215U, 88U, 0U, 0U,
    0U, 78U, 234U, 173U, 0U, 0U, 0U, 23U, 199U, 24U, 149U, 97U, 0U, 0U, 0U, 0U,
    0U, 0U, 0U, 0U, 0U, 35U, 186U, 242U, 216U, 91U, 0U, 3U, 211U, 136U, 14U, 62U,
    242U, 55U, 52U, 249U, 8U, 0U, 0U, 161U, 148U, 79U, 230U, 0U, 0U, 0U, 135U,
    174U, 51U, 248U, 7U, 0U, 0U, 163U, 146U, 2U, 210U, 134U, 13U, 64U, 243U, 53U,
    0U, 33U, 187U, 243U, 215U, 88U, 0U, 0U, 7U, 216U, 178U, 66U, 149U, 0U, 0U,
    54U, 148U, 103U, 239U, 68U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 35U, 186U,
    242U, 216U, 91U, 0U, 3U, 211U, 136U, 14U, 62U, 242U, 55U, 52U, 249U, 8U, 0U,
    0U, 161U, 148U, 79U, 230U, 0U, 0U, 0U, 135U, 174U, 51U, 248U, 7U, 0U, 0U,
    163U, 146U, 2U, 210U, 134U, 13U, 64U, 243U, 53U, 0U, 33U, 187U, 243U, 215U,
    88U, 0U, 0U, 0U, 252U, 8U, 168U, 88U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
    35U, 186U, 242U, 216U, 91U, 0U, 3U, 211U, 136U, 14U, 62U, 242U, 55U, 52U,
    249U, 8U, 0U, 0U, 161U, 148U, 79U, 230U, 0U, 0U, 0U, 135U, 174U, 51U, 248U,
    7U, 0U, 0U, 163U, 146U, 2U, 210U, 134U, 13U, 64U, 243U, 53U, 0U, 33U, 187U,
    243U, 215U, 88U, 0U, 0U, 0U, 0U, 244U, 128U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
    0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 180U, MAX_uint8_T, MAX_uint8_T,
    MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, 60U, 0U, 0U, 0U, 0U, 0U,
    0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 244U, 128U, 0U, 0U,
    0U, 0U, 33U, 185U, 242U, 221U, 179U, 89U, 2U, 210U, 135U, 14U, 114U,
    MAX_uint8_T, 53U, 51U, 248U, 7U, 21U, 188U, 185U, 147U, 79U, 229U, 2U, 176U,
    43U, 137U, 175U, 54U, 249U, 141U, 90U, 0U, 161U, 147U, 2U, 213U, 201U, 12U,
    62U, 242U, 55U, 20U, 191U, 190U, 245U, 217U, 91U, 0U, 5U, 182U, 125U, 0U, 0U,
    0U, 0U, 9U, 189U, 54U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 236U, 60U, 0U, 0U,
    220U, 72U, 236U, 60U, 0U, 0U, 220U, 72U, 236U, 60U, 0U, 0U, 220U, 72U, 236U,
    60U, 0U, 0U, 220U, 72U, 235U, 61U, 0U, 4U, 230U, 72U, 212U, 112U, 21U, 162U,
    246U, 72U, 80U, 234U, 227U, 88U, 220U, 72U, 0U, 0U, 42U, 228U, 43U, 0U, 0U,
    6U, 194U, 53U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 236U, 60U, 0U, 0U, 220U, 72U,
    236U, 60U, 0U, 0U, 220U, 72U, 236U, 60U, 0U, 0U, 220U, 72U, 236U, 60U, 0U,
    0U, 220U, 72U, 235U, 61U, 0U, 4U, 230U, 72U, 212U, 112U, 21U, 162U, 246U,
    72U, 80U, 234U, 227U, 88U, 220U, 72U, 0U, 78U, 234U, 173U, 0U, 0U, 23U, 199U,
    24U, 149U, 97U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 236U, 60U, 0U, 0U, 220U, 72U,
    236U, 60U, 0U, 0U, 220U, 72U, 236U, 60U, 0U, 0U, 220U, 72U, 236U, 60U, 0U,
    0U, 220U, 72U, 235U, 61U, 0U, 4U, 230U, 72U, 212U, 112U, 21U, 162U, 246U,
    72U, 80U, 234U, 227U, 88U, 220U, 72U, 0U, 252U, 8U, 168U, 88U, 0U, 0U, 0U,
    0U, 0U, 0U, 0U, 236U, 60U, 0U, 0U, 220U, 72U, 236U, 60U, 0U, 0U, 220U, 72U,
    236U, 60U, 0U, 0U, 220U, 72U, 236U, 60U, 0U, 0U, 220U, 72U, 235U, 61U, 0U,
    4U, 230U, 72U, 212U, 112U, 21U, 162U, 246U, 72U, 80U, 234U, 227U, 88U, 220U,
    72U, 0U, 0U, 0U, 166U, 147U, 0U, 0U, 0U, 0U, 89U, 162U, 1U, 0U, 0U, 0U, 0U,
    0U, 0U, 0U, 0U, 0U, 187U, 131U, 0U, 0U, 23U, 240U, 13U, 97U, 220U, 0U, 0U,
    114U, 158U, 0U, 14U, 247U, 53U, 0U, 208U, 59U, 0U, 0U, 172U, 142U, 45U, 216U,
    0U, 0U, 0U, 82U, 229U, 142U, 117U, 0U, 0U, 0U, 7U, 240U, 244U, 23U, 0U, 0U,
    0U, 0U, 157U, 175U, 0U, 0U, 0U, 0U, 0U, 179U, 76U, 0U, 0U, 0U, 0U, 54U, 229U,
    3U, 0U, 0U, 0U, 216U, 80U, 0U, 0U, 0U, 0U, 216U, 80U, 0U, 0U, 0U, 0U, 216U,
    90U, 179U, 242U, 173U, 14U, 216U, 196U, 66U, 22U, 206U, 137U, 216U, 105U, 0U,
    0U, 108U, 207U, 216U, 80U, 0U, 0U, 90U, 222U, 216U, 104U, 0U, 0U, 120U, 191U,
    216U, 209U, 53U, 34U, 220U, 98U, 216U, 99U, 198U, 244U, 147U, 2U, 216U, 80U,
    0U, 0U, 0U, 0U, 216U, 80U, 0U, 0U, 0U, 0U, 0U, 116U, 144U, 32U, 224U, 0U, 0U,
    0U, 0U, 0U, 0U, 0U, 0U, 0U, 187U, 131U, 0U, 0U, 23U, 240U, 13U, 97U, 220U,
    0U, 0U, 114U, 158U, 0U, 14U, 247U, 53U, 0U, 208U, 59U, 0U, 0U, 172U, 142U,
    45U, 216U, 0U, 0U, 0U, 82U, 229U, 142U, 117U, 0U, 0U, 0U, 7U, 240U, 244U,
    23U, 0U, 0U, 0U, 0U, 157U, 175U, 0U, 0U, 0U, 0U, 0U, 179U, 76U, 0U, 0U, 0U,
    0U, 54U, 229U, 3U, 0U, 0U, 0U };

  static const signed char iv2[261] = { 9, 0, 0, 4, 4, 4, 8, 8, 8, 8, 3, 4, 4, 6,
    10, 4, 7, 4, 6, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 4, 4, 10, 10, 10, 5, 10, 8, 7,
    8, 9, 7, 6, 9, 9, 3, 4, 8, 6, 10, 9, 9, 7, 9, 8, 6, 8, 8, 8, 10, 8, 7, 7, 4,
    6, 4, 8, 6, 7, 7, 8, 6, 8, 7, 4, 7, 7, 3, 4, 7, 3, 11, 7, 7, 8, 8, 5, 6, 4,
    7, 6, 9, 7, 6, 7, 4, 4, 4, 8, 8, 8, 8, 7, 9, 9, 8, 7, 7, 7, 7, 7, 7, 6, 7, 7,
    7, 7, 3, 3, 3, 3, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 0, 5, 8, 8, 8, 0, 8, 7, 8,
    10, 0, 7, 7, 0, 11, 9, 0, 10, 0, 0, 8, 8, 0, 0, 0, 0, 0, 6, 6, 0, 10, 7, 5,
    4, 10, 0, 0, 0, 0, 6, 6, 0, 4, 8, 8, 9, 0, 0, 0, 0, 0, 0, 0, 0, 10, 0, 6, 0,
    0, 8, 0, 0, 0, 0, 0, 0, 0, 0, 0, 8, 7, 8, 7, 7, 3, 3, 3, 3, 9, 9, 0, 9, 8, 8,
    8, 0, 0, 0, 0, 0, 0, 0, 7, 0, 0, 0, 0, 0, 0, 0, 0, 0, 4, 9, 7, 7, 6, 7, 8, 0,
    10, 5, 5, 5, 10, 10, 10, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 7, 6, 4 };

  static const signed char iv4[261] = { 1, 0, 0, 0, 1, 0, 0, 1, 0, 0, 0, 0, 0, 0,
    1, 1, 0, 1, 1, 0, 1, 1, 1, 0, 1, 0, 1, 1, 0, 1, 1, 1, 1, 1, 0, 0, 0, 1, 0, 1,
    1, 1, 0, 1, 1, -2, 1, 1, 1, 1, 0, 1, 0, 1, 0, 0, 1, 0, 0, 0, 0, 0, 1, 1, 0,
    0, 0, 2, 0, 1, 0, 0, 0, 0, 0, 1, 1, -1, 1, 1, 1, 1, 0, 1, 0, 1, 0, 0, 1, 0,
    0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, -1, -1, 0, 1, 0, 0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 1, 1, 1, 0, 1, 1, 1, 0, 0,
    2, 2, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 1, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 1, 0, 1, 1, 0, -1, 0, -1, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0,
    0, 0, 0, 0, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, -1, 0, 0, 0, 1, 1, 0, 1, 1, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1 };

  static const signed char iv5[261] = { 8, 0, 0, 0, 9, 9, 9, 10, 9, 9, 9, 9, 9,
    9, 7, 1, 4, 1, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 7, 7, 7, 5, 7, 9, 9, 9, 9, 9,
    9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9,
    9, 0, 10, 7, 9, 7, 9, 7, 9, 7, 9, 9, 9, 9, 9, 7, 7, 7, 7, 7, 7, 7, 8, 7, 7,
    7, 7, 7, 7, 9, 9, 9, 5, 11, 12, 9, 12, 12, 11, 11, 10, 10, 10, 9, 10, 11, 7,
    10, 10, 10, 9, 10, 10, 10, 9, 10, 10, 10, 10, 9, 10, 10, 10, 10, 9, 0, 9, 9,
    9, 9, 0, 9, 9, 9, 9, 0, 10, 9, 0, 9, 9, 0, 7, 0, 0, 9, 7, 0, 0, 0, 0, 0, 9,
    9, 0, 7, 7, 7, 7, 5, 0, 0, 0, 0, 6, 6, 0, 0, 12, 12, 12, 0, 0, 0, 0, 0, 0, 0,
    0, 7, 0, 9, 0, 0, 8, 0, 0, 0, 0, 0, 0, 0, 0, 0, 12, 12, 12, 11, 12, 12, 12,
    11, 12, 12, 12, 0, 12, 12, 12, 12, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 9, 9, 10, 12, 10, 9, 9, 0, 7, 9, 9, 9, 9, 9, 9, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 4, 10, 4 };

  static const signed char uv2[261] = { 7, 0, 0, 0, 2, 4, 8, 6, 8, 8, 3, 4, 4, 6,
    8, 2, 7, 2, 5, 7, 6, 6, 6, 7, 6, 7, 6, 6, 7, 2, 2, 8, 8, 8, 5, 10, 9, 6, 8,
    8, 6, 5, 8, 7, 2, 5, 7, 6, 9, 7, 9, 6, 10, 7, 6, 8, 7, 8, 11, 8, 8, 7, 3, 5,
    3, 7, 6, 4, 7, 6, 6, 7, 6, 5, 7, 6, 2, 4, 6, 2, 10, 6, 7, 6, 7, 4, 6, 5, 6,
    7, 10, 7, 7, 7, 4, 2, 4, 7, 9, 9, 8, 6, 7, 9, 7, 7, 7, 7, 7, 7, 7, 6, 6, 6,
    6, 6, 4, 4, 5, 4, 6, 7, 7, 7, 7, 7, 6, 6, 6, 6, 0, 5, 6, 6, 6, 0, 6, 6, 6,
    10, 0, 4, 4, 0, 11, 9, 0, 8, 0, 0, 8, 6, 0, 0, 0, 0, 0, 4, 5, 0, 10, 7, 5, 2,
    8, 0, 0, 0, 0, 6, 6, 0, 0, 9, 9, 9, 0, 0, 0, 0, 0, 0, 0, 0, 8, 0, 7, 0, 0, 8,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 9, 6, 9, 6, 6, 4, 5, 4, 4, 9, 9, 0, 9, 7, 7, 7, 0,
    0, 0, 0, 0, 0, 0, 4, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2, 10, 7, 8, 7, 6, 6, 0, 8,
    4, 5, 5, 10, 10, 10, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 7, 6, 2 };

  static const signed char uv3[261] = { 8, 0, 0, 0, 9, 3, 9, 11, 9, 9, 3, 11, 11,
    4, 7, 3, 1, 1, 11, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 7, 9, 7, 3, 7, 9, 9, 9, 9,
    9, 9, 9, 9, 9, 9, 9, 11, 9, 9, 9, 9, 9, 9, 11, 9, 9, 9, 9, 9, 9, 9, 9, 9, 11,
    11, 11, 6, 1, 2, 7, 9, 7, 9, 7, 9, 9, 9, 9, 11, 9, 9, 7, 7, 7, 9, 9, 7, 7, 8,
    7, 7, 7, 7, 9, 7, 11, 11, 11, 2, 11, 12, 12, 12, 12, 11, 11, 10, 10, 10, 9,
    10, 11, 10, 10, 10, 10, 9, 10, 10, 10, 9, 10, 10, 10, 10, 9, 10, 10, 10, 10,
    9, 0, 3, 9, 9, 11, 0, 11, 9, 5, 9, 0, 2, 1, 0, 9, 9, 0, 7, 0, 0, 9, 9, 0, 0,
    0, 0, 0, 4, 4, 0, 7, 7, 9, 9, 3, 0, 0, 0, 0, 5, 5, 0, 0, 12, 12, 12, 0, 0, 0,
    0, 0, 0, 0, 0, 7, 0, 11, 0, 0, 7, 0, 0, 0, 0, 0, 0, 0, 0, 0, 12, 12, 12, 11,
    12, 12, 12, 11, 12, 12, 12, 0, 12, 12, 12, 12, 0, 0, 0, 0, 0, 0, 0, 3, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 11, 9, 10, 12, 12, 9, 11, 0, 7, 6, 6, 6, 9, 9, 9, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1 };

  static const char cv2[22] = "Error opening camera.";
  static const signed char dv[16] = { 19, 40, 15, 27, 68, 40, 9, 47, 29, 42, 20,
    40, 115, 92, 11, 60 };

  static const char cv3[3] = { 'c', 'a', 'r' };

  static const unsigned char uv[3] = { MAX_uint8_T, MAX_uint8_T, 0U };

  static unsigned char out[150528];
  static unsigned char tmpRGB[150528];
  static unsigned char pln0[76800];
  static unsigned char pln1[76800];
  static unsigned char pln2[76800];
  static unsigned char varargin_1[50176];
  static const bool bv1[2] = { false, true };

  dim3 ab_block;
  dim3 ab_grid;
  dim3 ac_block;
  dim3 ac_grid;
  dim3 b_block;
  dim3 b_grid;
  dim3 bb_block;
  dim3 bb_grid;
  dim3 bc_block;
  dim3 bc_grid;
  dim3 block;
  dim3 c_block;
  dim3 c_grid;
  dim3 cb_block;
  dim3 cb_grid;
  dim3 cc_block;
  dim3 cc_grid;
  dim3 d_block;
  dim3 d_grid;
  dim3 db_block;
  dim3 db_grid;
  dim3 dc_block;
  dim3 dc_grid;
  dim3 e_block;
  dim3 e_grid;
  dim3 eb_block;
  dim3 eb_grid;
  dim3 ec_block;
  dim3 ec_grid;
  dim3 f_block;
  dim3 f_grid;
  dim3 fb_block;
  dim3 fb_grid;
  dim3 fc_block;
  dim3 fc_grid;
  dim3 g_block;
  dim3 g_grid;
  dim3 gb_block;
  dim3 gb_grid;
  dim3 gc_block;
  dim3 gc_grid;
  dim3 grid;
  dim3 h_block;
  dim3 h_grid;
  dim3 hb_block;
  dim3 hb_grid;
  dim3 hc_block;
  dim3 hc_grid;
  dim3 i_block;
  dim3 i_grid;
  dim3 ib_block;
  dim3 ib_grid;
  dim3 ic_block;
  dim3 ic_grid;
  dim3 j_block;
  dim3 j_grid;
  dim3 jb_block;
  dim3 jb_grid;
  dim3 jc_block;
  dim3 jc_grid;
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
  dim3 xb_block;
  dim3 xb_grid;
  dim3 y_block;
  dim3 y_grid;
  dim3 yb_block;
  dim3 yb_grid;
  emxArray_cell_wrap_7_512 labelCells;
  emxArray_cell_wrap_7_512 *gpu_labelCells;
  double b_bboxPred_data[2048];
  double bboxPred_data[2048];
  double bboxesX1Y1X2Y2_data[2048];
  double (*b_gpu_bboxPred_data)[2048];
  double (*gpu_bboxPred_data)[2048];
  double (*gpu_bboxesX1Y1X2Y2_data)[2048];
  double (*gpu_colWeights)[1344];
  double (*gpu_rowWeights)[1120];
  double (*b_gpu_colWeights)[896];
  double (*b_gpu_rowWeights)[896];
  double x1_data[512];
  double x2_data[512];
  double y1_data[512];
  double y2_data[512];
  double (*gpu_idx_data)[512];
  double (*gpu_x1_data)[512];
  double (*gpu_x2_data)[512];
  double (*gpu_y1_data)[512];
  double (*gpu_y2_data)[512];
  double (*gpu_colWeightsTotal)[224];
  double (*gpu_rowWeightsTotal)[224];
  double (*b_gpu_colWeightsTotal)[128];
  double (*b_gpu_rowWeightsTotal)[128];
  double (*gpu_anchors)[16];
  double (*gpu_dv1)[8];
  double height;
  double *gpu_height;
  float (*c_gpu_out)[49152];
  float tmpFeatureMap[3072];
  float (*gpu_boxOut)[3072];
  float (*gpu_thresholdedPrediction_data)[3072];
  float (*gpu_tmpFeatureMap)[3072];
  float b_scores_data[512];
  float scorePred_data[512];
  float scores_data[512];
  float (*b_gpu_scores_data)[512];
  float (*gpu_scorePred_data)[512];
  float (*gpu_scores_data)[512];
  int iv6_data[512];
  int (*gpu_iv6_data)[512];
  int position[4];
  int positionOut[4];
  int (*gpu_position)[4];
  int (*gpu_positionOut)[4];
  int b_b_size[2];
  int b_bboxPred_size[2];
  int b_size[2];
  int b_x_size[2];
  int bboxPred_size[2];
  int bboxesX1Y1X2Y2_size[2];
  int c_b_size[2];
  int c_bboxPred_size[2];
  int d_bboxPred_size[2];
  int inDims[2];
  int inputBbox_size[2];
  int iv[2];
  int thisCharcodes_1b_size[2];
  int thisTextU16_size[2];
  int thresholdedPrediction_size[2];
  int uv5_size[2];
  int x_size[2];
  int (*b_gpu_b_size)[2];
  int (*b_gpu_bboxPred_size)[2];
  int (*b_gpu_x_size)[2];
  int (*c_gpu_b_size)[2];
  int (*c_gpu_bboxPred_size)[2];
  int (*d_gpu_bboxPred_size)[2];
  int (*gpu_b_size)[2];
  int (*gpu_bboxPred_size)[2];
  int (*gpu_bboxesX1Y1X2Y2_size)[2];
  unsigned int (*gpu_castRed)[2];
  int (*gpu_inputBbox_size)[2];
  int (*gpu_iv)[2];
  int (*gpu_thisCharcodes_1b_size)[2];
  int (*gpu_thisTextU16_size)[2];
  int (*gpu_thresholdedPrediction_size)[2];
  int (*gpu_uv5_size)[2];
  int (*gpu_x_size)[2];
  int iv_size[1];
  int v_size[1];
  int x2_size[1];
  int y1_size[1];
  int y2_size[1];
  int (*gpu_idx_size)[1];
  int (*gpu_iv_size)[1];
  int (*gpu_scorePred_size)[1];
  int (*gpu_v_size)[1];
  int (*gpu_y1_size)[1];
  int b_i;
  int d_isInitialized;
  int k;
  int nrows;
  int penX;
  int status;
  int tbWidth;
  int w_isInitialized;
  int *gpu_i;
  int *gpu_ind;
  int *gpu_nrows;
  int *gpu_status;
  int *gpu_tbWidth;
  short (*gpu_ipColIndices)[1344];
  short (*gpu_ipRowIndices)[1120];
  short (*b_gpu_ipColIndices)[896];
  short (*b_gpu_ipRowIndices)[896];
  short (*gpu_aux2)[640];
  short idx_data[512];
  short iv_data[512];
  short (*b_gpu_idx_data)[512];
  short (*gpu_iv_data)[512];
  short (*gpu_aux1)[480];
  short (*b_gpu_aux1)[448];
  short (*b_gpu_aux2)[448];
  unsigned short (*gpu_uv1)[256];
  short dv2[2];
  short (*gpu_dv2)[2];
  unsigned char (*gpu_img)[230400];
  unsigned char (*gpu_partialResize)[161280];
  unsigned char (*b_gpu_out)[150528];
  unsigned char (*gpu_tmpRGB)[150528];
  unsigned char (*b_gpu_partialResize)[86016];
  unsigned char (*gpu_pln0)[76800];
  unsigned char (*gpu_pln1)[76800];
  unsigned char (*gpu_pln2)[76800];
  unsigned char varargin_2[50176];
  unsigned char varargin_3[50176];
  unsigned char (*gpu_varargin_1)[50176];
  unsigned char (*gpu_varargin_2)[50176];
  unsigned char (*gpu_varargin_3)[50176];
  unsigned char (*gpu_out)[49152];
  unsigned char (*gpu_uv5)[10664];
  unsigned char (*gpu_uv5_data)[10664];
  signed char (*gpu_iv2)[261];
  unsigned char pixCount[224];
  unsigned char (*gpu_pixCount)[224];
  unsigned char thisGlyphBitmap_data[144];
  unsigned char (*b_gpu_uv5_data)[144];
  unsigned char (*gpu_thisGlyphBitmap_data)[144];
  char cv1[22];
  char (*gpu_cv1)[22];
  char (*gpu_cv2)[22];
  signed char (*gpu_dv)[16];
  unsigned char color[3];
  signed char thisTextU16_data[3];
  unsigned char (*gpu_color)[3];
  char (*gpu_cv3)[3];
  signed char (*gpu_thisCharcodes_1b_data)[3];
  signed char (*gpu_thisTextU16_data)[3];
  unsigned char (*gpu_uv)[3];
  char (*gpu_v_data)[3];
  signed char (*gpu_x_data)[3];
  signed char num[2];
  unsigned char outVal[2];
  signed char (*gpu_num)[2];
  unsigned char (*gpu_outVal)[2];
  bool b_data[512];
  bool index_data[512];
  bool (*gpu_b_data)[512];
  bool (*gpu_index_data)[512];
  bool (*b_gpu_x_data)[3];
  bool b_bboxPred_data_dirtyOnGpu;
  bool b_data_dirtyOnCpu;
  bool b_scores_data_dirtyOnCpu;
  bool b_scores_data_dirtyOnGpu;
  bool bboxPred_data_dirtyOnCpu;
  bool bboxPred_data_dirtyOnGpu;
  bool bboxesX1Y1X2Y2_data_dirtyOnGpu;
  bool cv1_dirtyOnCpu;
  bool cv2_dirtyOnCpu;
  bool cv3_dirtyOnCpu;
  bool dv1_dirtyOnCpu;
  bool dv_dirtyOnCpu;
  bool height_dirtyOnCpu;
  bool idx_data_dirtyOnGpu;
  bool index_data_dirtyOnGpu;
  bool iv2_dirtyOnCpu;
  bool iv6_data_dirtyOnCpu;
  bool iv_data_dirtyOnCpu;
  bool labelCells_dirtyOnCpu;
  bool labelCells_dirtyOnGpu;
  bool out_dirtyOnCpu;
  bool pixCount_dirtyOnCpu;
  bool scorePred_data_dirtyOnCpu;
  bool scorePred_data_dirtyOnGpu;
  bool scores_data_dirtyOnCpu;
  bool scores_data_dirtyOnGpu;
  bool thisGlyphBitmap_data_dirtyOnGpu;
  bool thisTextU16_data_dirtyOnGpu;
  bool tmpRGB_dirtyOnCpu;
  bool uv1_dirtyOnCpu;
  bool uv5_dirtyOnCpu;
  bool uv_dirtyOnCpu;
  bool varargin_1_dirtyOnCpu;
  bool varargin_2_dirtyOnCpu;
  bool varargin_3_dirtyOnCpu;
  bool w_Initialized;
  bool x1_data_dirtyOnGpu;
  bool x2_data_dirtyOnGpu;
  bool y1_data_dirtyOnGpu;
  bool y2_data_dirtyOnGpu;
  if (!isInitialized_detectFunction) {
    detectFunction_initialize();
  }

  cudaMalloc(&gpu_varargin_3, 50176UL);
  cudaMalloc(&gpu_varargin_1, 50176UL);
  cudaMalloc(&gpu_varargin_2, 50176UL);
  cudaMalloc(&gpu_thisGlyphBitmap_data, 144UL);
  cudaMalloc(&b_gpu_uv5_data, 144UL);
  cudaMalloc(&gpu_uv5_size, 8UL);
  cudaMalloc(&gpu_num, 2UL);
  cudaMalloc(&gpu_uv5_data, 10664UL);
  cudaMalloc(&gpu_uv5, 10664UL);
  cudaMalloc(&b_gpu_x_data, 3UL);
  cudaMalloc(&b_gpu_x_size, 8UL);
  cudaMalloc(&gpu_x_data, 3UL);
  cudaMalloc(&gpu_uv1, 512UL);
  cudaMalloc(&gpu_iv2, 261UL);
  cudaMalloc(&gpu_x_size, 8UL);
  cudaMalloc(&gpu_thisCharcodes_1b_data, 3UL);
  cudaMalloc(&gpu_thisCharcodes_1b_size, 8UL);
  cudaMalloc(&gpu_thisTextU16_data, 3UL);
  cudaMalloc(&gpu_thisTextU16_size, 8UL);
  cudaMalloc(&gpu_pixCount, 224UL);
  cudaMalloc(&gpu_positionOut, 16UL);
  cudaMalloc(&gpu_color, 3UL);
  cudaMalloc(&gpu_uv, 3UL);
  cudaMalloc(&gpu_tmpRGB, 150528UL);
  cudaMalloc(&gpu_position, 16UL);
  cudaMalloc(&gpu_i, 4UL);
  cudaMalloc(&gpu_v_data, 3UL);
  cudaMalloc(&gpu_cv3, 3UL);
  cudaMalloc(&gpu_v_size, 4UL);
  cudaMalloc(&gpu_ind, 4UL);
  cudaMalloc(&b_gpu_scores_data, 2048UL);
  cudaMalloc(&d_gpu_bboxPred_size, 8UL);
  cudaMalloc(&gpu_iv6_data, 2048UL);
  cudaMalloc(&gpu_index_data, 512UL);
  cudaMalloc(&gpu_iv, 8UL);
  cudaMalloc(&gpu_height, 8UL);
  cudaMalloc(&gpu_inputBbox_size, 8UL);
  cudaMalloc(&gpu_idx_data, 4096UL);
  cudaMalloc(&gpu_dv2, 4UL);
  cudaMalloc(&gpu_idx_size, 4UL);
  cudaMalloc(&c_gpu_b_size, 8UL);
  cudaMalloc(&b_gpu_b_size, 8UL);
  cudaMalloc(&c_gpu_bboxPred_size, 8UL);
  cudaMalloc(&gpu_b_size, 8UL);
  cudaMalloc(&b_gpu_idx_data, 1024UL);
  cudaMalloc(&gpu_scorePred_data, 2048UL);
  cudaMalloc(&gpu_scores_data, 2048UL);
  cudaMalloc(&b_gpu_bboxPred_data, 16384UL);
  cudaMalloc(&gpu_scorePred_size, 4UL);
  cudaMalloc(&gpu_bboxPred_data, 16384UL);
  cudaMalloc(&gpu_bboxPred_size, 8UL);
  cudaMalloc(&gpu_tbWidth, 4UL);
  cudaMalloc(&gpu_y2_data, 4096UL);
  cudaMalloc(&gpu_x2_data, 4096UL);
  cudaMalloc(&gpu_y1_data, 4096UL);
  cudaMalloc(&gpu_y1_size, 4UL);
  cudaMalloc(&gpu_x1_data, 4096UL);
  cudaMalloc(&gpu_bboxesX1Y1X2Y2_data, 16384UL);
  cudaMalloc(&gpu_bboxesX1Y1X2Y2_size, 8UL);
  cudaMalloc(&gpu_labelCells, 6148UL);
  cudaMalloc(&b_gpu_bboxPred_size, 8UL);
  cudaMalloc(&gpu_thresholdedPrediction_data, 12288UL);
  cudaMalloc(&gpu_thresholdedPrediction_size, 8UL);
  cudaMalloc(&gpu_nrows, 4UL);
  cudaMalloc(&gpu_iv_data, 1024UL);
  cudaMalloc(&gpu_iv_size, 4UL);
  cudaMalloc(&gpu_b_data, 512UL);
  cudaMalloc(&gpu_boxOut, 12288UL);
  cudaMalloc(&gpu_tmpFeatureMap, 12288UL);
  cudaMalloc(&gpu_dv1, 64UL);
  cudaMalloc(&gpu_anchors, 128UL);
  cudaMalloc(&gpu_dv, 16UL);
  cudaMalloc(&c_gpu_out, 196608UL);
  cudaMalloc(&gpu_outVal, 2UL);
  cudaMalloc(&gpu_castRed, 8UL);
  cudaMalloc(&gpu_out, 49152UL);
  cudaMalloc(&b_gpu_partialResize, 86016UL);
  cudaMalloc(&b_gpu_colWeightsTotal, 1024UL);
  cudaMalloc(&b_gpu_rowWeightsTotal, 1024UL);
  cudaMalloc(&b_gpu_colWeights, 7168UL);
  cudaMalloc(&b_gpu_ipColIndices, 1792UL);
  cudaMalloc(&b_gpu_rowWeights, 7168UL);
  cudaMalloc(&b_gpu_ipRowIndices, 1792UL);
  cudaMalloc(&b_gpu_aux1, 896UL);
  cudaMalloc(&b_gpu_aux2, 896UL);
  cudaMalloc(&b_gpu_out, 150528UL);
  cudaMalloc(&gpu_partialResize, 161280UL);
  cudaMalloc(&gpu_colWeightsTotal, 1792UL);
  cudaMalloc(&gpu_rowWeightsTotal, 1792UL);
  cudaMalloc(&gpu_colWeights, 10752UL);
  cudaMalloc(&gpu_ipColIndices, 2688UL);
  cudaMalloc(&gpu_rowWeights, 8960UL);
  cudaMalloc(&gpu_ipRowIndices, 2240UL);
  cudaMalloc(&gpu_aux2, 1280UL);
  cudaMalloc(&gpu_aux1, 960UL);
  cudaMalloc(&gpu_img, 230400UL);
  cudaMalloc(&gpu_pln2, 76800UL);
  cudaMalloc(&gpu_pln0, 76800UL);
  cudaMalloc(&gpu_pln1, 76800UL);
  cudaMalloc(&gpu_cv1, 22UL);
  cudaMalloc(&gpu_cv2, 22UL);
  cudaMalloc(&gpu_status, 4UL);
  varargin_3_dirtyOnCpu = false;
  varargin_1_dirtyOnCpu = false;
  varargin_2_dirtyOnCpu = false;
  height_dirtyOnCpu = false;
  pixCount_dirtyOnCpu = false;
  tmpRGB_dirtyOnCpu = false;
  b_scores_data_dirtyOnCpu = false;
  iv6_data_dirtyOnCpu = false;
  scorePred_data_dirtyOnCpu = false;
  bboxPred_data_dirtyOnCpu = false;
  labelCells_dirtyOnCpu = false;
  iv_data_dirtyOnCpu = false;
  b_data_dirtyOnCpu = false;
  out_dirtyOnCpu = false;
  cv1_dirtyOnCpu = false;
  scores_data_dirtyOnCpu = false;
  thisGlyphBitmap_data_dirtyOnGpu = false;
  thisTextU16_data_dirtyOnGpu = false;
  b_scores_data_dirtyOnGpu = false;
  index_data_dirtyOnGpu = false;
  idx_data_dirtyOnGpu = false;
  scorePred_data_dirtyOnGpu = false;
  b_bboxPred_data_dirtyOnGpu = false;
  bboxPred_data_dirtyOnGpu = false;
  y2_data_dirtyOnGpu = false;
  x2_data_dirtyOnGpu = false;
  y1_data_dirtyOnGpu = false;
  x1_data_dirtyOnGpu = false;
  bboxesX1Y1X2Y2_data_dirtyOnGpu = false;
  labelCells_dirtyOnGpu = false;
  scores_data_dirtyOnGpu = false;
  uv5_dirtyOnCpu = true;
  uv1_dirtyOnCpu = true;
  iv2_dirtyOnCpu = true;
  uv_dirtyOnCpu = true;
  cv3_dirtyOnCpu = true;
  dv1_dirtyOnCpu = true;
  dv_dirtyOnCpu = true;
  cv2_dirtyOnCpu = true;

  //  Signature
  //  inputs    inputImgSize:    the input size the network is specified (input layer) 
  //            imageDispay :    if the captured picture should be dispayed or not  
  // function detect-function(inputImgSize,imageDisplay)
  //     %% init       -> everything set up
  //  Function will be called several times. Just the first time the
  //  variable will be initialized mynet. Comparable to static in C or C++
  //  create a jetson object to access the hardware
  //  access data req.
  // hwobj = jetson('192.168.1.6', 'jetson', '1111');
  //  https://de.mathworks.com/help/supportpkg/nvidia/ref/webcam.webcam.html?searchHighlight=webcam&s_tid=srchtitle 
  w_Initialized = false;
  w_isInitialized = 0;

  //  just for now results will be displayes
  d_isInitialized = 0;

  //  load the trained network
  if (!mynet_not_empty) {
    //  todo: yet to specify
    coder::DeepLearningNetwork_setup(&gobj_0);
    mynet.Network = &gobj_0;
    mynet_not_empty = true;
  }

  //     %% doom loop  -> there is no escape
  //  must fit in int32 variable
  thresholdedPrediction_size[1] = 6;
  for (int i = 0; i < 100000; i++) {
    float ex;
    int count;
    int nrowx;
    int startC;
    unsigned char resolutionStatus;
    bool b_bboxPred_size_dirtyOnCpu;
    bool b_data_dirtyOnGpu;
    bool bboxPred_size_dirtyOnCpu;
    bool guard1 = false;
    bool isInitialise;
    bool thresholdedPrediction_size_dirtyOnCpu;
    bool validLaunchParams;

    //       %% main code  -> for detection
    if (w_isInitialized != 1) {
      w_isInitialized = 1;
      getCameraList();
      resolutionStatus = validateResolution(0, 320, 240);
      if (resolutionStatus != 0) {
        status = EXT_webcamInit(1, 0, 0, 0, 0, 0, 320U, 240U, 2U, 2U, 1U,
          0.033333333333333333);
        if (status == 0) {
          w_Initialized = true;
        } else {
          if (cv2_dirtyOnCpu) {
            cudaMemcpy(gpu_cv2, (void *)&cv2[0], 22UL, cudaMemcpyHostToDevice);
            cv2_dirtyOnCpu = false;
          }

          if (cv1_dirtyOnCpu) {
            cudaMemcpy(gpu_cv1, &cv1[0], 22UL, cudaMemcpyHostToDevice);
          }

          detectFunction_kernel1<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
            (*gpu_cv2, *gpu_cv1);
          cudaMemcpy(&cv1[0], gpu_cv1, 22UL, cudaMemcpyDeviceToHost);
          printf(&cv1[0]);
          cv1_dirtyOnCpu = true;
        }
      }
    }

    if (w_Initialized) {
      EXT_webcamCapture(1, 0, &pln0[0], &pln1[0], &pln2[0]);
      cudaMemcpy(gpu_pln1, &pln1[0], 76800UL, cudaMemcpyHostToDevice);
      cudaMemcpy(gpu_pln0, &pln0[0], 76800UL, cudaMemcpyHostToDevice);
      detectFunction_kernel2<<<dim3(150U, 1U, 1U), dim3(512U, 1U, 1U)>>>
        (*gpu_pln1, *gpu_pln0, *gpu_img);
      cudaMemcpy(gpu_pln2, &pln2[0], 76800UL, cudaMemcpyHostToDevice);
      detectFunction_kernel3<<<dim3(150U, 1U, 1U), dim3(512U, 1U, 1U)>>>
        (*gpu_pln2, *gpu_img);
    }

    detectFunction_kernel4<<<dim3(1U, 1U, 1U), dim3(480U, 1U, 1U)>>>(*gpu_aux1);
    detectFunction_kernel5<<<dim3(2U, 1U, 1U), dim3(512U, 1U, 1U)>>>(*gpu_aux2);
    detectFunction_kernel6<<<dim3(3U, 1U, 1U), dim3(512U, 1U, 1U)>>>(*gpu_aux1, *
      gpu_ipRowIndices, *gpu_rowWeights);
    detectFunction_kernel7<<<dim3(3U, 1U, 1U), dim3(512U, 1U, 1U)>>>(*gpu_aux2, *
      gpu_ipColIndices, *gpu_colWeights);
    detectFunction_kernel8<<<dim3(1U, 1U, 1U), dim3(224U, 1U, 1U)>>>
      (*gpu_rowWeights, *gpu_rowWeightsTotal);
    for (k = 0; k < 4; k++) {
      status = (k + 1) * 224;
      cudaMemcpy(gpu_status, &status, 4UL, cudaMemcpyHostToDevice);
      detectFunction_kernel9<<<dim3(1U, 1U, 1U), dim3(224U, 1U, 1U)>>>
        (*gpu_rowWeights, gpu_status, *gpu_rowWeightsTotal);
    }

    detectFunction_kernel10<<<dim3(1U, 1U, 1U), dim3(224U, 1U, 1U)>>>
      (*gpu_colWeights, *gpu_colWeightsTotal);
    for (k = 0; k < 5; k++) {
      status = (k + 1) * 224;
      cudaMemcpy(gpu_status, &status, 4UL, cudaMemcpyHostToDevice);
      detectFunction_kernel11<<<dim3(1U, 1U, 1U), dim3(224U, 1U, 1U)>>>
        (*gpu_colWeights, gpu_status, *gpu_colWeightsTotal);
    }

    detectFunction_kernel12<<<dim3(315U, 1U, 1U), dim3(512U, 1U, 1U)>>>
      (*gpu_colWeightsTotal, *gpu_colWeights, *gpu_img, *gpu_ipColIndices,
       *gpu_partialResize);
    detectFunction_kernel13<<<dim3(294U, 1U, 1U), dim3(512U, 1U, 1U)>>>
      (*gpu_rowWeightsTotal, *gpu_rowWeights, *gpu_partialResize,
       *gpu_ipRowIndices, *b_gpu_out);
    detectFunction_kernel14<<<dim3(1U, 1U, 1U), dim3(448U, 1U, 1U)>>>
      (*b_gpu_aux2, *b_gpu_aux1);
    detectFunction_kernel15<<<dim3(2U, 1U, 1U), dim3(512U, 1U, 1U)>>>
      (*b_gpu_aux1, *b_gpu_ipRowIndices, *b_gpu_rowWeights);
    detectFunction_kernel16<<<dim3(2U, 1U, 1U), dim3(512U, 1U, 1U)>>>
      (*b_gpu_aux2, *b_gpu_ipColIndices, *b_gpu_colWeights);
    detectFunction_kernel17<<<dim3(1U, 1U, 1U), dim3(128U, 1U, 1U)>>>
      (*b_gpu_rowWeights, *b_gpu_rowWeightsTotal);
    for (k = 0; k < 6; k++) {
      status = (k + 1) << 7;
      cudaMemcpy(gpu_status, &status, 4UL, cudaMemcpyHostToDevice);
      detectFunction_kernel18<<<dim3(1U, 1U, 1U), dim3(128U, 1U, 1U)>>>
        (*b_gpu_rowWeights, gpu_status, *b_gpu_rowWeightsTotal);
    }

    detectFunction_kernel19<<<dim3(1U, 1U, 1U), dim3(128U, 1U, 1U)>>>
      (*b_gpu_colWeights, *b_gpu_colWeightsTotal);
    for (k = 0; k < 6; k++) {
      status = (k + 1) << 7;
      cudaMemcpy(gpu_status, &status, 4UL, cudaMemcpyHostToDevice);
      detectFunction_kernel20<<<dim3(1U, 1U, 1U), dim3(128U, 1U, 1U)>>>
        (*b_gpu_colWeights, gpu_status, *b_gpu_colWeightsTotal);
    }

    detectFunction_kernel21<<<dim3(168U, 1U, 1U), dim3(512U, 1U, 1U)>>>
      (*b_gpu_rowWeightsTotal, *b_gpu_rowWeights, *b_gpu_out,
       *b_gpu_ipRowIndices, *b_gpu_partialResize);
    detectFunction_kernel22<<<dim3(96U, 1U, 1U), dim3(512U, 1U, 1U)>>>
      (*b_gpu_colWeightsTotal, *b_gpu_colWeights, *b_gpu_partialResize,
       *b_gpu_ipColIndices, *gpu_out);
    detectFunction_kernel23<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>(*gpu_out,
      *gpu_castRed);
    coder_reduce0<<<dim3(96U, 1U, 1U), dim3(512U, 1U, 1U)>>>(*gpu_out,
      *gpu_castRed);
    detectFunction_kernel24<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>(*gpu_outVal,
      *gpu_castRed);
    cudaMemcpy(&outVal[0], gpu_outVal, 2UL, cudaMemcpyDeviceToHost);
    status = outVal[1] - outVal[0];
    cudaMemcpy(gpu_status, &status, 4UL, cudaMemcpyHostToDevice);
    detectFunction_kernel25<<<dim3(96U, 1U, 1U), dim3(512U, 1U, 1U)>>>
      (gpu_status, static_cast<short>(outVal[0]), *gpu_out, *c_gpu_out);
    cudaMemcpy(&b_out[0], c_gpu_out, 196608UL, cudaMemcpyDeviceToHost);
    coder::DeepLearningNetwork_activations(mynet.Network, b_out, tmpFeatureMap);
    if (dv_dirtyOnCpu) {
      cudaMemcpy(gpu_dv, (void *)&dv[0], 16UL, cudaMemcpyHostToDevice);
      dv_dirtyOnCpu = false;
    }

    detectFunction_kernel26<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>(*gpu_dv,
      *gpu_anchors);
    if (dv1_dirtyOnCpu) {
      cudaMemcpy(gpu_dv1, (void *)&dv1[0], 64UL, cudaMemcpyHostToDevice);
      dv1_dirtyOnCpu = false;
    }

    detectFunction_kernel27<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>(*gpu_dv1,
      *gpu_anchors);
    cudaMemcpy(gpu_tmpFeatureMap, &tmpFeatureMap[0], 12288UL,
               cudaMemcpyHostToDevice);
    detectFunction_kernel28<<<dim3(1U, 1U, 1U), dim3(512U, 1U, 1U)>>>
      (*gpu_anchors, *gpu_tmpFeatureMap, *gpu_boxOut);
    if (b_data_dirtyOnCpu) {
      cudaMemcpy(gpu_b_data, &b_data[0], 512UL, cudaMemcpyHostToDevice);
      b_data_dirtyOnCpu = false;
    }

    detectFunction_kernel29<<<dim3(1U, 1U, 1U), dim3(512U, 1U, 1U)>>>
      (*gpu_boxOut, *gpu_b_data);
    b_data_dirtyOnGpu = true;
    status = 0;
    for (b_i = 0; b_i < 512; b_i++) {
      if (b_data_dirtyOnGpu) {
        cudaMemcpy(&b_data[0], gpu_b_data, 512UL, cudaMemcpyDeviceToHost);
        b_data_dirtyOnGpu = false;
      }

      if (b_data[b_i]) {
        status++;
      }
    }

    iv_size[0] = status;
    nrows = 0;
    for (b_i = 0; b_i < 512; b_i++) {
      if (b_data_dirtyOnGpu) {
        cudaMemcpy(&b_data[0], gpu_b_data, 512UL, cudaMemcpyDeviceToHost);
        b_data_dirtyOnGpu = false;
      }

      if (b_data[b_i]) {
        iv_data[nrows] = static_cast<short>(b_i + 1);
        iv_data_dirtyOnCpu = true;
        nrows++;
      }
    }

    thresholdedPrediction_size[0] = status;
    thresholdedPrediction_size_dirtyOnCpu = true;
    validLaunchParams = mwGetLaunchParameters(static_cast<double>(((iv_size[0] -
      1) + 1L) * 6L), &grid, &block, 1024U, 65535U);
    if (validLaunchParams) {
      if (iv_data_dirtyOnCpu) {
        cudaMemcpy(gpu_iv_data, &iv_data[0], 1024UL, cudaMemcpyHostToDevice);
        iv_data_dirtyOnCpu = false;
      }

      cudaMemcpy(gpu_thresholdedPrediction_size, &thresholdedPrediction_size[0],
                 8UL, cudaMemcpyHostToDevice);
      thresholdedPrediction_size_dirtyOnCpu = false;
      cudaMemcpy(gpu_iv_size, &iv_size[0], 4UL, cudaMemcpyHostToDevice);
      detectFunction_kernel30<<<grid, block>>>(*gpu_boxOut, *gpu_iv_data,
        *gpu_thresholdedPrediction_size, *gpu_iv_size,
        *gpu_thresholdedPrediction_data);
    }

    if (iv_size[0] != 0) {
      short c_i;
      k = iv_size[0] - 1;
      bboxesX1Y1X2Y2_size[0] = iv_size[0];
      bboxesX1Y1X2Y2_size[1] = 4;
      isInitialise = true;
      validLaunchParams = mwGetLaunchParameters(static_cast<double>((k + 1L) *
        4L), &b_grid, &b_block, 1024U, 65535U);
      if (validLaunchParams) {
        if (thresholdedPrediction_size_dirtyOnCpu) {
          cudaMemcpy(gpu_thresholdedPrediction_size,
                     &thresholdedPrediction_size[0], 8UL, cudaMemcpyHostToDevice);
          thresholdedPrediction_size_dirtyOnCpu = false;
        }

        cudaMemcpy(gpu_bboxesX1Y1X2Y2_size, &bboxesX1Y1X2Y2_size[0], 8UL,
                   cudaMemcpyHostToDevice);
        isInitialise = false;
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
        detectFunction_kernel33<<<c_grid, c_block>>>(*gpu_bboxesX1Y1X2Y2_data, k,
          *gpu_x1_data);
        x1_data_dirtyOnGpu = true;
      }

      k = bboxesX1Y1X2Y2_size[0] - 1;
      y1_size[0] = bboxesX1Y1X2Y2_size[0];
      validLaunchParams = mwGetLaunchParameters(static_cast<double>(k + 1L),
        &d_grid, &d_block, 1024U, 65535U);
      if (validLaunchParams) {
        if (isInitialise) {
          cudaMemcpy(gpu_bboxesX1Y1X2Y2_size, &bboxesX1Y1X2Y2_size[0], 8UL,
                     cudaMemcpyHostToDevice);
          isInitialise = false;
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
        if (isInitialise) {
          cudaMemcpy(gpu_bboxesX1Y1X2Y2_size, &bboxesX1Y1X2Y2_size[0], 8UL,
                     cudaMemcpyHostToDevice);
          isInitialise = false;
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
        if (isInitialise) {
          cudaMemcpy(gpu_bboxesX1Y1X2Y2_size, &bboxesX1Y1X2Y2_size[0], 8UL,
                     cudaMemcpyHostToDevice);
        }

        detectFunction_kernel36<<<f_grid, f_block>>>(*gpu_bboxesX1Y1X2Y2_data,
          *gpu_bboxesX1Y1X2Y2_size, k, *gpu_y2_data);
        y2_data_dirtyOnGpu = true;
      }

      tbWidth = bboxesX1Y1X2Y2_size[0];
      validLaunchParams = mwGetLaunchParameters(static_cast<double>((tbWidth - 1)
        + 1L), &g_grid, &g_block, 1024U, 65535U);
      if (validLaunchParams) {
        cudaMemcpy(gpu_tbWidth, &tbWidth, 4UL, cudaMemcpyHostToDevice);
        detectFunction_kernel37<<<g_grid, g_block>>>(gpu_tbWidth, *gpu_x1_data);
        x1_data_dirtyOnGpu = true;
      }

      tbWidth = bboxesX1Y1X2Y2_size[0];
      validLaunchParams = mwGetLaunchParameters(static_cast<double>((tbWidth - 1)
        + 1L), &h_grid, &h_block, 1024U, 65535U);
      if (validLaunchParams) {
        cudaMemcpy(gpu_tbWidth, &tbWidth, 4UL, cudaMemcpyHostToDevice);
        detectFunction_kernel38<<<h_grid, h_block>>>(gpu_tbWidth, *gpu_y1_data);
        y1_data_dirtyOnGpu = true;
      }

      tbWidth = bboxesX1Y1X2Y2_size[0];
      validLaunchParams = mwGetLaunchParameters(static_cast<double>((tbWidth - 1)
        + 1L), &i_grid, &i_block, 1024U, 65535U);
      if (validLaunchParams) {
        cudaMemcpy(gpu_tbWidth, &tbWidth, 4UL, cudaMemcpyHostToDevice);
        detectFunction_kernel39<<<i_grid, i_block>>>(gpu_tbWidth, *gpu_x2_data);
        x2_data_dirtyOnGpu = true;
      }

      tbWidth = bboxesX1Y1X2Y2_size[0];
      validLaunchParams = mwGetLaunchParameters(static_cast<double>((tbWidth - 1)
        + 1L), &j_grid, &j_block, 1024U, 65535U);
      if (validLaunchParams) {
        cudaMemcpy(gpu_tbWidth, &tbWidth, 4UL, cudaMemcpyHostToDevice);
        detectFunction_kernel40<<<j_grid, j_block>>>(gpu_tbWidth, *gpu_y2_data);
        y2_data_dirtyOnGpu = true;
      }

      bboxesX1Y1X2Y2_size[0] = iv_size[0];
      bboxesX1Y1X2Y2_size[1] = 4;
      isInitialise = true;
      status = iv_size[0] - 1;
      validLaunchParams = mwGetLaunchParameters(static_cast<double>(status + 1L),
        &k_grid, &k_block, 1024U, 65535U);
      if (validLaunchParams) {
        cudaMemcpy(gpu_status, &status, 4UL, cudaMemcpyHostToDevice);
        detectFunction_kernel41<<<k_grid, k_block>>>(*gpu_x1_data, gpu_status,
          *gpu_bboxesX1Y1X2Y2_data);
        bboxesX1Y1X2Y2_data_dirtyOnGpu = true;
      }

      status = y1_size[0] - 1;
      validLaunchParams = mwGetLaunchParameters(static_cast<double>(status + 1L),
        &l_grid, &l_block, 1024U, 65535U);
      if (validLaunchParams) {
        cudaMemcpy(gpu_status, &status, 4UL, cudaMemcpyHostToDevice);
        cudaMemcpy(gpu_bboxesX1Y1X2Y2_size, &bboxesX1Y1X2Y2_size[0], 8UL,
                   cudaMemcpyHostToDevice);
        isInitialise = false;
        detectFunction_kernel42<<<l_grid, l_block>>>(*gpu_y1_data,
          *gpu_bboxesX1Y1X2Y2_size, gpu_status, *gpu_bboxesX1Y1X2Y2_data);
        bboxesX1Y1X2Y2_data_dirtyOnGpu = true;
      }

      status = x2_size[0] - 1;
      validLaunchParams = mwGetLaunchParameters(static_cast<double>(status + 1L),
        &m_grid, &m_block, 1024U, 65535U);
      if (validLaunchParams) {
        cudaMemcpy(gpu_status, &status, 4UL, cudaMemcpyHostToDevice);
        if (isInitialise) {
          cudaMemcpy(gpu_bboxesX1Y1X2Y2_size, &bboxesX1Y1X2Y2_size[0], 8UL,
                     cudaMemcpyHostToDevice);
          isInitialise = false;
        }

        detectFunction_kernel43<<<m_grid, m_block>>>(*gpu_x2_data,
          *gpu_bboxesX1Y1X2Y2_size, gpu_status, *gpu_bboxesX1Y1X2Y2_data);
        bboxesX1Y1X2Y2_data_dirtyOnGpu = true;
      }

      status = y2_size[0] - 1;
      validLaunchParams = mwGetLaunchParameters(static_cast<double>(status + 1L),
        &n_grid, &n_block, 1024U, 65535U);
      if (validLaunchParams) {
        cudaMemcpy(gpu_status, &status, 4UL, cudaMemcpyHostToDevice);
        if (isInitialise) {
          cudaMemcpy(gpu_bboxesX1Y1X2Y2_size, &bboxesX1Y1X2Y2_size[0], 8UL,
                     cudaMemcpyHostToDevice);
          isInitialise = false;
        }

        detectFunction_kernel44<<<n_grid, n_block>>>(*gpu_y2_data,
          *gpu_bboxesX1Y1X2Y2_size, gpu_status, *gpu_bboxesX1Y1X2Y2_data);
        bboxesX1Y1X2Y2_data_dirtyOnGpu = true;
      }

      k = bboxesX1Y1X2Y2_size[0];
      nrows = bboxesX1Y1X2Y2_size[0];
      tbWidth = bboxesX1Y1X2Y2_size[0];
      b_bboxPred_size[0] = bboxesX1Y1X2Y2_size[0];
      b_bboxPred_size[1] = 4;
      bboxPred_size_dirtyOnCpu = true;
      status = bboxesX1Y1X2Y2_size[0] - 1;
      validLaunchParams = mwGetLaunchParameters(static_cast<double>(status + 1L),
        &o_grid, &o_block, 1024U, 65535U);
      if (validLaunchParams) {
        cudaMemcpy(gpu_status, &status, 4UL, cudaMemcpyHostToDevice);
        detectFunction_kernel45<<<o_grid, o_block>>>(*gpu_bboxesX1Y1X2Y2_data,
          gpu_status, *gpu_bboxPred_data);
        bboxPred_data_dirtyOnGpu = true;
      }

      status = k - 1;
      validLaunchParams = mwGetLaunchParameters(static_cast<double>(status + 1L),
        &p_grid, &p_block, 1024U, 65535U);
      if (validLaunchParams) {
        cudaMemcpy(gpu_status, &status, 4UL, cudaMemcpyHostToDevice);
        if (isInitialise) {
          cudaMemcpy(gpu_bboxesX1Y1X2Y2_size, &bboxesX1Y1X2Y2_size[0], 8UL,
                     cudaMemcpyHostToDevice);
          isInitialise = false;
        }

        cudaMemcpy(gpu_bboxPred_size, &b_bboxPred_size[0], 8UL,
                   cudaMemcpyHostToDevice);
        bboxPred_size_dirtyOnCpu = false;
        detectFunction_kernel46<<<p_grid, p_block>>>(*gpu_bboxesX1Y1X2Y2_data,
          *gpu_bboxesX1Y1X2Y2_size, *gpu_bboxPred_size, gpu_status,
          *gpu_bboxPred_data);
        bboxPred_data_dirtyOnGpu = true;
      }

      status = nrows - 1;
      validLaunchParams = mwGetLaunchParameters(static_cast<double>(status + 1L),
        &q_grid, &q_block, 1024U, 65535U);
      if (validLaunchParams) {
        cudaMemcpy(gpu_status, &status, 4UL, cudaMemcpyHostToDevice);
        if (isInitialise) {
          cudaMemcpy(gpu_bboxesX1Y1X2Y2_size, &bboxesX1Y1X2Y2_size[0], 8UL,
                     cudaMemcpyHostToDevice);
          isInitialise = false;
        }

        if (bboxPred_size_dirtyOnCpu) {
          cudaMemcpy(gpu_bboxPred_size, &b_bboxPred_size[0], 8UL,
                     cudaMemcpyHostToDevice);
          bboxPred_size_dirtyOnCpu = false;
        }

        detectFunction_kernel47<<<q_grid, q_block>>>(*gpu_bboxesX1Y1X2Y2_data,
          *gpu_bboxesX1Y1X2Y2_size, *gpu_bboxPred_size, gpu_status,
          *gpu_bboxPred_data);
        bboxPred_data_dirtyOnGpu = true;
      }

      status = tbWidth - 1;
      validLaunchParams = mwGetLaunchParameters(static_cast<double>(status + 1L),
        &r_grid, &r_block, 1024U, 65535U);
      if (validLaunchParams) {
        cudaMemcpy(gpu_status, &status, 4UL, cudaMemcpyHostToDevice);
        if (isInitialise) {
          cudaMemcpy(gpu_bboxesX1Y1X2Y2_size, &bboxesX1Y1X2Y2_size[0], 8UL,
                     cudaMemcpyHostToDevice);
        }

        if (bboxPred_size_dirtyOnCpu) {
          cudaMemcpy(gpu_bboxPred_size, &b_bboxPred_size[0], 8UL,
                     cudaMemcpyHostToDevice);
          bboxPred_size_dirtyOnCpu = false;
        }

        detectFunction_kernel48<<<r_grid, r_block>>>(*gpu_bboxesX1Y1X2Y2_data,
          *gpu_bboxesX1Y1X2Y2_size, *gpu_bboxPred_size, gpu_status,
          *gpu_bboxPred_data);
        bboxPred_data_dirtyOnGpu = true;
      }

      status = b_bboxPred_size[0] << 2;
      validLaunchParams = mwGetLaunchParameters(static_cast<double>((status - 1)
        + 1L), &s_grid, &s_block, 1024U, 65535U);
      if (validLaunchParams) {
        cudaMemcpy(gpu_status, &status, 4UL, cudaMemcpyHostToDevice);
        detectFunction_kernel49<<<s_grid, s_block>>>(gpu_status,
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

      validLaunchParams = mwGetLaunchParameters(static_cast<double>((iv_size[0]
        - 1) + 1L), &u_grid, &u_block, 1024U, 65535U);
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

      validLaunchParams = mwGetLaunchParameters(static_cast<double>((iv_size[0]
        - 1) + 1L), &w_grid, &w_block, 1024U, 65535U);
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
      x2_size[0] = b_bboxPred_size[0];
      iv_size[0] = b_bboxPred_size[0];
      status = b_bboxPred_size[0];
      for (b_i = 0; b_i < status; b_i++) {
        if (bboxPred_data_dirtyOnGpu) {
          cudaMemcpy(&b_bboxPred_data[0], gpu_bboxPred_data, 16384UL,
                     cudaMemcpyDeviceToHost);
          bboxPred_data_dirtyOnGpu = false;
        }

        if ((b_bboxPred_data[b_i + b_bboxPred_size[0] * 3] >= 1.0) &&
            (b_bboxPred_data[b_i + (b_bboxPred_size[0] << 1)] >= 1.0) &&
            (b_bboxPred_data[b_i + b_bboxPred_size[0] * 3] <= 224.0) &&
            (b_bboxPred_data[b_i + (b_bboxPred_size[0] << 1)] <= 224.0)) {
          count++;
          nrows = count - 1;
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

          cudaMemcpy(gpu_nrows, &nrows, 4UL, cudaMemcpyHostToDevice);
          if (bboxPred_data_dirtyOnCpu) {
            cudaMemcpy(b_gpu_bboxPred_data, &bboxPred_data[0], 16384UL,
                       cudaMemcpyHostToDevice);
            bboxPred_data_dirtyOnCpu = false;
          }

          detectFunction_kernel54<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
            (*gpu_bboxPred_data, *gpu_bboxPred_size, b_i, *b_gpu_bboxPred_size,
             gpu_nrows, *b_gpu_bboxPred_data);
          b_bboxPred_data_dirtyOnGpu = true;
          if (scores_data_dirtyOnCpu) {
            cudaMemcpy(gpu_scores_data, &scores_data[0], 2048UL,
                       cudaMemcpyHostToDevice);
            scores_data_dirtyOnCpu = false;
          }

          if (thresholdedPrediction_size_dirtyOnCpu) {
            cudaMemcpy(gpu_thresholdedPrediction_size,
                       &thresholdedPrediction_size[0], 8UL,
                       cudaMemcpyHostToDevice);
            thresholdedPrediction_size_dirtyOnCpu = false;
          }

          if (scorePred_data_dirtyOnCpu) {
            cudaMemcpy(gpu_scorePred_data, &scorePred_data[0], 2048UL,
                       cudaMemcpyHostToDevice);
            scorePred_data_dirtyOnCpu = false;
          }

          detectFunction_kernel55<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
            (*gpu_thresholdedPrediction_data, *gpu_thresholdedPrediction_size,
             b_i, count, *gpu_scores_data, *gpu_scorePred_data);
          scorePred_data_dirtyOnGpu = true;
          scores_data_dirtyOnGpu = true;
        }
      }

      c_i = static_cast<short>(bboxPred_size[0]);
      validLaunchParams = mwGetLaunchParameters(static_cast<double>((c_i -
        static_cast<short>(count + 1)) + 1L), &x_grid, &x_block, 1024U, 65535U);
      if (validLaunchParams) {
        detectFunction_kernel56<<<x_grid, x_block>>>(static_cast<short>(count +
          1), c_i, *b_gpu_idx_data);
        idx_data_dirtyOnGpu = true;
      }

      nrowx = bboxPred_size[0];
      if (static_cast<short>(bboxPred_size[0] - count) == 1) {
        nrows = bboxPred_size[0] - 1;
        cudaMemcpy(gpu_status, &status, 4UL, cudaMemcpyHostToDevice);
        detectFunction_kernel59<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
          (*b_gpu_idx_data, gpu_status);
        isInitialise = true;
        for (startC = 0; startC < 4; startC++) {
          if (isInitialise) {
            cudaMemcpy(&status, gpu_status, 4UL, cudaMemcpyDeviceToHost);
            isInitialise = false;
          }

          for (b_i = 0; b_i <= nrows - status; b_i++) {
            tbWidth = status + b_i;
            if (b_bboxPred_data_dirtyOnGpu) {
              cudaMemcpy(&bboxPred_data[0], b_gpu_bboxPred_data, 16384UL,
                         cudaMemcpyDeviceToHost);
              b_bboxPred_data_dirtyOnGpu = false;
            }

            bboxPred_data[(tbWidth + bboxPred_size[0] * startC) - 1] =
              bboxPred_data[tbWidth + bboxPred_size[0] * startC];
            bboxPred_data_dirtyOnCpu = true;
          }
        }
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

        status = static_cast<short>(bboxPred_size[0] - count);
        for (k = 0; k < status; k++) {
          if (b_data_dirtyOnGpu) {
            cudaMemcpy(&b_data[0], gpu_b_data, 512UL, cudaMemcpyDeviceToHost);
            b_data_dirtyOnGpu = false;
          }

          if (idx_data_dirtyOnGpu) {
            cudaMemcpy(&idx_data[0], b_gpu_idx_data, 1024UL,
                       cudaMemcpyDeviceToHost);
            idx_data_dirtyOnGpu = false;
          }

          b_data[idx_data[k] - 1] = true;
          b_data_dirtyOnCpu = true;
        }

        tbWidth = 0;
        status = b_size[1];
        validLaunchParams = mwGetLaunchParameters(static_cast<double>((status -
          1) + 1L), &ab_grid, &ab_block, 1024U, 65535U);
        if (validLaunchParams) {
          cudaMemcpy(gpu_status, &status, 4UL, cudaMemcpyHostToDevice);
          if (b_data_dirtyOnCpu) {
            cudaMemcpy(gpu_b_data, &b_data[0], 512UL, cudaMemcpyHostToDevice);
            b_data_dirtyOnCpu = false;
          }

          cudaMemcpy(gpu_tbWidth, &tbWidth, 4UL, cudaMemcpyHostToDevice);
          cudaMemcpy(gpu_b_size, &b_size[0], 8UL, cudaMemcpyHostToDevice);
          detectFunction_kernel58<<<ab_grid, ab_block>>>(*gpu_b_data, gpu_status,
            gpu_tbWidth);
          cudaMemcpy(&tbWidth, gpu_tbWidth, 4UL, cudaMemcpyDeviceToHost);
        }

        nrows = bboxPred_size[0] - tbWidth;
        b_i = 0;
        for (k = 0; k < nrowx; k++) {
          guard1 = false;
          if (k + 1 > b_size[1]) {
            guard1 = true;
          } else {
            if (b_data_dirtyOnGpu) {
              cudaMemcpy(&b_data[0], gpu_b_data, 512UL, cudaMemcpyDeviceToHost);
              b_data_dirtyOnGpu = false;
            }

            if (!b_data[k]) {
              guard1 = true;
            }
          }

          if (guard1) {
            for (startC = 0; startC < 4; startC++) {
              if (b_bboxPred_data_dirtyOnGpu) {
                cudaMemcpy(&bboxPred_data[0], b_gpu_bboxPred_data, 16384UL,
                           cudaMemcpyDeviceToHost);
                b_bboxPred_data_dirtyOnGpu = false;
              }

              bboxPred_data[b_i + bboxPred_size[0] * startC] = bboxPred_data[k +
                bboxPred_size[0] * startC];
              bboxPred_data_dirtyOnCpu = true;
            }

            b_i++;
          }
        }
      }

      if (1 > nrows) {
        status = -1;
      } else {
        status = nrows - 1;
      }

      c_bboxPred_size[0] = status + 1;
      c_bboxPred_size[1] = 4;
      bboxPred_size_dirtyOnCpu = true;
      validLaunchParams = mwGetLaunchParameters(static_cast<double>((status + 1L)
        * 4L), &bb_grid, &bb_block, 1024U, 65535U);
      if (validLaunchParams) {
        cudaMemcpy(gpu_status, &status, 4UL, cudaMemcpyHostToDevice);
        if (b_bboxPred_size_dirtyOnCpu) {
          cudaMemcpy(b_gpu_bboxPred_size, &bboxPred_size[0], 8UL,
                     cudaMemcpyHostToDevice);
        }

        if (bboxPred_data_dirtyOnCpu) {
          cudaMemcpy(b_gpu_bboxPred_data, &bboxPred_data[0], 16384UL,
                     cudaMemcpyHostToDevice);
          bboxPred_data_dirtyOnCpu = false;
        }

        cudaMemcpy(c_gpu_bboxPred_size, &c_bboxPred_size[0], 8UL,
                   cudaMemcpyHostToDevice);
        bboxPred_size_dirtyOnCpu = false;
        detectFunction_kernel60<<<bb_grid, bb_block>>>(*b_gpu_bboxPred_data,
          *b_gpu_bboxPred_size, *c_gpu_bboxPred_size, gpu_status,
          *gpu_bboxPred_data);
        bboxPred_data_dirtyOnGpu = true;
      }

      bboxPred_size[0] = c_bboxPred_size[0];
      bboxPred_size[1] = 4;
      b_bboxPred_size_dirtyOnCpu = true;
      validLaunchParams = mwGetLaunchParameters(static_cast<double>
        ((c_bboxPred_size[0] * 4 - 1) + 1L), &cb_grid, &cb_block, 1024U, 65535U);
      if (validLaunchParams) {
        if (bboxPred_data_dirtyOnCpu) {
          cudaMemcpy(b_gpu_bboxPred_data, &bboxPred_data[0], 16384UL,
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

      c_i = static_cast<short>(x2_size[0]);
      validLaunchParams = mwGetLaunchParameters(static_cast<double>((c_i -
        static_cast<short>(count + 1)) + 1L), &db_grid, &db_block, 1024U, 65535U);
      if (validLaunchParams) {
        detectFunction_kernel62<<<db_grid, db_block>>>(static_cast<short>(count
          + 1), c_i, *b_gpu_idx_data);
        idx_data_dirtyOnGpu = true;
      }

      nrowx = x2_size[0];
      b_b_size[0] = 1;
      b_b_size[1] = x2_size[0];
      validLaunchParams = mwGetLaunchParameters(static_cast<double>((x2_size[0]
        - 1) + 1L), &eb_grid, &eb_block, 1024U, 65535U);
      if (validLaunchParams) {
        if (b_data_dirtyOnCpu) {
          cudaMemcpy(gpu_b_data, &b_data[0], 512UL, cudaMemcpyHostToDevice);
          b_data_dirtyOnCpu = false;
        }

        cudaMemcpy(gpu_scorePred_size, &x2_size[0], 4UL, cudaMemcpyHostToDevice);
        detectFunction_kernel63<<<eb_grid, eb_block>>>(*gpu_scorePred_size,
          *gpu_b_data);
        b_data_dirtyOnGpu = true;
      }

      status = static_cast<short>(x2_size[0] - count);
      for (k = 0; k < status; k++) {
        if (b_data_dirtyOnGpu) {
          cudaMemcpy(&b_data[0], gpu_b_data, 512UL, cudaMemcpyDeviceToHost);
          b_data_dirtyOnGpu = false;
        }

        if (idx_data_dirtyOnGpu) {
          cudaMemcpy(&idx_data[0], b_gpu_idx_data, 1024UL,
                     cudaMemcpyDeviceToHost);
          idx_data_dirtyOnGpu = false;
        }

        b_data[idx_data[k] - 1] = true;
        b_data_dirtyOnCpu = true;
      }

      tbWidth = 0;
      isInitialise = false;
      status = b_b_size[1];
      validLaunchParams = mwGetLaunchParameters(static_cast<double>((status - 1)
        + 1L), &fb_grid, &fb_block, 1024U, 65535U);
      if (validLaunchParams) {
        cudaMemcpy(gpu_status, &status, 4UL, cudaMemcpyHostToDevice);
        if (b_data_dirtyOnCpu) {
          cudaMemcpy(gpu_b_data, &b_data[0], 512UL, cudaMemcpyHostToDevice);
          b_data_dirtyOnCpu = false;
        }

        cudaMemcpy(gpu_tbWidth, &tbWidth, 4UL, cudaMemcpyHostToDevice);
        cudaMemcpy(b_gpu_b_size, &b_b_size[0], 8UL, cudaMemcpyHostToDevice);
        detectFunction_kernel64<<<fb_grid, fb_block>>>(*gpu_b_data, gpu_status,
          gpu_tbWidth);
        cudaMemcpy(&tbWidth, gpu_tbWidth, 4UL, cudaMemcpyDeviceToHost);
        isInitialise = false;
      }

      status = x2_size[0] - tbWidth;
      nrows = -1;
      for (k = 0; k < nrowx; k++) {
        guard1 = false;
        if (k + 1 > b_b_size[1]) {
          guard1 = true;
        } else {
          if (b_data_dirtyOnGpu) {
            cudaMemcpy(&b_data[0], gpu_b_data, 512UL, cudaMemcpyDeviceToHost);
            b_data_dirtyOnGpu = false;
          }

          if (!b_data[k]) {
            guard1 = true;
          }
        }

        if (guard1) {
          nrows++;
          if (scorePred_data_dirtyOnGpu) {
            cudaMemcpy(&scorePred_data[0], gpu_scorePred_data, 2048UL,
                       cudaMemcpyDeviceToHost);
            scorePred_data_dirtyOnGpu = false;
          }

          scorePred_data[nrows] = scorePred_data[k];
          scorePred_data_dirtyOnCpu = true;
        }
      }

      if (1 > status) {
        x2_size[0] = 0;
      } else {
        x2_size[0] = status;
      }

      c_i = static_cast<short>(iv_size[0]);
      validLaunchParams = mwGetLaunchParameters(static_cast<double>((c_i -
        static_cast<short>(count + 1)) + 1L), &gb_grid, &gb_block, 1024U, 65535U);
      if (validLaunchParams) {
        detectFunction_kernel65<<<gb_grid, gb_block>>>(static_cast<short>(count
          + 1), c_i, *b_gpu_idx_data);
        idx_data_dirtyOnGpu = true;
      }

      nrowx = iv_size[0];
      c_b_size[0] = 1;
      c_b_size[1] = iv_size[0];
      validLaunchParams = mwGetLaunchParameters(static_cast<double>((iv_size[0]
        - 1) + 1L), &hb_grid, &hb_block, 1024U, 65535U);
      if (validLaunchParams) {
        if (b_data_dirtyOnCpu) {
          cudaMemcpy(gpu_b_data, &b_data[0], 512UL, cudaMemcpyHostToDevice);
          b_data_dirtyOnCpu = false;
        }

        cudaMemcpy(gpu_iv_size, &iv_size[0], 4UL, cudaMemcpyHostToDevice);
        detectFunction_kernel66<<<hb_grid, hb_block>>>(*gpu_iv_size, *gpu_b_data);
        b_data_dirtyOnGpu = true;
      }

      status = static_cast<short>(iv_size[0] - count);
      for (k = 0; k < status; k++) {
        if (b_data_dirtyOnGpu) {
          cudaMemcpy(&b_data[0], gpu_b_data, 512UL, cudaMemcpyDeviceToHost);
          b_data_dirtyOnGpu = false;
        }

        if (idx_data_dirtyOnGpu) {
          cudaMemcpy(&idx_data[0], b_gpu_idx_data, 1024UL,
                     cudaMemcpyDeviceToHost);
          idx_data_dirtyOnGpu = false;
        }

        b_data[idx_data[k] - 1] = true;
        b_data_dirtyOnCpu = true;
      }

      tbWidth = 0;
      status = c_b_size[1];
      validLaunchParams = mwGetLaunchParameters(static_cast<double>((status - 1)
        + 1L), &ib_grid, &ib_block, 1024U, 65535U);
      if (validLaunchParams) {
        cudaMemcpy(gpu_status, &status, 4UL, cudaMemcpyHostToDevice);
        if (b_data_dirtyOnCpu) {
          cudaMemcpy(gpu_b_data, &b_data[0], 512UL, cudaMemcpyHostToDevice);
          b_data_dirtyOnCpu = false;
        }

        cudaMemcpy(gpu_tbWidth, &tbWidth, 4UL, cudaMemcpyHostToDevice);
        cudaMemcpy(c_gpu_b_size, &c_b_size[0], 8UL, cudaMemcpyHostToDevice);
        detectFunction_kernel67<<<ib_grid, ib_block>>>(*gpu_b_data, gpu_status,
          gpu_tbWidth);
        isInitialise = true;
      }

      if (isInitialise) {
        cudaMemcpy(&tbWidth, gpu_tbWidth, 4UL, cudaMemcpyDeviceToHost);
      }

      status = iv_size[0] - tbWidth;
      nrows = -1;
      for (k = 0; k < nrowx; k++) {
        guard1 = false;
        if (k + 1 > c_b_size[1]) {
          guard1 = true;
        } else {
          if (b_data_dirtyOnGpu) {
            cudaMemcpy(&b_data[0], gpu_b_data, 512UL, cudaMemcpyDeviceToHost);
            b_data_dirtyOnGpu = false;
          }

          if (!b_data[k]) {
            guard1 = true;
          }
        }

        if (guard1) {
          nrows++;
          if (scores_data_dirtyOnGpu) {
            cudaMemcpy(&scores_data[0], gpu_scores_data, 2048UL,
                       cudaMemcpyDeviceToHost);
            scores_data_dirtyOnGpu = false;
          }

          scores_data[nrows] = scores_data[k];
          scores_data_dirtyOnCpu = true;
        }
      }

      if (1 > status) {
        iv_size[0] = 0;
      } else {
        iv_size[0] = status;
      }

      if (bboxPred_size[0] == 0) {
        y1_size[0] = x2_size[0];
        validLaunchParams = mwGetLaunchParameters(static_cast<double>((x2_size[0]
          - 1) + 1L), &xb_grid, &xb_block, 1024U, 65535U);
        if (validLaunchParams) {
          if (scorePred_data_dirtyOnCpu) {
            cudaMemcpy(gpu_scorePred_data, &scorePred_data[0], 2048UL,
                       cudaMemcpyHostToDevice);
            scorePred_data_dirtyOnCpu = false;
          }

          cudaMemcpy(gpu_scorePred_size, &x2_size[0], 4UL,
                     cudaMemcpyHostToDevice);
          if (b_scores_data_dirtyOnCpu) {
            cudaMemcpy(b_gpu_scores_data, &b_scores_data[0], 2048UL,
                       cudaMemcpyHostToDevice);
            b_scores_data_dirtyOnCpu = false;
          }

          detectFunction_kernel82<<<xb_grid, xb_block>>>(*gpu_scorePred_data,
            *gpu_scorePred_size, *b_gpu_scores_data);
          b_scores_data_dirtyOnGpu = true;
        }
      } else {
        validLaunchParams = mwGetLaunchParameters(static_cast<double>((iv_size[0]
          - 1) + 1L), &jb_grid, &jb_block, 1024U, 65535U);
        if (validLaunchParams) {
          if (scores_data_dirtyOnCpu) {
            cudaMemcpy(gpu_scores_data, &scores_data[0], 2048UL,
                       cudaMemcpyHostToDevice);
            scores_data_dirtyOnCpu = false;
          }

          cudaMemcpy(gpu_iv_size, &iv_size[0], 4UL, cudaMemcpyHostToDevice);
          detectFunction_kernel68<<<jb_grid, jb_block>>>(*gpu_scores_data,
            *gpu_iv_size, *gpu_y1_data);
          y1_data_dirtyOnGpu = true;
        }

        y2_size[0] = x2_size[0];
        isInitialise = true;
        if (x2_size[0] != 0) {
          status = 2;
          if (x2_size[0] != 1) {
            status = 1;
          }

          inDims[0] = x2_size[0];
          inDims[1] = 1;
          validLaunchParams = mwGetLaunchParameters(static_cast<double>
            ((x2_size[0] - 1) + 1L), &kb_grid, &kb_block, 1024U, 65535U);
          if (validLaunchParams) {
            if (scores_data_dirtyOnCpu) {
              cudaMemcpy(gpu_scores_data, &scores_data[0], 2048UL,
                         cudaMemcpyHostToDevice);
              scores_data_dirtyOnCpu = false;
            }

            if (scorePred_data_dirtyOnCpu) {
              cudaMemcpy(gpu_scorePred_data, &scorePred_data[0], 2048UL,
                         cudaMemcpyHostToDevice);
              scorePred_data_dirtyOnCpu = false;
            }

            cudaMemcpy(gpu_scorePred_size, &x2_size[0], 4UL,
                       cudaMemcpyHostToDevice);
            detectFunction_kernel69<<<kb_grid, kb_block>>>(*gpu_scorePred_data, *
              gpu_scorePred_size, *gpu_scores_data);
          }

          dv2[0] = static_cast<short>(x2_size[0]);
          y2_size[0] = static_cast<short>(x2_size[0]);
          validLaunchParams = mwGetLaunchParameters(static_cast<double>((dv2[0]
            - 1) + 1L), &lb_grid, &lb_block, 1024U, 65535U);
          if (validLaunchParams) {
            cudaMemcpy(gpu_dv2, &dv2[0], 4UL, cudaMemcpyHostToDevice);
            detectFunction_kernel70<<<lb_grid, lb_block>>>(*gpu_dv2,
              *gpu_idx_data);
          }

          if (scores_data_dirtyOnCpu) {
            cudaMemcpy(gpu_scores_data, &scores_data[0], 2048UL,
                       cudaMemcpyHostToDevice);
            scores_data_dirtyOnCpu = false;
          }

          thrustSortImplWithIndex(&(*gpu_scores_data)[0], &(*gpu_idx_data)[0], 2,
            &inDims[0], status, 'd', false);
          scores_data_dirtyOnGpu = true;
        }

        inputBbox_size[0] = y2_size[0];
        inputBbox_size[1] = 4;
        bboxPred_size_dirtyOnCpu = true;
        validLaunchParams = mwGetLaunchParameters(static_cast<double>(((y2_size
          [0] - 1) + 1L) * 4L), &mb_grid, &mb_block, 1024U, 65535U);
        if (validLaunchParams) {
          cudaMemcpy(b_gpu_bboxPred_size, &bboxPred_size[0], 8UL,
                     cudaMemcpyHostToDevice);
          b_bboxPred_size_dirtyOnCpu = false;
          if (bboxPred_data_dirtyOnCpu) {
            cudaMemcpy(b_gpu_bboxPred_data, &bboxPred_data[0], 16384UL,
                       cudaMemcpyHostToDevice);
            bboxPred_data_dirtyOnCpu = false;
          }

          cudaMemcpy(gpu_inputBbox_size, &inputBbox_size[0], 8UL,
                     cudaMemcpyHostToDevice);
          bboxPred_size_dirtyOnCpu = false;
          cudaMemcpy(gpu_idx_size, &y2_size[0], 4UL, cudaMemcpyHostToDevice);
          isInitialise = false;
          detectFunction_kernel71<<<mb_grid, mb_block>>>(*b_gpu_bboxPred_data,
            *b_gpu_bboxPred_size, *gpu_idx_data, *gpu_inputBbox_size,
            *gpu_idx_size, *gpu_bboxesX1Y1X2Y2_data);
          bboxesX1Y1X2Y2_data_dirtyOnGpu = true;
        }

        iv_size[0] = y2_size[0];
        validLaunchParams = mwGetLaunchParameters(static_cast<double>((y2_size[0]
          - 1) + 1L), &nb_grid, &nb_block, 1024U, 65535U);
        if (validLaunchParams) {
          if (isInitialise) {
            cudaMemcpy(gpu_idx_size, &y2_size[0], 4UL, cudaMemcpyHostToDevice);
            isInitialise = false;
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

        y1_size[0] = y2_size[0];
        validLaunchParams = mwGetLaunchParameters(static_cast<double>((y2_size[0]
          - 1) + 1L), &pb_grid, &pb_block, 1024U, 65535U);
        if (validLaunchParams) {
          if (b_data_dirtyOnCpu) {
            cudaMemcpy(gpu_b_data, &b_data[0], 512UL, cudaMemcpyHostToDevice);
            b_data_dirtyOnCpu = false;
          }

          if (isInitialise) {
            cudaMemcpy(gpu_idx_size, &y2_size[0], 4UL, cudaMemcpyHostToDevice);
          }

          detectFunction_kernel74<<<pb_grid, pb_block>>>(*gpu_idx_size,
            *gpu_b_data);
          b_data_dirtyOnGpu = true;
        }

        k = y2_size[0] - 1;
        validLaunchParams = mwGetLaunchParameters(static_cast<double>(k + 1L),
          &qb_grid, &qb_block, 1024U, 65535U);
        if (validLaunchParams) {
          if (bboxPred_size_dirtyOnCpu) {
            cudaMemcpy(gpu_inputBbox_size, &inputBbox_size[0], 8UL,
                       cudaMemcpyHostToDevice);
            bboxPred_size_dirtyOnCpu = false;
          }

          detectFunction_kernel75<<<qb_grid, qb_block>>>
            (*gpu_bboxesX1Y1X2Y2_data, *gpu_inputBbox_size, k, *gpu_x1_data);
          x1_data_dirtyOnGpu = true;
        }

        k = y2_size[0] - 1;
        validLaunchParams = mwGetLaunchParameters(static_cast<double>(k + 1L),
          &rb_grid, &rb_block, 1024U, 65535U);
        if (validLaunchParams) {
          if (bboxPred_size_dirtyOnCpu) {
            cudaMemcpy(gpu_inputBbox_size, &inputBbox_size[0], 8UL,
                       cudaMemcpyHostToDevice);
            bboxPred_size_dirtyOnCpu = false;
          }

          detectFunction_kernel76<<<rb_grid, rb_block>>>(*gpu_inputBbox_size,
            *gpu_bboxesX1Y1X2Y2_data, k, *gpu_x2_data);
          x2_data_dirtyOnGpu = true;
        }

        k = y2_size[0] - 1;
        validLaunchParams = mwGetLaunchParameters(static_cast<double>(k + 1L),
          &sb_grid, &sb_block, 1024U, 65535U);
        if (validLaunchParams) {
          if (bboxPred_size_dirtyOnCpu) {
            cudaMemcpy(gpu_inputBbox_size, &inputBbox_size[0], 8UL,
                       cudaMemcpyHostToDevice);
          }

          detectFunction_kernel77<<<sb_grid, sb_block>>>
            (*gpu_bboxesX1Y1X2Y2_data, *gpu_inputBbox_size, k, *gpu_y2_data);
          y2_data_dirtyOnGpu = true;
        }

        status = -1;
        tbWidth = y2_size[0];
        for (b_i = 0; b_i < tbWidth; b_i++) {
          status = b_i;
          if (y1_data_dirtyOnGpu) {
            cudaMemcpy(&y1_data[0], gpu_y1_data, 4096UL, cudaMemcpyDeviceToHost);
            y1_data_dirtyOnGpu = false;
          }

          if (rtIsNaN(y1_data[b_i])) {
            if (b_data_dirtyOnGpu) {
              cudaMemcpy(&b_data[0], gpu_b_data, 512UL, cudaMemcpyDeviceToHost);
              b_data_dirtyOnGpu = false;
            }

            b_data[b_i] = false;
            b_data_dirtyOnCpu = true;
          } else {
            if (b_data_dirtyOnGpu) {
              cudaMemcpy(&b_data[0], gpu_b_data, 512UL, cudaMemcpyDeviceToHost);
              b_data_dirtyOnGpu = false;
            }

            if (b_data[b_i]) {
              nrows = (y2_size[0] - b_i) - 2;
              for (startC = 0; startC <= nrows; startC++) {
                nrowx = (b_i + startC) + 1;
                if (b_data[nrowx] && (!(y1_data[nrowx] != y1_data[b_i]))) {
                  double maxval;
                  double width;
                  if (x2_data_dirtyOnGpu) {
                    cudaMemcpy(&x2_data[0], gpu_x2_data, 4096UL,
                               cudaMemcpyDeviceToHost);
                    x2_data_dirtyOnGpu = false;
                  }

                  if ((x2_data[b_i] < x2_data[nrowx]) || rtIsNaN(x2_data[nrowx]))
                  {
                    height = x2_data[b_i];
                    height_dirtyOnCpu = true;
                  } else {
                    height = x2_data[nrowx];
                    height_dirtyOnCpu = true;
                  }

                  if (bboxesX1Y1X2Y2_data_dirtyOnGpu) {
                    cudaMemcpy(&bboxesX1Y1X2Y2_data[0], gpu_bboxesX1Y1X2Y2_data,
                               16384UL, cudaMemcpyDeviceToHost);
                    bboxesX1Y1X2Y2_data_dirtyOnGpu = false;
                  }

                  if ((bboxesX1Y1X2Y2_data[b_i] > bboxesX1Y1X2Y2_data[nrowx]) ||
                      rtIsNaN(bboxesX1Y1X2Y2_data[nrowx])) {
                    maxval = bboxesX1Y1X2Y2_data[b_i];
                  } else {
                    maxval = bboxesX1Y1X2Y2_data[nrowx];
                  }

                  width = height - maxval;
                  if (!(width <= 0.0)) {
                    if (y2_data_dirtyOnGpu) {
                      cudaMemcpy(&y2_data[0], gpu_y2_data, 4096UL,
                                 cudaMemcpyDeviceToHost);
                      y2_data_dirtyOnGpu = false;
                    }

                    if ((y2_data[b_i] < y2_data[nrowx]) || rtIsNaN(y2_data[nrowx]))
                    {
                      height = y2_data[b_i];
                    } else {
                      height = y2_data[nrowx];
                    }

                    if ((bboxesX1Y1X2Y2_data[b_i + inputBbox_size[0]] >
                         bboxesX1Y1X2Y2_data[nrowx + inputBbox_size[0]]) ||
                        rtIsNaN(bboxesX1Y1X2Y2_data[nrowx + inputBbox_size[0]]))
                    {
                      maxval = bboxesX1Y1X2Y2_data[b_i + inputBbox_size[0]];
                    } else {
                      maxval = bboxesX1Y1X2Y2_data[nrowx + inputBbox_size[0]];
                    }

                    height -= maxval;
                    if (!(height <= 0.0)) {
                      height *= width;
                      if (x1_data_dirtyOnGpu) {
                        cudaMemcpy(&x1_data[0], gpu_x1_data, 4096UL,
                                   cudaMemcpyDeviceToHost);
                        x1_data_dirtyOnGpu = false;
                      }

                      if (height / ((x1_data[b_i] + x1_data[nrowx]) - height) >
                          0.5) {
                        b_data[nrowx] = false;
                        b_data_dirtyOnCpu = true;
                      }
                    }
                  }
                }
              }
            }
          }
        }

        if (status + 2 > y1_size[0]) {
          nrows = 0;
          status = 0;
        } else {
          nrows = status + 1;
          status = y1_size[0];
        }

        iv[1] = status - nrows;
        validLaunchParams = mwGetLaunchParameters(static_cast<double>((iv[1] - 1)
          + 1L), &tb_grid, &tb_block, 1024U, 65535U);
        if (validLaunchParams) {
          if (b_data_dirtyOnCpu) {
            cudaMemcpy(gpu_b_data, &b_data[0], 512UL, cudaMemcpyHostToDevice);
            b_data_dirtyOnCpu = false;
          }

          cudaMemcpy(gpu_nrows, &nrows, 4UL, cudaMemcpyHostToDevice);
          cudaMemcpy(gpu_iv, &iv[0], 8UL, cudaMemcpyHostToDevice);
          detectFunction_kernel78<<<tb_grid, tb_block>>>(gpu_nrows, *gpu_iv,
            *gpu_b_data);
        }

        x2_size[0] = y1_size[0];
        validLaunchParams = mwGetLaunchParameters(static_cast<double>((y1_size[0]
          - 1) + 1L), &ub_grid, &ub_block, 1024U, 65535U);
        if (validLaunchParams) {
          if (b_data_dirtyOnCpu) {
            cudaMemcpy(gpu_b_data, &b_data[0], 512UL, cudaMemcpyHostToDevice);
            b_data_dirtyOnCpu = false;
          }

          cudaMemcpy(gpu_y1_size, &y1_size[0], 4UL, cudaMemcpyHostToDevice);
          detectFunction_kernel79<<<ub_grid, ub_block>>>(*gpu_b_data,
            *gpu_idx_data, *gpu_y1_size, *gpu_index_data);
          index_data_dirtyOnGpu = true;
        }

        tbWidth = x2_size[0] - 1;
        status = 0;
        for (b_i = 0; b_i <= tbWidth; b_i++) {
          if (index_data_dirtyOnGpu) {
            cudaMemcpy(&index_data[0], gpu_index_data, 512UL,
                       cudaMemcpyDeviceToHost);
            index_data_dirtyOnGpu = false;
          }

          if (index_data[b_i]) {
            status++;
          }
        }

        iv_size[0] = status;
        nrows = 0;
        for (b_i = 0; b_i <= tbWidth; b_i++) {
          if (index_data_dirtyOnGpu) {
            cudaMemcpy(&index_data[0], gpu_index_data, 512UL,
                       cudaMemcpyDeviceToHost);
            index_data_dirtyOnGpu = false;
          }

          if (index_data[b_i]) {
            iv6_data[nrows] = b_i + 1;
            iv6_data_dirtyOnCpu = true;
            nrows++;
          }
        }

        d_bboxPred_size[0] = status;
        d_bboxPred_size[1] = 4;
        bboxPred_size_dirtyOnCpu = true;
        validLaunchParams = mwGetLaunchParameters(static_cast<double>(((iv_size
          [0] - 1) + 1L) * 4L), &vb_grid, &vb_block, 1024U, 65535U);
        if (validLaunchParams) {
          cudaMemcpy(gpu_iv_size, &iv_size[0], 4UL, cudaMemcpyHostToDevice);
          if (b_bboxPred_size_dirtyOnCpu) {
            cudaMemcpy(b_gpu_bboxPred_size, &bboxPred_size[0], 8UL,
                       cudaMemcpyHostToDevice);
          }

          if (bboxPred_data_dirtyOnCpu) {
            cudaMemcpy(b_gpu_bboxPred_data, &bboxPred_data[0], 16384UL,
                       cudaMemcpyHostToDevice);
            bboxPred_data_dirtyOnCpu = false;
          }

          if (iv6_data_dirtyOnCpu) {
            cudaMemcpy(gpu_iv6_data, &iv6_data[0], 2048UL,
                       cudaMemcpyHostToDevice);
            iv6_data_dirtyOnCpu = false;
          }

          cudaMemcpy(d_gpu_bboxPred_size, &d_bboxPred_size[0], 8UL,
                     cudaMemcpyHostToDevice);
          bboxPred_size_dirtyOnCpu = false;
          detectFunction_kernel80<<<vb_grid, vb_block>>>(*b_gpu_bboxPred_data,
            *b_gpu_bboxPred_size, *gpu_iv6_data, *d_gpu_bboxPred_size,
            *gpu_iv_size, *gpu_bboxPred_data);
          bboxPred_data_dirtyOnGpu = true;
        }

        bboxPred_size[0] = d_bboxPred_size[0];
        bboxPred_size[1] = 4;
        validLaunchParams = mwGetLaunchParameters(static_cast<double>
          ((d_bboxPred_size[0] * 4 - 1) + 1L), &wb_grid, &wb_block, 1024U,
          65535U);
        if (validLaunchParams) {
          if (bboxPred_data_dirtyOnCpu) {
            cudaMemcpy(b_gpu_bboxPred_data, &bboxPred_data[0], 16384UL,
                       cudaMemcpyHostToDevice);
            bboxPred_data_dirtyOnCpu = false;
          }

          if (bboxPred_size_dirtyOnCpu) {
            cudaMemcpy(d_gpu_bboxPred_size, &d_bboxPred_size[0], 8UL,
                       cudaMemcpyHostToDevice);
          }

          detectFunction_kernel81<<<wb_grid, wb_block>>>(*gpu_bboxPred_data,
            *d_gpu_bboxPred_size, *b_gpu_bboxPred_data);
          b_bboxPred_data_dirtyOnGpu = true;
        }

        tbWidth = x2_size[0] - 1;
        status = 0;
        for (b_i = 0; b_i <= tbWidth; b_i++) {
          if (index_data_dirtyOnGpu) {
            cudaMemcpy(&index_data[0], gpu_index_data, 512UL,
                       cudaMemcpyDeviceToHost);
            index_data_dirtyOnGpu = false;
          }

          if (index_data[b_i]) {
            status++;
          }
        }

        y1_size[0] = status;
        nrows = 0;
        for (b_i = 0; b_i <= tbWidth; b_i++) {
          if (index_data_dirtyOnGpu) {
            cudaMemcpy(&index_data[0], gpu_index_data, 512UL,
                       cudaMemcpyDeviceToHost);
            index_data_dirtyOnGpu = false;
          }

          if (index_data[b_i]) {
            if (scorePred_data_dirtyOnGpu) {
              cudaMemcpy(&scorePred_data[0], gpu_scorePred_data, 2048UL,
                         cudaMemcpyDeviceToHost);
              scorePred_data_dirtyOnGpu = false;
            }

            if (b_scores_data_dirtyOnGpu) {
              cudaMemcpy(&b_scores_data[0], b_gpu_scores_data, 2048UL,
                         cudaMemcpyDeviceToHost);
              b_scores_data_dirtyOnGpu = false;
            }

            b_scores_data[nrows] = scorePred_data[b_i];
            b_scores_data_dirtyOnCpu = true;
            nrows++;
          }
        }
      }

      iv_size[0] = y1_size[0];
      validLaunchParams = mwGetLaunchParameters(static_cast<double>((y1_size[0]
        - 1) + 1L), &yb_grid, &yb_block, 1024U, 65535U);
      if (validLaunchParams) {
        if (scores_data_dirtyOnCpu) {
          cudaMemcpy(gpu_scores_data, &scores_data[0], 2048UL,
                     cudaMemcpyHostToDevice);
          scores_data_dirtyOnCpu = false;
        }

        cudaMemcpy(gpu_y1_size, &y1_size[0], 4UL, cudaMemcpyHostToDevice);
        if (b_scores_data_dirtyOnCpu) {
          cudaMemcpy(b_gpu_scores_data, &b_scores_data[0], 2048UL,
                     cudaMemcpyHostToDevice);
          b_scores_data_dirtyOnCpu = false;
        }

        detectFunction_kernel83<<<yb_grid, yb_block>>>(*b_gpu_scores_data,
          *gpu_y1_size, *gpu_scores_data);
        scores_data_dirtyOnGpu = true;
      }

      if (labelCells_dirtyOnGpu) {
        cudaMemcpy(&labelCells, gpu_labelCells, 6148UL, cudaMemcpyDeviceToHost);
        labelCells_dirtyOnGpu = false;
      }

      labelCells.size[0] = bboxPred_size[0];
      labelCells_dirtyOnCpu = true;
      status = bboxPred_size[0];
      for (b_i = 0; b_i < status; b_i++) {
        tbWidth = 0;
        for (k = 0; k < 3; k++) {
          tbWidth = k + 1;
        }

        cudaMemcpy(gpu_ind, &tbWidth, 4UL, cudaMemcpyHostToDevice);
        detectFunction_kernel84<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>(gpu_ind,
          *gpu_v_size);
        if (cv3_dirtyOnCpu) {
          cudaMemcpy(gpu_cv3, (void *)&cv3[0], 3UL, cudaMemcpyHostToDevice);
          cv3_dirtyOnCpu = false;
        }

        detectFunction_kernel85<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
          (*gpu_cv3, 0, *gpu_v_data);
        if (labelCells_dirtyOnGpu) {
          cudaMemcpy(&labelCells, gpu_labelCells, 6148UL, cudaMemcpyDeviceToHost);
          labelCells_dirtyOnGpu = false;
        }

        labelCells.data[b_i].f1.size[0] = 1;
        labelCells.data[b_i].f1.size[1] = tbWidth;
        labelCells_dirtyOnCpu = true;
        cudaMemcpy(&v_size[0], gpu_v_size, 4UL, cudaMemcpyDeviceToHost);
        validLaunchParams = mwGetLaunchParameters(static_cast<double>((v_size[0]
          - 1) + 1L), &ac_grid, &ac_block, 1024U, 65535U);
        if (validLaunchParams) {
          cudaMemcpy(gpu_labelCells, &labelCells, 6148UL, cudaMemcpyHostToDevice);
          cudaMemcpy(gpu_i, &b_i, 4UL, cudaMemcpyHostToDevice);
          detectFunction_kernel86<<<ac_grid, ac_block>>>(*gpu_v_data, gpu_i,
            *gpu_v_size, gpu_labelCells);
          labelCells_dirtyOnCpu = false;
          labelCells_dirtyOnGpu = true;
        }
      }
    } else {
      bboxPred_size[0] = 0;
      bboxPred_size[1] = 4;
      iv_size[0] = 0;
      if (labelCells_dirtyOnCpu) {
        cudaMemcpy(gpu_labelCells, &labelCells, 6148UL, cudaMemcpyHostToDevice);
        labelCells_dirtyOnCpu = false;
      }

      detectFunction_kernel31<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
        (gpu_labelCells);
      labelCells_dirtyOnGpu = true;
    }

    tbWidth = 1;
    status = iv_size[0];
    if (scores_data_dirtyOnGpu) {
      cudaMemcpy(&scores_data[0], gpu_scores_data, 2048UL,
                 cudaMemcpyDeviceToHost);
      scores_data_dirtyOnGpu = false;
    }

    ex = scores_data[0];
    for (b_i = 0; b_i <= status - 2; b_i++) {
      if (rtIsNaNF(scores_data[b_i + 1])) {
        isInitialise = false;
      } else if (rtIsNaNF(ex)) {
        isInitialise = true;
      } else {
        isInitialise = (ex < scores_data[b_i + 1]);
      }

      if (isInitialise) {
        ex = scores_data[b_i + 1];
        tbWidth = b_i + 2;
      }
    }

    //  annotate detecttions in the image
    if (bboxPred_size[0] != 0) {
      void* colPtr;
      void* posPtr;
      void* ptrObj;
      int qY;
      status = tbWidth - 1;
      cudaMemcpy(gpu_status, &status, 4UL, cudaMemcpyHostToDevice);
      cudaMemcpy(b_gpu_bboxPred_size, &bboxPred_size[0], 8UL,
                 cudaMemcpyHostToDevice);
      if (bboxPred_data_dirtyOnCpu) {
        cudaMemcpy(b_gpu_bboxPred_data, &bboxPred_data[0], 16384UL,
                   cudaMemcpyHostToDevice);
        bboxPred_data_dirtyOnCpu = false;
      }

      detectFunction_kernel87<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
        (*b_gpu_bboxPred_data, *b_gpu_bboxPred_size, gpu_status, *gpu_position);
      if (tmpRGB_dirtyOnCpu) {
        cudaMemcpy(gpu_tmpRGB, &tmpRGB[0], 150528UL, cudaMemcpyHostToDevice);
        tmpRGB_dirtyOnCpu = false;
      }

      detectFunction_kernel88<<<dim3(294U, 1U, 1U), dim3(512U, 1U, 1U)>>>
        (*b_gpu_out, *gpu_tmpRGB);
      bboxPred_size_dirtyOnCpu = true;
      if (uv_dirtyOnCpu) {
        cudaMemcpy(gpu_uv, (void *)&uv[0], 3UL, cudaMemcpyHostToDevice);
        uv_dirtyOnCpu = false;
      }

      detectFunction_kernel89<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>(*gpu_uv, *
        gpu_color);
      detectFunction_kernel90<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
        (*gpu_position, *gpu_positionOut);
      detectFunction_kernel91<<<dim3(294U, 1U, 1U), dim3(512U, 1U, 1U)>>>
        (*b_gpu_out);
      b_bboxPred_size_dirtyOnCpu = true;
      if (pixCount_dirtyOnCpu) {
        cudaMemcpy(gpu_pixCount, &pixCount[0], 224UL, cudaMemcpyHostToDevice);
        pixCount_dirtyOnCpu = false;
      }

      detectFunction_kernel92<<<dim3(1U, 1U, 1U), dim3(224U, 1U, 1U)>>>
        (*gpu_pixCount);
      thresholdedPrediction_size_dirtyOnCpu = true;
      ptrObj = NULL;
      constructDrawBaseObjectShape(&ptrObj);
      posPtr = NULL;
      cudaMemcpy(&positionOut[0], gpu_positionOut, 16UL, cudaMemcpyDeviceToHost);
      getPositionDataPointer(&posPtr, &positionOut[0], 1U, 4U);
      colPtr = NULL;
      cudaMemcpy(&color[0], gpu_color, 3UL, cudaMemcpyDeviceToHost);
      getColorDataPointer_uint8(&colPtr, &color[0], 1U, 3U);
      for (b_i = 0; b_i < 2; b_i++) {
        isInitialise = initialiseDrawbaseShape(ptrObj, static_cast<short>(b_i),
          1);
        if (!isInitialise) {
          if (b_bboxPred_size_dirtyOnCpu) {
            cudaMemcpy(&out[0], b_gpu_out, 150528UL, cudaMemcpyDeviceToHost);
            b_bboxPred_size_dirtyOnCpu = false;
          }

          if (bboxPred_size_dirtyOnCpu) {
            cudaMemcpy(&tmpRGB[0], gpu_tmpRGB, 150528UL, cudaMemcpyDeviceToHost);
            bboxPred_size_dirtyOnCpu = false;
          }

          if (thresholdedPrediction_size_dirtyOnCpu) {
            cudaMemcpy(&pixCount[0], gpu_pixCount, 224UL, cudaMemcpyDeviceToHost);
            thresholdedPrediction_size_dirtyOnCpu = false;
          }

          instantiateDrawBaseShape_uint8(ptrObj, &out[0], &tmpRGB[0], posPtr,
            colPtr, 0.6, 1, 1, true, 224, 224, 3, 2, 1, 4, 1, false, bv1[b_i],
            &pixCount[0], static_cast<short>(b_i));
          pixCount_dirtyOnCpu = true;
          tmpRGB_dirtyOnCpu = true;
          out_dirtyOnCpu = true;
        }
      }

      mDrawShapes(ptrObj, false, true, 1, 1, 224, 224);
      deallocateMemoryShape(ptrObj);
      deletePositionDataPointer(posPtr);
      deleteColorDataPointer_uint8(colPtr);
      cudaMemcpy(gpu_positionOut, &positionOut[0], 16UL, cudaMemcpyHostToDevice);
      detectFunction_kernel93<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
        (*gpu_position, *gpu_positionOut);
      cudaMemcpy(&position[0], gpu_position, 16UL, cudaMemcpyDeviceToHost);
      if (position[1] < -2147483647) {
        qY = MIN_int32_T;
      } else {
        qY = position[1] - 1;
      }

      detectFunction_kernel94<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>(qY,
        *gpu_positionOut);
      cudaMemcpy(gpu_color, &color[0], 3UL, cudaMemcpyHostToDevice);
      detectFunction_kernel95<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>(*gpu_uv, *
        gpu_color);
      bboxPred_size_dirtyOnCpu = true;
      thisTextU16_size[0] = 1;
      if (labelCells_dirtyOnGpu) {
        cudaMemcpy(&labelCells, gpu_labelCells, 6148UL, cudaMemcpyDeviceToHost);
        labelCells_dirtyOnGpu = false;
      }

      thisTextU16_size[1] = labelCells.data[tbWidth - 1].f1.size[1];
      validLaunchParams = mwGetLaunchParameters(static_cast<double>
        ((labelCells.data[tbWidth - 1].f1.size[1] - 1) + 1L), &bc_grid,
        &bc_block, 1024U, 65535U);
      if (validLaunchParams) {
        if (labelCells_dirtyOnCpu) {
          cudaMemcpy(gpu_labelCells, &labelCells, 6148UL, cudaMemcpyHostToDevice);
          labelCells_dirtyOnCpu = false;
        }

        cudaMemcpy(gpu_tbWidth, &tbWidth, 4UL, cudaMemcpyHostToDevice);
        detectFunction_kernel96<<<bc_grid, bc_block>>>(gpu_labelCells,
          gpu_tbWidth, *gpu_thisTextU16_data);
        thisTextU16_data_dirtyOnGpu = true;
      }

      if (thisTextU16_size[1] != 0) {
        int b_r;
        int c;
        int endC_gl;
        int i17;
        int penY;
        int r;
        thisCharcodes_1b_size[0] = 1;
        thisCharcodes_1b_size[1] = thisTextU16_size[1];
        thresholdedPrediction_size_dirtyOnCpu = true;
        validLaunchParams = mwGetLaunchParameters(static_cast<double>
          ((thisTextU16_size[1] - 1) + 1L), &cc_grid, &cc_block, 1024U, 65535U);
        if (validLaunchParams) {
          cudaMemcpy(gpu_thisTextU16_size, &thisTextU16_size[0], 8UL,
                     cudaMemcpyHostToDevice);
          detectFunction_kernel97<<<cc_grid, cc_block>>>(*gpu_thisTextU16_data, *
            gpu_thisTextU16_size, *gpu_thisCharcodes_1b_data);
        }

        x_size[0] = 1;
        x_size[1] = thisCharcodes_1b_size[1];
        validLaunchParams = mwGetLaunchParameters(static_cast<double>
          ((thisCharcodes_1b_size[1] - 1) + 1L), &dc_grid, &dc_block, 1024U,
          65535U);
        if (validLaunchParams) {
          if (iv2_dirtyOnCpu) {
            cudaMemcpy(gpu_iv2, (void *)&iv2[0], 261UL, cudaMemcpyHostToDevice);
            iv2_dirtyOnCpu = false;
          }

          if (uv1_dirtyOnCpu) {
            cudaMemcpy(gpu_uv1, (void *)&uv1[0], 512UL, cudaMemcpyHostToDevice);
            uv1_dirtyOnCpu = false;
          }

          cudaMemcpy(gpu_thisCharcodes_1b_size, &thisCharcodes_1b_size[0], 8UL,
                     cudaMemcpyHostToDevice);
          thresholdedPrediction_size_dirtyOnCpu = false;
          detectFunction_kernel98<<<dc_grid, dc_block>>>(*gpu_iv2, *gpu_uv1,
            *gpu_thisCharcodes_1b_data, *gpu_thisCharcodes_1b_size, *gpu_x_data);
        }

        status = thisCharcodes_1b_size[1];
        if (iv2_dirtyOnCpu) {
          cudaMemcpy(gpu_iv2, (void *)&iv2[0], 261UL, cudaMemcpyHostToDevice);
          iv2_dirtyOnCpu = false;
        }

        if (uv1_dirtyOnCpu) {
          cudaMemcpy(gpu_uv1, (void *)&uv1[0], 512UL, cudaMemcpyHostToDevice);
          uv1_dirtyOnCpu = false;
        }

        if (height_dirtyOnCpu) {
          cudaMemcpy(gpu_height, &height, 8UL, cudaMemcpyHostToDevice);
        }

        detectFunction_kernel99<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
          (*gpu_iv2, *gpu_uv1, *gpu_thisCharcodes_1b_data, gpu_height);
        validLaunchParams = mwGetLaunchParameters(static_cast<double>((status -
          2) + 1L), &ec_grid, &ec_block, 1024U, 65535U);
        if (validLaunchParams) {
          cudaMemcpy(gpu_status, &status, 4UL, cudaMemcpyHostToDevice);
          cudaMemcpy(gpu_x_size, &x_size[0], 8UL, cudaMemcpyHostToDevice);
          detectFunction_kernel100<<<ec_grid, ec_block>>>(*gpu_x_data,
            gpu_status, gpu_height);
        }

        b_x_size[0] = 1;
        b_x_size[1] = thisCharcodes_1b_size[1];
        validLaunchParams = mwGetLaunchParameters(static_cast<double>
          ((thisCharcodes_1b_size[1] - 1) + 1L), &fc_grid, &fc_block, 1024U,
          65535U);
        if (validLaunchParams) {
          if (thresholdedPrediction_size_dirtyOnCpu) {
            cudaMemcpy(gpu_thisCharcodes_1b_size, &thisCharcodes_1b_size[0], 8UL,
                       cudaMemcpyHostToDevice);
          }

          detectFunction_kernel101<<<fc_grid, fc_block>>>(*gpu_uv1,
            *gpu_thisCharcodes_1b_data, *gpu_thisCharcodes_1b_size,
            *b_gpu_x_data);
        }

        status = b_x_size[1];
        cudaMemcpy(gpu_nrows, &nrows, 4UL, cudaMemcpyHostToDevice);
        detectFunction_kernel102<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
          (*b_gpu_x_data, gpu_nrows);
        validLaunchParams = mwGetLaunchParameters(static_cast<double>((status -
          2) + 1L), &gc_grid, &gc_block, 1024U, 65535U);
        if (validLaunchParams) {
          cudaMemcpy(gpu_status, &status, 4UL, cudaMemcpyHostToDevice);
          cudaMemcpy(b_gpu_x_size, &b_x_size[0], 8UL, cudaMemcpyHostToDevice);
          detectFunction_kernel103<<<gc_grid, gc_block>>>(*b_gpu_x_data,
            gpu_status, gpu_nrows);
        }

        cudaMemcpy(&height, gpu_height, 8UL, cudaMemcpyDeviceToHost);
        if (height < 2.147483648E+9) {
          status = static_cast<int>(height);
        } else {
          status = MAX_int32_T;
        }

        cudaMemcpy(&nrows, gpu_nrows, 4UL, cudaMemcpyDeviceToHost);
        height = static_cast<double>(nrows) * 4.0;
        height_dirtyOnCpu = true;
        if (height < 2.147483648E+9) {
          if (height >= -2.147483648E+9) {
            nrows = static_cast<int>(height);
          } else {
            nrows = MIN_int32_T;
          }
        } else {
          nrows = MAX_int32_T;
        }

        if ((status > 0) && (nrows > MAX_int32_T - status)) {
          tbWidth = MAX_int32_T;
        } else {
          tbWidth = status + nrows;
        }

        if (tbWidth <= position[2]) {
          tbWidth = position[2];
        }

        if (tbWidth > position[2]) {
          if (tbWidth > 2147483639) {
            tbWidth = MAX_int32_T;
          } else {
            tbWidth += 8;
          }
        }

        cudaMemcpy(&positionOut[0], gpu_positionOut, 16UL,
                   cudaMemcpyDeviceToHost);
        if (positionOut[1] < -2147483625) {
          qY = MIN_int32_T;
        } else {
          qY = positionOut[1] - 23;
        }

        count = qY + 1;
        endC_gl = positionOut[0];
        if ((position[2] > 0) && (position[3] > 0)) {
          if (qY + 1 > 2147483624) {
            k = MAX_int32_T;
          } else {
            k = qY + 24;
          }

          guard1 = false;
          if (positionOut[0] <= 224) {
            if ((positionOut[0] < 0) && (position[2] < MIN_int32_T
                 - positionOut[0])) {
              status = MIN_int32_T;
            } else if ((positionOut[0] > 0) && (position[2] > MAX_int32_T
                        - positionOut[0])) {
              status = MAX_int32_T;
            } else {
              status = positionOut[0] + position[2];
            }

            if ((status - 1 >= 1) && (k <= 224)) {
              if ((k < 0) && (position[3] < MIN_int32_T - k)) {
                k = MIN_int32_T;
              } else if ((k > 0) && (position[3] > MAX_int32_T - k)) {
                k = MAX_int32_T;
              } else {
                k += position[3];
              }

              if (k - 1 >= 1) {
                if (qY + 1 < 1) {
                  if ((qY + 24 < 0) && (position[3] < 2147483624 - qY)) {
                    qY = MIN_int32_T;
                  } else if ((qY + 24 > 0) && (position[3] > 2147483623 - qY)) {
                    qY = MAX_int32_T;
                  } else {
                    qY = (qY + position[3]) + 24;
                  }

                  if (qY >= 1) {
                    if ((positionOut[1] < 0) && (position[3] < MIN_int32_T
                         - positionOut[1])) {
                      qY = MIN_int32_T;
                    } else if ((positionOut[1] > 0) && (position[3] >
                                MAX_int32_T - positionOut[1])) {
                      qY = MAX_int32_T;
                    } else {
                      qY = positionOut[1] + position[3];
                    }

                    if (qY > 2147483646) {
                      count = MAX_int32_T;
                    } else {
                      count = qY + 1;
                    }
                  }
                }

                if ((positionOut[0] < 0) && (tbWidth < MIN_int32_T
                     - positionOut[0])) {
                  qY = MIN_int32_T;
                } else if ((positionOut[0] > 0) && (tbWidth > MAX_int32_T
                            - positionOut[0])) {
                  qY = MAX_int32_T;
                } else {
                  qY = positionOut[0] + tbWidth;
                }

                if (static_cast<double>(qY) - 224.0 >= -2.147483648E+9) {
                  nrows = qY - 224;
                } else {
                  nrows = MIN_int32_T;
                }

                if (nrows > 0) {
                  if ((positionOut[0] >= 0) && (nrows < positionOut[0] -
                       MAX_int32_T)) {
                    qY = MAX_int32_T;
                  } else if ((positionOut[0] < 0) && (nrows > positionOut[0] -
                              MIN_int32_T)) {
                    qY = MIN_int32_T;
                  } else {
                    qY = positionOut[0] - nrows;
                  }

                  endC_gl = qY + 1;
                }

                if (endC_gl < 1) {
                  if ((positionOut[0] < 0) && (position[2] < MIN_int32_T
                       - positionOut[0])) {
                    qY = MIN_int32_T;
                  } else if ((positionOut[0] > 0) && (position[2] > MAX_int32_T
                              - positionOut[0])) {
                    qY = MAX_int32_T;
                  } else {
                    qY = positionOut[0] + position[2];
                  }

                  if (qY >= 1) {
                    endC_gl = 1;
                  }
                }
              } else {
                guard1 = true;
              }
            } else {
              guard1 = true;
            }
          } else {
            guard1 = true;
          }

          if (guard1) {
            count = -32767;
            endC_gl = -32767;
          }
        }

        nrows = count;
        if (count > 2147483624) {
          qY = MAX_int32_T;
        } else {
          qY = count + 23;
        }

        nrowx = qY - 1;
        startC = endC_gl;
        if ((endC_gl < 0) && (tbWidth < MIN_int32_T - endC_gl)) {
          k = MIN_int32_T;
        } else if ((endC_gl > 0) && (tbWidth > MAX_int32_T - endC_gl)) {
          k = MAX_int32_T;
        } else {
          k = endC_gl + tbWidth;
        }

        if (k < -2147483647) {
          status = MIN_int32_T;
        } else {
          status = k - 1;
        }

        if ((count <= 224) && (qY - 1 >= 1) && (endC_gl <= 224) && (status >= 1))
        {
          if (count < 1) {
            nrows = 1;
          }

          if (qY - 1 > 224) {
            nrowx = 224;
          }

          if (endC_gl < 1) {
            startC = 1;
          }

          if (status > 224) {
            status = 224;
          }

          for (b_i = 0; b_i < 3; b_i++) {
            for (qY = 0; qY <= status - startC; qY++) {
              c = (startC + qY) - 1;
              for (r = 0; r <= nrowx - nrows; r++) {
                unsigned char tmp22;
                b_r = (nrows + r) - 1;
                if (bboxPred_size_dirtyOnCpu) {
                  cudaMemcpy(&color[0], gpu_color, 3UL, cudaMemcpyDeviceToHost);
                  bboxPred_size_dirtyOnCpu = false;
                }

                resolutionStatus = (uint8_T)(0.6 * static_cast<double>(color[b_i])
                  + 0.5);
                if (b_bboxPred_size_dirtyOnCpu) {
                  cudaMemcpy(&out[0], b_gpu_out, 150528UL,
                             cudaMemcpyDeviceToHost);
                  b_bboxPred_size_dirtyOnCpu = false;
                }

                tmp22 = (uint8_T)(0.4 * static_cast<double>(out[(b_r + 224 * c)
                  + 50176 * b_i]) + 0.5);
                k = static_cast<int>(static_cast<unsigned int>(resolutionStatus)
                                     + tmp22);
                if (static_cast<unsigned int>(k) > 255U) {
                  k = 255;
                }

                out[(b_r + 224 * c) + 50176 * b_i] = static_cast<unsigned char>
                  (k);
                out_dirtyOnCpu = true;
              }
            }
          }
        }

        if (endC_gl > 2147483643) {
          penX = MAX_int32_T;
        } else {
          penX = endC_gl + 4;
        }

        if (count > 2147483643) {
          qY = MAX_int32_T;
        } else {
          qY = count + 4;
        }

        if (qY > 2147483635) {
          penY = MAX_int32_T;
        } else {
          penY = qY + 12;
        }

        i17 = thisTextU16_size[1];
        for (b_i = 0; b_i < i17; b_i++) {
          if (thisTextU16_data_dirtyOnGpu) {
            cudaMemcpy(&thisTextU16_data[0], gpu_thisTextU16_data, 3UL,
                       cudaMemcpyDeviceToHost);
            thisTextU16_data_dirtyOnGpu = false;
          }

          if (uv1[thisTextU16_data[b_i]] == 0) {
            if (penX > 2147483643) {
              penX = MAX_int32_T;
            } else {
              penX += 4;
            }
          } else {
            int endR_im;
            int startC_im;
            int yy;
            nrows = iv4[uv1[thisTextU16_data[b_i]]];
            if ((penX < 0) && (nrows < MIN_int32_T - penX)) {
              startC_im = MIN_int32_T;
            } else if ((penX > 0) && (nrows > MAX_int32_T - penX)) {
              startC_im = MAX_int32_T;
            } else {
              startC_im = penX + nrows;
            }

            yy = penY - iv5[uv1[thisTextU16_data[b_i]]];
            nrows = uv3[uv1[thisTextU16_data[b_i]]];
            if ((yy < 0) && (nrows < MIN_int32_T - yy)) {
              qY = MIN_int32_T;
            } else if ((yy > 0) && (nrows > MAX_int32_T - yy)) {
              qY = MAX_int32_T;
            } else {
              qY = yy + nrows;
            }

            endR_im = qY - 1;
            nrows = uv2[uv1[thisTextU16_data[b_i]]];
            isInitialise = true;
            if ((startC_im < 0) && (nrows < MIN_int32_T - startC_im)) {
              k = MIN_int32_T;
            } else if ((startC_im > 0) && (nrows > MAX_int32_T - startC_im)) {
              k = MAX_int32_T;
            } else {
              k = startC_im + nrows;
            }

            count = k - 1;
            if ((yy <= 224) && (qY - 1 >= 1) && (startC_im <= 224) && (k - 1 >=
                 1)) {
              signed char num_idx_0;
              signed char num_idx_1;
              nrowx = 1;
              startC = 1;
              tbWidth = uv3[uv1[thisTextU16_data[b_i]]];
              endC_gl = uv2[uv1[thisTextU16_data[b_i]]];
              if (yy < 1) {
                nrowx = 2 - yy;
                yy = 1;
              }

              if (qY - 1 > 224) {
                tbWidth = (uv3[uv1[thisTextU16_data[b_i]]] - qY) + 225;
                endR_im = 224;
              }

              if (startC_im < 1) {
                if (-startC_im > 2147483645) {
                  startC = MAX_int32_T;
                } else {
                  startC = 2 - startC_im;
                }

                startC_im = 1;
              }

              if (k - 1 > 224) {
                endC_gl = (uv2[uv1[thisTextU16_data[b_i]]] - k) + 225;
                count = 224;
              }

              status = static_cast<int>(static_cast<unsigned int>
                (uv4[uv1[thisTextU16_data[b_i]]]) + uv2[uv1[thisTextU16_data[b_i]]]
                * uv3[uv1[thisTextU16_data[b_i]]]);
              if (uv4[uv1[thisTextU16_data[b_i]]] + 1U > static_cast<unsigned
                  int>(status)) {
                nrows = 0;
                status = -1;
              } else {
                nrows = uv4[uv1[thisTextU16_data[b_i]]];
                status--;
              }

              num_idx_0 = uv2[uv1[thisTextU16_data[b_i]]];
              num_idx_1 = uv3[uv1[thisTextU16_data[b_i]]];
              validLaunchParams = mwGetLaunchParameters(static_cast<double>
                ((status - nrows) + 1L), &hc_grid, &hc_block, 1024U, 65535U);
              if (validLaunchParams) {
                cudaMemcpy(gpu_status, &status, 4UL, cudaMemcpyHostToDevice);
                cudaMemcpy(gpu_nrows, &nrows, 4UL, cudaMemcpyHostToDevice);
                isInitialise = false;
                if (uv5_dirtyOnCpu) {
                  cudaMemcpy(gpu_uv5, (void *)&uv5[0], 10664UL,
                             cudaMemcpyHostToDevice);
                  uv5_dirtyOnCpu = false;
                }

                detectFunction_kernel104<<<hc_grid, hc_block>>>(*gpu_uv5,
                  gpu_nrows, gpu_status, *gpu_uv5_data);
              }

              num[0] = num_idx_0;
              num[1] = num_idx_1;
              uv5_size[0] = num_idx_1;
              uv5_size[1] = num_idx_0;
              validLaunchParams = mwGetLaunchParameters(static_cast<double>
                (((num[1] - 1) + 1L) * ((num[0] - 1) + 1L)), &ic_grid, &ic_block,
                1024U, 65535U);
              if (validLaunchParams) {
                cudaMemcpy(gpu_uv5_size, &uv5_size[0], 8UL,
                           cudaMemcpyHostToDevice);
                cudaMemcpy(gpu_num, &num[0], 2UL, cudaMemcpyHostToDevice);
                detectFunction_kernel105<<<ic_grid, ic_block>>>(*gpu_uv5_data,
                  *gpu_uv5_size, *gpu_num, *b_gpu_uv5_data);
              }

              num[0] = num_idx_1;
              num[1] = num_idx_0;
              validLaunchParams = mwGetLaunchParameters(static_cast<double>
                ((num[0] * num[1] - 1) + 1L), &jc_grid, &jc_block, 1024U, 65535U);
              if (validLaunchParams) {
                cudaMemcpy(gpu_num, &num[0], 2UL, cudaMemcpyHostToDevice);
                detectFunction_kernel106<<<jc_grid, jc_block>>>(*b_gpu_uv5_data,
                  *gpu_num, *gpu_thisGlyphBitmap_data);
                thisGlyphBitmap_data_dirtyOnGpu = true;
              }

              if (nrowx > tbWidth) {
                tbWidth = 1;
              } else {
                tbWidth = nrowx;
              }

              if (startC > endC_gl) {
                status = -1;
              } else {
                status = startC - 2;
              }

              for (nrows = 0; nrows < 3; nrows++) {
                isInitialise = true;
                height = 1.0;
                for (qY = 0; qY <= count - startC_im; qY++) {
                  c = (startC_im + qY) - 1;
                  nrowx = 0;
                  for (r = 0; r <= endR_im - yy; r++) {
                    b_r = (yy + r) - 1;
                    if (thisGlyphBitmap_data_dirtyOnGpu) {
                      cudaMemcpy(&thisGlyphBitmap_data[0],
                                 gpu_thisGlyphBitmap_data, 144UL,
                                 cudaMemcpyDeviceToHost);
                      thisGlyphBitmap_data_dirtyOnGpu = false;
                    }

                    resolutionStatus = thisGlyphBitmap_data[((tbWidth + nrowx) +
                      num_idx_1 * (status + static_cast<int>(height))) - 1];
                    if (resolutionStatus == 255) {
                      if (b_bboxPred_size_dirtyOnCpu) {
                        cudaMemcpy(&out[0], b_gpu_out, 150528UL,
                                   cudaMemcpyDeviceToHost);
                        b_bboxPred_size_dirtyOnCpu = false;
                      }

                      out[(b_r + 224 * c) + 50176 * nrows] = 0U;
                      out_dirtyOnCpu = true;
                    } else {
                      if (resolutionStatus != 0) {
                        unsigned short b_x;
                        unsigned short tmp3;
                        if (b_bboxPred_size_dirtyOnCpu) {
                          cudaMemcpy(&out[0], b_gpu_out, 150528UL,
                                     cudaMemcpyDeviceToHost);
                          b_bboxPred_size_dirtyOnCpu = false;
                        }

                        b_x = static_cast<unsigned short>(out[(b_r + 224 * c) +
                          50176 * nrows] * (255 - resolutionStatus));
                        tmp3 = static_cast<unsigned short>(b_x / 255U);
                        b_x = static_cast<unsigned short>(static_cast<unsigned
                          int>(b_x) - tmp3 * 255);
                        if ((b_x > 0) && (b_x >= 128)) {
                          tmp3 = static_cast<unsigned short>(tmp3 + 1);
                        }

                        out[(b_r + 224 * c) + 50176 * nrows] = static_cast<
                          unsigned char>(tmp3);
                        out_dirtyOnCpu = true;
                      }
                    }

                    nrowx++;
                  }

                  height++;
                }
              }
            }

            if (isInitialise) {
              cudaMemcpy(gpu_nrows, &nrows, 4UL, cudaMemcpyHostToDevice);
            }

            detectFunction_kernel107<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
              (*gpu_iv2, *gpu_uv1, b_i, *gpu_thisTextU16_data, gpu_nrows);
            cudaMemcpy(&nrows, gpu_nrows, 4UL, cudaMemcpyDeviceToHost);
            if ((penX < 0) && (nrows < MIN_int32_T - penX)) {
              penX = MIN_int32_T;
            } else if ((penX > 0) && (nrows > MAX_int32_T - penX)) {
              penX = MAX_int32_T;
            } else {
              penX += nrows;
            }
          }
        }
      }
    }

    if (out_dirtyOnCpu) {
      cudaMemcpy(b_gpu_out, &out[0], 150528UL, cudaMemcpyHostToDevice);
      out_dirtyOnCpu = false;
    }

    if (varargin_2_dirtyOnCpu) {
      cudaMemcpy(gpu_varargin_2, &varargin_2[0], 50176UL, cudaMemcpyHostToDevice);
    }

    if (varargin_1_dirtyOnCpu) {
      cudaMemcpy(gpu_varargin_1, &varargin_1[0], 50176UL, cudaMemcpyHostToDevice);
    }

    detectFunction_kernel108<<<dim3(98U, 1U, 1U), dim3(512U, 1U, 1U)>>>
      (*b_gpu_out, *gpu_varargin_2, *gpu_varargin_1);
    if (varargin_3_dirtyOnCpu) {
      cudaMemcpy(gpu_varargin_3, &varargin_3[0], 50176UL, cudaMemcpyHostToDevice);
    }

    detectFunction_kernel109<<<dim3(98U, 1U, 1U), dim3(512U, 1U, 1U)>>>
      (*b_gpu_out, *gpu_varargin_3);
    if (d_isInitialized != 1) {
      d_isInitialized = 1;
      MW_SDL_videoDisplayInit(1, 1, 1, 224.0, 224.0);
    }

    cudaMemcpy(&varargin_2[0], gpu_varargin_2, 50176UL, cudaMemcpyDeviceToHost);
    cudaMemcpy(&varargin_1[0], gpu_varargin_1, 50176UL, cudaMemcpyDeviceToHost);
    cudaMemcpy(&varargin_3[0], gpu_varargin_3, 50176UL, cudaMemcpyDeviceToHost);
    MW_SDL_videoDisplayOutput(&varargin_1[0], &varargin_2[0], &varargin_3[0]);
    varargin_3_dirtyOnCpu = true;
    varargin_1_dirtyOnCpu = true;
    varargin_2_dirtyOnCpu = true;
  }

  if (d_isInitialized == 1) {
    MW_SDL_videoDisplayTerminate(0, 0);
  }

  if ((w_isInitialized == 1) && w_Initialized) {
    EXT_webcamTerminate(1, 0);
  }

  cudaFree(gpu_status);
  cudaFree(*gpu_cv2);
  cudaFree(*gpu_cv1);
  cudaFree(*gpu_pln1);
  cudaFree(*gpu_pln0);
  cudaFree(*gpu_pln2);
  cudaFree(*gpu_img);
  cudaFree(*gpu_aux1);
  cudaFree(*gpu_aux2);
  cudaFree(*gpu_ipRowIndices);
  cudaFree(*gpu_rowWeights);
  cudaFree(*gpu_ipColIndices);
  cudaFree(*gpu_colWeights);
  cudaFree(*gpu_rowWeightsTotal);
  cudaFree(*gpu_colWeightsTotal);
  cudaFree(*gpu_partialResize);
  cudaFree(*b_gpu_out);
  cudaFree(*b_gpu_aux2);
  cudaFree(*b_gpu_aux1);
  cudaFree(*b_gpu_ipRowIndices);
  cudaFree(*b_gpu_rowWeights);
  cudaFree(*b_gpu_ipColIndices);
  cudaFree(*b_gpu_colWeights);
  cudaFree(*b_gpu_rowWeightsTotal);
  cudaFree(*b_gpu_colWeightsTotal);
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
  cudaFree(*gpu_iv_size);
  cudaFree(*gpu_iv_data);
  cudaFree(gpu_nrows);
  cudaFree(*gpu_thresholdedPrediction_size);
  cudaFree(*gpu_thresholdedPrediction_data);
  cudaFree(*b_gpu_bboxPred_size);
  cudaFree(gpu_labelCells);
  cudaFree(*gpu_bboxesX1Y1X2Y2_size);
  cudaFree(*gpu_bboxesX1Y1X2Y2_data);
  cudaFree(*gpu_x1_data);
  cudaFree(*gpu_y1_size);
  cudaFree(*gpu_y1_data);
  cudaFree(*gpu_x2_data);
  cudaFree(*gpu_y2_data);
  cudaFree(gpu_tbWidth);
  cudaFree(*gpu_bboxPred_size);
  cudaFree(*gpu_bboxPred_data);
  cudaFree(*gpu_scorePred_size);
  cudaFree(*b_gpu_bboxPred_data);
  cudaFree(*gpu_scores_data);
  cudaFree(*gpu_scorePred_data);
  cudaFree(*b_gpu_idx_data);
  cudaFree(*gpu_b_size);
  cudaFree(*c_gpu_bboxPred_size);
  cudaFree(*b_gpu_b_size);
  cudaFree(*c_gpu_b_size);
  cudaFree(*gpu_idx_size);
  cudaFree(*gpu_dv2);
  cudaFree(*gpu_idx_data);
  cudaFree(*gpu_inputBbox_size);
  cudaFree(gpu_height);
  cudaFree(*gpu_iv);
  cudaFree(*gpu_index_data);
  cudaFree(*gpu_iv6_data);
  cudaFree(*d_gpu_bboxPred_size);
  cudaFree(*b_gpu_scores_data);
  cudaFree(gpu_ind);
  cudaFree(*gpu_v_size);
  cudaFree(*gpu_cv3);
  cudaFree(*gpu_v_data);
  cudaFree(gpu_i);
  cudaFree(*gpu_position);
  cudaFree(*gpu_tmpRGB);
  cudaFree(*gpu_uv);
  cudaFree(*gpu_color);
  cudaFree(*gpu_positionOut);
  cudaFree(*gpu_pixCount);
  cudaFree(*gpu_thisTextU16_size);
  cudaFree(*gpu_thisTextU16_data);
  cudaFree(*gpu_thisCharcodes_1b_size);
  cudaFree(*gpu_thisCharcodes_1b_data);
  cudaFree(*gpu_x_size);
  cudaFree(*gpu_iv2);
  cudaFree(*gpu_uv1);
  cudaFree(*gpu_x_data);
  cudaFree(*b_gpu_x_size);
  cudaFree(*b_gpu_x_data);
  cudaFree(*gpu_uv5);
  cudaFree(*gpu_uv5_data);
  cudaFree(*gpu_num);
  cudaFree(*gpu_uv5_size);
  cudaFree(*b_gpu_uv5_data);
  cudaFree(*gpu_thisGlyphBitmap_data);
  cudaFree(*gpu_varargin_2);
  cudaFree(*gpu_varargin_1);
  cudaFree(*gpu_varargin_3);
}

//
// %% standart input values if no are given
//  % nargin returns the number of function input arguments given int the
//  % call to the currently executing function.
//  switch nargin
//    case 0
//      inputImgSize = [];
//      imageDisplay = [];
//    case 1
//      imageDisplay = [];
//    case 2
//    otherwise
//      error('2 inputs are accepted.')
//  end
//  if isempty(inputImgSize)
//    inputImgSize = [416 416];
//  end
//  if isempty(lwRadiation)
//    imageDisplay = true;
//  end
// Arguments    : void
// Return Type  : void
//
void detectFunction_init()
{
  mynet_not_empty = false;
}

//
// File trailer for detectFunction.cu
//
// [EOF]
//
