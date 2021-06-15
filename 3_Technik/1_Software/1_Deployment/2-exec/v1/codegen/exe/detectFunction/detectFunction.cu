//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: detectFunction.cu
//
// GPU Coder version                    : 2.0
// CUDA/C/C++ source code generated on  : 12-Mar-2021 11:28:36
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
#include "frameReader.h"
#include "getCameraProps.h"
#include "insertShapeUtilsCore_api.hpp"
#include "math_constants.h"
#include "rt_nonfinite.h"

// Custom Source Code
#include "main.h"

class yoloNetwork0_0;

// Type Definitions
namespace coder
{
  namespace nvidiacoder
  {
    namespace common
    {
      struct imageDisplay
      {
        bool matlabCodegenIsDeleted;
        int isInitialized;
        bool isSetupComplete;
        int PixelFormatEnum;
      };
    }
  }
}

struct emxArray_char_T_1x3
{
  char data[3];
  int size[2];
};

struct cell_wrap_7
{
  emxArray_char_T_1x3 f1;
};

struct emxArray_cell_wrap_7_1024
{
  cell_wrap_7 data[1024];
  int size[1];
};

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
static yoloNetwork0_0 gobj_3;
static coder::YOLOv2Network mynet;
static bool mynet_not_empty;
static bool hwobj_not_empty;
static coder::nvidiacoder::common::camera cam;
static bool cam_not_empty;
static coder::nvidiacoder::common::imageDisplay display;
static bool display_not_empty;
static __device__ coder::nvidiacoder::common::camera gpu_cam;

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
static __global__ void detectFunction_kernel1(const char cv[33]);
static __global__ void detectFunction_kernel10(const double rowWeights[2912],
  const int initStatus, double rowWeightsTotal[224]);
static __global__ void detectFunction_kernel100(const signed char
  thisTextU16_data[3], const int uv7_size[2], signed char thisCharcodes_1b_data
  [3]);
static __global__ void detectFunction_kernel101(const signed char iv5[261],
  const unsigned short uv[256], const signed char thisCharcodes_1b_data[3],
  const int uv7_size[2], signed char x_data[3]);
static __global__ void detectFunction_kernel102(const signed char iv5[261],
  const unsigned short uv[256], const signed char thisCharcodes_1b_data[3],
  double *validSampleTime);
static __global__ void detectFunction_kernel103(const signed char x_data[3], int
  initStatus, double *validSampleTime);
static __global__ void detectFunction_kernel104(const unsigned short uv[256],
  const signed char thisCharcodes_1b_data[3], const int uv7_size[2], bool
  x_data[3]);
static __global__ void detectFunction_kernel105(const bool x_data[3], int
  *camIndex);
static __global__ void detectFunction_kernel106(const bool x_data[3], int
  initStatus, int *camIndex);
static __global__ void detectFunction_kernel107(const unsigned char uv11[10664],
  const int *camIndex, const int initStatus, unsigned char uv11_data[10664]);
static __global__ void detectFunction_kernel108(const unsigned char uv11_data
  [10664], const int uv11_size[2], const signed char num[2], unsigned char
  b_uv11_data[144]);
static __global__ void detectFunction_kernel109(const unsigned char uv11_data
  [144], const signed char num[2], unsigned char thisGlyphBitmap_data[144]);
static __global__ void detectFunction_kernel11(const double colWeights[5152],
  double colWeightsTotal[224]);
static __global__ void detectFunction_kernel110(const signed char iv5[261],
  const unsigned short uv[256], const signed char thisTextU16_data[3], const int
  i, int *cameraDevice);
static __global__ void detectFunction_kernel111(const unsigned char out[150528],
  unsigned char varargin_2[50176], unsigned char varargin_1[50176]);
static __global__ void detectFunction_kernel112(const unsigned char out[150528],
  unsigned char varargin_3[50176]);
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
static __global__ void detectFunction_kernel32(emxArray_cell_wrap_7_1024
  *labelCells);
static __global__ void detectFunction_kernel33(const float
  thresholdedPrediction_data[6144], const int thresholdedPrediction_size[2],
  const int bboxesX1Y1X2Y2_size[2], const int qY, double bboxesX1Y1X2Y2_data
  [4096]);
static __global__ void detectFunction_kernel34(const double bboxesX1Y1X2Y2_data
  [4096], const int qY, double x1_data[1024]);
static __global__ void detectFunction_kernel35(const double bboxesX1Y1X2Y2_data
  [4096], const int bboxesX1Y1X2Y2_size[2], const int qY, double y1_data[1024]);
static __global__ void detectFunction_kernel36(const double bboxesX1Y1X2Y2_data
  [4096], const int bboxesX1Y1X2Y2_size[2], const int qY, double x2_data[1024]);
static __global__ void detectFunction_kernel37(const double bboxesX1Y1X2Y2_data
  [4096], const int bboxesX1Y1X2Y2_size[2], const int qY, double y2_data[1024]);
static __global__ void detectFunction_kernel38(const int *cameraDevice, double
  x1_data[1024]);
static __global__ void detectFunction_kernel39(const int *cameraDevice, double
  y1_data[1024]);
static __global__ void detectFunction_kernel4(const unsigned char pln2[921600],
  unsigned char img[2764800]);
static __global__ void detectFunction_kernel40(const int *cameraDevice, double
  x2_data[1024]);
static __global__ void detectFunction_kernel41(const int *cameraDevice, double
  y2_data[1024]);
static __global__ void detectFunction_kernel42(const double x1_data[1024], const
  int initStatus, double bboxesX1Y1X2Y2_data[4096]);
static __global__ void detectFunction_kernel43(const double y1_data[1024], const
  int bboxesX1Y1X2Y2_size[2], const int initStatus, double bboxesX1Y1X2Y2_data
  [4096]);
static __global__ void detectFunction_kernel44(const double x2_data[1024], const
  int bboxesX1Y1X2Y2_size[2], const int initStatus, double bboxesX1Y1X2Y2_data
  [4096]);
static __global__ void detectFunction_kernel45(const double y2_data[1024], const
  int bboxesX1Y1X2Y2_size[2], const int initStatus, double bboxesX1Y1X2Y2_data
  [4096]);
static __global__ void detectFunction_kernel46(const double bboxesX1Y1X2Y2_data
  [4096], const int initStatus, double bboxPred_data[4096]);
static __global__ void detectFunction_kernel47(const double bboxesX1Y1X2Y2_data
  [4096], const int bboxesX1Y1X2Y2_size[2], const int bboxPred_size[2], const
  int initStatus, double bboxPred_data[4096]);
static __global__ void detectFunction_kernel48(const double bboxesX1Y1X2Y2_data
  [4096], const int bboxesX1Y1X2Y2_size[2], const int bboxPred_size[2], const
  int initStatus, double bboxPred_data[4096]);
static __global__ void detectFunction_kernel49(const double bboxesX1Y1X2Y2_data
  [4096], const int bboxesX1Y1X2Y2_size[2], const int bboxPred_size[2], const
  int initStatus, double bboxPred_data[4096]);
static __global__ void detectFunction_kernel5(short aux1[1440]);
static __global__ void detectFunction_kernel50(const int initStatus, double
  bboxPred_data[4096]);
static __global__ void detectFunction_kernel51(const double bboxPred_data[4096],
  const int bboxPred_size[2], const int qY, double x1_data[1024]);
static __global__ void detectFunction_kernel52(const double x1_data[1024], const
  int bboxPred_size[2], const int iv_size[1], double bboxPred_data[4096]);
static __global__ void detectFunction_kernel53(const double bboxPred_data[4096],
  const int bboxPred_size[2], const int qY, double x1_data[1024]);
static __global__ void detectFunction_kernel54(const double x1_data[1024], const
  int bboxPred_size[2], const int iv_size[1], double bboxPred_data[4096]);
static __global__ void detectFunction_kernel55(const double bboxPred_data[4096],
  const int bboxPred_size[2], const int i, const int b_bboxPred_size[2], const
  int *camIndex, double b_bboxPred_data[4096]);
static __global__ void detectFunction_kernel56(const float
  thresholdedPrediction_data[6144], const int thresholdedPrediction_size[2],
  const int i, const int count, float classPred_data[1024], float
  scorePred_data[1024]);
static __global__ void detectFunction_kernel57(const short outVal, const short i,
  short iv_data[1024]);
static __global__ void detectFunction_kernel58(const int bboxPred_size[2], bool
  b_data[1024]);
static __global__ void detectFunction_kernel59(const bool b_data[1024], int
  initStatus, int *camIndex);
static __global__ void detectFunction_kernel6(short aux2[2560]);
static __global__ void detectFunction_kernel60(const int bboxPred_size[2], const
  int initStatus, const short iv_data[1024], double bboxPred_data[4096]);
static __global__ void detectFunction_kernel61(const double bboxPred_data[4096],
  const int bboxPred_size[2], const int b_bboxPred_size[2], const int initStatus,
  double b_bboxPred_data[4096]);
static __global__ void detectFunction_kernel62(const double bboxPred_data[4096],
  const int bboxPred_size[2], double b_bboxPred_data[4096]);
static __global__ void detectFunction_kernel63(const short outVal, const short i,
  short iv_data[1024]);
static __global__ void detectFunction_kernel64(const int scorePred_size[1], bool
  b_data[1024]);
static __global__ void detectFunction_kernel65(const bool b_data[1024], int
  initStatus, int *camIndex);
static __global__ void detectFunction_kernel66(const short outVal, const short i,
  short iv_data[1024]);
static __global__ void detectFunction_kernel67(const int iv_size[1], bool
  b_data[1024]);
static __global__ void detectFunction_kernel68(const bool b_data[1024], int
  initStatus, int *camIndex);
static __global__ void detectFunction_kernel69(const float classPred_data[1024],
  const int iv_size[1], double y1_data[1024]);
static __global__ void detectFunction_kernel7(const short aux1[1440], short
  ipRowIndices[2912], double rowWeights[2912]);
static __global__ void detectFunction_kernel70(const float scorePred_data[1024],
  const int scorePred_size[1], float classPred_data[1024]);
static __global__ void detectFunction_kernel71(const short dv2[2], double
  idx_data[1024]);
static __global__ void detectFunction_kernel72(const double bboxPred_data[4096],
  const int bboxPred_size[2], const double idx_data[1024], const int
  inputBbox_size[2], const int idx_size[1], double bboxesX1Y1X2Y2_data[4096]);
static __global__ void detectFunction_kernel73(const double y1_data[1024], const
  double idx_data[1024], const int idx_size[1], double x1_data[1024]);
static __global__ void detectFunction_kernel74(const double x1_data[1024], const
  int iv_size[1], double y1_data[1024]);
static __global__ void detectFunction_kernel75(const int idx_size[1], bool
  b_data[1024]);
static __global__ void detectFunction_kernel76(const double bboxesX1Y1X2Y2_data
  [4096], const int inputBbox_size[2], const int qY, double x1_data[1024]);
static __global__ void detectFunction_kernel77(const int inputBbox_size[2],
  const double bboxesX1Y1X2Y2_data[4096], const int qY, double x2_data[1024]);
static __global__ void detectFunction_kernel78(const double bboxesX1Y1X2Y2_data
  [4096], const int inputBbox_size[2], const int qY, double y2_data[1024]);
static __global__ void detectFunction_kernel79(const int *camIndex, const int
  thresholdedPrediction_size[2], bool b_data[1024]);
static __global__ void detectFunction_kernel8(const short aux2[2560], short
  ipColIndices[5152], double colWeights[5152]);
static __global__ void detectFunction_kernel80(const bool b_data[1024], const
  double idx_data[1024], const int selectedIndex_size[1], bool index_data[1024]);
static __global__ void detectFunction_kernel81(const double bboxPred_data[4096],
  const int bboxPred_size[2], const short iv9_data[1024], const int
  b_bboxPred_size[2], const int iv_size[1], double b_bboxPred_data[4096]);
static __global__ void detectFunction_kernel82(const double bboxPred_data[4096],
  const int bboxPred_size[2], double b_bboxPred_data[4096]);
static __global__ void detectFunction_kernel83(const float scorePred_data[1024],
  const int scorePred_size[1], float scores_data[1024]);
static __global__ void detectFunction_kernel84(const int *oldIdx, int v_size[1]);
static __global__ void detectFunction_kernel85(const char cv3[3], const int
  initAuxVar, char v_data[3]);
static __global__ void detectFunction_kernel86(const char v_data[3], const int
  *i, const int v_size[1], emxArray_cell_wrap_7_1024 *labelCells);
static __global__ void detectFunction_kernel87(const unsigned char uv7[22591],
  const int *camIndex, const int initStatus, unsigned char uv7_data[22591]);
static __global__ void detectFunction_kernel88(const unsigned char uv7_data
  [22591], const int uv7_size[2], const signed char num[2], unsigned char
  b_uv7_data[324]);
static __global__ void detectFunction_kernel89(const unsigned char uv7_data[324],
  const signed char num[2], unsigned char thisGlyphBitmap_data[324]);
static __global__ void detectFunction_kernel9(const double rowWeights[2912],
  double rowWeightsTotal[224]);
static __global__ void detectFunction_kernel90(const double bboxPred_data[4096],
  const int bboxPred_size[2], const int initStatus, int position[4]);
static __global__ void detectFunction_kernel91(const unsigned char out[150528],
  unsigned char tmpRGB[150528]);
static __global__ void detectFunction_kernel92(const unsigned char uv5[3],
  unsigned char color[3]);
static __global__ void detectFunction_kernel93(const int position[4], int
  positionOut[4]);
static __global__ void detectFunction_kernel94(unsigned char out[150528]);
static __global__ void detectFunction_kernel95(unsigned char pixCount[224]);
static __global__ void detectFunction_kernel96(const int position[4], int
  positionOut[4]);
static __global__ void detectFunction_kernel97(const int c, int positionOut[4]);
static __global__ void detectFunction_kernel98(const unsigned char uv5[3],
  unsigned char color[3]);
static __global__ void detectFunction_kernel99(const emxArray_cell_wrap_7_1024
  *labelCells, const int *camIndex, signed char thisTextU16_data[3]);
static __device__ double rt_powd_snf_device(double u0, double u1);
static __device__ double rt_roundd_snf_device(double u);
static __device__ int shflDown1(int in1, unsigned int offset, unsigned int mask);
static __device__ unsigned int shflDown1(unsigned int in1, unsigned int offset,
  unsigned int mask);
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
  int oldIdx;
  oldIdx = static_cast<int>(mwGetGlobalThreadIndex());
  if (oldIdx < 224) {
    rowWeightsTotal[oldIdx] += rowWeights[initStatus + oldIdx];
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const signed char thisTextU16_data[3]
//                const int uv7_size[2]
//                signed char thisCharcodes_1b_data[3]
// Return Type  : void
//
static __global__ __launch_bounds__(1024, 1) void detectFunction_kernel100(const
  signed char thisTextU16_data[3], const int uv7_size[2], signed char
  thisCharcodes_1b_data[3])
{
  unsigned long idx;
  long loopEnd;
  unsigned long threadId;
  unsigned long threadStride;
  threadId = mwGetGlobalThreadIndex();
  threadStride = mwGetTotalThreadsLaunched();
  loopEnd = static_cast<long>(uv7_size[1] - 1);
  for (idx = threadId; idx <= static_cast<unsigned long>(loopEnd); idx +=
       threadStride) {
    int oldIdx;
    oldIdx = static_cast<int>(idx);
    thisCharcodes_1b_data[oldIdx] = static_cast<signed char>(static_cast<int>
      (thisTextU16_data[oldIdx]) + 1);
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const signed char iv5[261]
//                const unsigned short uv[256]
//                const signed char thisCharcodes_1b_data[3]
//                const int uv7_size[2]
//                signed char x_data[3]
// Return Type  : void
//
static __global__ __launch_bounds__(1024, 1) void detectFunction_kernel101(const
  signed char iv5[261], const unsigned short uv[256], const signed char
  thisCharcodes_1b_data[3], const int uv7_size[2], signed char x_data[3])
{
  unsigned long idx;
  long loopEnd;
  unsigned long threadId;
  unsigned long threadStride;
  threadId = mwGetGlobalThreadIndex();
  threadStride = mwGetTotalThreadsLaunched();
  loopEnd = static_cast<long>(uv7_size[1] - 1);
  for (idx = threadId; idx <= static_cast<unsigned long>(loopEnd); idx +=
       threadStride) {
    int oldIdx;
    oldIdx = static_cast<int>(idx);
    x_data[oldIdx] = iv5[uv[static_cast<int>(thisCharcodes_1b_data[oldIdx]) - 1]];
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const signed char iv5[261]
//                const unsigned short uv[256]
//                const signed char thisCharcodes_1b_data[3]
//                double *validSampleTime
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void detectFunction_kernel102(const
  signed char iv5[261], const unsigned short uv[256], const signed char
  thisCharcodes_1b_data[3], double *validSampleTime)
{
  int tmpIdx;
  tmpIdx = static_cast<int>(mwGetGlobalThreadIndex());
  if (tmpIdx < 1) {
    *validSampleTime = static_cast<double>(iv5[uv[static_cast<int>
      (thisCharcodes_1b_data[0]) - 1]]);
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const signed char x_data[3]
//                int initStatus
//                double *validSampleTime
// Return Type  : void
//
static __global__ __launch_bounds__(1024, 1) void detectFunction_kernel103(const
  signed char x_data[3], int initStatus, double *validSampleTime)
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
  loopEnd = static_cast<long>(initStatus - 2);
  if (mwIsLastBlock()) {
    m = (static_cast<long>(initStatus - 2) + 1L) % static_cast<long>(blockStride);
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
    atomicOpreal_T(&validSampleTime[0], tmpRed0);
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const unsigned short uv[256]
//                const signed char thisCharcodes_1b_data[3]
//                const int uv7_size[2]
//                bool x_data[3]
// Return Type  : void
//
static __global__ __launch_bounds__(1024, 1) void detectFunction_kernel104(const
  unsigned short uv[256], const signed char thisCharcodes_1b_data[3], const int
  uv7_size[2], bool x_data[3])
{
  unsigned long idx;
  long loopEnd;
  unsigned long threadId;
  unsigned long threadStride;
  threadId = mwGetGlobalThreadIndex();
  threadStride = mwGetTotalThreadsLaunched();
  loopEnd = static_cast<long>(uv7_size[1] - 1);
  for (idx = threadId; idx <= static_cast<unsigned long>(loopEnd); idx +=
       threadStride) {
    int oldIdx;
    oldIdx = static_cast<int>(idx);
    x_data[oldIdx] = (static_cast<int>(uv[static_cast<int>
      (thisCharcodes_1b_data[oldIdx]) - 1]) == 0);
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const bool x_data[3]
//                int *camIndex
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void detectFunction_kernel105(const
  bool x_data[3], int *camIndex)
{
  int tmpIdx;
  tmpIdx = static_cast<int>(mwGetGlobalThreadIndex());
  if (tmpIdx < 1) {
    *camIndex = static_cast<int>(x_data[0]);
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const bool x_data[3]
//                int initStatus
//                int *camIndex
// Return Type  : void
//
static __global__ __launch_bounds__(1024, 1) void detectFunction_kernel106(const
  bool x_data[3], int initStatus, int *camIndex)
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
  loopEnd = static_cast<long>(initStatus - 2);
  if (mwIsLastBlock()) {
    m = (static_cast<long>(initStatus - 2) + 1L) % static_cast<long>(blockStride);
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
    atomicAdd(&camIndex[0], tmpRed0);
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const unsigned char uv11[10664]
//                const int *camIndex
//                const int initStatus
//                unsigned char uv11_data[10664]
// Return Type  : void
//
static __global__ __launch_bounds__(1024, 1) void detectFunction_kernel107(const
  unsigned char uv11[10664], const int *camIndex, const int initStatus, unsigned
  char uv11_data[10664])
{
  unsigned long idx;
  long loopEnd;
  unsigned long threadId;
  unsigned long threadStride;
  threadId = mwGetGlobalThreadIndex();
  threadStride = mwGetTotalThreadsLaunched();
  loopEnd = static_cast<long>(initStatus - *camIndex);
  for (idx = threadId; idx <= static_cast<unsigned long>(loopEnd); idx +=
       threadStride) {
    int oldIdx;
    oldIdx = static_cast<int>(idx);
    uv11_data[oldIdx] = uv11[*camIndex + oldIdx];
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const unsigned char uv11_data[10664]
//                const int uv11_size[2]
//                const signed char num[2]
//                unsigned char b_uv11_data[144]
// Return Type  : void
//
static __global__ __launch_bounds__(1024, 1) void detectFunction_kernel108(const
  unsigned char uv11_data[10664], const int uv11_size[2], const signed char num
  [2], unsigned char b_uv11_data[144])
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
    int ind;
    int oldIdx;
    ind = static_cast<int>(idx % (static_cast<unsigned long>(static_cast<int>
      (num[1]) - 1) + 1UL));
    oldIdx = static_cast<int>((idx - static_cast<unsigned long>(ind)) / (
      static_cast<unsigned long>(static_cast<int>(num[1]) - 1) + 1UL));
    b_uv11_data[ind + uv11_size[0] * oldIdx] = uv11_data[oldIdx + static_cast<
      int>(num[0]) * ind];
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const unsigned char uv11_data[144]
//                const signed char num[2]
//                unsigned char thisGlyphBitmap_data[144]
// Return Type  : void
//
static __global__ __launch_bounds__(1024, 1) void detectFunction_kernel109(const
  unsigned char uv11_data[144], const signed char num[2], unsigned char
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
    int oldIdx;
    oldIdx = static_cast<int>(idx);
    thisGlyphBitmap_data[oldIdx] = uv11_data[oldIdx];
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
  int oldIdx;
  oldIdx = static_cast<int>(mwGetGlobalThreadIndex());
  if (oldIdx < 224) {
    colWeightsTotal[oldIdx] = colWeights[oldIdx];
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const signed char iv5[261]
//                const unsigned short uv[256]
//                const signed char thisTextU16_data[3]
//                const int i
//                int *cameraDevice
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void detectFunction_kernel110(const
  signed char iv5[261], const unsigned short uv[256], const signed char
  thisTextU16_data[3], const int i, int *cameraDevice)
{
  int tmpIdx;
  tmpIdx = static_cast<int>(mwGetGlobalThreadIndex());
  if (tmpIdx < 1) {
    *cameraDevice = static_cast<int>(iv5[uv[thisTextU16_data[i]]]);
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
static __global__ __launch_bounds__(512, 1) void detectFunction_kernel111(const
  unsigned char out[150528], unsigned char varargin_2[50176], unsigned char
  varargin_1[50176])
{
  unsigned long threadId;
  int ind;
  int oldIdx;
  threadId = mwGetGlobalThreadIndex();
  ind = static_cast<int>(threadId % 224UL);
  oldIdx = static_cast<int>((threadId - static_cast<unsigned long>(ind)) / 224UL);
  if ((static_cast<int>(oldIdx < 224)) && (static_cast<int>(ind < 224))) {
    varargin_1[ind + 224 * oldIdx] = out[oldIdx + 224 * ind];
    varargin_2[ind + 224 * oldIdx] = out[(oldIdx + 224 * ind) + 50176];
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const unsigned char out[150528]
//                unsigned char varargin_3[50176]
// Return Type  : void
//
static __global__ __launch_bounds__(512, 1) void detectFunction_kernel112(const
  unsigned char out[150528], unsigned char varargin_3[50176])
{
  unsigned long threadId;
  int ind;
  int oldIdx;
  threadId = mwGetGlobalThreadIndex();
  ind = static_cast<int>(threadId % 224UL);
  oldIdx = static_cast<int>((threadId - static_cast<unsigned long>(ind)) / 224UL);
  if ((static_cast<int>(oldIdx < 224)) && (static_cast<int>(ind < 224))) {
    varargin_3[ind + 224 * oldIdx] = out[(oldIdx + 224 * ind) + 100352];
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
  int oldIdx;
  oldIdx = static_cast<int>(mwGetGlobalThreadIndex());
  if (oldIdx < 224) {
    colWeightsTotal[oldIdx] += colWeights[initStatus + oldIdx];
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
  int b_i;
  int i;
  int oldIdx;
  threadId = mwGetGlobalThreadIndex();
  oldIdx = static_cast<int>(threadId % 3UL);
  threadId = (threadId - static_cast<unsigned long>(oldIdx)) / 3UL;
  i = static_cast<int>(threadId % 224UL);
  threadId = (threadId - static_cast<unsigned long>(i)) / 224UL;
  b_i = static_cast<int>(threadId);
  if ((static_cast<int>((static_cast<int>(b_i < 720)) && (static_cast<int>(i <
          224)))) && (static_cast<int>(oldIdx < 3))) {
    unsigned char u1;
    sumVal = 0.0;
    for (int ind = 0; ind < 23; ind++) {
      sumVal += static_cast<double>(img[(b_i + 720 * (static_cast<int>
        (ipColIndices[i + 224 * ind]) - 1)) + 921600 * oldIdx]) * (colWeights[i
        + 224 * ind] / colWeightsTotal[i]);
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

    partialResize[(b_i + 720 * i) + 161280 * oldIdx] = u1;
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
  int b_i;
  int i;
  int oldIdx;
  threadId = mwGetGlobalThreadIndex();
  oldIdx = static_cast<int>(threadId % 3UL);
  threadId = (threadId - static_cast<unsigned long>(oldIdx)) / 3UL;
  b_i = static_cast<int>(threadId % 224UL);
  threadId = (threadId - static_cast<unsigned long>(b_i)) / 224UL;
  i = static_cast<int>(threadId);
  if ((static_cast<int>((static_cast<int>(i < 224)) && (static_cast<int>(b_i <
          224)))) && (static_cast<int>(oldIdx < 3))) {
    unsigned char u1;
    sumVal = 0.0;
    for (int ind = 0; ind < 13; ind++) {
      sumVal += static_cast<double>(partialResize[((static_cast<int>
        (ipRowIndices[b_i + 224 * ind]) + 720 * i) + 161280 * oldIdx) - 1]) *
        (rowWeights[b_i + 224 * ind] / rowWeightsTotal[b_i]);
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

    out[(b_i + 224 * i) + 50176 * oldIdx] = u1;
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
  int k;
  threadId = mwGetGlobalThreadIndex();
  k = static_cast<int>(threadId % 7UL);
  i = static_cast<int>((threadId - static_cast<unsigned long>(k)) / 7UL);
  if ((static_cast<int>(i < 128)) && (static_cast<int>(k < 7))) {
    double absx;
    double absx2;
    double sumVal;
    int oldIdx;
    sumVal = (static_cast<double>(i) + 1.0) / 0.5714285714285714 + -0.375;
    oldIdx = static_cast<int>(floor(sumVal - 3.5));
    absx = fabs(0.5714285714285714 * (sumVal - (static_cast<double>(oldIdx + k)
      + 1.0)));
    absx2 = absx * absx;
    sumVal = rt_powd_snf_device(absx, 3.0);
    rowWeights[i + (k << 7)] = 0.5714285714285714 * (((1.5 * sumVal - 2.5 *
      absx2) + 1.0) * static_cast<double>(absx <= 1.0) + (((-0.5 * sumVal + 2.5 *
      absx2) - 4.0 * absx) + 2.0) * static_cast<double>((static_cast<int>(1.0 <
      absx)) && (static_cast<int>(absx <= 2.0))));
    oldIdx = (oldIdx + k) + 1;
    if (oldIdx - 1 == 0) {
      ind = 0;
    } else {
      ind = static_cast<int>(fmod(static_cast<double>(oldIdx) - 1.0, 448.0));
      if ((static_cast<int>(ind != 0)) && (static_cast<int>(oldIdx - 1 < 0))) {
        ind += 448;
      }
    }

    ipRowIndices[i + (k << 7)] = aux1[ind];
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
  int i;
  int ind;
  int k;
  threadId = mwGetGlobalThreadIndex();
  k = static_cast<int>(threadId % 7UL);
  i = static_cast<int>((threadId - static_cast<unsigned long>(k)) / 7UL);
  if ((static_cast<int>(i < 128)) && (static_cast<int>(k < 7))) {
    double absx;
    double absx2;
    double sumVal;
    int oldIdx;
    sumVal = (static_cast<double>(i) + 1.0) / 0.5714285714285714 + -0.375;
    oldIdx = static_cast<int>(floor(sumVal - 3.5));
    absx = fabs(0.5714285714285714 * (sumVal - (static_cast<double>(oldIdx + k)
      + 1.0)));
    absx2 = absx * absx;
    sumVal = rt_powd_snf_device(absx, 3.0);
    colWeights[i + (k << 7)] = 0.5714285714285714 * (((1.5 * sumVal - 2.5 *
      absx2) + 1.0) * static_cast<double>(absx <= 1.0) + (((-0.5 * sumVal + 2.5 *
      absx2) - 4.0 * absx) + 2.0) * static_cast<double>((static_cast<int>(1.0 <
      absx)) && (static_cast<int>(absx <= 2.0))));
    oldIdx = (oldIdx + k) + 1;
    if (oldIdx - 1 == 0) {
      ind = 0;
    } else {
      ind = static_cast<int>(fmod(static_cast<double>(oldIdx) - 1.0, 448.0));
      if ((static_cast<int>(ind != 0)) && (static_cast<int>(oldIdx - 1 < 0))) {
        ind += 448;
      }
    }

    ipColIndices[i + (k << 7)] = aux2[ind];
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
  int oldIdx;
  oldIdx = static_cast<int>(mwGetGlobalThreadIndex());
  if (oldIdx < 128) {
    rowWeightsTotal[oldIdx] = rowWeights[oldIdx];
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
  int oldIdx;
  oldIdx = static_cast<int>(mwGetGlobalThreadIndex());
  if (oldIdx < 128) {
    rowWeightsTotal[oldIdx] += rowWeights[initStatus + oldIdx];
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
  int oldIdx;
  oldIdx = static_cast<int>(mwGetGlobalThreadIndex());
  if (oldIdx < 128) {
    colWeightsTotal[oldIdx] = colWeights[oldIdx];
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
  int oldIdx;
  oldIdx = static_cast<int>(mwGetGlobalThreadIndex());
  if (oldIdx < 128) {
    colWeightsTotal[oldIdx] += colWeights[initStatus + oldIdx];
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
  int b_i;
  int i;
  int oldIdx;
  threadId = mwGetGlobalThreadIndex();
  oldIdx = static_cast<int>(threadId % 3UL);
  threadId = (threadId - static_cast<unsigned long>(oldIdx)) / 3UL;
  b_i = static_cast<int>(threadId % 128UL);
  threadId = (threadId - static_cast<unsigned long>(b_i)) / 128UL;
  i = static_cast<int>(threadId);
  if ((static_cast<int>((static_cast<int>(i < 224)) && (static_cast<int>(b_i <
          128)))) && (static_cast<int>(oldIdx < 3))) {
    unsigned char u1;
    sumVal = 0.0;
    for (int ind = 0; ind < 7; ind++) {
      sumVal += static_cast<double>(out[((static_cast<int>(ipRowIndices[b_i +
        (ind << 7)]) + 224 * i) + 50176 * oldIdx) - 1]) * (rowWeights[b_i + (ind
        << 7)] / rowWeightsTotal[b_i]);
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

    partialResize[(b_i + (i << 7)) + 28672 * oldIdx] = u1;
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
  int b_i;
  int i;
  int oldIdx;
  threadId = mwGetGlobalThreadIndex();
  oldIdx = static_cast<int>(threadId % 3UL);
  threadId = (threadId - static_cast<unsigned long>(oldIdx)) / 3UL;
  b_i = static_cast<int>(threadId % 128UL);
  threadId = (threadId - static_cast<unsigned long>(b_i)) / 128UL;
  i = static_cast<int>(threadId);
  if ((static_cast<int>((static_cast<int>(i < 128)) && (static_cast<int>(b_i <
          128)))) && (static_cast<int>(oldIdx < 3))) {
    unsigned char u1;
    sumVal = 0.0;
    for (int ind = 0; ind < 7; ind++) {
      sumVal += static_cast<double>(partialResize[(b_i + ((static_cast<int>
        (ipColIndices[i + (ind << 7)]) - 1) << 7)) + 28672 * oldIdx]) *
        (colWeights[i + (ind << 7)] / colWeightsTotal[i]);
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

    out[(b_i + (i << 7)) + (oldIdx << 14)] = u1;
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
  int b_i;
  int i;
  int oldIdx;
  threadId = mwGetGlobalThreadIndex();
  b_i = static_cast<int>(threadId % 16UL);
  threadId = (threadId - static_cast<unsigned long>(b_i)) / 16UL;
  i = static_cast<int>(threadId % 16UL);
  threadId = (threadId - static_cast<unsigned long>(i)) / 16UL;
  oldIdx = static_cast<int>(threadId);
  if ((static_cast<int>((static_cast<int>(oldIdx < 4)) && (static_cast<int>(i <
          16)))) && (static_cast<int>(b_i < 16))) {
    float bh;
    float bw;
    float cx;
    float cy;
    int ind;
    ind = (((b_i << 6) + (i << 2)) + oldIdx) + 1;
    cx = (tmpFeatureMap[((b_i + (i << 4)) + (oldIdx << 8)) + 1024] +
          static_cast<float>(i)) * 8.0F;
    cy = (tmpFeatureMap[((b_i + (i << 4)) + (oldIdx << 8)) + 2048] +
          static_cast<float>(b_i)) * 8.0F;
    bw = tmpFeatureMap[((b_i + (i << 4)) + (oldIdx << 8)) + 3072] * static_cast<
      float>(anchors[oldIdx + 4]) * 8.0F;
    bh = tmpFeatureMap[((b_i + (i << 4)) + (oldIdx << 8)) + 4096] * static_cast<
      float>(anchors[oldIdx]) * 8.0F;
    boxOut[ind - 1] = cx - bw / 2.0F;
    boxOut[ind + 1023] = cy - bh / 2.0F;
    boxOut[ind + 2047] = cx + bw / 2.0F;
    boxOut[ind + 3071] = cy + bh / 2.0F;
    boxOut[ind + 4095] = tmpFeatureMap[(b_i + (i << 4)) + (oldIdx << 8)] *
      tmpFeatureMap[((b_i + (i << 4)) + (oldIdx << 8)) + 5120];
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
//                emxArray_cell_wrap_7_1024 *labelCells
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void detectFunction_kernel32
  (emxArray_cell_wrap_7_1024 *labelCells)
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
//                const float thresholdedPrediction_data[6144]
//                const int thresholdedPrediction_size[2]
//                const int bboxesX1Y1X2Y2_size[2]
//                const int qY
//                double bboxesX1Y1X2Y2_data[4096]
// Return Type  : void
//
static __global__ __launch_bounds__(1024, 1) void detectFunction_kernel33(const
  float thresholdedPrediction_data[6144], const int thresholdedPrediction_size[2],
  const int bboxesX1Y1X2Y2_size[2], const int qY, double bboxesX1Y1X2Y2_data
  [4096])
{
  unsigned long idx;
  long loopEnd;
  unsigned long threadId;
  unsigned long threadStride;
  threadId = mwGetGlobalThreadIndex();
  threadStride = mwGetTotalThreadsLaunched();
  loopEnd = (static_cast<long>(qY) + 1L) * 4L - 1L;
  for (idx = threadId; idx <= static_cast<unsigned long>(loopEnd); idx +=
       threadStride) {
    int ind;
    int oldIdx;
    ind = static_cast<int>(idx % (static_cast<unsigned long>(qY) + 1UL));
    oldIdx = static_cast<int>((idx - static_cast<unsigned long>(ind)) / (
      static_cast<unsigned long>(qY) + 1UL));
    bboxesX1Y1X2Y2_data[ind + bboxesX1Y1X2Y2_size[0] * oldIdx] = static_cast<
      double>(thresholdedPrediction_data[ind + thresholdedPrediction_size[0] *
              oldIdx]);
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double bboxesX1Y1X2Y2_data[4096]
//                const int qY
//                double x1_data[1024]
// Return Type  : void
//
static __global__ __launch_bounds__(1024, 1) void detectFunction_kernel34(const
  double bboxesX1Y1X2Y2_data[4096], const int qY, double x1_data[1024])
{
  unsigned long idx;
  long loopEnd;
  unsigned long threadId;
  unsigned long threadStride;
  threadId = mwGetGlobalThreadIndex();
  threadStride = mwGetTotalThreadsLaunched();
  loopEnd = static_cast<long>(qY);
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
//                const int qY
//                double y1_data[1024]
// Return Type  : void
//
static __global__ __launch_bounds__(1024, 1) void detectFunction_kernel35(const
  double bboxesX1Y1X2Y2_data[4096], const int bboxesX1Y1X2Y2_size[2], const int
  qY, double y1_data[1024])
{
  unsigned long idx;
  long loopEnd;
  unsigned long threadId;
  unsigned long threadStride;
  threadId = mwGetGlobalThreadIndex();
  threadStride = mwGetTotalThreadsLaunched();
  loopEnd = static_cast<long>(qY);
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
//                const int qY
//                double x2_data[1024]
// Return Type  : void
//
static __global__ __launch_bounds__(1024, 1) void detectFunction_kernel36(const
  double bboxesX1Y1X2Y2_data[4096], const int bboxesX1Y1X2Y2_size[2], const int
  qY, double x2_data[1024])
{
  unsigned long idx;
  long loopEnd;
  unsigned long threadId;
  unsigned long threadStride;
  threadId = mwGetGlobalThreadIndex();
  threadStride = mwGetTotalThreadsLaunched();
  loopEnd = static_cast<long>(qY);
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
//                const int qY
//                double y2_data[1024]
// Return Type  : void
//
static __global__ __launch_bounds__(1024, 1) void detectFunction_kernel37(const
  double bboxesX1Y1X2Y2_data[4096], const int bboxesX1Y1X2Y2_size[2], const int
  qY, double y2_data[1024])
{
  unsigned long idx;
  long loopEnd;
  unsigned long threadId;
  unsigned long threadStride;
  threadId = mwGetGlobalThreadIndex();
  threadStride = mwGetTotalThreadsLaunched();
  loopEnd = static_cast<long>(qY);
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
//                const int *cameraDevice
//                double x1_data[1024]
// Return Type  : void
//
static __global__ __launch_bounds__(1024, 1) void detectFunction_kernel38(const
  int *cameraDevice, double x1_data[1024])
{
  unsigned long idx;
  long loopEnd;
  unsigned long threadId;
  unsigned long threadStride;
  threadId = mwGetGlobalThreadIndex();
  threadStride = mwGetTotalThreadsLaunched();
  loopEnd = static_cast<long>(*cameraDevice - 1);
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
//                const int *cameraDevice
//                double y1_data[1024]
// Return Type  : void
//
static __global__ __launch_bounds__(1024, 1) void detectFunction_kernel39(const
  int *cameraDevice, double y1_data[1024])
{
  unsigned long idx;
  long loopEnd;
  unsigned long threadId;
  unsigned long threadStride;
  threadId = mwGetGlobalThreadIndex();
  threadStride = mwGetTotalThreadsLaunched();
  loopEnd = static_cast<long>(*cameraDevice - 1);
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
//                const int *cameraDevice
//                double x2_data[1024]
// Return Type  : void
//
static __global__ __launch_bounds__(1024, 1) void detectFunction_kernel40(const
  int *cameraDevice, double x2_data[1024])
{
  unsigned long idx;
  long loopEnd;
  unsigned long threadId;
  unsigned long threadStride;
  threadId = mwGetGlobalThreadIndex();
  threadStride = mwGetTotalThreadsLaunched();
  loopEnd = static_cast<long>(*cameraDevice - 1);
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
//                const int *cameraDevice
//                double y2_data[1024]
// Return Type  : void
//
static __global__ __launch_bounds__(1024, 1) void detectFunction_kernel41(const
  int *cameraDevice, double y2_data[1024])
{
  unsigned long idx;
  long loopEnd;
  unsigned long threadId;
  unsigned long threadStride;
  threadId = mwGetGlobalThreadIndex();
  threadStride = mwGetTotalThreadsLaunched();
  loopEnd = static_cast<long>(*cameraDevice - 1);
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
static __global__ __launch_bounds__(1024, 1) void detectFunction_kernel42(const
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
static __global__ __launch_bounds__(1024, 1) void detectFunction_kernel43(const
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
static __global__ __launch_bounds__(1024, 1) void detectFunction_kernel44(const
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
static __global__ __launch_bounds__(1024, 1) void detectFunction_kernel45(const
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
static __global__ __launch_bounds__(1024, 1) void detectFunction_kernel46(const
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
static __global__ __launch_bounds__(1024, 1) void detectFunction_kernel49(const
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
//                const int initStatus
//                double bboxPred_data[4096]
// Return Type  : void
//
static __global__ __launch_bounds__(1024, 1) void detectFunction_kernel50(const
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
    int k;
    k = static_cast<int>(idx);
    bboxPred_data[k] = floor(bboxPred_data[k]);
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double bboxPred_data[4096]
//                const int bboxPred_size[2]
//                const int qY
//                double x1_data[1024]
// Return Type  : void
//
static __global__ __launch_bounds__(1024, 1) void detectFunction_kernel51(const
  double bboxPred_data[4096], const int bboxPred_size[2], const int qY, double
  x1_data[1024])
{
  unsigned long idx;
  long loopEnd;
  unsigned long threadId;
  unsigned long threadStride;
  threadId = mwGetGlobalThreadIndex();
  threadStride = mwGetTotalThreadsLaunched();
  loopEnd = static_cast<long>(qY);
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
static __global__ __launch_bounds__(1024, 1) void detectFunction_kernel52(const
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
//                const int qY
//                double x1_data[1024]
// Return Type  : void
//
static __global__ __launch_bounds__(1024, 1) void detectFunction_kernel53(const
  double bboxPred_data[4096], const int bboxPred_size[2], const int qY, double
  x1_data[1024])
{
  unsigned long idx;
  long loopEnd;
  unsigned long threadId;
  unsigned long threadStride;
  threadId = mwGetGlobalThreadIndex();
  threadStride = mwGetTotalThreadsLaunched();
  loopEnd = static_cast<long>(qY);
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
static __global__ __launch_bounds__(1024, 1) void detectFunction_kernel54(const
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
//                const int i
//                const int b_bboxPred_size[2]
//                const int *camIndex
//                double b_bboxPred_data[4096]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void detectFunction_kernel55(const
  double bboxPred_data[4096], const int bboxPred_size[2], const int i, const int
  b_bboxPred_size[2], const int *camIndex, double b_bboxPred_data[4096])
{
  int oldIdx;
  oldIdx = static_cast<int>(mwGetGlobalThreadIndex());
  if (oldIdx < 4) {
    b_bboxPred_data[*camIndex + b_bboxPred_size[0] * oldIdx] = bboxPred_data[i +
      bboxPred_size[0] * oldIdx];
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const float thresholdedPrediction_data[6144]
//                const int thresholdedPrediction_size[2]
//                const int i
//                const int count
//                float classPred_data[1024]
//                float scorePred_data[1024]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void detectFunction_kernel56(const
  float thresholdedPrediction_data[6144], const int thresholdedPrediction_size[2],
  const int i, const int count, float classPred_data[1024], float
  scorePred_data[1024])
{
  int tmpIdx;
  tmpIdx = static_cast<int>(mwGetGlobalThreadIndex());
  if (tmpIdx < 1) {
    scorePred_data[count - 1] = thresholdedPrediction_data[i +
      (thresholdedPrediction_size[0] << 2)];
    classPred_data[count - 1] = thresholdedPrediction_data[i +
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
static __global__ __launch_bounds__(1024, 1) void detectFunction_kernel57(const
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
static __global__ __launch_bounds__(1024, 1) void detectFunction_kernel58(const
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
static __global__ __launch_bounds__(1024, 1) void detectFunction_kernel59(const
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
//                const int bboxPred_size[2]
//                const int initStatus
//                const short iv_data[1024]
//                double bboxPred_data[4096]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void detectFunction_kernel60(const
  int bboxPred_size[2], const int initStatus, const short iv_data[1024], double
  bboxPred_data[4096])
{
  int oldIdx;
  oldIdx = static_cast<int>(mwGetGlobalThreadIndex());
  if (oldIdx < 4) {
    int ind;
    ind = static_cast<int>(iv_data[0]);
    for (int i = 0; i <= initStatus - ind; i++) {
      int b_i;
      b_i = ind + i;
      bboxPred_data[(b_i + bboxPred_size[0] * oldIdx) - 1] = bboxPred_data[b_i +
        bboxPred_size[0] * oldIdx];
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
static __global__ __launch_bounds__(1024, 1) void detectFunction_kernel61(const
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
static __global__ __launch_bounds__(1024, 1) void detectFunction_kernel62(const
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
static __global__ __launch_bounds__(1024, 1) void detectFunction_kernel63(const
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
static __global__ __launch_bounds__(1024, 1) void detectFunction_kernel64(const
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
static __global__ __launch_bounds__(1024, 1) void detectFunction_kernel65(const
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
static __global__ __launch_bounds__(1024, 1) void detectFunction_kernel66(const
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
static __global__ __launch_bounds__(1024, 1) void detectFunction_kernel67(const
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
static __global__ __launch_bounds__(1024, 1) void detectFunction_kernel68(const
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
static __global__ __launch_bounds__(1024, 1) void detectFunction_kernel69(const
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
  int k;
  threadId = mwGetGlobalThreadIndex();
  k = static_cast<int>(threadId % 13UL);
  i = static_cast<int>((threadId - static_cast<unsigned long>(k)) / 13UL);
  if ((static_cast<int>(i < 224)) && (static_cast<int>(k < 13))) {
    double absx;
    double absx2;
    double sumVal;
    int oldIdx;
    sumVal = (static_cast<double>(i) + 1.0) / 0.31111111111111112 +
      -1.1071428571428572;
    oldIdx = static_cast<int>(floor(sumVal - 6.4285714285714288));
    absx = fabs(0.31111111111111112 * (sumVal - (static_cast<double>(oldIdx + k)
      + 1.0)));
    absx2 = absx * absx;
    sumVal = rt_powd_snf_device(absx, 3.0);
    rowWeights[i + 224 * k] = 0.31111111111111112 * (((1.5 * sumVal - 2.5 *
      absx2) + 1.0) * static_cast<double>(absx <= 1.0) + (((-0.5 * sumVal + 2.5 *
      absx2) - 4.0 * absx) + 2.0) * static_cast<double>((static_cast<int>(1.0 <
      absx)) && (static_cast<int>(absx <= 2.0))));
    oldIdx = (oldIdx + k) + 1;
    if (oldIdx - 1 == 0) {
      ind = 0;
    } else {
      ind = static_cast<int>(fmod(static_cast<double>(oldIdx) - 1.0, 1440.0));
      if ((static_cast<int>(ind != 0)) && (static_cast<int>(oldIdx - 1 < 0))) {
        ind += 1440;
      }
    }

    ipRowIndices[i + 224 * k] = aux1[ind];
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
static __global__ __launch_bounds__(1024, 1) void detectFunction_kernel70(const
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
//                const short dv2[2]
//                double idx_data[1024]
// Return Type  : void
//
static __global__ __launch_bounds__(1024, 1) void detectFunction_kernel71(const
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
static __global__ __launch_bounds__(1024, 1) void detectFunction_kernel72(const
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
static __global__ __launch_bounds__(1024, 1) void detectFunction_kernel73(const
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
static __global__ __launch_bounds__(1024, 1) void detectFunction_kernel74(const
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
static __global__ __launch_bounds__(1024, 1) void detectFunction_kernel75(const
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
//                const int qY
//                double x1_data[1024]
// Return Type  : void
//
static __global__ __launch_bounds__(1024, 1) void detectFunction_kernel76(const
  double bboxesX1Y1X2Y2_data[4096], const int inputBbox_size[2], const int qY,
  double x1_data[1024])
{
  unsigned long idx;
  long loopEnd;
  unsigned long threadId;
  unsigned long threadStride;
  threadId = mwGetGlobalThreadIndex();
  threadStride = mwGetTotalThreadsLaunched();
  loopEnd = static_cast<long>(qY);
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
//                const int qY
//                double x2_data[1024]
// Return Type  : void
//
static __global__ __launch_bounds__(1024, 1) void detectFunction_kernel77(const
  int inputBbox_size[2], const double bboxesX1Y1X2Y2_data[4096], const int qY,
  double x2_data[1024])
{
  unsigned long idx;
  long loopEnd;
  unsigned long threadId;
  unsigned long threadStride;
  threadId = mwGetGlobalThreadIndex();
  threadStride = mwGetTotalThreadsLaunched();
  loopEnd = static_cast<long>(qY);
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
//                const int qY
//                double y2_data[1024]
// Return Type  : void
//
static __global__ __launch_bounds__(1024, 1) void detectFunction_kernel78(const
  double bboxesX1Y1X2Y2_data[4096], const int inputBbox_size[2], const int qY,
  double y2_data[1024])
{
  unsigned long idx;
  long loopEnd;
  unsigned long threadId;
  unsigned long threadStride;
  threadId = mwGetGlobalThreadIndex();
  threadStride = mwGetTotalThreadsLaunched();
  loopEnd = static_cast<long>(qY);
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
static __global__ __launch_bounds__(1024, 1) void detectFunction_kernel79(const
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
//                const short aux2[2560]
//                short ipColIndices[5152]
//                double colWeights[5152]
// Return Type  : void
//
static __global__ __launch_bounds__(512, 1) void detectFunction_kernel8(const
  short aux2[2560], short ipColIndices[5152], double colWeights[5152])
{
  unsigned long threadId;
  int i;
  int ind;
  int k;
  threadId = mwGetGlobalThreadIndex();
  k = static_cast<int>(threadId % 23UL);
  i = static_cast<int>((threadId - static_cast<unsigned long>(k)) / 23UL);
  if ((static_cast<int>(i < 224)) && (static_cast<int>(k < 23))) {
    double absx;
    double absx2;
    double sumVal;
    int oldIdx;
    sumVal = (static_cast<double>(i) + 1.0) / 0.175 + -2.3571428571428572;
    oldIdx = static_cast<int>(floor(sumVal - 11.428571428571429));
    absx = fabs(0.175 * (sumVal - (static_cast<double>(oldIdx + k) + 1.0)));
    absx2 = absx * absx;
    sumVal = rt_powd_snf_device(absx, 3.0);
    colWeights[i + 224 * k] = 0.175 * (((1.5 * sumVal - 2.5 * absx2) + 1.0) *
      static_cast<double>(absx <= 1.0) + (((-0.5 * sumVal + 2.5 * absx2) - 4.0 *
      absx) + 2.0) * static_cast<double>((static_cast<int>(1.0 < absx)) && (
      static_cast<int>(absx <= 2.0))));
    oldIdx = (oldIdx + k) + 1;
    if (oldIdx - 1 == 0) {
      ind = 0;
    } else {
      ind = static_cast<int>(fmod(static_cast<double>(oldIdx) - 1.0, 2560.0));
      if ((static_cast<int>(ind != 0)) && (static_cast<int>(oldIdx - 1 < 0))) {
        ind += 2560;
      }
    }

    ipColIndices[i + 224 * k] = aux2[ind];
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
static __global__ __launch_bounds__(1024, 1) void detectFunction_kernel80(const
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
//                const double bboxPred_data[4096]
//                const int bboxPred_size[2]
//                const short iv9_data[1024]
//                const int b_bboxPred_size[2]
//                const int iv_size[1]
//                double b_bboxPred_data[4096]
// Return Type  : void
//
static __global__ __launch_bounds__(1024, 1) void detectFunction_kernel81(const
  double bboxPred_data[4096], const int bboxPred_size[2], const short iv9_data
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
      static_cast<int>(iv9_data[ind]) + bboxPred_size[0] * oldIdx) - 1];
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
static __global__ __launch_bounds__(1024, 1) void detectFunction_kernel82(const
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
static __global__ __launch_bounds__(1024, 1) void detectFunction_kernel83(const
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
//                const int *oldIdx
//                int v_size[1]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void detectFunction_kernel84(const
  int *oldIdx, int v_size[1])
{
  int tmpIdx;
  tmpIdx = static_cast<int>(mwGetGlobalThreadIndex());
  if (tmpIdx < 1) {
    v_size[0] = *oldIdx;
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
//                emxArray_cell_wrap_7_1024 *labelCells
// Return Type  : void
//
static __global__ __launch_bounds__(1024, 1) void detectFunction_kernel86(const
  char v_data[3], const int *i, const int v_size[1], emxArray_cell_wrap_7_1024
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
    int oldIdx;
    oldIdx = static_cast<int>(idx);
    labelCells->data[*i].f1.data[oldIdx] = v_data[oldIdx];
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const unsigned char uv7[22591]
//                const int *camIndex
//                const int initStatus
//                unsigned char uv7_data[22591]
// Return Type  : void
//
static __global__ __launch_bounds__(1024, 1) void detectFunction_kernel87(const
  unsigned char uv7[22591], const int *camIndex, const int initStatus, unsigned
  char uv7_data[22591])
{
  unsigned long idx;
  long loopEnd;
  unsigned long threadId;
  unsigned long threadStride;
  threadId = mwGetGlobalThreadIndex();
  threadStride = mwGetTotalThreadsLaunched();
  loopEnd = static_cast<long>(initStatus - *camIndex);
  for (idx = threadId; idx <= static_cast<unsigned long>(loopEnd); idx +=
       threadStride) {
    int oldIdx;
    oldIdx = static_cast<int>(idx);
    uv7_data[oldIdx] = uv7[*camIndex + oldIdx];
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const unsigned char uv7_data[22591]
//                const int uv7_size[2]
//                const signed char num[2]
//                unsigned char b_uv7_data[324]
// Return Type  : void
//
static __global__ __launch_bounds__(1024, 1) void detectFunction_kernel88(const
  unsigned char uv7_data[22591], const int uv7_size[2], const signed char num[2],
  unsigned char b_uv7_data[324])
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
    int ind;
    int oldIdx;
    ind = static_cast<int>(idx % (static_cast<unsigned long>(static_cast<int>
      (num[1]) - 1) + 1UL));
    oldIdx = static_cast<int>((idx - static_cast<unsigned long>(ind)) / (
      static_cast<unsigned long>(static_cast<int>(num[1]) - 1) + 1UL));
    b_uv7_data[ind + uv7_size[0] * oldIdx] = uv7_data[oldIdx + static_cast<int>
      (num[0]) * ind];
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const unsigned char uv7_data[324]
//                const signed char num[2]
//                unsigned char thisGlyphBitmap_data[324]
// Return Type  : void
//
static __global__ __launch_bounds__(1024, 1) void detectFunction_kernel89(const
  unsigned char uv7_data[324], const signed char num[2], unsigned char
  thisGlyphBitmap_data[324])
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
    int oldIdx;
    oldIdx = static_cast<int>(idx);
    thisGlyphBitmap_data[oldIdx] = uv7_data[oldIdx];
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
  int oldIdx;
  oldIdx = static_cast<int>(mwGetGlobalThreadIndex());
  if (oldIdx < 224) {
    rowWeightsTotal[oldIdx] = rowWeights[oldIdx];
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double bboxPred_data[4096]
//                const int bboxPred_size[2]
//                const int initStatus
//                int position[4]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void detectFunction_kernel90(const
  double bboxPred_data[4096], const int bboxPred_size[2], const int initStatus,
  int position[4])
{
  int oldIdx;
  oldIdx = static_cast<int>(mwGetGlobalThreadIndex());
  if (oldIdx < 4) {
    double sumVal;
    int ind;
    sumVal = rt_roundd_snf_device(bboxPred_data[initStatus + bboxPred_size[0] *
      oldIdx]);
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

    position[oldIdx] = ind;
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const unsigned char out[150528]
//                unsigned char tmpRGB[150528]
// Return Type  : void
//
static __global__ __launch_bounds__(512, 1) void detectFunction_kernel91(const
  unsigned char out[150528], unsigned char tmpRGB[150528])
{
  int oldIdx;
  oldIdx = static_cast<int>(mwGetGlobalThreadIndex());
  if (oldIdx < 150528) {
    tmpRGB[oldIdx] = out[oldIdx];
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const unsigned char uv5[3]
//                unsigned char color[3]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void detectFunction_kernel92(const
  unsigned char uv5[3], unsigned char color[3])
{
  int oldIdx;
  oldIdx = static_cast<int>(mwGetGlobalThreadIndex());
  if (oldIdx < 3) {
    color[oldIdx] = uv5[oldIdx];
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
  int oldIdx;
  oldIdx = static_cast<int>(mwGetGlobalThreadIndex());
  if (oldIdx < 4) {
    positionOut[oldIdx] = position[oldIdx];
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                unsigned char out[150528]
// Return Type  : void
//
static __global__ __launch_bounds__(512, 1) void detectFunction_kernel94
  (unsigned char out[150528])
{
  int oldIdx;
  oldIdx = static_cast<int>(mwGetGlobalThreadIndex());
  if (oldIdx < 150528) {
    out[oldIdx] = static_cast<unsigned char>(0U);
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                unsigned char pixCount[224]
// Return Type  : void
//
static __global__ __launch_bounds__(224, 1) void detectFunction_kernel95
  (unsigned char pixCount[224])
{
  int oldIdx;
  oldIdx = static_cast<int>(mwGetGlobalThreadIndex());
  if (oldIdx < 224) {
    pixCount[oldIdx] = static_cast<unsigned char>(0U);
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const int position[4]
//                int positionOut[4]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void detectFunction_kernel96(const
  int position[4], int positionOut[4])
{
  int oldIdx;
  oldIdx = static_cast<int>(mwGetGlobalThreadIndex());
  if (oldIdx < 4) {
    positionOut[oldIdx] = position[oldIdx];
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const int c
//                int positionOut[4]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void detectFunction_kernel97(const
  int c, int positionOut[4])
{
  int tmpIdx;
  tmpIdx = static_cast<int>(mwGetGlobalThreadIndex());
  if (tmpIdx < 1) {
    positionOut[1] = c;
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const unsigned char uv5[3]
//                unsigned char color[3]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void detectFunction_kernel98(const
  unsigned char uv5[3], unsigned char color[3])
{
  int oldIdx;
  oldIdx = static_cast<int>(mwGetGlobalThreadIndex());
  if (oldIdx < 3) {
    color[oldIdx] = uv5[oldIdx];
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const emxArray_cell_wrap_7_1024 *labelCells
//                const int *camIndex
//                signed char thisTextU16_data[3]
// Return Type  : void
//
static __global__ __launch_bounds__(1024, 1) void detectFunction_kernel99(const
  emxArray_cell_wrap_7_1024 *labelCells, const int *camIndex, signed char
  thisTextU16_data[3])
{
  unsigned long idx;
  long loopEnd;
  unsigned long threadId;
  unsigned long threadStride;
  threadId = mwGetGlobalThreadIndex();
  threadStride = mwGetTotalThreadsLaunched();
  loopEnd = static_cast<long>(labelCells->data[*camIndex - 1].f1.size[1] - 1);
  for (idx = threadId; idx <= static_cast<unsigned long>(loopEnd); idx +=
       threadStride) {
    int oldIdx;
    oldIdx = static_cast<int>(idx);
    thisTextU16_data[oldIdx] = static_cast<signed char>(labelCells->data
      [*camIndex - 1].f1.data[oldIdx]);
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
// Arguments    : void
// Return Type  : void
//
void detectFunction()
{
  static const double dv1[4] = { 1.25, 6.25, 2.375, 5.0 };

  static float b_out[49152];
  static const short uv10[261] = { 0, 0, 0, 56, 56, 74, 86, 158, 224, 296, 368,
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

  static const short uv6[261] = { 0, 0, 0, 120, 120, 159, 183, 339, 474, 630,
    786, 802, 882, 962, 1018, 1150, 1165, 1174, 1180, 1292, 1422, 1526, 1643,
    1760, 1903, 2007, 2137, 2267, 2397, 2527, 2557, 2596, 2728, 2776, 2908, 3012,
    3207, 3376, 3493, 3649, 3805, 3922, 4026, 4182, 4325, 4364, 4460, 4603, 4720,
    4889, 5032, 5214, 5331, 5571, 5714, 5831, 5987, 6117, 6261, 6469, 6612, 6744,
    6887, 6967, 7079, 7159, 7269, 7278, 7293, 7393, 7523, 7613, 7743, 7833, 7937,
    8077, 8194, 8233, 8335, 8465, 8504, 8654, 8744, 8854, 8994, 9134, 9194, 9264,
    9348, 9438, 9538, 9678, 9788, 9928, 10018, 10098, 10146, 10226, 13693, 13901,
    14314, 14671, 15688, 16802, 17838, 18552, 18412, 18692, 18962, 18832, 19092,
    19392, 19644, 19518, 19770, 19896, 20083, 20013, 20153, 20251, 20469, 20740,
    20586, 20894, 21191, 21048, 21702, 21576, 21828, 21954, 0, 11448, 10298,
    10415, 10820, 0, 11838, 18282, 11367, 10976, 0, 11706, 10964, 0, 14122,
    17146, 0, 11478, 0, 0, 10629, 11721, 0, 0, 0, 0, 0, 11186, 12052, 0, 19242,
    21466, 12710, 10259, 11298, 0, 0, 0, 0, 11235, 12101, 0, 10259, 12822, 13485,
    16578, 0, 0, 0, 0, 0, 0, 0, 0, 21334, 0, 22421, 0, 0, 10519, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 13264, 14824, 13043, 14977, 14518, 15206, 15291, 15410, 15121,
    16102, 16340, 0, 15864, 17498, 17668, 17328, 0, 0, 0, 0, 0, 0, 0, 11988, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 10772, 15506, 20329, 17998, 22071, 18174, 22251, 0,
    17026, 12004, 11610, 11658, 12346, 12164, 12528, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 11358, 11439, 11982 };

  static const unsigned short uv[256] = { 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
    0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
    0U, 0U, 0U, 3U, 4U, 5U, 6U, 7U, 8U, 9U, 10U, 11U, 12U, 13U, 14U, 15U, 16U,
    17U, 18U, 19U, 20U, 21U, 22U, 23U, 24U, 25U, 26U, 27U, 28U, 29U, 30U, 31U,
    32U, 33U, 34U, 35U, 36U, 37U, 38U, 39U, 40U, 41U, 42U, 43U, 44U, 45U, 46U,
    47U, 48U, 49U, 50U, 51U, 52U, 53U, 54U, 55U, 56U, 57U, 58U, 59U, 60U, 61U,
    62U, 63U, 64U, 65U, 66U, 67U, 68U, 69U, 70U, 71U, 72U, 73U, 74U, 75U, 76U,
    77U, 78U, 79U, 80U, 81U, 82U, 83U, 84U, 85U, 86U, 87U, 88U, 89U, 90U, 91U,
    92U, 93U, 94U, 95U, 96U, 97U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
    0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
    0U, 0U, 172U, 163U, 132U, 133U, 189U, 150U, 232U, 134U, 142U, 139U, 157U,
    169U, 164U, 258U, 138U, 259U, 131U, 147U, 242U, 243U, 141U, 151U, 136U, 260U,
    222U, 241U, 158U, 170U, 245U, 244U, 246U, 162U, 173U, 201U, 199U, 174U, 98U,
    99U, 144U, 100U, 203U, 101U, 200U, 202U, 207U, 204U, 205U, 206U, 233U, 102U,
    211U, 208U, 209U, 175U, 103U, 240U, 145U, 214U, 212U, 213U, 104U, 235U, 237U,
    137U, 106U, 105U, 107U, 109U, 108U, 110U, 160U, 111U, 113U, 112U, 114U, 115U,
    117U, 116U, 118U, 119U, 234U, 120U, 122U, 121U, 123U, 125U, 124U, 184U, 161U,
    127U, 126U, 128U, 129U, 236U, 238U, 186U };

  static const unsigned char uv7[22591] = { 48U, 64U, 64U, 64U, 64U, 64U, 64U,
    64U, 64U, 16U, 159U, 80U, 80U, 80U, 80U, 80U, 80U, 80U, 135U, 64U, 144U, 0U,
    0U, 0U, 0U, 0U, 0U, 0U, 80U, 64U, 144U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 80U, 64U,
    144U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 80U, 64U, 144U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
    80U, 64U, 144U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 80U, 64U, 144U, 0U, 0U, 0U, 0U,
    0U, 0U, 0U, 80U, 64U, 144U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 80U, 64U, 144U, 0U,
    0U, 0U, 0U, 0U, 0U, 0U, 80U, 64U, 144U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 80U, 64U,
    171U, 144U, 144U, 144U, 144U, 144U, 144U, 144U, 179U, 64U, 4U, MAX_uint8_T,
    180U, 4U, MAX_uint8_T, 180U, 2U, MAX_uint8_T, 178U, 0U, 251U, 171U, 0U, 243U,
    163U, 0U, 234U, 155U, 0U, 226U, 147U, 0U, 217U, 139U, 0U, 208U, 131U, 0U, 0U,
    0U, 0U, 0U, 0U, 4U, MAX_uint8_T, 180U, 4U, MAX_uint8_T, 180U, 2U, 254U, 172U,
    0U, 246U, 184U, 0U, 244U, 158U, 0U, 232U, 170U, 0U, 230U, 144U, 0U, 218U,
    156U, 0U, 215U, 130U, 0U, 203U, 142U, 0U, 0U, 0U, 0U, 21U, 250U, 12U, 0U,
    168U, 120U, 0U, 0U, 0U, 0U, 0U, 0U, 83U, 202U, 0U, 0U, 229U, 58U, 0U, 0U, 0U,
    0U, 0U, 0U, 146U, 140U, 0U, 36U, 246U, 6U, 0U, 0U, 0U, 0U, 0U, 0U, 209U, 78U,
    0U, 97U, 190U, 0U, 0U, 0U, 1U, 239U, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T,
    MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T,
    37U, 0U, 0U, 0U, 85U, 199U, 0U, 0U, 228U, 54U, 0U, 0U, 0U, 0U, 0U, 0U, 150U,
    135U, 0U, 38U, 241U, 3U, 0U, 0U, 0U, 0U, 0U, 0U, 215U, 71U, 0U, 103U, 180U,
    0U, 0U, 0U, 0U, 196U, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T,
    MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, 83U, 0U, 0U,
    0U, 92U, 192U, 0U, 1U, 237U, 51U, 0U, 0U, 0U, 0U, 0U, 0U, 155U, 130U, 0U,
    46U, 241U, 3U, 0U, 0U, 0U, 0U, 0U, 0U, 218U, 68U, 0U, 108U, 181U, 0U, 0U, 0U,
    0U, 0U, 0U, 24U, 251U, 11U, 0U, 169U, 119U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
    0U, 160U, 60U, 0U, 0U, 0U, 0U, 0U, 73U, 202U, 251U, 241U, 187U, 71U, 0U, 0U,
    58U, 253U, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T,
    12U, 0U, 156U, 244U, 82U, 169U, 78U, 85U, 193U, 11U, 0U, 168U, 222U, 0U,
    160U, 60U, 0U, 0U, 0U, 0U, 98U, MAX_uint8_T, 113U, 160U, 60U, 0U, 0U, 0U, 0U,
    3U, 169U, MAX_uint8_T, 244U, 67U, 0U, 0U, 0U, 0U, 0U, 1U, 116U, 247U, 238U,
    93U, 0U, 0U, 0U, 0U, 0U, 0U, 160U, 221U, MAX_uint8_T, 134U, 0U, 0U, 0U, 0U,
    0U, 160U, 63U, 173U, 253U, 29U, 0U, 0U, 0U, 0U, 160U, 60U, 75U, MAX_uint8_T,
    57U, 23U, 194U, 93U, 34U, 162U, 103U, 190U, 251U, 19U, 24U, MAX_uint8_T,
    MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, 143U, 0U,
    0U, 62U, 165U, 222U, 253U, 228U, 121U, 1U, 0U, 0U, 0U, 0U, 0U, 160U, 60U, 0U,
    0U, 0U, 4U, 141U, 235U, 222U, 95U, 0U, 0U, 0U, 0U, 18U, 222U, 106U, 110U,
    229U, 35U, 78U, 253U, 46U, 0U, 0U, 2U, 183U, 161U, 0U, 195U, 136U, 0U, 0U,
    205U, 130U, 0U, 0U, 132U, 206U, 9U, 0U, 220U, 110U, 0U, 0U, 178U, 155U, 0U,
    79U, 236U, 33U, 0U, 0U, 195U, 135U, 0U, 0U, 200U, 130U, 38U, 238U, 71U, 0U,
    0U, 0U, 109U, 228U, 34U, 65U, 251U, 58U, 212U, 123U, 0U, 0U, 0U, 0U, 4U,
    143U, 236U, 222U, 95U, 169U, 176U, 88U, 220U, 236U, 146U, 6U, 0U, 0U, 0U, 0U,
    116U, 217U, 52U, 252U, 83U, 33U, 226U, 117U, 0U, 0U, 0U, 64U, 240U, 42U,
    119U, 212U, 0U, 0U, 129U, 202U, 0U, 0U, 28U, 233U, 86U, 0U, 144U, 186U, 0U,
    0U, 102U, 227U, 0U, 6U, 201U, 139U, 0U, 0U, 119U, 211U, 0U, 0U, 124U, 202U,
    0U, 154U, 190U, 3U, 0U, 0U, 37U, 252U, 80U, 27U, 217U, 116U, 99U, 227U, 21U,
    0U, 0U, 0U, 0U, 89U, 222U, 236U, 147U, 6U, 0U, 0U, 0U, 14U, 149U, 231U, 237U,
    160U, 16U, 0U, 0U, 0U, 0U, 0U, 0U, 175U, MAX_uint8_T, MAX_uint8_T,
    MAX_uint8_T, MAX_uint8_T, 165U, 0U, 0U, 0U, 0U, 0U, 4U, 251U, 226U, 28U, 24U,
    215U, 240U, 0U, 0U, 0U, 0U, 0U, 0U, 230U, 193U, 0U, 0U, 176U, 217U, 0U, 0U,
    0U, 0U, 0U, 0U, 135U, 247U, 27U, 70U, 250U, 91U, 0U, 0U, 0U, 0U, 0U, 39U,
    167U, MAX_uint8_T, 225U, 244U, 101U, 0U, 0U, 0U, 0U, 0U, 80U, 247U, 172U,
    140U, MAX_uint8_T, 97U, 0U, 0U, 2U, MAX_uint8_T, 169U, 14U, 237U, 164U, 0U,
    2U, 199U, 228U, 17U, 0U, 10U, MAX_uint8_T, 145U, 66U, MAX_uint8_T, 111U, 0U,
    0U, 42U, 249U, 173U, 1U, 43U, MAX_uint8_T, 88U, 76U, MAX_uint8_T, 190U, 2U,
    0U, 0U, 108U, MAX_uint8_T, 133U, 115U, 239U, 11U, 19U, 246U, MAX_uint8_T,
    175U, 49U, 6U, 0U, 169U, MAX_uint8_T, 242U, 116U, 0U, 0U, 102U, 254U,
    MAX_uint8_T, MAX_uint8_T, 254U, 242U, 219U, MAX_uint8_T, MAX_uint8_T, 78U,
    0U, 0U, 0U, 64U, 181U, 239U, 249U, 225U, 173U, 100U, 239U, 249U, 77U, 1U,
    249U, MAX_uint8_T, 21U, 0U, 223U, 248U, 1U, 0U, 195U, 221U, 0U, 0U, 166U,
    193U, 0U, 0U, 0U, 0U, 100U, 47U, 0U, 0U, 119U, MAX_uint8_T, 52U, 0U, 54U,
    252U, 188U, 5U, 1U, 212U, 231U, 14U, 0U, 72U, MAX_uint8_T, 109U, 0U, 0U,
    152U, MAX_uint8_T, 29U, 0U, 0U, 204U, 236U, 0U, 0U, 0U, 229U, 217U, 0U, 0U,
    0U, 229U, 216U, 0U, 0U, 0U, 204U, 236U, 0U, 0U, 0U, 152U, MAX_uint8_T, 29U,
    0U, 0U, 72U, MAX_uint8_T, 109U, 0U, 0U, 1U, 212U, 231U, 14U, 0U, 0U, 54U,
    252U, 187U, 5U, 0U, 0U, 119U, MAX_uint8_T, 52U, 0U, 0U, 0U, 101U, 47U, 74U,
    72U, 0U, 0U, 0U, 87U, 251U, 86U, 0U, 0U, 17U, 213U, 240U, 29U, 0U, 0U, 34U,
    248U, 177U, 0U, 0U, 0U, 145U, 254U, 36U, 0U, 0U, 65U, MAX_uint8_T, 116U, 0U,
    0U, 16U, MAX_uint8_T, 167U, 0U, 0U, 1U, 252U, 191U, 0U, 0U, 1U, 252U, 191U,
    0U, 0U, 16U, MAX_uint8_T, 167U, 0U, 0U, 65U, MAX_uint8_T, 116U, 0U, 0U, 145U,
    254U, 36U, 0U, 34U, 248U, 177U, 0U, 16U, 213U, 240U, 29U, 0U, 87U, 252U, 86U,
    0U, 0U, 74U, 72U, 0U, 0U, 0U, 0U, 0U, 0U, 69U, 244U, 0U, 0U, 0U, 0U, 0U, 0U,
    47U, 222U, 0U, 0U, 0U, 0U, 185U, 107U, 31U, 199U, 26U, 148U, 123U, 0U, 52U,
    148U, 152U, 40U, 198U, 116U, 23U, 0U, 0U, 1U, 140U, 98U, 85U, 0U, 0U, 0U, 0U,
    133U, 192U, 30U, 238U, 59U, 0U, 0U, 4U, 175U, 50U, 0U, 127U, 105U, 0U, 0U,
    0U, 0U, 0U, 0U, 128U, 208U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 128U,
    208U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 128U, 208U, 0U, 0U, 0U, 0U, 0U,
    0U, 0U, 0U, 0U, 0U, 128U, 208U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 128U,
    208U, 0U, 0U, 0U, 0U, 0U, 12U, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T,
    MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T,
    MAX_uint8_T, 92U, 0U, 0U, 0U, 0U, 0U, 128U, 208U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
    0U, 0U, 0U, 128U, 208U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 128U, 208U,
    0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 128U, 208U, 0U, 0U, 0U, 0U, 0U, 0U,
    0U, 0U, 0U, 0U, 128U, 208U, 0U, 0U, 0U, 0U, 0U, 60U, MAX_uint8_T, 236U, 60U,
    MAX_uint8_T, 235U, 0U, 107U, 219U, 3U, 178U, 163U, 58U, 194U, 28U, 180U,
    MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T,
    MAX_uint8_T, 28U, 60U, MAX_uint8_T, 236U, 60U, MAX_uint8_T, 236U, 0U, 0U, 0U,
    0U, 0U, 194U, 153U, 0U, 0U, 0U, 0U, 16U, 251U, 78U, 0U, 0U, 0U, 0U, 88U,
    248U, 11U, 0U, 0U, 0U, 0U, 163U, 184U, 0U, 0U, 0U, 0U, 2U, 235U, 109U, 0U,
    0U, 0U, 0U, 57U, MAX_uint8_T, 35U, 0U, 0U, 0U, 0U, 132U, 215U, 0U, 0U, 0U,
    0U, 0U, 207U, 140U, 0U, 0U, 0U, 0U, 26U, MAX_uint8_T, 65U, 0U, 0U, 0U, 0U,
    101U, 241U, 5U, 0U, 0U, 0U, 0U, 176U, 171U, 0U, 0U, 0U, 0U, 7U, 243U, 96U,
    0U, 0U, 0U, 0U, 70U, 254U, 23U, 0U, 0U, 0U, 0U, 145U, 202U, 0U, 0U, 0U, 0U,
    0U, 219U, 127U, 0U, 0U, 0U, 0U, 39U, MAX_uint8_T, 53U, 0U, 0U, 0U, 0U, 0U,
    0U, 78U, 202U, 247U, 228U, 137U, 9U, 0U, 0U, 0U, 92U, MAX_uint8_T,
    MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, 187U, 3U, 0U, 18U, 239U,
    246U, 84U, 8U, 39U, 198U, MAX_uint8_T, 102U, 0U, 108U, MAX_uint8_T, 125U, 0U,
    0U, 0U, 29U, 252U, 208U, 0U, 172U, MAX_uint8_T, 44U, 0U, 0U, 0U, 0U, 201U,
    MAX_uint8_T, 16U, 206U, MAX_uint8_T, 7U, 0U, 0U, 0U, 0U, 164U, MAX_uint8_T,
    49U, 216U, 254U, 0U, 0U, 0U, 0U, 0U, 154U, MAX_uint8_T, 60U, 205U,
    MAX_uint8_T, 7U, 0U, 0U, 0U, 0U, 165U, MAX_uint8_T, 48U, 171U, MAX_uint8_T,
    43U, 0U, 0U, 0U, 0U, 203U, MAX_uint8_T, 15U, 107U, MAX_uint8_T, 123U, 0U, 0U,
    0U, 33U, 253U, 206U, 0U, 18U, 240U, 244U, 81U, 7U, 41U, 201U, MAX_uint8_T,
    102U, 0U, 0U, 94U, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T,
    MAX_uint8_T, 188U, 3U, 0U, 0U, 0U, 81U, 206U, 248U, 227U, 136U, 9U, 0U, 0U,
    2U, 46U, 105U, 164U, 223U, 11U, 0U, 0U, 76U, 252U, 208U, 228U, MAX_uint8_T,
    12U, 0U, 0U, 14U, 12U, 0U, 176U, MAX_uint8_T, 12U, 0U, 0U, 0U, 0U, 0U, 176U,
    MAX_uint8_T, 12U, 0U, 0U, 0U, 0U, 0U, 176U, MAX_uint8_T, 12U, 0U, 0U, 0U, 0U,
    0U, 176U, MAX_uint8_T, 12U, 0U, 0U, 0U, 0U, 0U, 176U, MAX_uint8_T, 12U, 0U,
    0U, 0U, 0U, 0U, 176U, MAX_uint8_T, 12U, 0U, 0U, 0U, 0U, 0U, 176U,
    MAX_uint8_T, 12U, 0U, 0U, 0U, 0U, 0U, 176U, MAX_uint8_T, 12U, 0U, 0U, 0U, 0U,
    0U, 176U, MAX_uint8_T, 12U, 0U, 0U, 76U, MAX_uint8_T, MAX_uint8_T,
    MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, 168U, 76U, MAX_uint8_T,
    MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, 168U, 1U,
    76U, 179U, 227U, 244U, 210U, 114U, 6U, 0U, 48U, MAX_uint8_T, MAX_uint8_T,
    MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, 177U, 0U, 46U, 180U, 75U,
    22U, 11U, 69U, 229U, MAX_uint8_T, 43U, 0U, 0U, 0U, 0U, 0U, 0U, 148U,
    MAX_uint8_T, 66U, 0U, 0U, 0U, 0U, 0U, 0U, 203U, 250U, 24U, 0U, 0U, 0U, 0U,
    0U, 102U, MAX_uint8_T, 136U, 0U, 0U, 0U, 0U, 0U, 97U, 252U, 154U, 2U, 0U, 0U,
    0U, 0U, 132U, 252U, 117U, 0U, 0U, 0U, 0U, 1U, 153U, 248U, 78U, 0U, 0U, 0U,
    0U, 0U, 132U, MAX_uint8_T, 93U, 0U, 0U, 0U, 0U, 0U, 56U, 254U, 200U, 0U, 0U,
    0U, 0U, 0U, 0U, 128U, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T,
    MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, 60U, 128U, MAX_uint8_T, MAX_uint8_T,
    MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, 60U, 0U,
    63U, 179U, 234U, 242U, 213U, 129U, 9U, 0U, 0U, MAX_uint8_T, MAX_uint8_T,
    MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, 158U, 0U, 0U, 192U, 74U,
    18U, 13U, 80U, 245U, 235U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 214U, 215U, 0U, 0U,
    0U, 1U, 14U, 48U, 146U, 249U, 79U, 0U, 0U, 0U, 246U, MAX_uint8_T,
    MAX_uint8_T, 201U, 49U, 0U, 0U, 0U, 0U, 248U, MAX_uint8_T, MAX_uint8_T,
    MAX_uint8_T, 198U, 41U, 0U, 0U, 0U, 0U, 16U, 46U, 134U, 253U, 228U, 12U, 0U,
    0U, 0U, 0U, 0U, 0U, 152U, MAX_uint8_T, 76U, 0U, 0U, 0U, 0U, 0U, 0U, 143U,
    MAX_uint8_T, 94U, 42U, 178U, 71U, 15U, 22U, 107U, 249U, MAX_uint8_T, 43U,
    44U, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T,
    MAX_uint8_T, 156U, 0U, 1U, 77U, 183U, 238U, 245U, 203U, 105U, 2U, 0U, 0U, 0U,
    0U, 0U, 0U, 0U, 144U, MAX_uint8_T, 104U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 63U,
    254U, MAX_uint8_T, 104U, 0U, 0U, 0U, 0U, 0U, 0U, 13U, 223U, 198U,
    MAX_uint8_T, 104U, 0U, 0U, 0U, 0U, 0U, 0U, 155U, 224U, 45U, MAX_uint8_T,
    104U, 0U, 0U, 0U, 0U, 0U, 73U, 250U, 57U, 28U, MAX_uint8_T, 104U, 0U, 0U, 0U,
    0U, 18U, 230U, 119U, 0U, 28U, MAX_uint8_T, 104U, 0U, 0U, 0U, 0U, 167U, 185U,
    1U, 0U, 28U, MAX_uint8_T, 104U, 0U, 0U, 0U, 85U, 231U, 22U, 0U, 0U, 28U,
    MAX_uint8_T, 104U, 0U, 0U, 9U, 236U, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T,
    MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, 36U, 20U,
    MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T,
    MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, 36U, 0U, 0U, 0U, 0U, 0U, 0U, 56U,
    MAX_uint8_T, 104U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 56U, MAX_uint8_T, 104U,
    0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 56U, MAX_uint8_T, 104U, 0U, 0U, 152U,
    MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, 252U, 0U,
    152U, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, 252U,
    0U, 152U, 236U, 0U, 0U, 0U, 0U, 0U, 0U, 152U, 236U, 0U, 0U, 0U, 0U, 0U, 0U,
    152U, 236U, 0U, 0U, 0U, 0U, 0U, 0U, 152U, MAX_uint8_T, 246U, 218U, 152U, 37U,
    0U, 0U, 152U, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, 247U, 69U,
    0U, 0U, 3U, 21U, 62U, 163U, MAX_uint8_T, 228U, 3U, 0U, 0U, 0U, 0U, 0U, 193U,
    MAX_uint8_T, 46U, 0U, 0U, 0U, 0U, 0U, 180U, MAX_uint8_T, 54U, 157U, 71U, 12U,
    25U, 121U, 254U, 245U, 11U, 208U, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T,
    MAX_uint8_T, MAX_uint8_T, 106U, 0U, 53U, 193U, 248U, 235U, 180U, 69U, 0U, 0U,
    0U, 0U, 31U, 157U, 228U, 246U, 206U, 110U, 10U, 0U, 0U, 47U, 238U,
    MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, 108U, 0U,
    3U, 212U, MAX_uint8_T, 153U, 33U, 10U, 54U, 150U, 97U, 0U, 74U, MAX_uint8_T,
    183U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 144U, MAX_uint8_T, 83U, 0U, 0U, 0U, 0U, 0U,
    0U, 0U, 184U, MAX_uint8_T, 58U, 152U, 233U, 248U, 205U, 82U, 0U, 0U, 199U,
    MAX_uint8_T, 227U, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T,
    MAX_uint8_T, 93U, 0U, 194U, MAX_uint8_T, 212U, 66U, 11U, 23U, 130U,
    MAX_uint8_T, 227U, 0U, 168U, MAX_uint8_T, 63U, 0U, 0U, 0U, 0U, 200U,
    MAX_uint8_T, 16U, 113U, MAX_uint8_T, 97U, 0U, 0U, 0U, 0U, 193U, 251U, 6U,
    26U, 247U, 235U, 84U, 15U, 20U, 119U, MAX_uint8_T, 185U, 0U, 0U, 113U,
    MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, 240U, 42U,
    0U, 0U, 0U, 90U, 205U, 248U, 238U, 170U, 38U, 0U, 0U, 12U, MAX_uint8_T,
    MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T,
    MAX_uint8_T, 76U, 12U, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T,
    MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, 76U, 0U, 0U, 0U, 0U, 0U,
    0U, 0U, 171U, 227U, 16U, 0U, 0U, 0U, 0U, 0U, 0U, 79U, MAX_uint8_T, 70U, 0U,
    0U, 0U, 0U, 0U, 0U, 15U, 229U, 158U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 155U, 235U,
    17U, 0U, 0U, 0U, 0U, 0U, 0U, 65U, 254U, 100U, 0U, 0U, 0U, 0U, 0U, 0U, 8U,
    219U, 210U, 2U, 0U, 0U, 0U, 0U, 0U, 0U, 128U, MAX_uint8_T, 79U, 0U, 0U, 0U,
    0U, 0U, 0U, 27U, 247U, 212U, 1U, 0U, 0U, 0U, 0U, 0U, 0U, 146U, MAX_uint8_T,
    106U, 0U, 0U, 0U, 0U, 0U, 0U, 3U, 236U, 252U, 20U, 0U, 0U, 0U, 0U, 0U, 0U,
    47U, MAX_uint8_T, 206U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 55U, 182U, 233U,
    243U, 202U, 87U, 0U, 0U, 0U, 63U, 251U, MAX_uint8_T, MAX_uint8_T,
    MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, 88U, 0U, 0U, 172U, 252U, 97U, 13U,
    15U, 103U, 253U, 171U, 0U, 0U, 181U, 226U, 0U, 0U, 0U, 0U, 228U, 141U, 0U,
    0U, 83U, MAX_uint8_T, 150U, 13U, 0U, 100U, 228U, 25U, 0U, 0U, 0U, 112U, 252U,
    236U, 184U, 204U, 35U, 0U, 0U, 0U, 7U, 154U, 236U, 211U, MAX_uint8_T, 228U,
    86U, 0U, 0U, 0U, 169U, 217U, 27U, 1U, 82U, 217U, MAX_uint8_T, 110U, 0U, 53U,
    MAX_uint8_T, 93U, 0U, 0U, 0U, 10U, 206U, 246U, 16U, 102U, MAX_uint8_T, 103U,
    0U, 0U, 0U, 0U, 155U, MAX_uint8_T, 54U, 77U, MAX_uint8_T, 233U, 78U, 13U,
    17U, 97U, 246U, 252U, 21U, 4U, 199U, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T,
    MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, 130U, 0U, 0U, 10U, 119U, 210U, 243U,
    233U, 190U, 76U, 0U, 0U, 0U, 2U, 114U, 215U, 249U, 230U, 148U, 17U, 0U, 0U,
    0U, 164U, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T,
    208U, 10U, 0U, 68U, MAX_uint8_T, 199U, 48U, 9U, 43U, 180U, MAX_uint8_T, 127U,
    0U, 141U, MAX_uint8_T, 51U, 0U, 0U, 0U, 8U, 234U, 225U, 0U, 152U,
    MAX_uint8_T, 59U, 0U, 0U, 0U, 0U, 207U, MAX_uint8_T, 23U, 98U, MAX_uint8_T,
    207U, 53U, 8U, 30U, 138U, MAX_uint8_T, MAX_uint8_T, 47U, 6U, 202U,
    MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, 250U, 233U, MAX_uint8_T,
    48U, 0U, 13U, 147U, 232U, 248U, 196U, 61U, 210U, MAX_uint8_T, 29U, 0U, 0U,
    0U, 0U, 0U, 0U, 13U, 245U, 237U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 120U,
    MAX_uint8_T, 156U, 0U, 0U, 186U, 80U, 18U, 19U, 111U, 250U, 247U, 38U, 0U,
    0U, 244U, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, 253U, 91U, 0U,
    0U, 0U, 59U, 181U, 242U, 235U, 175U, 54U, 0U, 0U, 0U, 4U, MAX_uint8_T, 180U,
    4U, MAX_uint8_T, 180U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
    0U, 0U, 0U, 0U, 0U, 4U, MAX_uint8_T, 180U, 4U, MAX_uint8_T, 180U, 4U,
    MAX_uint8_T, 180U, 4U, MAX_uint8_T, 180U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
    0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 4U, MAX_uint8_T, 180U, 4U, MAX_uint8_T,
    178U, 0U, 105U, 160U, 0U, 163U, 103U, 3U, 181U, 7U, 0U, 0U, 0U, 0U, 0U, 0U,
    0U, 0U, 0U, 19U, 139U, 83U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 12U, 124U, 241U,
    237U, 57U, 0U, 0U, 0U, 0U, 0U, 6U, 110U, 233U, 244U, 132U, 15U, 0U, 0U, 0U,
    0U, 2U, 95U, 224U, 249U, 147U, 23U, 0U, 0U, 0U, 0U, 0U, 80U, 213U, 253U,
    162U, 32U, 0U, 0U, 0U, 0U, 0U, 0U, 146U, MAX_uint8_T, 253U, 87U, 0U, 0U, 0U,
    0U, 0U, 0U, 0U, 0U, 0U, 81U, 213U, 253U, 162U, 33U, 0U, 0U, 0U, 0U, 0U, 0U,
    0U, 0U, 3U, 96U, 224U, 250U, 148U, 24U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 6U,
    111U, 233U, 245U, 134U, 16U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 12U, 125U, 241U,
    238U, 58U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 19U, 140U, 83U, 12U,
    MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T,
    MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, 92U, 0U, 0U, 0U, 0U, 0U,
    0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
    12U, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T,
    MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, 92U, 11U,
    181U, 48U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 6U, 208U, 254U, 166U, 36U, 0U,
    0U, 0U, 0U, 0U, 0U, 0U, 0U, 1U, 89U, 220U, 251U, 152U, 26U, 0U, 0U, 0U, 0U,
    0U, 0U, 0U, 0U, 5U, 104U, 230U, 246U, 137U, 18U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
    0U, 9U, 119U, 238U, 240U, 122U, 11U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 32U,
    228U, MAX_uint8_T, 209U, 17U, 0U, 0U, 0U, 0U, 0U, 10U, 119U, 238U, 240U,
    123U, 11U, 0U, 0U, 0U, 0U, 5U, 106U, 231U, 246U, 138U, 18U, 0U, 0U, 0U, 0U,
    2U, 92U, 222U, 251U, 153U, 27U, 0U, 0U, 0U, 0U, 0U, 6U, 211U, 254U, 167U,
    36U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 11U, 182U, 48U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
    0U, 0U, 27U, 142U, 218U, 247U, 222U, 148U, 26U, 0U, 172U, MAX_uint8_T,
    MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, 221U, 7U, 142U, 105U,
    32U, 5U, 36U, 197U, MAX_uint8_T, 58U, 0U, 0U, 0U, 0U, 0U, 204U, 254U, 30U,
    0U, 0U, 0U, 0U, 114U, MAX_uint8_T, 123U, 0U, 0U, 0U, 0U, 84U, 252U, 113U, 0U,
    0U, 0U, 0U, 40U, 247U, 114U, 0U, 0U, 0U, 0U, 0U, 156U, 249U, 5U, 0U, 0U, 0U,
    0U, 0U, 200U, 236U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
    0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 204U, 236U, 0U, 0U, 0U, 0U, 0U, 0U, 204U,
    236U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 41U, 137U, 209U, 241U, 246U, 211U,
    132U, 19U, 0U, 0U, 0U, 0U, 0U, 8U, 149U, 225U, 117U, 42U, 10U, 11U, 56U,
    158U, 229U, 51U, 0U, 0U, 0U, 13U, 195U, 159U, 9U, 0U, 0U, 0U, 0U, 0U, 0U,
    86U, 231U, 19U, 0U, 0U, 173U, 145U, 0U, 0U, 61U, 192U, 244U, MAX_uint8_T,
    MAX_uint8_T, 55U, 0U, 154U, 127U, 0U, 78U, 200U, 2U, 0U, 83U, 205U, 49U, 11U,
    109U, 248U, 6U, 0U, 68U, 187U, 0U, 193U, 68U, 0U, 23U, 234U, 33U, 0U, 0U,
    137U, 197U, 0U, 0U, 56U, 195U, 11U, 237U, 3U, 0U, 128U, 174U, 0U, 0U, 38U,
    245U, 140U, 0U, 0U, 103U, 154U, 38U, 212U, 0U, 0U, 198U, 134U, 0U, 13U, 192U,
    226U, 83U, 0U, 6U, 218U, 56U, 33U, 224U, 0U, 0U, 213U, 156U, 32U, 195U, 105U,
    254U, 66U, 33U, 181U, 130U, 0U, 2U, 231U, 48U, 0U, 103U, 242U, 218U, 82U,
    22U, 222U, 248U, 206U, 91U, 0U, 0U, 0U, 111U, 206U, 17U, 0U, 0U, 0U, 0U, 0U,
    0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 147U, 225U, 101U, 29U, 6U, 39U, 138U, 109U,
    0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 71U, 178U, 234U, 248U, 215U, 136U, 22U, 0U,
    0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 2U, 224U, MAX_uint8_T, 85U, 0U, 0U, 0U, 0U,
    0U, 0U, 0U, 0U, 0U, 73U, MAX_uint8_T, MAX_uint8_T, 185U, 0U, 0U, 0U, 0U, 0U,
    0U, 0U, 0U, 0U, 174U, 247U, 232U, 253U, 31U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 24U,
    251U, 162U, 132U, MAX_uint8_T, 129U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 121U,
    MAX_uint8_T, 59U, 32U, 254U, 226U, 2U, 0U, 0U, 0U, 0U, 0U, 1U, 221U, 212U,
    0U, 0U, 186U, MAX_uint8_T, 73U, 0U, 0U, 0U, 0U, 0U, 68U, MAX_uint8_T, 109U,
    0U, 0U, 84U, MAX_uint8_T, 173U, 0U, 0U, 0U, 0U, 0U, 170U, 246U, 16U, 0U, 0U,
    5U, 233U, 250U, 22U, 0U, 0U, 0U, 21U, 249U, MAX_uint8_T, MAX_uint8_T,
    MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, 117U, 0U,
    0U, 0U, 117U, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T,
    MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, 216U, 0U, 0U, 0U, 217U,
    247U, 23U, 0U, 0U, 0U, 0U, 9U, 234U, MAX_uint8_T, 61U, 0U, 64U, MAX_uint8_T,
    147U, 0U, 0U, 0U, 0U, 0U, 0U, 125U, MAX_uint8_T, 161U, 0U, 165U, 250U, 30U,
    0U, 0U, 0U, 0U, 0U, 0U, 17U, 242U, 246U, 15U, 84U, MAX_uint8_T, MAX_uint8_T,
    MAX_uint8_T, 244U, 215U, 133U, 17U, 0U, 84U, MAX_uint8_T, MAX_uint8_T,
    MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, 194U, 0U, 84U,
    MAX_uint8_T, 128U, 5U, 27U, 106U, 252U, MAX_uint8_T, 15U, 84U, MAX_uint8_T,
    128U, 0U, 0U, 0U, 225U, 241U, 1U, 84U, MAX_uint8_T, 128U, 8U, 40U, 146U,
    MAX_uint8_T, 115U, 0U, 84U, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T,
    MAX_uint8_T, 222U, 88U, 0U, 0U, 84U, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T,
    MAX_uint8_T, 249U, 159U, 17U, 0U, 84U, MAX_uint8_T, 128U, 14U, 56U, 163U,
    MAX_uint8_T, 215U, 8U, 84U, MAX_uint8_T, 128U, 0U, 0U, 0U, 150U, MAX_uint8_T,
    103U, 84U, MAX_uint8_T, 128U, 0U, 0U, 0U, 101U, MAX_uint8_T, 146U, 84U,
    MAX_uint8_T, 128U, 1U, 14U, 62U, 217U, MAX_uint8_T, 125U, 84U, MAX_uint8_T,
    MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, 241U, 31U,
    84U, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, 252U, 225U, 165U, 39U, 0U, 0U,
    0U, 0U, 19U, 126U, 201U, 239U, 247U, 216U, 167U, 81U, 8U, 0U, 0U, 56U, 233U,
    MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T,
    MAX_uint8_T, 112U, 0U, 23U, 234U, MAX_uint8_T, 204U, 77U, 17U, 5U, 35U, 95U,
    173U, 103U, 0U, 136U, MAX_uint8_T, 198U, 7U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
    216U, MAX_uint8_T, 60U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 3U, 254U, 245U, 2U,
    0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 15U, MAX_uint8_T, 228U, 0U, 0U, 0U, 0U, 0U,
    0U, 0U, 0U, 0U, 3U, 254U, 245U, 2U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 218U,
    MAX_uint8_T, 64U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 141U, MAX_uint8_T,
    207U, 12U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 27U, 238U, MAX_uint8_T, 216U, 90U,
    23U, 6U, 33U, 77U, 166U, 106U, 0U, 0U, 64U, 238U, MAX_uint8_T, MAX_uint8_T,
    MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, 116U, 0U,
    0U, 0U, 24U, 134U, 208U, 242U, 247U, 220U, 180U, 91U, 9U, 84U, MAX_uint8_T,
    MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, 247U, 230U, 181U, 93U, 2U, 0U, 0U,
    84U, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T,
    MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, 184U, 10U, 0U, 84U, MAX_uint8_T, 136U,
    0U, 4U, 16U, 52U, 137U, 251U, MAX_uint8_T, 152U, 0U, 84U, MAX_uint8_T, 136U,
    0U, 0U, 0U, 0U, 0U, 77U, MAX_uint8_T, 252U, 28U, 84U, MAX_uint8_T, 136U, 0U,
    0U, 0U, 0U, 0U, 0U, 187U, MAX_uint8_T, 97U, 84U, MAX_uint8_T, 136U, 0U, 0U,
    0U, 0U, 0U, 0U, 121U, MAX_uint8_T, 131U, 84U, MAX_uint8_T, 136U, 0U, 0U, 0U,
    0U, 0U, 0U, 104U, MAX_uint8_T, 137U, 84U, MAX_uint8_T, 136U, 0U, 0U, 0U, 0U,
    0U, 0U, 127U, MAX_uint8_T, 118U, 84U, MAX_uint8_T, 136U, 0U, 0U, 0U, 0U, 0U,
    0U, 196U, MAX_uint8_T, 68U, 84U, MAX_uint8_T, 136U, 0U, 0U, 0U, 0U, 0U, 80U,
    MAX_uint8_T, 231U, 7U, 84U, MAX_uint8_T, 136U, 0U, 0U, 9U, 44U, 131U, 249U,
    MAX_uint8_T, 98U, 0U, 84U, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T,
    MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, 127U, 0U,
    0U, 84U, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, 247U, 221U,
    159U, 51U, 0U, 0U, 0U, 84U, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T,
    MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, 244U, 0U, 84U, MAX_uint8_T,
    MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, 244U, 0U,
    84U, MAX_uint8_T, 136U, 0U, 0U, 0U, 0U, 0U, 0U, 84U, MAX_uint8_T, 136U, 0U,
    0U, 0U, 0U, 0U, 0U, 84U, MAX_uint8_T, 136U, 0U, 0U, 0U, 0U, 0U, 0U, 84U,
    MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T,
    16U, 0U, 84U, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T,
    MAX_uint8_T, MAX_uint8_T, 16U, 0U, 84U, MAX_uint8_T, 136U, 0U, 0U, 0U, 0U,
    0U, 0U, 84U, MAX_uint8_T, 136U, 0U, 0U, 0U, 0U, 0U, 0U, 84U, MAX_uint8_T,
    136U, 0U, 0U, 0U, 0U, 0U, 0U, 84U, MAX_uint8_T, 136U, 0U, 0U, 0U, 0U, 0U, 0U,
    84U, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T,
    MAX_uint8_T, MAX_uint8_T, 84U, 84U, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T,
    MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, 84U, 84U, MAX_uint8_T,
    MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, 244U, 84U,
    MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T,
    244U, 84U, MAX_uint8_T, 136U, 0U, 0U, 0U, 0U, 0U, 84U, MAX_uint8_T, 136U, 0U,
    0U, 0U, 0U, 0U, 84U, MAX_uint8_T, 136U, 0U, 0U, 0U, 0U, 0U, 84U, MAX_uint8_T,
    136U, 0U, 0U, 0U, 0U, 0U, 84U, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T,
    MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, 20U, 84U, MAX_uint8_T, MAX_uint8_T,
    MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, 20U, 84U, MAX_uint8_T,
    136U, 0U, 0U, 0U, 0U, 0U, 84U, MAX_uint8_T, 136U, 0U, 0U, 0U, 0U, 0U, 84U,
    MAX_uint8_T, 136U, 0U, 0U, 0U, 0U, 0U, 84U, MAX_uint8_T, 136U, 0U, 0U, 0U,
    0U, 0U, 84U, MAX_uint8_T, 136U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 18U, 126U,
    202U, 239U, 247U, 218U, 173U, 85U, 8U, 0U, 0U, 54U, 231U, MAX_uint8_T,
    MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T,
    112U, 0U, 22U, 233U, MAX_uint8_T, 204U, 77U, 17U, 5U, 35U, 95U, 173U, 103U,
    0U, 135U, MAX_uint8_T, 197U, 7U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 216U,
    MAX_uint8_T, 58U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 3U, 254U, 245U, 2U, 0U, 0U,
    0U, 0U, 0U, 0U, 0U, 0U, 16U, MAX_uint8_T, 228U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
    0U, 0U, 3U, 254U, 245U, 3U, 0U, 0U, 0U, 0U, 0U, 100U, MAX_uint8_T, 116U, 0U,
    218U, MAX_uint8_T, 65U, 0U, 0U, 0U, 0U, 0U, 100U, MAX_uint8_T, 116U, 0U,
    141U, MAX_uint8_T, 207U, 12U, 0U, 0U, 0U, 0U, 100U, MAX_uint8_T, 116U, 0U,
    27U, 239U, MAX_uint8_T, 216U, 92U, 25U, 4U, 19U, 134U, MAX_uint8_T, 116U, 0U,
    0U, 64U, 238U, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T,
    MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, 116U, 0U, 0U, 0U, 24U, 133U, 206U,
    242U, 250U, 229U, 195U, 137U, 42U, 84U, MAX_uint8_T, 136U, 0U, 0U, 0U, 0U,
    0U, 72U, MAX_uint8_T, 144U, 84U, MAX_uint8_T, 136U, 0U, 0U, 0U, 0U, 0U, 72U,
    MAX_uint8_T, 144U, 84U, MAX_uint8_T, 136U, 0U, 0U, 0U, 0U, 0U, 72U,
    MAX_uint8_T, 144U, 84U, MAX_uint8_T, 136U, 0U, 0U, 0U, 0U, 0U, 72U,
    MAX_uint8_T, 144U, 84U, MAX_uint8_T, 136U, 0U, 0U, 0U, 0U, 0U, 72U,
    MAX_uint8_T, 144U, 84U, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T,
    MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, 144U, 84U,
    MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T,
    MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, 144U, 84U, MAX_uint8_T, 136U, 0U, 0U,
    0U, 0U, 0U, 72U, MAX_uint8_T, 144U, 84U, MAX_uint8_T, 136U, 0U, 0U, 0U, 0U,
    0U, 72U, MAX_uint8_T, 144U, 84U, MAX_uint8_T, 136U, 0U, 0U, 0U, 0U, 0U, 72U,
    MAX_uint8_T, 144U, 84U, MAX_uint8_T, 136U, 0U, 0U, 0U, 0U, 0U, 72U,
    MAX_uint8_T, 144U, 84U, MAX_uint8_T, 136U, 0U, 0U, 0U, 0U, 0U, 72U,
    MAX_uint8_T, 144U, 84U, MAX_uint8_T, 136U, 0U, 0U, 0U, 0U, 0U, 72U,
    MAX_uint8_T, 144U, 84U, MAX_uint8_T, 132U, 84U, MAX_uint8_T, 132U, 84U,
    MAX_uint8_T, 132U, 84U, MAX_uint8_T, 132U, 84U, MAX_uint8_T, 132U, 84U,
    MAX_uint8_T, 132U, 84U, MAX_uint8_T, 132U, 84U, MAX_uint8_T, 132U, 84U,
    MAX_uint8_T, 132U, 84U, MAX_uint8_T, 132U, 84U, MAX_uint8_T, 132U, 84U,
    MAX_uint8_T, 132U, 84U, MAX_uint8_T, 132U, 0U, 0U, 0U, 0U, 220U, 252U, 0U,
    0U, 0U, 0U, 220U, 252U, 0U, 0U, 0U, 0U, 220U, 252U, 0U, 0U, 0U, 0U, 220U,
    252U, 0U, 0U, 0U, 0U, 220U, 252U, 0U, 0U, 0U, 0U, 220U, 252U, 0U, 0U, 0U, 0U,
    220U, 252U, 0U, 0U, 0U, 0U, 220U, 252U, 0U, 0U, 0U, 0U, 220U, 252U, 0U, 0U,
    0U, 0U, 220U, 252U, 0U, 0U, 0U, 0U, 220U, 252U, 0U, 0U, 0U, 0U, 225U, 250U,
    0U, 0U, 0U, 5U, 246U, 235U, 0U, 0U, 11U, 123U, MAX_uint8_T, 184U, 160U,
    MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, 254U, 67U, 160U, MAX_uint8_T, 246U,
    198U, 74U, 0U, 84U, MAX_uint8_T, 104U, 0U, 0U, 0U, 3U, 186U, 242U, 43U, 0U,
    84U, MAX_uint8_T, 104U, 0U, 0U, 0U, 144U, 254U, 78U, 0U, 0U, 84U,
    MAX_uint8_T, 104U, 0U, 0U, 99U, MAX_uint8_T, 122U, 0U, 0U, 0U, 84U,
    MAX_uint8_T, 104U, 0U, 60U, 249U, 167U, 0U, 0U, 0U, 0U, 84U, MAX_uint8_T,
    104U, 31U, 234U, 204U, 9U, 0U, 0U, 0U, 0U, 84U, MAX_uint8_T, 115U, 208U,
    231U, 28U, 0U, 0U, 0U, 0U, 0U, 84U, MAX_uint8_T, 201U, MAX_uint8_T, 184U, 2U,
    0U, 0U, 0U, 0U, 0U, 84U, MAX_uint8_T, 105U, 166U, MAX_uint8_T, 148U, 0U, 0U,
    0U, 0U, 0U, 84U, MAX_uint8_T, 104U, 7U, 195U, MAX_uint8_T, 119U, 0U, 0U, 0U,
    0U, 84U, MAX_uint8_T, 104U, 0U, 19U, 218U, 254U, 89U, 0U, 0U, 0U, 84U,
    MAX_uint8_T, 104U, 0U, 0U, 36U, 235U, 249U, 65U, 0U, 0U, 84U, MAX_uint8_T,
    104U, 0U, 0U, 0U, 59U, 248U, 240U, 44U, 0U, 84U, MAX_uint8_T, 104U, 0U, 0U,
    0U, 0U, 88U, 254U, 227U, 27U, 84U, MAX_uint8_T, 136U, 0U, 0U, 0U, 0U, 0U, 0U,
    84U, MAX_uint8_T, 136U, 0U, 0U, 0U, 0U, 0U, 0U, 84U, MAX_uint8_T, 136U, 0U,
    0U, 0U, 0U, 0U, 0U, 84U, MAX_uint8_T, 136U, 0U, 0U, 0U, 0U, 0U, 0U, 84U,
    MAX_uint8_T, 136U, 0U, 0U, 0U, 0U, 0U, 0U, 84U, MAX_uint8_T, 136U, 0U, 0U,
    0U, 0U, 0U, 0U, 84U, MAX_uint8_T, 136U, 0U, 0U, 0U, 0U, 0U, 0U, 84U,
    MAX_uint8_T, 136U, 0U, 0U, 0U, 0U, 0U, 0U, 84U, MAX_uint8_T, 136U, 0U, 0U,
    0U, 0U, 0U, 0U, 84U, MAX_uint8_T, 136U, 0U, 0U, 0U, 0U, 0U, 0U, 84U,
    MAX_uint8_T, 136U, 0U, 0U, 0U, 0U, 0U, 0U, 84U, MAX_uint8_T, MAX_uint8_T,
    MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, 96U, 84U,
    MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T,
    MAX_uint8_T, 96U, 84U, MAX_uint8_T, MAX_uint8_T, 105U, 0U, 0U, 0U, 0U, 0U,
    0U, 164U, MAX_uint8_T, 212U, 84U, MAX_uint8_T, MAX_uint8_T, 198U, 0U, 0U, 0U,
    0U, 0U, 13U, 245U, MAX_uint8_T, 212U, 84U, MAX_uint8_T, 238U, MAX_uint8_T,
    36U, 0U, 0U, 0U, 0U, 98U, 251U, 243U, 212U, 84U, MAX_uint8_T, 153U,
    MAX_uint8_T, 128U, 0U, 0U, 0U, 0U, 193U, 177U, 228U, 212U, 84U, MAX_uint8_T,
    76U, 240U, 220U, 0U, 0U, 0U, 34U, 254U, 82U, 228U, 212U, 84U, MAX_uint8_T,
    68U, 156U, MAX_uint8_T, 58U, 0U, 0U, 128U, 236U, 6U, 228U, 212U, 84U,
    MAX_uint8_T, 68U, 64U, MAX_uint8_T, 151U, 0U, 1U, 222U, 147U, 0U, 228U, 212U,
    84U, MAX_uint8_T, 68U, 1U, 226U, 237U, 6U, 62U, MAX_uint8_T, 52U, 0U, 228U,
    212U, 84U, MAX_uint8_T, 68U, 0U, 135U, MAX_uint8_T, 80U, 158U, 213U, 0U, 0U,
    228U, 212U, 84U, MAX_uint8_T, 68U, 0U, 42U, MAX_uint8_T, 183U, 242U, 117U,
    0U, 0U, 228U, 212U, 84U, MAX_uint8_T, 68U, 0U, 0U, 207U, MAX_uint8_T, 252U,
    25U, 0U, 0U, 228U, 212U, 84U, MAX_uint8_T, 68U, 0U, 0U, 114U, MAX_uint8_T,
    183U, 0U, 0U, 0U, 228U, 212U, 84U, MAX_uint8_T, 68U, 0U, 0U, 0U, 0U, 0U, 0U,
    0U, 0U, 228U, 212U, 84U, MAX_uint8_T, 203U, 3U, 0U, 0U, 0U, 0U, 0U, 248U,
    160U, 84U, MAX_uint8_T, MAX_uint8_T, 117U, 0U, 0U, 0U, 0U, 0U, 248U, 160U,
    84U, MAX_uint8_T, MAX_uint8_T, 246U, 37U, 0U, 0U, 0U, 0U, 248U, 160U, 84U,
    MAX_uint8_T, 165U, MAX_uint8_T, 193U, 1U, 0U, 0U, 0U, 248U, 160U, 84U,
    MAX_uint8_T, 68U, 185U, MAX_uint8_T, 106U, 0U, 0U, 0U, 248U, 160U, 84U,
    MAX_uint8_T, 68U, 31U, 243U, 242U, 30U, 0U, 0U, 248U, 160U, 84U, MAX_uint8_T,
    68U, 0U, 108U, MAX_uint8_T, 183U, 0U, 0U, 248U, 160U, 84U, MAX_uint8_T, 68U,
    0U, 1U, 196U, MAX_uint8_T, 94U, 0U, 248U, 160U, 84U, MAX_uint8_T, 68U, 0U,
    0U, 39U, 247U, 237U, 23U, 248U, 160U, 84U, MAX_uint8_T, 68U, 0U, 0U, 0U,
    120U, MAX_uint8_T, 172U, 248U, 160U, 84U, MAX_uint8_T, 68U, 0U, 0U, 0U, 4U,
    205U, MAX_uint8_T, MAX_uint8_T, 160U, 84U, MAX_uint8_T, 68U, 0U, 0U, 0U, 0U,
    47U, 250U, MAX_uint8_T, 160U, 84U, MAX_uint8_T, 68U, 0U, 0U, 0U, 0U, 0U,
    131U, MAX_uint8_T, 160U, 0U, 0U, 0U, 19U, 133U, 210U, 244U, 244U, 210U, 133U,
    19U, 0U, 0U, 0U, 0U, 0U, 50U, 231U, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T,
    MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, 231U, 49U, 0U, 0U, 0U, 18U, 229U,
    MAX_uint8_T, 190U, 62U, 10U, 10U, 62U, 190U, MAX_uint8_T, 228U, 18U, 0U, 0U,
    130U, MAX_uint8_T, 188U, 3U, 0U, 0U, 0U, 0U, 3U, 188U, MAX_uint8_T, 129U, 0U,
    0U, 213U, MAX_uint8_T, 53U, 0U, 0U, 0U, 0U, 0U, 0U, 53U, MAX_uint8_T, 212U,
    0U, 3U, 254U, 244U, 1U, 0U, 0U, 0U, 0U, 0U, 0U, 1U, 244U, 254U, 2U, 15U,
    MAX_uint8_T, 228U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 228U, MAX_uint8_T, 14U,
    3U, 253U, 244U, 1U, 0U, 0U, 0U, 0U, 0U, 0U, 2U, 245U, 253U, 2U, 0U, 211U,
    MAX_uint8_T, 55U, 0U, 0U, 0U, 0U, 0U, 0U, 55U, MAX_uint8_T, 211U, 0U, 0U,
    128U, MAX_uint8_T, 188U, 3U, 0U, 0U, 0U, 0U, 3U, 188U, MAX_uint8_T, 129U, 0U,
    0U, 17U, 228U, MAX_uint8_T, 188U, 60U, 9U, 10U, 63U, 191U, MAX_uint8_T, 230U,
    19U, 0U, 0U, 0U, 50U, 233U, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T,
    MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, 232U, 52U, 0U, 0U, 0U, 0U, 0U, 20U,
    136U, 214U, 246U, 244U, 210U, 133U, 20U, 0U, 0U, 0U, 84U, MAX_uint8_T,
    MAX_uint8_T, MAX_uint8_T, 252U, 233U, 180U, 61U, 0U, 84U, MAX_uint8_T,
    MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, 249U, 45U,
    84U, MAX_uint8_T, 128U, 1U, 16U, 63U, 213U, MAX_uint8_T, 135U, 84U,
    MAX_uint8_T, 128U, 0U, 0U, 0U, 97U, MAX_uint8_T, 158U, 84U, MAX_uint8_T,
    128U, 0U, 0U, 0U, 135U, MAX_uint8_T, 133U, 84U, MAX_uint8_T, 128U, 7U, 35U,
    126U, 252U, 254U, 50U, 84U, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T,
    MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, 127U, 0U, 84U, MAX_uint8_T,
    MAX_uint8_T, 252U, 232U, 176U, 69U, 0U, 0U, 84U, MAX_uint8_T, 128U, 0U, 0U,
    0U, 0U, 0U, 0U, 84U, MAX_uint8_T, 128U, 0U, 0U, 0U, 0U, 0U, 0U, 84U,
    MAX_uint8_T, 128U, 0U, 0U, 0U, 0U, 0U, 0U, 84U, MAX_uint8_T, 128U, 0U, 0U,
    0U, 0U, 0U, 0U, 84U, MAX_uint8_T, 128U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
    19U, 133U, 210U, 244U, 244U, 210U, 133U, 18U, 0U, 0U, 0U, 0U, 0U, 0U, 50U,
    231U, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T,
    MAX_uint8_T, 230U, 47U, 0U, 0U, 0U, 0U, 18U, 228U, MAX_uint8_T, 190U, 62U,
    10U, 10U, 62U, 190U, MAX_uint8_T, 226U, 16U, 0U, 0U, 0U, 129U, MAX_uint8_T,
    188U, 3U, 0U, 0U, 0U, 0U, 3U, 188U, MAX_uint8_T, 126U, 0U, 0U, 0U, 213U,
    MAX_uint8_T, 53U, 0U, 0U, 0U, 0U, 0U, 0U, 54U, MAX_uint8_T, 210U, 0U, 0U, 3U,
    254U, 244U, 1U, 0U, 0U, 0U, 0U, 0U, 0U, 1U, 244U, 253U, 2U, 0U, 15U,
    MAX_uint8_T, 228U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 228U, MAX_uint8_T, 14U,
    0U, 3U, 253U, 244U, 1U, 0U, 0U, 0U, 0U, 0U, 0U, 2U, 245U, 248U, 0U, 0U, 0U,
    211U, MAX_uint8_T, 55U, 0U, 0U, 0U, 0U, 0U, 0U, 54U, MAX_uint8_T, 213U, 0U,
    0U, 0U, 126U, MAX_uint8_T, 188U, 3U, 0U, 0U, 0U, 0U, 3U, 186U, MAX_uint8_T,
    124U, 0U, 0U, 0U, 16U, 226U, MAX_uint8_T, 188U, 60U, 9U, 10U, 62U, 188U,
    MAX_uint8_T, 225U, 19U, 0U, 0U, 0U, 0U, 48U, 232U, MAX_uint8_T, MAX_uint8_T,
    MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, 224U, 48U, 0U, 0U, 0U,
    0U, 0U, 0U, 21U, 139U, 217U, 248U, 247U, 253U, 247U, 78U, 0U, 0U, 0U, 0U, 0U,
    0U, 0U, 0U, 0U, 0U, 0U, 0U, 58U, 223U, MAX_uint8_T, 214U, 127U, 60U, 5U, 0U,
    0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 15U, 156U, 252U, MAX_uint8_T, 247U, 33U, 0U,
    0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 37U, 163U, 127U, 0U, 84U,
    MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, 253U, 225U, 162U, 30U, 0U, 0U, 0U,
    84U, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T,
    MAX_uint8_T, 220U, 12U, 0U, 0U, 84U, MAX_uint8_T, 128U, 1U, 18U, 80U, 232U,
    MAX_uint8_T, 82U, 0U, 0U, 84U, MAX_uint8_T, 128U, 0U, 0U, 0U, 126U,
    MAX_uint8_T, 114U, 0U, 0U, 84U, MAX_uint8_T, 128U, 0U, 0U, 0U, 159U,
    MAX_uint8_T, 85U, 0U, 0U, 84U, MAX_uint8_T, 128U, 9U, 38U, 133U, 254U, 235U,
    10U, 0U, 0U, 84U, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T,
    MAX_uint8_T, 236U, 58U, 0U, 0U, 0U, 84U, MAX_uint8_T, MAX_uint8_T,
    MAX_uint8_T, MAX_uint8_T, 251U, 37U, 0U, 0U, 0U, 0U, 84U, MAX_uint8_T, 128U,
    0U, 105U, MAX_uint8_T, 183U, 2U, 0U, 0U, 0U, 84U, MAX_uint8_T, 128U, 0U, 0U,
    170U, MAX_uint8_T, 127U, 0U, 0U, 0U, 84U, MAX_uint8_T, 128U, 0U, 0U, 15U,
    221U, 253U, 71U, 0U, 0U, 84U, MAX_uint8_T, 128U, 0U, 0U, 0U, 52U, 249U, 236U,
    30U, 0U, 84U, MAX_uint8_T, 128U, 0U, 0U, 0U, 0U, 111U, MAX_uint8_T, 202U, 6U,
    0U, 0U, 86U, 201U, 242U, 227U, 169U, 57U, 0U, 0U, 118U, MAX_uint8_T,
    MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, 252U, 0U, 0U, 237U, 234U,
    69U, 9U, 21U, 94U, 195U, 0U, 4U, 251U, 215U, 1U, 0U, 0U, 0U, 0U, 0U, 0U,
    188U, MAX_uint8_T, 158U, 13U, 0U, 0U, 0U, 0U, 0U, 31U, 216U, MAX_uint8_T,
    230U, 101U, 3U, 0U, 0U, 0U, 0U, 12U, 141U, 251U, MAX_uint8_T, 207U, 44U, 0U,
    0U, 0U, 0U, 0U, 39U, 186U, MAX_uint8_T, 241U, 39U, 0U, 0U, 0U, 0U, 0U, 0U,
    149U, MAX_uint8_T, 159U, 0U, 0U, 0U, 0U, 0U, 0U, 38U, MAX_uint8_T, 195U, 31U,
    196U, 100U, 39U, 7U, 31U, 153U, MAX_uint8_T, 155U, 32U, MAX_uint8_T,
    MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, 234U, 33U,
    0U, 65U, 166U, 225U, 245U, 214U, 141U, 23U, 0U, 216U, MAX_uint8_T,
    MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T,
    MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, 60U, 216U, MAX_uint8_T, MAX_uint8_T,
    MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T,
    MAX_uint8_T, MAX_uint8_T, 60U, 0U, 0U, 0U, 0U, 60U, MAX_uint8_T, 160U, 0U,
    0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 60U, MAX_uint8_T, 160U, 0U, 0U, 0U, 0U, 0U,
    0U, 0U, 0U, 0U, 60U, MAX_uint8_T, 160U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
    60U, MAX_uint8_T, 160U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 60U, MAX_uint8_T,
    160U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 60U, MAX_uint8_T, 160U, 0U, 0U, 0U,
    0U, 0U, 0U, 0U, 0U, 0U, 60U, MAX_uint8_T, 160U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
    0U, 0U, 60U, MAX_uint8_T, 160U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 60U,
    MAX_uint8_T, 160U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 60U, MAX_uint8_T,
    160U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 60U, MAX_uint8_T, 160U, 0U, 0U, 0U,
    0U, 0U, 108U, MAX_uint8_T, 108U, 0U, 0U, 0U, 0U, 0U, 180U, 232U, 108U,
    MAX_uint8_T, 108U, 0U, 0U, 0U, 0U, 0U, 180U, 232U, 108U, MAX_uint8_T, 108U,
    0U, 0U, 0U, 0U, 0U, 180U, 232U, 108U, MAX_uint8_T, 108U, 0U, 0U, 0U, 0U, 0U,
    180U, 232U, 108U, MAX_uint8_T, 108U, 0U, 0U, 0U, 0U, 0U, 180U, 232U, 108U,
    MAX_uint8_T, 108U, 0U, 0U, 0U, 0U, 0U, 180U, 232U, 108U, MAX_uint8_T, 108U,
    0U, 0U, 0U, 0U, 0U, 180U, 232U, 108U, MAX_uint8_T, 110U, 0U, 0U, 0U, 0U, 0U,
    181U, 232U, 100U, MAX_uint8_T, 135U, 0U, 0U, 0U, 0U, 0U, 195U, 224U, 72U,
    MAX_uint8_T, 203U, 0U, 0U, 0U, 0U, 10U, 242U, 200U, 14U, 244U, MAX_uint8_T,
    151U, 36U, 9U, 42U, 178U, MAX_uint8_T, 137U, 0U, 117U, MAX_uint8_T,
    MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, 236U, 27U,
    0U, 0U, 87U, 194U, 241U, 251U, 230U, 164U, 36U, 0U, 124U, MAX_uint8_T, 93U,
    0U, 0U, 0U, 0U, 0U, 0U, 33U, 254U, 125U, 24U, 250U, 200U, 0U, 0U, 0U, 0U, 0U,
    0U, 135U, 252U, 28U, 0U, 170U, MAX_uint8_T, 51U, 0U, 0U, 0U, 0U, 5U, 232U,
    179U, 0U, 0U, 65U, MAX_uint8_T, 158U, 0U, 0U, 0U, 0U, 86U, MAX_uint8_T, 79U,
    0U, 0U, 0U, 215U, 246U, 18U, 0U, 0U, 0U, 189U, 230U, 4U, 0U, 0U, 0U, 111U,
    MAX_uint8_T, 116U, 0U, 0U, 37U, 254U, 133U, 0U, 0U, 0U, 0U, 16U, 245U, 221U,
    2U, 0U, 140U, 254U, 34U, 0U, 0U, 0U, 0U, 0U, 157U, MAX_uint8_T, 74U, 7U,
    235U, 188U, 0U, 0U, 0U, 0U, 0U, 0U, 52U, MAX_uint8_T, 181U, 90U, MAX_uint8_T,
    87U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 203U, 253U, 214U, 236U, 7U, 0U, 0U, 0U, 0U,
    0U, 0U, 0U, 98U, MAX_uint8_T, MAX_uint8_T, 142U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
    0U, 9U, 238U, MAX_uint8_T, 42U, 0U, 0U, 0U, 0U, 220U, 235U, 1U, 0U, 0U, 0U,
    55U, MAX_uint8_T, 204U, 0U, 0U, 0U, 0U, 69U, MAX_uint8_T, 59U, 154U,
    MAX_uint8_T, 47U, 0U, 0U, 0U, 116U, MAX_uint8_T, 251U, 11U, 0U, 0U, 0U, 141U,
    239U, 3U, 88U, MAX_uint8_T, 113U, 0U, 0U, 0U, 177U, MAX_uint8_T, MAX_uint8_T,
    65U, 0U, 0U, 0U, 214U, 171U, 0U, 22U, MAX_uint8_T, 179U, 0U, 0U, 1U, 236U,
    194U, MAX_uint8_T, 124U, 0U, 0U, 30U, MAX_uint8_T, 99U, 0U, 0U, 212U, 241U,
    3U, 0U, 43U, MAX_uint8_T, 75U, 254U, 182U, 0U, 0U, 103U, MAX_uint8_T, 28U,
    0U, 0U, 146U, MAX_uint8_T, 56U, 0U, 105U, 246U, 6U, 212U, 239U, 1U, 0U, 176U,
    211U, 0U, 0U, 0U, 80U, MAX_uint8_T, 122U, 0U, 166U, 191U, 0U, 154U,
    MAX_uint8_T, 43U, 5U, 242U, 139U, 0U, 0U, 0U, 17U, 253U, 188U, 0U, 227U,
    130U, 0U, 95U, MAX_uint8_T, 102U, 65U, MAX_uint8_T, 67U, 0U, 0U, 0U, 0U,
    205U, 246U, 40U, MAX_uint8_T, 68U, 0U, 36U, MAX_uint8_T, 160U, 138U, 244U,
    6U, 0U, 0U, 0U, 0U, 139U, MAX_uint8_T, 158U, 251U, 11U, 0U, 0U, 233U, 219U,
    211U, 179U, 0U, 0U, 0U, 0U, 0U, 73U, MAX_uint8_T, 251U, 201U, 0U, 0U, 0U,
    175U, MAX_uint8_T, MAX_uint8_T, 107U, 0U, 0U, 0U, 0U, 0U, 12U, 250U,
    MAX_uint8_T, 139U, 0U, 0U, 0U, 116U, MAX_uint8_T, MAX_uint8_T, 35U, 0U, 0U,
    0U, 0U, 0U, 0U, 197U, MAX_uint8_T, 78U, 0U, 0U, 0U, 57U, MAX_uint8_T, 219U,
    0U, 0U, 0U, 0U, 83U, MAX_uint8_T, 211U, 5U, 0U, 0U, 0U, 0U, 99U, MAX_uint8_T,
    90U, 0U, 177U, MAX_uint8_T, 124U, 0U, 0U, 0U, 29U, 241U, 177U, 0U, 0U, 28U,
    242U, 248U, 39U, 0U, 0U, 185U, 238U, 25U, 0U, 0U, 0U, 109U, MAX_uint8_T,
    193U, 1U, 103U, MAX_uint8_T, 94U, 0U, 0U, 0U, 0U, 2U, 200U, MAX_uint8_T,
    131U, 242U, 181U, 0U, 0U, 0U, 0U, 0U, 0U, 45U, 251U, MAX_uint8_T, 240U, 27U,
    0U, 0U, 0U, 0U, 0U, 0U, 0U, 181U, MAX_uint8_T, 184U, 0U, 0U, 0U, 0U, 0U, 0U,
    0U, 60U, 253U, 251U, MAX_uint8_T, 77U, 0U, 0U, 0U, 0U, 0U, 9U, 218U, 216U,
    77U, MAX_uint8_T, 227U, 13U, 0U, 0U, 0U, 0U, 140U, 253U, 57U, 0U, 161U,
    MAX_uint8_T, 146U, 0U, 0U, 0U, 56U, 253U, 140U, 0U, 0U, 19U, 234U, 253U, 55U,
    0U, 7U, 214U, 217U, 9U, 0U, 0U, 0U, 90U, MAX_uint8_T, 210U, 5U, 135U, 253U,
    58U, 0U, 0U, 0U, 0U, 0U, 181U, MAX_uint8_T, 122U, 143U, MAX_uint8_T, 141U,
    0U, 0U, 0U, 0U, 0U, 25U, 241U, 172U, 16U, 236U, 250U, 38U, 0U, 0U, 0U, 0U,
    169U, 243U, 28U, 0U, 107U, MAX_uint8_T, 179U, 0U, 0U, 0U, 70U, MAX_uint8_T,
    114U, 0U, 0U, 3U, 212U, MAX_uint8_T, 70U, 0U, 8U, 220U, 209U, 4U, 0U, 0U, 0U,
    70U, MAX_uint8_T, 213U, 4U, 131U, 254U, 57U, 0U, 0U, 0U, 0U, 0U, 180U,
    MAX_uint8_T, 145U, 249U, 153U, 0U, 0U, 0U, 0U, 0U, 0U, 39U, 250U,
    MAX_uint8_T, 234U, 18U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 156U, MAX_uint8_T, 111U,
    0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 132U, MAX_uint8_T, 88U, 0U, 0U, 0U, 0U, 0U,
    0U, 0U, 0U, 132U, MAX_uint8_T, 88U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 132U,
    MAX_uint8_T, 88U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 132U, MAX_uint8_T, 88U, 0U,
    0U, 0U, 0U, 0U, 188U, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T,
    MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, 8U, 0U, 188U,
    MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T,
    MAX_uint8_T, MAX_uint8_T, 8U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 144U, MAX_uint8_T,
    164U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 87U, MAX_uint8_T, 211U, 10U, 0U, 0U, 0U,
    0U, 0U, 0U, 42U, 243U, 241U, 37U, 0U, 0U, 0U, 0U, 0U, 0U, 13U, 215U,
    MAX_uint8_T, 81U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 171U, MAX_uint8_T, 137U, 0U,
    0U, 0U, 0U, 0U, 0U, 0U, 115U, MAX_uint8_T, 191U, 3U, 0U, 0U, 0U, 0U, 0U, 0U,
    62U, 251U, 229U, 23U, 0U, 0U, 0U, 0U, 0U, 0U, 25U, 231U, 250U, 59U, 0U, 0U,
    0U, 0U, 0U, 0U, 4U, 195U, MAX_uint8_T, 111U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 44U,
    MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T,
    MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, 8U, 44U, MAX_uint8_T, MAX_uint8_T,
    MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T,
    MAX_uint8_T, 8U, 68U, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, 52U, 68U,
    MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, 52U, 68U, MAX_uint8_T, 64U, 0U, 0U,
    68U, MAX_uint8_T, 64U, 0U, 0U, 68U, MAX_uint8_T, 64U, 0U, 0U, 68U,
    MAX_uint8_T, 64U, 0U, 0U, 68U, MAX_uint8_T, 64U, 0U, 0U, 68U, MAX_uint8_T,
    64U, 0U, 0U, 68U, MAX_uint8_T, 64U, 0U, 0U, 68U, MAX_uint8_T, 64U, 0U, 0U,
    68U, MAX_uint8_T, 64U, 0U, 0U, 68U, MAX_uint8_T, 64U, 0U, 0U, 68U,
    MAX_uint8_T, 64U, 0U, 0U, 68U, MAX_uint8_T, 64U, 0U, 0U, 68U, MAX_uint8_T,
    MAX_uint8_T, MAX_uint8_T, 52U, 68U, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T,
    52U, 43U, MAX_uint8_T, 48U, 0U, 0U, 0U, 0U, 0U, 223U, 123U, 0U, 0U, 0U, 0U,
    0U, 149U, 198U, 0U, 0U, 0U, 0U, 0U, 74U, 253U, 20U, 0U, 0U, 0U, 0U, 8U, 246U,
    92U, 0U, 0U, 0U, 0U, 0U, 180U, 167U, 0U, 0U, 0U, 0U, 0U, 105U, 238U, 3U, 0U,
    0U, 0U, 0U, 30U, MAX_uint8_T, 61U, 0U, 0U, 0U, 0U, 0U, 211U, 136U, 0U, 0U,
    0U, 0U, 0U, 136U, 211U, 0U, 0U, 0U, 0U, 0U, 61U, MAX_uint8_T, 31U, 0U, 0U,
    0U, 0U, 3U, 238U, 105U, 0U, 0U, 0U, 0U, 0U, 167U, 180U, 0U, 0U, 0U, 0U, 0U,
    92U, 246U, 9U, 0U, 0U, 0U, 0U, 19U, 253U, 74U, 0U, 0U, 0U, 0U, 0U, 197U,
    149U, 87U, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, 31U, 87U, MAX_uint8_T,
    MAX_uint8_T, MAX_uint8_T, 31U, 0U, 0U, 99U, MAX_uint8_T, 31U, 0U, 0U, 99U,
    MAX_uint8_T, 31U, 0U, 0U, 99U, MAX_uint8_T, 31U, 0U, 0U, 99U, MAX_uint8_T,
    31U, 0U, 0U, 99U, MAX_uint8_T, 31U, 0U, 0U, 99U, MAX_uint8_T, 31U, 0U, 0U,
    99U, MAX_uint8_T, 31U, 0U, 0U, 99U, MAX_uint8_T, 31U, 0U, 0U, 99U,
    MAX_uint8_T, 31U, 0U, 0U, 99U, MAX_uint8_T, 31U, 0U, 0U, 99U, MAX_uint8_T,
    31U, 0U, 0U, 99U, MAX_uint8_T, 31U, 87U, MAX_uint8_T, MAX_uint8_T,
    MAX_uint8_T, 31U, 87U, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, 31U, 0U, 0U,
    0U, 0U, 0U, 113U, 8U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 17U, 242U, 106U, 0U,
    0U, 0U, 0U, 0U, 0U, 0U, 0U, 126U, MAX_uint8_T, 223U, 4U, 0U, 0U, 0U, 0U, 0U,
    0U, 11U, 236U, 151U, 249U, 94U, 0U, 0U, 0U, 0U, 0U, 0U, 115U, 241U, 16U,
    155U, 214U, 1U, 0U, 0U, 0U, 0U, 7U, 230U, 135U, 0U, 36U, 252U, 82U, 0U, 0U,
    0U, 0U, 104U, 246U, 22U, 0U, 0U, 167U, 203U, 0U, 0U, 0U, 4U, 222U, 146U, 0U,
    0U, 0U, 46U, 254U, 70U, 0U, 0U, 93U, 250U, 29U, 0U, 0U, 0U, 0U, 179U, 192U,
    0U, 1U, 213U, 157U, 0U, 0U, 0U, 0U, 0U, 57U, MAX_uint8_T, 58U, 64U,
    MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T,
    MAX_uint8_T, 60U, 38U, 233U, 204U, 5U, 0U, 0U, 47U, 239U, 135U, 0U, 0U, 0U,
    57U, 241U, 63U, 0U, 3U, 86U, 188U, 235U, 241U, 192U, 50U, 0U, 0U, 0U, 68U,
    MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, 222U, 0U,
    0U, 0U, 63U, 161U, 57U, 12U, 32U, 214U, MAX_uint8_T, 21U, 0U, 0U, 0U, 0U, 0U,
    0U, 0U, 153U, MAX_uint8_T, 32U, 0U, 0U, 0U, 71U, 174U, 227U, 249U,
    MAX_uint8_T, MAX_uint8_T, 32U, 0U, 0U, 114U, MAX_uint8_T, MAX_uint8_T,
    MAX_uint8_T, 254U, MAX_uint8_T, MAX_uint8_T, 32U, 0U, 4U, 242U, 236U, 92U,
    24U, 2U, 152U, MAX_uint8_T, 33U, 0U, 12U, MAX_uint8_T, 211U, 39U, 12U, 60U,
    202U, MAX_uint8_T, 89U, 0U, 0U, 200U, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T,
    MAX_uint8_T, 217U, MAX_uint8_T, MAX_uint8_T, 105U, 0U, 33U, 185U, 244U, 217U,
    106U, 12U, 198U, 240U, 84U, 68U, MAX_uint8_T, 120U, 0U, 0U, 0U, 0U, 0U, 0U,
    0U, 68U, MAX_uint8_T, 120U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 68U, MAX_uint8_T,
    120U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 68U, MAX_uint8_T, 120U, 39U, 187U, 245U,
    225U, 121U, 1U, 0U, 68U, MAX_uint8_T, 135U, 228U, MAX_uint8_T, MAX_uint8_T,
    MAX_uint8_T, MAX_uint8_T, 117U, 0U, 68U, MAX_uint8_T, 238U, 167U, 40U, 13U,
    96U, 253U, 237U, 11U, 68U, MAX_uint8_T, 162U, 0U, 0U, 0U, 0U, 178U,
    MAX_uint8_T, 52U, 68U, MAX_uint8_T, 120U, 0U, 0U, 0U, 0U, 134U, MAX_uint8_T,
    82U, 68U, MAX_uint8_T, 120U, 0U, 0U, 0U, 0U, 144U, MAX_uint8_T, 78U, 68U,
    MAX_uint8_T, 126U, 0U, 0U, 0U, 0U, 201U, MAX_uint8_T, 38U, 68U, MAX_uint8_T,
    248U, 119U, 20U, 22U, 130U, MAX_uint8_T, 209U, 0U, 68U, MAX_uint8_T, 158U,
    250U, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, 250U, 64U, 0U, 68U, MAX_uint8_T,
    77U, 72U, 214U, 248U, 201U, 67U, 0U, 0U, 0U, 0U, 15U, 138U, 220U, 248U, 215U,
    109U, 4U, 0U, 14U, 213U, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T,
    MAX_uint8_T, 64U, 0U, 138U, MAX_uint8_T, 234U, 87U, 16U, 39U, 148U, 59U, 0U,
    222U, MAX_uint8_T, 79U, 0U, 0U, 0U, 0U, 0U, 2U, 253U, 251U, 6U, 0U, 0U, 0U,
    0U, 0U, 1U, 252U, 250U, 5U, 0U, 0U, 0U, 0U, 0U, 0U, 211U, MAX_uint8_T, 80U,
    0U, 0U, 0U, 0U, 0U, 0U, 115U, MAX_uint8_T, 235U, 90U, 17U, 30U, 123U, 82U,
    0U, 5U, 193U, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T,
    MAX_uint8_T, 92U, 0U, 0U, 9U, 133U, 224U, 246U, 204U, 105U, 7U, 0U, 0U, 0U,
    0U, 0U, 0U, 0U, 36U, MAX_uint8_T, 152U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 36U,
    MAX_uint8_T, 152U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 36U, MAX_uint8_T, 152U, 0U,
    0U, 25U, 165U, 239U, 236U, 130U, 37U, MAX_uint8_T, 152U, 0U, 12U, 218U,
    MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, 167U, MAX_uint8_T, 152U,
    0U, 126U, MAX_uint8_T, 195U, 42U, 9U, 77U, 218U, MAX_uint8_T, 152U, 0U, 210U,
    254U, 33U, 0U, 0U, 0U, 41U, MAX_uint8_T, 152U, 0U, 250U, 231U, 0U, 0U, 0U,
    0U, 36U, MAX_uint8_T, 152U, 2U, 252U, 221U, 0U, 0U, 0U, 0U, 36U, MAX_uint8_T,
    152U, 0U, 226U, 249U, 15U, 0U, 0U, 0U, 79U, MAX_uint8_T, 152U, 0U, 167U,
    MAX_uint8_T, 166U, 25U, 17U, 114U, 229U, MAX_uint8_T, 152U, 0U, 44U, 250U,
    MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, 254U, 109U, MAX_uint8_T, 152U, 0U, 0U,
    69U, 205U, 248U, 217U, 87U, 36U, MAX_uint8_T, 152U, 0U, 0U, 23U, 160U, 236U,
    245U, 177U, 27U, 0U, 0U, 14U, 218U, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T,
    MAX_uint8_T, 209U, 3U, 0U, 135U, MAX_uint8_T, 151U, 21U, 17U, 146U,
    MAX_uint8_T, 83U, 0U, 220U, 241U, 4U, 0U, 0U, 12U, MAX_uint8_T, 151U, 2U,
    254U, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T,
    MAX_uint8_T, 183U, 1U, 252U, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T,
    MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, 194U, 0U, 210U, 235U, 8U, 0U, 0U, 0U,
    0U, 0U, 0U, 114U, MAX_uint8_T, 189U, 59U, 13U, 22U, 95U, 152U, 0U, 4U, 186U,
    MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, 188U, 0U,
    0U, 5U, 119U, 214U, 248U, 221U, 145U, 31U, 0U, 0U, 39U, 183U, 238U,
    MAX_uint8_T, MAX_uint8_T, 36U, 0U, 0U, 197U, MAX_uint8_T, MAX_uint8_T,
    MAX_uint8_T, MAX_uint8_T, 36U, 0U, 0U, 238U, 221U, 16U, 0U, 0U, 0U, 76U,
    MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, 44U, 0U,
    76U, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, 44U,
    0U, 0U, 0U, 240U, 204U, 0U, 0U, 0U, 0U, 0U, 0U, 240U, 204U, 0U, 0U, 0U, 0U,
    0U, 0U, 240U, 204U, 0U, 0U, 0U, 0U, 0U, 0U, 240U, 204U, 0U, 0U, 0U, 0U, 0U,
    0U, 240U, 204U, 0U, 0U, 0U, 0U, 0U, 0U, 240U, 204U, 0U, 0U, 0U, 0U, 0U, 0U,
    240U, 204U, 0U, 0U, 0U, 0U, 0U, 0U, 240U, 204U, 0U, 0U, 0U, 0U, 0U, 0U, 22U,
    160U, 238U, 238U, 131U, 37U, MAX_uint8_T, 152U, 0U, 11U, 215U, MAX_uint8_T,
    MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, 166U, MAX_uint8_T, 152U, 0U, 127U,
    MAX_uint8_T, 211U, 52U, 9U, 64U, 211U, MAX_uint8_T, 152U, 0U, 213U,
    MAX_uint8_T, 47U, 0U, 0U, 0U, 41U, MAX_uint8_T, 152U, 0U, 252U, 234U, 0U, 0U,
    0U, 0U, 36U, MAX_uint8_T, 152U, 3U, MAX_uint8_T, 221U, 0U, 0U, 0U, 0U, 36U,
    MAX_uint8_T, 152U, 0U, 232U, 249U, 14U, 0U, 0U, 0U, 77U, MAX_uint8_T, 152U,
    0U, 164U, MAX_uint8_T, 164U, 24U, 17U, 111U, 230U, MAX_uint8_T, 151U, 0U,
    40U, 245U, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, 119U,
    MAX_uint8_T, 146U, 0U, 0U, 57U, 199U, 248U, 222U, 99U, 55U, MAX_uint8_T,
    133U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 117U, MAX_uint8_T, 105U, 0U, 21U, 187U,
    71U, 18U, 20U, 95U, 242U, MAX_uint8_T, 38U, 0U, 47U, MAX_uint8_T,
    MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, 155U, 0U,
    0U, 3U, 90U, 194U, 238U, 247U, 209U, 114U, 2U, 0U, 68U, MAX_uint8_T, 120U,
    0U, 0U, 0U, 0U, 0U, 0U, 68U, MAX_uint8_T, 120U, 0U, 0U, 0U, 0U, 0U, 0U, 68U,
    MAX_uint8_T, 120U, 0U, 0U, 0U, 0U, 0U, 0U, 68U, MAX_uint8_T, 120U, 18U, 163U,
    240U, 223U, 97U, 0U, 68U, MAX_uint8_T, 127U, 205U, MAX_uint8_T, MAX_uint8_T,
    MAX_uint8_T, 254U, 49U, 68U, MAX_uint8_T, 233U, 213U, 62U, 11U, 143U,
    MAX_uint8_T, 117U, 68U, MAX_uint8_T, 221U, 18U, 0U, 0U, 57U, MAX_uint8_T,
    136U, 68U, MAX_uint8_T, 120U, 0U, 0U, 0U, 48U, MAX_uint8_T, 136U, 68U,
    MAX_uint8_T, 120U, 0U, 0U, 0U, 48U, MAX_uint8_T, 136U, 68U, MAX_uint8_T,
    120U, 0U, 0U, 0U, 48U, MAX_uint8_T, 136U, 68U, MAX_uint8_T, 120U, 0U, 0U, 0U,
    48U, MAX_uint8_T, 136U, 68U, MAX_uint8_T, 120U, 0U, 0U, 0U, 48U, MAX_uint8_T,
    136U, 68U, MAX_uint8_T, 120U, 0U, 0U, 0U, 48U, MAX_uint8_T, 136U, 68U,
    MAX_uint8_T, 120U, 68U, MAX_uint8_T, 120U, 0U, 0U, 0U, 68U, MAX_uint8_T,
    120U, 68U, MAX_uint8_T, 120U, 68U, MAX_uint8_T, 120U, 68U, MAX_uint8_T, 120U,
    68U, MAX_uint8_T, 120U, 68U, MAX_uint8_T, 120U, 68U, MAX_uint8_T, 120U, 68U,
    MAX_uint8_T, 120U, 68U, MAX_uint8_T, 120U, 68U, MAX_uint8_T, 120U, 0U, 0U,
    0U, 0U, MAX_uint8_T, 192U, 0U, 0U, 0U, 0U, MAX_uint8_T, 192U, 0U, 0U, 0U, 0U,
    0U, 0U, 0U, 0U, 0U, 0U, MAX_uint8_T, 192U, 0U, 0U, 0U, 0U, MAX_uint8_T, 192U,
    0U, 0U, 0U, 0U, MAX_uint8_T, 192U, 0U, 0U, 0U, 0U, MAX_uint8_T, 192U, 0U, 0U,
    0U, 0U, MAX_uint8_T, 192U, 0U, 0U, 0U, 0U, MAX_uint8_T, 192U, 0U, 0U, 0U, 0U,
    MAX_uint8_T, 192U, 0U, 0U, 0U, 0U, MAX_uint8_T, 192U, 0U, 0U, 0U, 0U,
    MAX_uint8_T, 192U, 0U, 0U, 0U, 1U, MAX_uint8_T, 192U, 0U, 0U, 0U, 15U,
    MAX_uint8_T, 179U, 0U, 0U, 5U, 105U, MAX_uint8_T, 149U, 92U, MAX_uint8_T,
    MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, 56U, 92U, MAX_uint8_T, 252U, 216U,
    96U, 0U, 68U, MAX_uint8_T, 120U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 68U,
    MAX_uint8_T, 120U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 68U, MAX_uint8_T, 120U, 0U,
    0U, 0U, 0U, 0U, 0U, 0U, 68U, MAX_uint8_T, 120U, 0U, 0U, 15U, 215U, 224U, 21U,
    0U, 68U, MAX_uint8_T, 120U, 0U, 3U, 184U, 244U, 46U, 0U, 0U, 68U,
    MAX_uint8_T, 120U, 0U, 145U, 254U, 81U, 0U, 0U, 0U, 68U, MAX_uint8_T, 120U,
    102U, MAX_uint8_T, 123U, 0U, 0U, 0U, 0U, 68U, MAX_uint8_T, 180U, 250U, 190U,
    0U, 0U, 0U, 0U, 0U, 68U, MAX_uint8_T, 146U, 226U, 250U, 66U, 0U, 0U, 0U, 0U,
    68U, MAX_uint8_T, 120U, 44U, 240U, 240U, 44U, 0U, 0U, 0U, 68U, MAX_uint8_T,
    120U, 0U, 66U, 250U, 226U, 26U, 0U, 0U, 68U, MAX_uint8_T, 120U, 0U, 0U, 93U,
    MAX_uint8_T, 207U, 12U, 0U, 68U, MAX_uint8_T, 120U, 0U, 0U, 0U, 124U,
    MAX_uint8_T, 183U, 4U, 68U, MAX_uint8_T, 120U, 68U, MAX_uint8_T, 120U, 68U,
    MAX_uint8_T, 120U, 68U, MAX_uint8_T, 120U, 68U, MAX_uint8_T, 120U, 68U,
    MAX_uint8_T, 120U, 68U, MAX_uint8_T, 120U, 68U, MAX_uint8_T, 120U, 68U,
    MAX_uint8_T, 120U, 68U, MAX_uint8_T, 120U, 68U, MAX_uint8_T, 120U, 68U,
    MAX_uint8_T, 120U, 68U, MAX_uint8_T, 120U, 68U, MAX_uint8_T, 120U, 65U, 197U,
    246U, 215U, 65U, 0U, 59U, 201U, 245U, 203U, 51U, 0U, 68U, MAX_uint8_T, 189U,
    252U, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, 239U, 63U, 247U, MAX_uint8_T,
    MAX_uint8_T, MAX_uint8_T, 221U, 1U, 68U, MAX_uint8_T, MAX_uint8_T, 176U, 43U,
    15U, 188U, MAX_uint8_T, 246U, 151U, 32U, 23U, 214U, MAX_uint8_T, 29U, 68U,
    MAX_uint8_T, 166U, 1U, 0U, 0U, 113U, MAX_uint8_T, 130U, 0U, 0U, 0U, 149U,
    MAX_uint8_T, 44U, 68U, MAX_uint8_T, 120U, 0U, 0U, 0U, 108U, MAX_uint8_T, 84U,
    0U, 0U, 0U, 144U, MAX_uint8_T, 44U, 68U, MAX_uint8_T, 120U, 0U, 0U, 0U, 108U,
    MAX_uint8_T, 84U, 0U, 0U, 0U, 144U, MAX_uint8_T, 44U, 68U, MAX_uint8_T, 120U,
    0U, 0U, 0U, 108U, MAX_uint8_T, 84U, 0U, 0U, 0U, 144U, MAX_uint8_T, 44U, 68U,
    MAX_uint8_T, 120U, 0U, 0U, 0U, 108U, MAX_uint8_T, 84U, 0U, 0U, 0U, 144U,
    MAX_uint8_T, 44U, 68U, MAX_uint8_T, 120U, 0U, 0U, 0U, 108U, MAX_uint8_T, 84U,
    0U, 0U, 0U, 144U, MAX_uint8_T, 44U, 68U, MAX_uint8_T, 120U, 0U, 0U, 0U, 108U,
    MAX_uint8_T, 84U, 0U, 0U, 0U, 144U, MAX_uint8_T, 44U, 68U, MAX_uint8_T, 120U,
    18U, 163U, 240U, 223U, 97U, 0U, 68U, MAX_uint8_T, 127U, 205U, MAX_uint8_T,
    MAX_uint8_T, MAX_uint8_T, 254U, 49U, 68U, MAX_uint8_T, 233U, 213U, 62U, 11U,
    143U, MAX_uint8_T, 117U, 68U, MAX_uint8_T, 221U, 18U, 0U, 0U, 57U,
    MAX_uint8_T, 136U, 68U, MAX_uint8_T, 120U, 0U, 0U, 0U, 48U, MAX_uint8_T,
    136U, 68U, MAX_uint8_T, 120U, 0U, 0U, 0U, 48U, MAX_uint8_T, 136U, 68U,
    MAX_uint8_T, 120U, 0U, 0U, 0U, 48U, MAX_uint8_T, 136U, 68U, MAX_uint8_T,
    120U, 0U, 0U, 0U, 48U, MAX_uint8_T, 136U, 68U, MAX_uint8_T, 120U, 0U, 0U, 0U,
    48U, MAX_uint8_T, 136U, 68U, MAX_uint8_T, 120U, 0U, 0U, 0U, 48U, MAX_uint8_T,
    136U, 0U, 0U, 18U, 145U, 225U, 250U, 228U, 151U, 21U, 0U, 0U, 0U, 14U, 216U,
    MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, 220U, 18U,
    0U, 0U, 136U, MAX_uint8_T, 199U, 49U, 10U, 44U, 187U, MAX_uint8_T, 146U, 0U,
    0U, 220U, 253U, 32U, 0U, 0U, 0U, 20U, 246U, 231U, 0U, 1U, 253U, 228U, 0U, 0U,
    0U, 0U, 0U, 208U, MAX_uint8_T, 10U, 2U, 253U, 227U, 0U, 0U, 0U, 0U, 0U, 209U,
    MAX_uint8_T, 10U, 0U, 219U, 253U, 30U, 0U, 0U, 0U, 21U, 246U, 231U, 0U, 0U,
    134U, MAX_uint8_T, 197U, 47U, 9U, 45U, 189U, MAX_uint8_T, 148U, 0U, 0U, 13U,
    214U, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, 222U,
    19U, 0U, 0U, 0U, 19U, 148U, 229U, 250U, 228U, 150U, 22U, 0U, 0U, 68U,
    MAX_uint8_T, 120U, 39U, 187U, 245U, 225U, 121U, 1U, 0U, 68U, MAX_uint8_T,
    135U, 228U, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, 117U, 0U,
    68U, MAX_uint8_T, 238U, 167U, 40U, 13U, 96U, 253U, 237U, 11U, 68U,
    MAX_uint8_T, 162U, 0U, 0U, 0U, 0U, 178U, MAX_uint8_T, 52U, 68U, MAX_uint8_T,
    120U, 0U, 0U, 0U, 0U, 134U, MAX_uint8_T, 82U, 68U, MAX_uint8_T, 120U, 0U, 0U,
    0U, 0U, 144U, MAX_uint8_T, 78U, 68U, MAX_uint8_T, 126U, 0U, 0U, 0U, 0U, 201U,
    MAX_uint8_T, 38U, 68U, MAX_uint8_T, 248U, 119U, 20U, 22U, 130U, MAX_uint8_T,
    209U, 0U, 68U, MAX_uint8_T, 173U, 250U, MAX_uint8_T, MAX_uint8_T,
    MAX_uint8_T, 250U, 64U, 0U, 68U, MAX_uint8_T, 120U, 72U, 214U, 248U, 201U,
    67U, 0U, 0U, 68U, MAX_uint8_T, 120U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 68U,
    MAX_uint8_T, 120U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 68U, MAX_uint8_T, 120U, 0U,
    0U, 0U, 0U, 0U, 0U, 0U, 68U, MAX_uint8_T, 120U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
    0U, 0U, 25U, 165U, 239U, 236U, 130U, 37U, MAX_uint8_T, 152U, 0U, 12U, 218U,
    MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, 167U, MAX_uint8_T, 152U,
    0U, 126U, MAX_uint8_T, 195U, 42U, 9U, 77U, 218U, MAX_uint8_T, 152U, 0U, 210U,
    254U, 33U, 0U, 0U, 0U, 41U, MAX_uint8_T, 152U, 0U, 250U, 231U, 0U, 0U, 0U,
    0U, 36U, MAX_uint8_T, 152U, 2U, 252U, 221U, 0U, 0U, 0U, 0U, 36U, MAX_uint8_T,
    152U, 0U, 226U, 249U, 15U, 0U, 0U, 0U, 79U, MAX_uint8_T, 152U, 0U, 167U,
    MAX_uint8_T, 166U, 25U, 17U, 114U, 229U, MAX_uint8_T, 152U, 0U, 44U, 250U,
    MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, 254U, 109U, MAX_uint8_T, 152U, 0U, 0U,
    69U, 205U, 248U, 217U, 87U, 36U, MAX_uint8_T, 152U, 0U, 0U, 0U, 0U, 0U, 0U,
    0U, 36U, MAX_uint8_T, 152U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 36U, MAX_uint8_T,
    152U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 36U, MAX_uint8_T, 152U, 0U, 0U, 0U, 0U, 0U,
    0U, 0U, 36U, MAX_uint8_T, 152U, 68U, MAX_uint8_T, 120U, 47U, 199U, 251U, 68U,
    MAX_uint8_T, 138U, 232U, MAX_uint8_T, MAX_uint8_T, 68U, MAX_uint8_T, 239U,
    176U, 37U, 0U, 68U, MAX_uint8_T, 197U, 3U, 0U, 0U, 68U, MAX_uint8_T, 120U,
    0U, 0U, 0U, 68U, MAX_uint8_T, 120U, 0U, 0U, 0U, 68U, MAX_uint8_T, 120U, 0U,
    0U, 0U, 68U, MAX_uint8_T, 120U, 0U, 0U, 0U, 68U, MAX_uint8_T, 120U, 0U, 0U,
    0U, 68U, MAX_uint8_T, 120U, 0U, 0U, 0U, 0U, 88U, 207U, 244U, 209U, 90U, 0U,
    62U, 254U, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, 20U, 130U,
    MAX_uint8_T, 73U, 5U, 48U, 171U, 19U, 97U, MAX_uint8_T, 182U, 31U, 0U, 0U,
    0U, 4U, 170U, MAX_uint8_T, 252U, 167U, 36U, 0U, 0U, 0U, 71U, 188U,
    MAX_uint8_T, 242U, 53U, 0U, 0U, 0U, 0U, 85U, MAX_uint8_T, 175U, 136U, 119U,
    38U, 6U, 56U, 252U, 181U, 160U, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T,
    MAX_uint8_T, MAX_uint8_T, 86U, 24U, 137U, 219U, 244U, 205U, 86U, 0U, 0U, 43U,
    229U, 132U, 0U, 0U, 0U, 0U, 52U, MAX_uint8_T, 136U, 0U, 0U, 0U, 100U,
    MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, 8U, 100U,
    MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, 8U, 0U, 52U,
    MAX_uint8_T, 136U, 0U, 0U, 0U, 0U, 52U, MAX_uint8_T, 136U, 0U, 0U, 0U, 0U,
    52U, MAX_uint8_T, 136U, 0U, 0U, 0U, 0U, 52U, MAX_uint8_T, 136U, 0U, 0U, 0U,
    0U, 52U, MAX_uint8_T, 141U, 0U, 0U, 0U, 0U, 42U, MAX_uint8_T, 217U, 33U, 0U,
    0U, 0U, 6U, 232U, MAX_uint8_T, MAX_uint8_T, 252U, 0U, 0U, 0U, 59U, 204U,
    249U, 252U, 0U, 96U, MAX_uint8_T, 92U, 0U, 0U, 0U, 76U, MAX_uint8_T, 112U,
    96U, MAX_uint8_T, 92U, 0U, 0U, 0U, 76U, MAX_uint8_T, 112U, 96U, MAX_uint8_T,
    92U, 0U, 0U, 0U, 76U, MAX_uint8_T, 112U, 96U, MAX_uint8_T, 92U, 0U, 0U, 0U,
    76U, MAX_uint8_T, 112U, 96U, MAX_uint8_T, 92U, 0U, 0U, 0U, 76U, MAX_uint8_T,
    112U, 96U, MAX_uint8_T, 92U, 0U, 0U, 0U, 76U, MAX_uint8_T, 112U, 96U,
    MAX_uint8_T, 101U, 0U, 0U, 4U, 188U, MAX_uint8_T, 112U, 77U, MAX_uint8_T,
    181U, 13U, 44U, 187U, 233U, MAX_uint8_T, 112U, 21U, 243U, MAX_uint8_T,
    MAX_uint8_T, MAX_uint8_T, 231U, 101U, MAX_uint8_T, 112U, 0U, 72U, 213U, 244U,
    184U, 37U, 76U, MAX_uint8_T, 112U, 171U, 251U, 21U, 0U, 0U, 0U, 0U, 152U,
    247U, 16U, 80U, MAX_uint8_T, 107U, 0U, 0U, 0U, 7U, 238U, 166U, 0U, 6U, 238U,
    197U, 0U, 0U, 0U, 85U, MAX_uint8_T, 69U, 0U, 0U, 154U, MAX_uint8_T, 32U, 0U,
    0U, 180U, 226U, 2U, 0U, 0U, 64U, MAX_uint8_T, 122U, 0U, 23U, 251U, 131U, 0U,
    0U, 0U, 1U, 227U, 213U, 0U, 114U, MAX_uint8_T, 36U, 0U, 0U, 0U, 0U, 138U,
    MAX_uint8_T, 47U, 209U, 194U, 0U, 0U, 0U, 0U, 0U, 47U, MAX_uint8_T, 184U,
    MAX_uint8_T, 97U, 0U, 0U, 0U, 0U, 0U, 0U, 212U, MAX_uint8_T, 244U, 12U, 0U,
    0U, 0U, 0U, 0U, 0U, 121U, MAX_uint8_T, 160U, 0U, 0U, 0U, 0U, 194U, 235U, 1U,
    0U, 0U, 7U, 245U, 249U, 10U, 0U, 0U, 0U, 226U, 151U, 124U, MAX_uint8_T, 50U,
    0U, 0U, 71U, MAX_uint8_T, MAX_uint8_T, 68U, 0U, 0U, 50U, MAX_uint8_T, 72U,
    54U, MAX_uint8_T, 118U, 0U, 0U, 145U, 210U, 251U, 132U, 0U, 0U, 129U, 242U,
    6U, 2U, 237U, 187U, 0U, 0U, 219U, 123U, 200U, 196U, 0U, 0U, 207U, 169U, 0U,
    0U, 170U, 247U, 8U, 36U, MAX_uint8_T, 49U, 134U, 250U, 10U, 30U, MAX_uint8_T,
    89U, 0U, 0U, 100U, MAX_uint8_T, 67U, 111U, 229U, 1U, 68U, MAX_uint8_T, 69U,
    108U, 250U, 15U, 0U, 0U, 29U, MAX_uint8_T, 136U, 185U, 156U, 0U, 9U, 249U,
    133U, 187U, 186U, 0U, 0U, 0U, 0U, 216U, 213U, 248U, 81U, 0U, 0U, 193U, 210U,
    250U, 106U, 0U, 0U, 0U, 0U, 146U, MAX_uint8_T, 250U, 13U, 0U, 0U, 127U,
    MAX_uint8_T, MAX_uint8_T, 28U, 0U, 0U, 0U, 0U, 75U, MAX_uint8_T, 189U, 0U,
    0U, 0U, 61U, MAX_uint8_T, 203U, 0U, 0U, 0U, 2U, 191U, MAX_uint8_T, 76U, 0U,
    0U, 0U, 32U, 243U, 156U, 0U, 0U, 27U, 236U, 235U, 26U, 0U, 0U, 188U, 229U,
    15U, 0U, 0U, 0U, 77U, MAX_uint8_T, 188U, 2U, 102U, MAX_uint8_T, 78U, 0U, 0U,
    0U, 0U, 0U, 145U, MAX_uint8_T, 144U, 241U, 167U, 0U, 0U, 0U, 0U, 0U, 0U, 7U,
    211U, MAX_uint8_T, 235U, 21U, 0U, 0U, 0U, 0U, 0U, 0U, 31U, 241U, MAX_uint8_T,
    224U, 15U, 0U, 0U, 0U, 0U, 0U, 2U, 193U, 230U, 115U, MAX_uint8_T, 169U, 0U,
    0U, 0U, 0U, 0U, 120U, 254U, 71U, 0U, 165U, MAX_uint8_T, 100U, 0U, 0U, 0U,
    49U, 249U, 146U, 0U, 0U, 14U, 221U, 245U, 41U, 0U, 8U, 213U, 213U, 8U, 0U,
    0U, 0U, 54U, 250U, 209U, 8U, 173U, MAX_uint8_T, 49U, 0U, 0U, 0U, 0U, 138U,
    254U, 30U, 79U, MAX_uint8_T, 142U, 0U, 0U, 0U, 4U, 231U, 188U, 0U, 5U, 234U,
    232U, 3U, 0U, 0U, 79U, MAX_uint8_T, 93U, 0U, 0U, 146U, MAX_uint8_T, 72U, 0U,
    0U, 178U, 243U, 11U, 0U, 0U, 52U, MAX_uint8_T, 165U, 0U, 24U, 251U, 159U, 0U,
    0U, 0U, 0U, 212U, 245U, 13U, 119U, MAX_uint8_T, 64U, 0U, 0U, 0U, 0U, 119U,
    MAX_uint8_T, 96U, 217U, 223U, 1U, 0U, 0U, 0U, 0U, 27U, 253U, 228U,
    MAX_uint8_T, 129U, 0U, 0U, 0U, 0U, 0U, 0U, 186U, MAX_uint8_T, MAX_uint8_T,
    35U, 0U, 0U, 0U, 0U, 0U, 0U, 91U, MAX_uint8_T, 195U, 0U, 0U, 0U, 0U, 0U, 0U,
    0U, 94U, MAX_uint8_T, 100U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 193U, 246U, 14U, 0U,
    0U, 0U, 0U, 0U, 0U, 36U, MAX_uint8_T, 166U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 135U,
    MAX_uint8_T, 71U, 0U, 0U, 0U, 0U, 0U, 164U, MAX_uint8_T, MAX_uint8_T,
    MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, 32U, 164U,
    MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T,
    MAX_uint8_T, 32U, 0U, 0U, 0U, 0U, 0U, 130U, MAX_uint8_T, 159U, 1U, 0U, 0U,
    0U, 0U, 126U, MAX_uint8_T, 163U, 2U, 0U, 0U, 0U, 0U, 122U, MAX_uint8_T, 166U,
    3U, 0U, 0U, 0U, 0U, 118U, MAX_uint8_T, 169U, 3U, 0U, 0U, 0U, 0U, 114U,
    MAX_uint8_T, 173U, 4U, 0U, 0U, 0U, 0U, 110U, MAX_uint8_T, 176U, 5U, 0U, 0U,
    0U, 0U, 0U, 236U, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T,
    MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, 60U, 236U, MAX_uint8_T, MAX_uint8_T,
    MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, 60U, 0U, 0U,
    74U, 198U, 161U, 0U, 52U, 252U, 86U, 3U, 0U, 113U, MAX_uint8_T, 24U, 0U, 0U,
    91U, MAX_uint8_T, 67U, 0U, 0U, 43U, MAX_uint8_T, 91U, 0U, 0U, 12U,
    MAX_uint8_T, 80U, 0U, 2U, 84U, 230U, 12U, 0U, 200U, MAX_uint8_T, 86U, 0U, 0U,
    2U, 93U, 216U, 4U, 0U, 0U, 11U, MAX_uint8_T, 65U, 0U, 0U, 29U, MAX_uint8_T,
    94U, 0U, 0U, 72U, MAX_uint8_T, 83U, 0U, 0U, 106U, MAX_uint8_T, 53U, 0U, 0U,
    106U, MAX_uint8_T, 17U, 0U, 0U, 33U, 245U, 95U, 3U, 0U, 0U, 58U, 191U, 160U,
    76U, MAX_uint8_T, 4U, 76U, MAX_uint8_T, 4U, 76U, MAX_uint8_T, 4U, 76U,
    MAX_uint8_T, 4U, 76U, MAX_uint8_T, 4U, 76U, MAX_uint8_T, 4U, 76U,
    MAX_uint8_T, 4U, 76U, MAX_uint8_T, 4U, 76U, MAX_uint8_T, 4U, 76U,
    MAX_uint8_T, 4U, 76U, MAX_uint8_T, 4U, 76U, MAX_uint8_T, 4U, 76U,
    MAX_uint8_T, 4U, 76U, MAX_uint8_T, 4U, 76U, MAX_uint8_T, 4U, 76U,
    MAX_uint8_T, 4U, 193U, 186U, 55U, 0U, 0U, 5U, 121U, 242U, 25U, 0U, 0U, 60U,
    MAX_uint8_T, 77U, 0U, 0U, 103U, MAX_uint8_T, 54U, 0U, 0U, 127U, 253U, 9U, 0U,
    0U, 117U, 232U, 0U, 0U, 0U, 32U, 241U, 53U, 0U, 0U, 0U, 122U, MAX_uint8_T,
    163U, 0U, 19U, 236U, 60U, 0U, 0U, 102U, 230U, 0U, 0U, 0U, 131U, 247U, 1U, 0U,
    0U, 119U, MAX_uint8_T, 35U, 0U, 0U, 89U, MAX_uint8_T, 68U, 0U, 0U, 54U,
    MAX_uint8_T, 69U, 0U, 6U, 130U, 227U, 13U, 0U, 191U, 177U, 39U, 0U, 0U, 0U,
    35U, 198U, 241U, 173U, 51U, 0U, 0U, 0U, 222U, 98U, 0U, 191U, 147U, 19U, 120U,
    238U, 167U, 43U, 49U, 252U, 38U, 5U, 252U, 66U, 0U, 0U, 17U, 128U, 223U,
    236U, 115U, 0U, 4U, MAX_uint8_T, 180U, 4U, MAX_uint8_T, 180U, 0U, 0U, 0U, 0U,
    0U, 0U, 0U, 209U, 131U, 0U, 217U, 139U, 0U, 226U, 147U, 0U, 234U, 155U, 0U,
    243U, 163U, 0U, 251U, 171U, 2U, MAX_uint8_T, 178U, 4U, MAX_uint8_T, 180U, 4U,
    MAX_uint8_T, 180U, 0U, 0U, 0U, 0U, 44U, 180U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
    44U, 180U, 0U, 0U, 0U, 0U, 0U, 30U, 160U, 236U, MAX_uint8_T, MAX_uint8_T,
    MAX_uint8_T, 48U, 0U, 36U, 234U, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T,
    MAX_uint8_T, MAX_uint8_T, 48U, 0U, 180U, MAX_uint8_T, 168U, 80U, 180U, 0U,
    0U, 0U, 12U, 252U, 238U, 9U, 44U, 180U, 0U, 0U, 0U, 44U, MAX_uint8_T, 193U,
    0U, 44U, 180U, 0U, 0U, 0U, 42U, MAX_uint8_T, 195U, 0U, 44U, 180U, 0U, 0U, 0U,
    8U, 248U, 245U, 20U, 44U, 180U, 0U, 0U, 0U, 0U, 162U, MAX_uint8_T, 202U, 97U,
    180U, 0U, 0U, 0U, 0U, 24U, 225U, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T,
    MAX_uint8_T, MAX_uint8_T, 48U, 0U, 0U, 26U, 162U, 239U, MAX_uint8_T,
    MAX_uint8_T, MAX_uint8_T, 48U, 0U, 0U, 0U, 0U, 44U, 180U, 0U, 0U, 0U, 0U, 0U,
    5U, 126U, 219U, 242U, 167U, 20U, 0U, 0U, 127U, MAX_uint8_T, MAX_uint8_T,
    MAX_uint8_T, MAX_uint8_T, 120U, 0U, 0U, 218U, 242U, 51U, 14U, 91U, 100U, 0U,
    0U, 249U, 194U, 0U, 0U, 0U, 0U, 0U, 0U, 252U, 192U, 0U, 0U, 0U, 0U, 144U,
    MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, 164U, 0U, 0U, 144U,
    MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, 164U, 0U, 0U, 0U, 0U,
    252U, 191U, 0U, 0U, 0U, 0U, 0U, 0U, 253U, 180U, 0U, 0U, 0U, 0U, 0U, 37U,
    MAX_uint8_T, 126U, 0U, 0U, 0U, 0U, 36U, 196U, 192U, 10U, 0U, 0U, 0U, 0U,
    212U, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T,
    MAX_uint8_T, 152U, 212U, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T,
    MAX_uint8_T, MAX_uint8_T, 152U, 4U, 192U, 44U, 0U, 0U, 0U, 0U, 0U, 5U, 175U,
    61U, 0U, 83U, 230U, 96U, 195U, 243U, 220U, 123U, 171U, 177U, 1U, 0U, 0U,
    156U, 246U, 97U, 15U, 52U, 209U, 241U, 13U, 0U, 0U, 2U, 223U, 119U, 0U, 0U,
    0U, 34U, 249U, 67U, 0U, 0U, 36U, MAX_uint8_T, 40U, 0U, 0U, 0U, 0U, 204U,
    132U, 0U, 0U, 36U, MAX_uint8_T, 40U, 0U, 0U, 0U, 0U, 205U, 131U, 0U, 0U, 2U,
    223U, 120U, 0U, 0U, 0U, 34U, 249U, 67U, 0U, 0U, 0U, 156U, 246U, 96U, 14U,
    53U, 209U, 241U, 13U, 0U, 0U, 82U, 229U, 93U, 194U, 244U, 220U, 122U, 169U,
    177U, 1U, 4U, 193U, 44U, 0U, 0U, 0U, 0U, 0U, 5U, 175U, 62U, 19U, 238U, 242U,
    28U, 0U, 0U, 0U, 0U, 22U, 235U, 150U, 0U, 109U, MAX_uint8_T, 174U, 0U, 0U,
    0U, 0U, 172U, 230U, 15U, 0U, 4U, 212U, MAX_uint8_T, 78U, 0U, 0U, 88U,
    MAX_uint8_T, 84U, 0U, 0U, 0U, 68U, MAX_uint8_T, 225U, 11U, 22U, 236U, 179U,
    0U, 0U, 0U, 0U, 0U, 176U, MAX_uint8_T, 141U, 174U, 244U, 30U, 0U, 0U, 0U, 0U,
    0U, 34U, 248U, 254U, MAX_uint8_T, 113U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 134U,
    MAX_uint8_T, 205U, 3U, 0U, 0U, 0U, 0U, 0U, 212U, MAX_uint8_T, MAX_uint8_T,
    MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, 28U, 0U, 0U, 0U, 0U, 0U,
    60U, MAX_uint8_T, 128U, 0U, 0U, 0U, 0U, 0U, 0U, 212U, MAX_uint8_T,
    MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, 28U, 0U, 0U,
    0U, 0U, 0U, 60U, MAX_uint8_T, 128U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 60U,
    MAX_uint8_T, 128U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 60U, MAX_uint8_T, 128U,
    0U, 0U, 0U, 0U, 76U, MAX_uint8_T, 4U, 76U, MAX_uint8_T, 4U, 76U, MAX_uint8_T,
    4U, 76U, MAX_uint8_T, 4U, 76U, MAX_uint8_T, 4U, 76U, MAX_uint8_T, 4U, 76U,
    MAX_uint8_T, 4U, 0U, 0U, 0U, 0U, 0U, 0U, 76U, MAX_uint8_T, 4U, 76U,
    MAX_uint8_T, 4U, 76U, MAX_uint8_T, 4U, 76U, MAX_uint8_T, 4U, 76U,
    MAX_uint8_T, 4U, 76U, MAX_uint8_T, 4U, 76U, MAX_uint8_T, 4U, 0U, 0U, 68U,
    187U, 233U, 239U, 187U, 81U, 2U, 0U, 94U, MAX_uint8_T, MAX_uint8_T,
    MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, 48U, 0U, 217U, 230U, 74U,
    14U, 16U, 69U, 176U, 46U, 0U, 239U, 199U, 8U, 0U, 0U, 0U, 0U, 0U, 0U, 162U,
    MAX_uint8_T, 233U, 130U, 14U, 0U, 0U, 0U, 0U, 18U, 239U, 247U, MAX_uint8_T,
    234U, 103U, 1U, 0U, 0U, 147U, 186U, 20U, 143U, 249U, MAX_uint8_T, 184U, 6U,
    0U, 245U, 160U, 0U, 0U, 43U, 206U, MAX_uint8_T, 104U, 0U, 215U, 254U, 142U,
    11U, 0U, 16U, 254U, 121U, 0U, 53U, 232U, MAX_uint8_T, 229U, 107U, 73U, 241U,
    27U, 0U, 0U, 20U, 153U, 252U, MAX_uint8_T, MAX_uint8_T, 135U, 0U, 0U, 0U, 0U,
    0U, 38U, 171U, MAX_uint8_T, 254U, 68U, 0U, 0U, 0U, 0U, 0U, 0U, 63U, 249U,
    161U, 15U, 200U, 102U, 40U, 7U, 21U, 97U, 253U, 153U, 16U, MAX_uint8_T,
    MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, 246U, 48U,
    0U, 58U, 158U, 221U, 247U, 222U, 160U, 42U, 0U, 248U, 140U, 0U, 128U,
    MAX_uint8_T, 8U, 248U, 140U, 0U, 128U, MAX_uint8_T, 8U, 0U, 0U, 0U, 0U, 45U,
    157U, 223U, 248U, 236U, 188U, 96U, 2U, 0U, 0U, 0U, 0U, 0U, 0U, 123U, 232U,
    116U, 37U, 7U, 20U, 79U, 193U, 200U, 24U, 0U, 0U, 0U, 0U, 126U, 203U, 23U,
    0U, 0U, 0U, 0U, 0U, 0U, 126U, 215U, 14U, 0U, 0U, 51U, 226U, 21U, 0U, 49U,
    186U, 245U, MAX_uint8_T, MAX_uint8_T, 52U, 0U, 150U, 154U, 0U, 0U, 171U, 99U,
    0U, 36U, 242U, 154U, 28U, 0U, 0U, 0U, 0U, 15U, 236U, 23U, 1U, 238U, 14U, 0U,
    150U, 208U, 1U, 0U, 0U, 0U, 0U, 0U, 0U, 167U, 92U, 21U, 227U, 0U, 0U, 202U,
    139U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 127U, 124U, 21U, 226U, 0U, 0U, 201U, 137U,
    0U, 0U, 0U, 0U, 0U, 0U, 0U, 127U, 123U, 2U, 238U, 14U, 0U, 147U, 204U, 1U,
    0U, 0U, 0U, 0U, 0U, 0U, 170U, 91U, 0U, 170U, 99U, 0U, 31U, 238U, 155U, 32U,
    0U, 0U, 0U, 0U, 17U, 239U, 22U, 0U, 51U, 226U, 20U, 0U, 43U, 183U, 245U,
    MAX_uint8_T, MAX_uint8_T, 52U, 0U, 153U, 153U, 0U, 0U, 0U, 125U, 200U, 20U,
    0U, 0U, 0U, 0U, 0U, 0U, 121U, 215U, 14U, 0U, 0U, 0U, 0U, 125U, 229U, 112U,
    33U, 6U, 21U, 78U, 189U, 200U, 24U, 0U, 0U, 0U, 0U, 0U, 0U, 47U, 159U, 226U,
    249U, 236U, 186U, 94U, 2U, 0U, 0U, 0U, 0U, 236U, MAX_uint8_T, 248U, 193U,
    26U, 0U, 0U, 0U, 0U, 28U, 235U, 137U, 0U, 0U, 0U, 0U, 0U, 196U, 163U, 0U, 0U,
    109U, 214U, 247U, MAX_uint8_T, 164U, 0U, 75U, 254U, 77U, 7U, 196U, 164U, 0U,
    107U, 251U, 37U, 47U, 227U, 193U, 2U, 17U, 194U, 241U, 165U, 113U, 240U,
    100U, 0U, 0U, 0U, 37U, 202U, 22U, 0U, 145U, 116U, 0U, 0U, 37U, 229U, 124U,
    0U, 143U, 221U, 23U, 0U, 36U, 228U, 152U, 0U, 142U, 235U, 37U, 0U, 0U, 200U,
    230U, 6U, 77U, MAX_uint8_T, 101U, 0U, 0U, 0U, 36U, 228U, 152U, 0U, 142U,
    234U, 37U, 0U, 0U, 0U, 37U, 229U, 124U, 0U, 144U, 221U, 23U, 0U, 0U, 0U, 38U,
    203U, 22U, 0U, 145U, 116U, 12U, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T,
    MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T,
    MAX_uint8_T, 92U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 240U, 92U, 0U, 0U,
    0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 240U, 92U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
    0U, 0U, 240U, 92U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 240U, 92U, 180U,
    MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T,
    MAX_uint8_T, 28U, 0U, 0U, 73U, 203U, 248U, 228U, 134U, 6U, 0U, 0U, 83U, 222U,
    76U, 14U, 41U, 172U, 180U, 0U, 2U, 221U, 41U, 224U, 251U, 201U, 17U, 189U,
    75U, 39U, 192U, 0U, 224U, 17U, 220U, 48U, 93U, 139U, 39U, 192U, 0U, 224U,
    MAX_uint8_T, 170U, 2U, 94U, 138U, 2U, 220U, 41U, 224U, 43U, 198U, 24U, 188U,
    74U, 0U, 82U, 222U, 74U, 13U, 41U, 169U, 179U, 0U, 0U, 0U, 74U, 204U, 249U,
    228U, 134U, 6U, 0U, 64U, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T,
    MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, 60U, 2U, 140U, 236U, 226U, 101U, 0U,
    109U, 199U, 25U, 46U, 231U, 58U, 167U, 108U, 0U, 0U, 161U, 115U, 110U, 199U,
    24U, 45U, 231U, 59U, 3U, 143U, 237U, 227U, 103U, 0U, 0U, 0U, 0U, 0U, 0U,
    128U, 208U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 128U, 208U, 0U, 0U, 0U,
    0U, 0U, 0U, 0U, 0U, 0U, 0U, 128U, 208U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
    0U, 128U, 208U, 0U, 0U, 0U, 0U, 0U, 12U, MAX_uint8_T, MAX_uint8_T,
    MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T,
    MAX_uint8_T, MAX_uint8_T, 92U, 0U, 0U, 0U, 0U, 0U, 128U, 208U, 0U, 0U, 0U,
    0U, 0U, 0U, 0U, 0U, 0U, 0U, 128U, 208U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
    0U, 128U, 208U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 128U, 208U, 0U, 0U,
    0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 12U, MAX_uint8_T,
    MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T,
    MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, 92U, 128U, MAX_uint8_T, 254U, 223U,
    96U, 0U, 0U, 0U, 3U, 88U, 253U, 32U, 0U, 0U, 0U, 76U, MAX_uint8_T, 56U, 0U,
    0U, 18U, 221U, 221U, 3U, 0U, 8U, 195U, 251U, 63U, 0U, 0U, 164U, MAX_uint8_T,
    99U, 0U, 0U, 93U, 253U, 111U, 0U, 0U, 0U, 176U, MAX_uint8_T, MAX_uint8_T,
    MAX_uint8_T, MAX_uint8_T, 60U, 128U, MAX_uint8_T, 252U, 222U, 93U, 0U, 0U,
    0U, 6U, 143U, 229U, 0U, 0U, 2U, 29U, 176U, 158U, 0U, 0U, 228U, MAX_uint8_T,
    203U, 26U, 0U, 0U, 2U, 32U, 172U, 223U, 7U, 0U, 0U, 0U, 53U, MAX_uint8_T,
    44U, 0U, 0U, 12U, 149U, 242U, 12U, 156U, MAX_uint8_T, 249U, 207U, 68U, 0U,
    0U, 2U, 192U, 239U, 47U, 0U, 120U, 244U, 57U, 0U, 50U, 242U, 68U, 0U, 0U,
    68U, MAX_uint8_T, 120U, 0U, 0U, 0U, 48U, MAX_uint8_T, 136U, 68U, MAX_uint8_T,
    120U, 0U, 0U, 0U, 48U, MAX_uint8_T, 136U, 68U, MAX_uint8_T, 120U, 0U, 0U, 0U,
    48U, MAX_uint8_T, 136U, 68U, MAX_uint8_T, 120U, 0U, 0U, 0U, 48U, MAX_uint8_T,
    136U, 68U, MAX_uint8_T, 120U, 0U, 0U, 0U, 48U, MAX_uint8_T, 136U, 68U,
    MAX_uint8_T, 120U, 0U, 0U, 0U, 48U, MAX_uint8_T, 136U, 68U, MAX_uint8_T,
    123U, 0U, 0U, 0U, 89U, MAX_uint8_T, 136U, 68U, MAX_uint8_T, 188U, 18U, 17U,
    113U, 247U, MAX_uint8_T, 136U, 68U, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T,
    MAX_uint8_T, MAX_uint8_T, 159U, MAX_uint8_T, 136U, 68U, MAX_uint8_T, 212U,
    236U, 233U, 114U, 48U, MAX_uint8_T, 136U, 68U, MAX_uint8_T, 120U, 0U, 0U, 0U,
    0U, 0U, 0U, 68U, MAX_uint8_T, 120U, 0U, 0U, 0U, 0U, 0U, 0U, 68U, MAX_uint8_T,
    120U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 31U, 174U, 235U, 253U, MAX_uint8_T,
    MAX_uint8_T, MAX_uint8_T, 12U, 0U, 192U, MAX_uint8_T, MAX_uint8_T,
    MAX_uint8_T, 172U, 8U, MAX_uint8_T, 12U, 6U, 252U, MAX_uint8_T, MAX_uint8_T,
    MAX_uint8_T, 172U, 8U, MAX_uint8_T, 12U, 9U, 254U, MAX_uint8_T, MAX_uint8_T,
    MAX_uint8_T, 172U, 8U, MAX_uint8_T, 12U, 0U, 215U, MAX_uint8_T, MAX_uint8_T,
    MAX_uint8_T, 172U, 8U, MAX_uint8_T, 12U, 0U, 83U, MAX_uint8_T, MAX_uint8_T,
    MAX_uint8_T, 172U, 8U, MAX_uint8_T, 12U, 0U, 0U, 71U, 193U, 246U, 172U, 8U,
    MAX_uint8_T, 12U, 0U, 0U, 0U, 0U, 108U, 172U, 8U, MAX_uint8_T, 12U, 0U, 0U,
    0U, 0U, 108U, 172U, 8U, MAX_uint8_T, 12U, 0U, 0U, 0U, 0U, 108U, 172U, 8U,
    MAX_uint8_T, 12U, 0U, 0U, 0U, 0U, 108U, 172U, 8U, MAX_uint8_T, 12U, 0U, 0U,
    0U, 0U, 108U, 172U, 8U, MAX_uint8_T, 12U, 0U, 0U, 0U, 0U, 108U, 172U, 8U,
    MAX_uint8_T, 12U, 0U, 0U, 0U, 0U, 108U, 172U, 8U, MAX_uint8_T, 12U, 0U, 0U,
    0U, 0U, 108U, 172U, 8U, MAX_uint8_T, 12U, 0U, 0U, 0U, 0U, 108U, 172U, 8U,
    MAX_uint8_T, 12U, 60U, MAX_uint8_T, 236U, 60U, MAX_uint8_T, 236U, 0U, 174U,
    30U, 0U, 33U, 253U, 210U, 34U, 0U, 26U, 220U, 121U, 132U, 254U, 209U, 33U,
    2U, 64U, 148U, 171U, 0U, 0U, 65U, 207U, 210U, 196U, 0U, 0U, 0U, 0U, 136U,
    196U, 0U, 0U, 0U, 0U, 136U, 196U, 0U, 0U, 0U, 0U, 136U, 196U, 0U, 0U, 0U, 0U,
    136U, 196U, 0U, 0U, 0U, 0U, 136U, 196U, 0U, 0U, 68U, MAX_uint8_T,
    MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, 128U, 2U, 119U, 222U, 240U, 179U, 30U,
    0U, 103U, 250U, 66U, 18U, 181U, 208U, 3U, 199U, 185U, 0U, 0U, 65U,
    MAX_uint8_T, 54U, 227U, 161U, 0U, 0U, 42U, MAX_uint8_T, 82U, 199U, 184U, 0U,
    0U, 67U, MAX_uint8_T, 53U, 102U, 248U, 63U, 18U, 184U, 208U, 2U, 2U, 121U,
    224U, 239U, 179U, 30U, 0U, 27U, 202U, 32U, 0U, 140U, 122U, 0U, 0U, 0U, 0U,
    133U, 224U, 31U, 35U, 233U, 121U, 0U, 0U, 0U, 0U, 160U, 223U, 30U, 52U, 244U,
    120U, 0U, 0U, 0U, 9U, 235U, 188U, 0U, 125U, MAX_uint8_T, 56U, 0U, 0U, 160U,
    223U, 31U, 52U, 244U, 121U, 0U, 0U, 133U, 224U, 31U, 35U, 233U, 122U, 0U, 0U,
    27U, 202U, 33U, 0U, 140U, 123U, 0U, 0U, 0U, 4U, 71U, 155U, 155U, 0U, 0U, 0U,
    0U, 0U, 0U, 127U, 176U, 0U, 0U, 84U, 202U, 217U, 176U, 0U, 0U, 0U, 0U, 0U,
    61U, 223U, 18U, 0U, 0U, 0U, 0U, 156U, 176U, 0U, 0U, 0U, 0U, 19U, 224U, 60U,
    0U, 0U, 0U, 0U, 0U, 156U, 176U, 0U, 0U, 0U, 0U, 178U, 124U, 0U, 0U, 0U, 0U,
    0U, 0U, 156U, 176U, 0U, 0U, 0U, 112U, 189U, 2U, 0U, 0U, 0U, 0U, 0U, 0U, 156U,
    176U, 0U, 0U, 50U, 227U, 25U, 0U, 37U, 244U, 120U, 0U, 0U, 0U, 156U, 176U,
    0U, 13U, 217U, 73U, 0U, 4U, 201U, 234U, 120U, 0U, 0U, 0U, 156U, 176U, 0U,
    164U, 139U, 0U, 0U, 131U, 148U, 184U, 120U, 0U, 0U, 0U, 0U, 0U, 97U, 201U,
    5U, 0U, 58U, 212U, 9U, 184U, 120U, 0U, 0U, 0U, 0U, 40U, 229U, 33U, 0U, 13U,
    216U, 51U, 0U, 184U, 120U, 0U, 0U, 0U, 8U, 209U, 86U, 0U, 0U, 75U,
    MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, 140U, 0U,
    0U, 150U, 154U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 184U, 120U, 0U, 0U, 82U, 212U,
    9U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 184U, 120U, 0U, 4U, 71U, 155U, 155U, 0U, 0U,
    0U, 0U, 0U, 15U, 220U, 67U, 0U, 0U, 84U, 202U, 217U, 176U, 0U, 0U, 0U, 0U,
    0U, 170U, 133U, 0U, 0U, 0U, 0U, 0U, 156U, 176U, 0U, 0U, 0U, 0U, 103U, 197U,
    4U, 0U, 0U, 0U, 0U, 0U, 156U, 176U, 0U, 0U, 0U, 44U, 229U, 30U, 0U, 0U, 0U,
    0U, 0U, 0U, 156U, 176U, 0U, 0U, 10U, 212U, 80U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
    156U, 176U, 0U, 0U, 156U, 148U, 52U, MAX_uint8_T, MAX_uint8_T, 237U, 147U,
    3U, 0U, 0U, 156U, 176U, 0U, 88U, 207U, 7U, 0U, 0U, 0U, 38U, 240U, 98U, 0U,
    0U, 156U, 176U, 34U, 229U, 39U, 0U, 0U, 0U, 0U, 18U, 241U, 123U, 0U, 0U, 0U,
    5U, 202U, 95U, 0U, 0U, 0U, 0U, 4U, 186U, 245U, 32U, 0U, 0U, 0U, 141U, 162U,
    0U, 0U, 0U, 0U, 0U, 153U, MAX_uint8_T, 97U, 0U, 0U, 0U, 74U, 216U, 12U, 0U,
    0U, 0U, 0U, 105U, MAX_uint8_T, 138U, 0U, 0U, 0U, 26U, 228U, 49U, 0U, 0U, 0U,
    0U, 33U, 246U, 161U, 1U, 0U, 0U, 2U, 191U, 110U, 0U, 0U, 0U, 0U, 0U, 104U,
    MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, 136U, 8U, MAX_uint8_T,
    MAX_uint8_T, 242U, 177U, 15U, 0U, 0U, 0U, 0U, 0U, 159U, 141U, 0U, 0U, 0U, 0U,
    37U, 244U, 97U, 0U, 0U, 0U, 0U, 90U, 203U, 5U, 0U, 0U, 0U, 10U, 77U, 237U,
    41U, 0U, 0U, 0U, 36U, 228U, 35U, 0U, 0U, 0U, 104U, MAX_uint8_T, 250U, 103U,
    0U, 0U, 0U, 6U, 204U, 88U, 0U, 0U, 0U, 0U, 0U, 11U, 82U, 246U, 99U, 0U, 0U,
    144U, 156U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 185U, 168U, 0U, 76U, 213U, 10U,
    37U, 244U, 120U, 0U, 0U, 0U, 1U, 53U, 238U, 122U, 27U, 227U, 44U, 4U, 201U,
    234U, 120U, 0U, 32U, MAX_uint8_T, MAX_uint8_T, 234U, 151U, 10U, 193U, 103U,
    0U, 131U, 148U, 184U, 120U, 0U, 0U, 0U, 0U, 0U, 0U, 129U, 171U, 0U, 58U,
    212U, 9U, 184U, 120U, 0U, 0U, 0U, 0U, 0U, 63U, 220U, 15U, 13U, 216U, 51U, 0U,
    184U, 120U, 0U, 0U, 0U, 0U, 20U, 224U, 55U, 0U, 75U, MAX_uint8_T,
    MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, 140U, 0U, 0U, 1U, 180U,
    118U, 0U, 0U, 0U, 0U, 0U, 0U, 184U, 120U, 0U, 0U, 0U, 114U, 184U, 1U, 0U, 0U,
    0U, 0U, 0U, 0U, 184U, 120U, 0U, 0U, 0U, 0U, 84U, MAX_uint8_T, 104U, 0U, 0U,
    0U, 0U, 0U, 84U, MAX_uint8_T, 104U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
    0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 84U, MAX_uint8_T, 100U, 0U, 0U,
    0U, 0U, 0U, 96U, MAX_uint8_T, 66U, 0U, 0U, 0U, 0U, 0U, 174U, 222U, 5U, 0U,
    0U, 0U, 0U, 116U, 251U, 64U, 0U, 0U, 0U, 0U, 115U, MAX_uint8_T, 109U, 0U, 0U,
    0U, 0U, 66U, MAX_uint8_T, 181U, 0U, 0U, 0U, 0U, 0U, 157U, MAX_uint8_T, 71U,
    0U, 0U, 0U, 0U, 0U, 155U, MAX_uint8_T, 145U, 25U, 9U, 50U, 153U, 66U, 60U,
    252U, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, 72U,
    0U, 59U, 182U, 233U, 242U, 197U, 96U, 4U, 0U, 0U, 0U, 50U, 241U, 188U, 1U,
    0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 60U, 245U, 115U, 0U, 0U, 0U, 0U, 0U,
    0U, 0U, 0U, 0U, 0U, 0U, 71U, 242U, 47U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
    0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 2U, 224U, MAX_uint8_T,
    85U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 73U, MAX_uint8_T, MAX_uint8_T, 185U,
    0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 174U, 247U, 232U, 253U, 31U, 0U, 0U, 0U,
    0U, 0U, 0U, 0U, 24U, 251U, 162U, 132U, MAX_uint8_T, 129U, 0U, 0U, 0U, 0U, 0U,
    0U, 0U, 121U, MAX_uint8_T, 59U, 32U, 254U, 226U, 2U, 0U, 0U, 0U, 0U, 0U, 1U,
    221U, 212U, 0U, 0U, 186U, MAX_uint8_T, 73U, 0U, 0U, 0U, 0U, 0U, 68U,
    MAX_uint8_T, 109U, 0U, 0U, 84U, MAX_uint8_T, 173U, 0U, 0U, 0U, 0U, 0U, 170U,
    246U, 16U, 0U, 0U, 5U, 233U, 250U, 22U, 0U, 0U, 0U, 21U, 249U, MAX_uint8_T,
    MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T,
    117U, 0U, 0U, 0U, 117U, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T,
    MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, 216U, 0U, 0U, 0U, 217U,
    247U, 23U, 0U, 0U, 0U, 0U, 9U, 234U, MAX_uint8_T, 61U, 0U, 64U, MAX_uint8_T,
    147U, 0U, 0U, 0U, 0U, 0U, 0U, 125U, MAX_uint8_T, 161U, 0U, 165U, 250U, 30U,
    0U, 0U, 0U, 0U, 0U, 0U, 17U, 242U, 246U, 15U, 0U, 0U, 0U, 0U, 0U, 0U, 79U,
    MAX_uint8_T, 146U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 24U, 235U, 160U, 1U,
    0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 1U, 184U, 173U, 3U, 0U, 0U, 0U, 0U, 0U, 0U,
    0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 2U, 224U,
    MAX_uint8_T, 85U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 73U, MAX_uint8_T,
    MAX_uint8_T, 185U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 174U, 247U, 232U,
    253U, 31U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 24U, 251U, 162U, 132U, MAX_uint8_T,
    129U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 121U, MAX_uint8_T, 59U, 32U, 254U, 226U,
    2U, 0U, 0U, 0U, 0U, 0U, 1U, 221U, 212U, 0U, 0U, 186U, MAX_uint8_T, 73U, 0U,
    0U, 0U, 0U, 0U, 68U, MAX_uint8_T, 109U, 0U, 0U, 84U, MAX_uint8_T, 173U, 0U,
    0U, 0U, 0U, 0U, 170U, 246U, 16U, 0U, 0U, 5U, 233U, 250U, 22U, 0U, 0U, 0U,
    21U, 249U, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T,
    MAX_uint8_T, MAX_uint8_T, 117U, 0U, 0U, 0U, 117U, MAX_uint8_T, MAX_uint8_T,
    MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T,
    216U, 0U, 0U, 0U, 217U, 247U, 23U, 0U, 0U, 0U, 0U, 9U, 234U, MAX_uint8_T,
    61U, 0U, 64U, MAX_uint8_T, 147U, 0U, 0U, 0U, 0U, 0U, 0U, 125U, MAX_uint8_T,
    161U, 0U, 165U, 250U, 30U, 0U, 0U, 0U, 0U, 0U, 0U, 17U, 242U, 246U, 15U, 0U,
    0U, 0U, 0U, 35U, 243U, MAX_uint8_T, 134U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 4U,
    199U, 194U, 104U, 252U, 62U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 129U, 211U, 16U, 0U,
    120U, 225U, 15U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
    0U, 0U, 0U, 0U, 0U, 2U, 224U, MAX_uint8_T, 85U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
    0U, 0U, 73U, MAX_uint8_T, MAX_uint8_T, 185U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
    0U, 174U, 247U, 232U, 253U, 31U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 24U, 251U, 162U,
    132U, MAX_uint8_T, 129U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 121U, MAX_uint8_T, 59U,
    32U, 254U, 226U, 2U, 0U, 0U, 0U, 0U, 0U, 1U, 221U, 212U, 0U, 0U, 186U,
    MAX_uint8_T, 73U, 0U, 0U, 0U, 0U, 0U, 68U, MAX_uint8_T, 109U, 0U, 0U, 84U,
    MAX_uint8_T, 173U, 0U, 0U, 0U, 0U, 0U, 170U, 246U, 16U, 0U, 0U, 5U, 233U,
    250U, 22U, 0U, 0U, 0U, 21U, 249U, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T,
    MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, 117U, 0U, 0U, 0U, 117U,
    MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T,
    MAX_uint8_T, MAX_uint8_T, 216U, 0U, 0U, 0U, 217U, 247U, 23U, 0U, 0U, 0U, 0U,
    9U, 234U, MAX_uint8_T, 61U, 0U, 64U, MAX_uint8_T, 147U, 0U, 0U, 0U, 0U, 0U,
    0U, 125U, MAX_uint8_T, 161U, 0U, 165U, 250U, 30U, 0U, 0U, 0U, 0U, 0U, 0U,
    17U, 242U, 246U, 15U, 0U, 0U, 0U, 59U, 233U, 224U, 98U, 43U, 253U, 9U, 0U,
    0U, 0U, 0U, 0U, 0U, 167U, 135U, 46U, 186U, 245U, 139U, 0U, 0U, 0U, 0U, 0U,
    0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 2U, 224U,
    MAX_uint8_T, 85U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 73U, MAX_uint8_T,
    MAX_uint8_T, 185U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 174U, 247U, 232U,
    253U, 31U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 24U, 251U, 162U, 132U, MAX_uint8_T,
    129U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 121U, MAX_uint8_T, 59U, 32U, 254U, 226U,
    2U, 0U, 0U, 0U, 0U, 0U, 1U, 221U, 212U, 0U, 0U, 186U, MAX_uint8_T, 73U, 0U,
    0U, 0U, 0U, 0U, 68U, MAX_uint8_T, 109U, 0U, 0U, 84U, MAX_uint8_T, 173U, 0U,
    0U, 0U, 0U, 0U, 170U, 246U, 16U, 0U, 0U, 5U, 233U, 250U, 22U, 0U, 0U, 0U,
    21U, 249U, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T,
    MAX_uint8_T, MAX_uint8_T, 117U, 0U, 0U, 0U, 117U, MAX_uint8_T, MAX_uint8_T,
    MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T,
    216U, 0U, 0U, 0U, 217U, 247U, 23U, 0U, 0U, 0U, 0U, 9U, 234U, MAX_uint8_T,
    61U, 0U, 64U, MAX_uint8_T, 147U, 0U, 0U, 0U, 0U, 0U, 0U, 125U, MAX_uint8_T,
    161U, 0U, 165U, 250U, 30U, 0U, 0U, 0U, 0U, 0U, 0U, 17U, 242U, 246U, 15U, 0U,
    0U, 0U, 80U, MAX_uint8_T, 52U, 0U, 216U, 176U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
    80U, MAX_uint8_T, 52U, 0U, 216U, 176U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
    0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 2U, 224U, MAX_uint8_T, 85U,
    0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 73U, MAX_uint8_T, MAX_uint8_T, 185U, 0U,
    0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 174U, 247U, 232U, 253U, 31U, 0U, 0U, 0U, 0U,
    0U, 0U, 0U, 24U, 251U, 162U, 132U, MAX_uint8_T, 129U, 0U, 0U, 0U, 0U, 0U, 0U,
    0U, 121U, MAX_uint8_T, 59U, 32U, 254U, 226U, 2U, 0U, 0U, 0U, 0U, 0U, 1U,
    221U, 212U, 0U, 0U, 186U, MAX_uint8_T, 73U, 0U, 0U, 0U, 0U, 0U, 68U,
    MAX_uint8_T, 109U, 0U, 0U, 84U, MAX_uint8_T, 173U, 0U, 0U, 0U, 0U, 0U, 170U,
    246U, 16U, 0U, 0U, 5U, 233U, 250U, 22U, 0U, 0U, 0U, 21U, 249U, MAX_uint8_T,
    MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T,
    117U, 0U, 0U, 0U, 117U, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T,
    MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, 216U, 0U, 0U, 0U, 217U,
    247U, 23U, 0U, 0U, 0U, 0U, 9U, 234U, MAX_uint8_T, 61U, 0U, 64U, MAX_uint8_T,
    147U, 0U, 0U, 0U, 0U, 0U, 0U, 125U, MAX_uint8_T, 161U, 0U, 165U, 250U, 30U,
    0U, 0U, 0U, 0U, 0U, 0U, 17U, 242U, 246U, 15U, 0U, 0U, 0U, 0U, 39U, 208U,
    239U, 116U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 172U, 68U, 18U, 196U, 29U,
    0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 172U, 68U, 17U, 195U, 29U, 0U, 0U, 0U, 0U,
    0U, 0U, 0U, 0U, 40U, 210U, 239U, 116U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
    2U, 224U, MAX_uint8_T, 85U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 73U,
    MAX_uint8_T, MAX_uint8_T, 185U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 174U,
    247U, 232U, 253U, 31U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 24U, 251U, 162U, 132U,
    MAX_uint8_T, 129U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 121U, MAX_uint8_T, 59U, 32U,
    254U, 226U, 2U, 0U, 0U, 0U, 0U, 0U, 1U, 221U, 212U, 0U, 0U, 186U,
    MAX_uint8_T, 73U, 0U, 0U, 0U, 0U, 0U, 68U, MAX_uint8_T, 109U, 0U, 0U, 84U,
    MAX_uint8_T, 173U, 0U, 0U, 0U, 0U, 0U, 170U, 246U, 16U, 0U, 0U, 5U, 233U,
    250U, 22U, 0U, 0U, 0U, 21U, 249U, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T,
    MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, 117U, 0U, 0U, 0U, 117U,
    MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T,
    MAX_uint8_T, MAX_uint8_T, 216U, 0U, 0U, 0U, 217U, 247U, 23U, 0U, 0U, 0U, 0U,
    9U, 234U, MAX_uint8_T, 61U, 0U, 64U, MAX_uint8_T, 147U, 0U, 0U, 0U, 0U, 0U,
    0U, 125U, MAX_uint8_T, 161U, 0U, 165U, 250U, 30U, 0U, 0U, 0U, 0U, 0U, 0U,
    17U, 242U, 246U, 15U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 153U, MAX_uint8_T,
    MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T,
    124U, 0U, 0U, 0U, 0U, 0U, 0U, 59U, 254U, MAX_uint8_T, MAX_uint8_T,
    MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, 124U, 0U,
    0U, 0U, 0U, 0U, 6U, 213U, 233U, 218U, MAX_uint8_T, 16U, 0U, 0U, 0U, 0U, 0U,
    0U, 0U, 0U, 0U, 0U, 125U, MAX_uint8_T, 99U, 204U, MAX_uint8_T, 16U, 0U, 0U,
    0U, 0U, 0U, 0U, 0U, 0U, 0U, 37U, 248U, 203U, 1U, 204U, MAX_uint8_T, 16U, 0U,
    0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 190U, 254U, 56U, 0U, 204U, MAX_uint8_T,
    MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, 156U, 0U, 0U, 0U, 0U,
    97U, MAX_uint8_T, 161U, 0U, 0U, 204U, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T,
    MAX_uint8_T, MAX_uint8_T, 156U, 0U, 0U, 0U, 21U, 236U, 242U, 24U, 0U, 0U,
    204U, MAX_uint8_T, 16U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 164U, MAX_uint8_T,
    MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T,
    16U, 0U, 0U, 0U, 0U, 0U, 0U, 69U, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T,
    MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, 16U, 0U, 0U,
    0U, 0U, 0U, 9U, 220U, 254U, 112U, 0U, 0U, 0U, 0U, 204U, MAX_uint8_T,
    MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, 228U, 135U,
    253U, 98U, 0U, 0U, 0U, 0U, 0U, 204U, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T,
    MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, 228U, 0U, 0U, 0U, 19U, 126U, 201U,
    239U, 247U, 216U, 167U, 81U, 8U, 0U, 0U, 56U, 233U, MAX_uint8_T, MAX_uint8_T,
    MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, 112U, 0U,
    23U, 234U, MAX_uint8_T, 204U, 77U, 17U, 5U, 35U, 95U, 173U, 103U, 0U, 136U,
    MAX_uint8_T, 198U, 7U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 216U, MAX_uint8_T,
    60U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 3U, 254U, 245U, 2U, 0U, 0U, 0U, 0U, 0U,
    0U, 0U, 0U, 15U, MAX_uint8_T, 228U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 3U,
    254U, 245U, 2U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 218U, MAX_uint8_T, 64U,
    0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 141U, MAX_uint8_T, 207U, 12U, 0U, 0U, 0U,
    0U, 0U, 0U, 0U, 0U, 27U, 238U, MAX_uint8_T, 216U, 90U, 23U, 6U, 33U, 77U,
    166U, 106U, 0U, 0U, 64U, 238U, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T,
    MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, 116U, 0U, 0U, 0U, 24U,
    134U, 208U, 242U, 247U, 220U, 180U, 91U, 9U, 0U, 0U, 0U, 0U, 0U, 0U, 114U,
    91U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 1U, 218U, 234U, 78U, 0U, 0U, 0U, 0U,
    0U, 0U, 0U, 0U, 0U, 11U, 166U, 189U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 64U,
    MAX_uint8_T, 232U, 78U, 0U, 0U, 0U, 0U, 22U, 217U, 225U, 16U, 0U, 0U, 0U, 0U,
    0U, 0U, 29U, 225U, 167U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 37U, 231U, 93U, 0U, 0U,
    0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 84U, MAX_uint8_T, MAX_uint8_T,
    MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, 244U, 0U, 84U,
    MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T,
    244U, 0U, 84U, MAX_uint8_T, 136U, 0U, 0U, 0U, 0U, 0U, 0U, 84U, MAX_uint8_T,
    136U, 0U, 0U, 0U, 0U, 0U, 0U, 84U, MAX_uint8_T, 136U, 0U, 0U, 0U, 0U, 0U, 0U,
    84U, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T,
    MAX_uint8_T, 16U, 0U, 84U, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T,
    MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, 16U, 0U, 84U, MAX_uint8_T, 136U, 0U,
    0U, 0U, 0U, 0U, 0U, 84U, MAX_uint8_T, 136U, 0U, 0U, 0U, 0U, 0U, 0U, 84U,
    MAX_uint8_T, 136U, 0U, 0U, 0U, 0U, 0U, 0U, 84U, MAX_uint8_T, 136U, 0U, 0U,
    0U, 0U, 0U, 0U, 84U, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T,
    MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, 84U, 84U, MAX_uint8_T, MAX_uint8_T,
    MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, 84U, 0U, 0U,
    0U, 0U, 61U, 253U, 164U, 2U, 0U, 0U, 0U, 0U, 15U, 224U, 177U, 4U, 0U, 0U, 0U,
    0U, 0U, 165U, 189U, 7U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 84U,
    MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T,
    244U, 0U, 84U, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T,
    MAX_uint8_T, MAX_uint8_T, 244U, 0U, 84U, MAX_uint8_T, 136U, 0U, 0U, 0U, 0U,
    0U, 0U, 84U, MAX_uint8_T, 136U, 0U, 0U, 0U, 0U, 0U, 0U, 84U, MAX_uint8_T,
    136U, 0U, 0U, 0U, 0U, 0U, 0U, 84U, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T,
    MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, 16U, 0U, 84U, MAX_uint8_T,
    MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, 16U, 0U,
    84U, MAX_uint8_T, 136U, 0U, 0U, 0U, 0U, 0U, 0U, 84U, MAX_uint8_T, 136U, 0U,
    0U, 0U, 0U, 0U, 0U, 84U, MAX_uint8_T, 136U, 0U, 0U, 0U, 0U, 0U, 0U, 84U,
    MAX_uint8_T, 136U, 0U, 0U, 0U, 0U, 0U, 0U, 84U, MAX_uint8_T, MAX_uint8_T,
    MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, 84U, 84U,
    MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T,
    MAX_uint8_T, 84U, 0U, 0U, 11U, 219U, MAX_uint8_T, 181U, 0U, 0U, 0U, 0U, 0U,
    156U, 224U, 83U, 246U, 107U, 0U, 0U, 0U, 81U, 235U, 39U, 0U, 76U, 242U, 42U,
    0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 84U, MAX_uint8_T, MAX_uint8_T,
    MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, 244U, 0U, 84U,
    MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T,
    244U, 0U, 84U, MAX_uint8_T, 136U, 0U, 0U, 0U, 0U, 0U, 0U, 84U, MAX_uint8_T,
    136U, 0U, 0U, 0U, 0U, 0U, 0U, 84U, MAX_uint8_T, 136U, 0U, 0U, 0U, 0U, 0U, 0U,
    84U, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T,
    MAX_uint8_T, 16U, 0U, 84U, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T,
    MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, 16U, 0U, 84U, MAX_uint8_T, 136U, 0U,
    0U, 0U, 0U, 0U, 0U, 84U, MAX_uint8_T, 136U, 0U, 0U, 0U, 0U, 0U, 0U, 84U,
    MAX_uint8_T, 136U, 0U, 0U, 0U, 0U, 0U, 0U, 84U, MAX_uint8_T, 136U, 0U, 0U,
    0U, 0U, 0U, 0U, 84U, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T,
    MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, 84U, 84U, MAX_uint8_T, MAX_uint8_T,
    MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, 84U, 0U, 0U,
    MAX_uint8_T, 132U, 0U, 136U, MAX_uint8_T, 0U, 0U, 0U, 0U, MAX_uint8_T, 132U,
    0U, 136U, MAX_uint8_T, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 84U,
    MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T,
    244U, 0U, 84U, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T,
    MAX_uint8_T, MAX_uint8_T, 244U, 0U, 84U, MAX_uint8_T, 136U, 0U, 0U, 0U, 0U,
    0U, 0U, 84U, MAX_uint8_T, 136U, 0U, 0U, 0U, 0U, 0U, 0U, 84U, MAX_uint8_T,
    136U, 0U, 0U, 0U, 0U, 0U, 0U, 84U, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T,
    MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, 16U, 0U, 84U, MAX_uint8_T,
    MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, 16U, 0U,
    84U, MAX_uint8_T, 136U, 0U, 0U, 0U, 0U, 0U, 0U, 84U, MAX_uint8_T, 136U, 0U,
    0U, 0U, 0U, 0U, 0U, 84U, MAX_uint8_T, 136U, 0U, 0U, 0U, 0U, 0U, 0U, 84U,
    MAX_uint8_T, 136U, 0U, 0U, 0U, 0U, 0U, 0U, 84U, MAX_uint8_T, MAX_uint8_T,
    MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, 84U, 84U,
    MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T,
    MAX_uint8_T, 84U, 7U, 188U, 245U, 40U, 0U, 0U, 11U, 198U, 205U, 5U, 0U, 0U,
    16U, 208U, 137U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 84U, MAX_uint8_T, 132U, 0U, 0U,
    84U, MAX_uint8_T, 132U, 0U, 0U, 84U, MAX_uint8_T, 132U, 0U, 0U, 84U,
    MAX_uint8_T, 132U, 0U, 0U, 84U, MAX_uint8_T, 132U, 0U, 0U, 84U, MAX_uint8_T,
    132U, 0U, 0U, 84U, MAX_uint8_T, 132U, 0U, 0U, 84U, MAX_uint8_T, 132U, 0U, 0U,
    84U, MAX_uint8_T, 132U, 0U, 0U, 84U, MAX_uint8_T, 132U, 0U, 0U, 84U,
    MAX_uint8_T, 132U, 0U, 0U, 84U, MAX_uint8_T, 132U, 0U, 0U, 84U, MAX_uint8_T,
    132U, 0U, 16U, 226U, 216U, 22U, 0U, 168U, 224U, 29U, 0U, 93U, 231U, 36U, 0U,
    0U, 0U, 0U, 0U, 0U, 0U, 84U, MAX_uint8_T, 132U, 0U, 0U, 84U, MAX_uint8_T,
    132U, 0U, 0U, 84U, MAX_uint8_T, 132U, 0U, 0U, 84U, MAX_uint8_T, 132U, 0U, 0U,
    84U, MAX_uint8_T, 132U, 0U, 0U, 84U, MAX_uint8_T, 132U, 0U, 0U, 84U,
    MAX_uint8_T, 132U, 0U, 0U, 84U, MAX_uint8_T, 132U, 0U, 0U, 84U, MAX_uint8_T,
    132U, 0U, 0U, 84U, MAX_uint8_T, 132U, 0U, 0U, 84U, MAX_uint8_T, 132U, 0U, 0U,
    84U, MAX_uint8_T, 132U, 0U, 0U, 84U, MAX_uint8_T, 132U, 0U, 0U, 0U, 0U, 182U,
    MAX_uint8_T, 218U, 11U, 0U, 0U, 108U, 244U, 82U, 227U, 155U, 0U, 42U, 242U,
    72U, 0U, 42U, 237U, 81U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 84U,
    MAX_uint8_T, 132U, 0U, 0U, 0U, 0U, 84U, MAX_uint8_T, 132U, 0U, 0U, 0U, 0U,
    84U, MAX_uint8_T, 132U, 0U, 0U, 0U, 0U, 84U, MAX_uint8_T, 132U, 0U, 0U, 0U,
    0U, 84U, MAX_uint8_T, 132U, 0U, 0U, 0U, 0U, 84U, MAX_uint8_T, 132U, 0U, 0U,
    0U, 0U, 84U, MAX_uint8_T, 132U, 0U, 0U, 0U, 0U, 84U, MAX_uint8_T, 132U, 0U,
    0U, 0U, 0U, 84U, MAX_uint8_T, 132U, 0U, 0U, 0U, 0U, 84U, MAX_uint8_T, 132U,
    0U, 0U, 0U, 0U, 84U, MAX_uint8_T, 132U, 0U, 0U, 0U, 0U, 84U, MAX_uint8_T,
    132U, 0U, 0U, 0U, 0U, 84U, MAX_uint8_T, 132U, 0U, 0U, 232U, 156U, 0U, 112U,
    MAX_uint8_T, 24U, 232U, 156U, 0U, 112U, MAX_uint8_T, 24U, 0U, 0U, 0U, 0U, 0U,
    0U, 0U, 84U, MAX_uint8_T, 132U, 0U, 0U, 0U, 84U, MAX_uint8_T, 132U, 0U, 0U,
    0U, 84U, MAX_uint8_T, 132U, 0U, 0U, 0U, 84U, MAX_uint8_T, 132U, 0U, 0U, 0U,
    84U, MAX_uint8_T, 132U, 0U, 0U, 0U, 84U, MAX_uint8_T, 132U, 0U, 0U, 0U, 84U,
    MAX_uint8_T, 132U, 0U, 0U, 0U, 84U, MAX_uint8_T, 132U, 0U, 0U, 0U, 84U,
    MAX_uint8_T, 132U, 0U, 0U, 0U, 84U, MAX_uint8_T, 132U, 0U, 0U, 0U, 84U,
    MAX_uint8_T, 132U, 0U, 0U, 0U, 84U, MAX_uint8_T, 132U, 0U, 0U, 0U, 84U,
    MAX_uint8_T, 132U, 0U, 0U, 0U, 0U, 84U, MAX_uint8_T, MAX_uint8_T,
    MAX_uint8_T, MAX_uint8_T, 247U, 230U, 181U, 93U, 2U, 0U, 0U, 0U, 0U, 84U,
    MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T,
    MAX_uint8_T, MAX_uint8_T, 184U, 10U, 0U, 0U, 0U, 84U, MAX_uint8_T, 136U, 0U,
    4U, 16U, 52U, 137U, 251U, MAX_uint8_T, 152U, 0U, 0U, 0U, 84U, MAX_uint8_T,
    136U, 0U, 0U, 0U, 0U, 0U, 77U, MAX_uint8_T, 252U, 28U, 0U, 0U, 84U,
    MAX_uint8_T, 136U, 0U, 0U, 0U, 0U, 0U, 0U, 187U, MAX_uint8_T, 97U, 68U,
    MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T,
    116U, 0U, 0U, 0U, 121U, MAX_uint8_T, 131U, 68U, MAX_uint8_T, MAX_uint8_T,
    MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, 116U, 0U, 0U, 0U, 104U,
    MAX_uint8_T, 137U, 0U, 0U, 84U, MAX_uint8_T, 136U, 0U, 0U, 0U, 0U, 0U, 0U,
    127U, MAX_uint8_T, 118U, 0U, 0U, 84U, MAX_uint8_T, 136U, 0U, 0U, 0U, 0U, 0U,
    0U, 196U, MAX_uint8_T, 68U, 0U, 0U, 84U, MAX_uint8_T, 136U, 0U, 0U, 0U, 0U,
    0U, 80U, MAX_uint8_T, 231U, 7U, 0U, 0U, 84U, MAX_uint8_T, 136U, 0U, 0U, 9U,
    44U, 130U, 249U, MAX_uint8_T, 98U, 0U, 0U, 0U, 84U, MAX_uint8_T, MAX_uint8_T,
    MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T,
    127U, 0U, 0U, 0U, 0U, 84U, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T,
    MAX_uint8_T, 247U, 221U, 159U, 51U, 0U, 0U, 0U, 0U, 0U, 0U, 125U, 244U, 195U,
    54U, 119U, 182U, 0U, 0U, 0U, 0U, 1U, 245U, 58U, 88U, 218U, 237U, 71U, 0U, 0U,
    0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 84U, MAX_uint8_T, 203U, 3U, 0U,
    0U, 0U, 0U, 0U, 248U, 160U, 84U, MAX_uint8_T, MAX_uint8_T, 117U, 0U, 0U, 0U,
    0U, 0U, 248U, 160U, 84U, MAX_uint8_T, MAX_uint8_T, 246U, 37U, 0U, 0U, 0U, 0U,
    248U, 160U, 84U, MAX_uint8_T, 165U, MAX_uint8_T, 193U, 1U, 0U, 0U, 0U, 248U,
    160U, 84U, MAX_uint8_T, 68U, 185U, MAX_uint8_T, 106U, 0U, 0U, 0U, 248U, 160U,
    84U, MAX_uint8_T, 68U, 31U, 243U, 242U, 30U, 0U, 0U, 248U, 160U, 84U,
    MAX_uint8_T, 68U, 0U, 108U, MAX_uint8_T, 183U, 0U, 0U, 248U, 160U, 84U,
    MAX_uint8_T, 68U, 0U, 1U, 196U, MAX_uint8_T, 94U, 0U, 248U, 160U, 84U,
    MAX_uint8_T, 68U, 0U, 0U, 39U, 247U, 237U, 23U, 248U, 160U, 84U, MAX_uint8_T,
    68U, 0U, 0U, 0U, 120U, MAX_uint8_T, 172U, 248U, 160U, 84U, MAX_uint8_T, 68U,
    0U, 0U, 0U, 4U, 205U, MAX_uint8_T, MAX_uint8_T, 160U, 84U, MAX_uint8_T, 68U,
    0U, 0U, 0U, 0U, 47U, 250U, MAX_uint8_T, 160U, 84U, MAX_uint8_T, 68U, 0U, 0U,
    0U, 0U, 0U, 131U, MAX_uint8_T, 160U, 0U, 0U, 0U, 0U, 93U, 254U, 134U, 0U, 0U,
    0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 106U, 252U, 62U, 0U, 0U, 0U, 0U, 0U,
    0U, 0U, 0U, 0U, 0U, 0U, 0U, 121U, 225U, 15U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
    0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 19U, 133U, 210U,
    244U, 244U, 210U, 133U, 19U, 0U, 0U, 0U, 0U, 0U, 50U, 231U, MAX_uint8_T,
    MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, 231U, 49U,
    0U, 0U, 0U, 18U, 229U, MAX_uint8_T, 190U, 62U, 10U, 10U, 62U, 190U,
    MAX_uint8_T, 228U, 18U, 0U, 0U, 130U, MAX_uint8_T, 188U, 3U, 0U, 0U, 0U, 0U,
    3U, 188U, MAX_uint8_T, 129U, 0U, 0U, 213U, MAX_uint8_T, 53U, 0U, 0U, 0U, 0U,
    0U, 0U, 53U, MAX_uint8_T, 212U, 0U, 3U, 254U, 244U, 1U, 0U, 0U, 0U, 0U, 0U,
    0U, 1U, 244U, 254U, 2U, 15U, MAX_uint8_T, 228U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
    0U, 228U, MAX_uint8_T, 14U, 3U, 253U, 244U, 1U, 0U, 0U, 0U, 0U, 0U, 0U, 2U,
    245U, 253U, 2U, 0U, 211U, MAX_uint8_T, 55U, 0U, 0U, 0U, 0U, 0U, 0U, 55U,
    MAX_uint8_T, 211U, 0U, 0U, 128U, MAX_uint8_T, 188U, 3U, 0U, 0U, 0U, 0U, 3U,
    188U, MAX_uint8_T, 129U, 0U, 0U, 17U, 228U, MAX_uint8_T, 188U, 60U, 9U, 10U,
    63U, 191U, MAX_uint8_T, 230U, 19U, 0U, 0U, 0U, 50U, 233U, MAX_uint8_T,
    MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, 232U, 52U,
    0U, 0U, 0U, 0U, 0U, 20U, 136U, 214U, 246U, 244U, 210U, 133U, 20U, 0U, 0U, 0U,
    0U, 0U, 0U, 0U, 0U, 0U, 0U, 139U, 253U, 89U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
    0U, 0U, 65U, 253U, 102U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 17U, 227U,
    116U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
    0U, 0U, 0U, 0U, 0U, 19U, 133U, 210U, 244U, 244U, 210U, 133U, 19U, 0U, 0U, 0U,
    0U, 0U, 50U, 231U, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T,
    MAX_uint8_T, MAX_uint8_T, 231U, 49U, 0U, 0U, 0U, 18U, 229U, MAX_uint8_T,
    190U, 62U, 10U, 10U, 62U, 190U, MAX_uint8_T, 228U, 18U, 0U, 0U, 130U,
    MAX_uint8_T, 188U, 3U, 0U, 0U, 0U, 0U, 3U, 188U, MAX_uint8_T, 129U, 0U, 0U,
    213U, MAX_uint8_T, 53U, 0U, 0U, 0U, 0U, 0U, 0U, 53U, MAX_uint8_T, 212U, 0U,
    3U, 254U, 244U, 1U, 0U, 0U, 0U, 0U, 0U, 0U, 1U, 244U, 254U, 2U, 15U,
    MAX_uint8_T, 228U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 228U, MAX_uint8_T, 14U,
    3U, 253U, 244U, 1U, 0U, 0U, 0U, 0U, 0U, 0U, 2U, 245U, 253U, 2U, 0U, 211U,
    MAX_uint8_T, 55U, 0U, 0U, 0U, 0U, 0U, 0U, 55U, MAX_uint8_T, 211U, 0U, 0U,
    128U, MAX_uint8_T, 188U, 3U, 0U, 0U, 0U, 0U, 3U, 188U, MAX_uint8_T, 129U, 0U,
    0U, 17U, 228U, MAX_uint8_T, 188U, 60U, 9U, 10U, 63U, 191U, MAX_uint8_T, 230U,
    19U, 0U, 0U, 0U, 50U, 233U, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T,
    MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, 232U, 52U, 0U, 0U, 0U, 0U, 0U, 20U,
    136U, 214U, 246U, 244U, 210U, 133U, 20U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 79U,
    MAX_uint8_T, MAX_uint8_T, 78U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 24U, 235U,
    146U, 151U, 235U, 24U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 1U, 184U, 169U, 2U, 2U,
    173U, 183U, 1U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
    0U, 0U, 0U, 0U, 0U, 19U, 133U, 210U, 244U, 244U, 210U, 133U, 19U, 0U, 0U, 0U,
    0U, 0U, 50U, 231U, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T,
    MAX_uint8_T, MAX_uint8_T, 231U, 49U, 0U, 0U, 0U, 18U, 229U, MAX_uint8_T,
    190U, 62U, 10U, 10U, 62U, 190U, MAX_uint8_T, 228U, 18U, 0U, 0U, 130U,
    MAX_uint8_T, 188U, 3U, 0U, 0U, 0U, 0U, 3U, 188U, MAX_uint8_T, 129U, 0U, 0U,
    213U, MAX_uint8_T, 53U, 0U, 0U, 0U, 0U, 0U, 0U, 53U, MAX_uint8_T, 212U, 0U,
    3U, 254U, 244U, 1U, 0U, 0U, 0U, 0U, 0U, 0U, 1U, 244U, 254U, 2U, 15U,
    MAX_uint8_T, 228U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 228U, MAX_uint8_T, 14U,
    3U, 253U, 244U, 1U, 0U, 0U, 0U, 0U, 0U, 0U, 2U, 245U, 253U, 2U, 0U, 211U,
    MAX_uint8_T, 55U, 0U, 0U, 0U, 0U, 0U, 0U, 55U, MAX_uint8_T, 211U, 0U, 0U,
    128U, MAX_uint8_T, 188U, 3U, 0U, 0U, 0U, 0U, 3U, 188U, MAX_uint8_T, 129U, 0U,
    0U, 17U, 228U, MAX_uint8_T, 188U, 60U, 9U, 10U, 63U, 191U, MAX_uint8_T, 230U,
    19U, 0U, 0U, 0U, 50U, 233U, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T,
    MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, 232U, 52U, 0U, 0U, 0U, 0U, 0U, 20U,
    136U, 214U, 246U, 244U, 210U, 133U, 20U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 97U,
    242U, 208U, 70U, 88U, 214U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 215U, 88U, 70U,
    206U, 242U, 97U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
    0U, 0U, 0U, 0U, 0U, 0U, 19U, 133U, 210U, 244U, 244U, 210U, 133U, 19U, 0U, 0U,
    0U, 0U, 0U, 50U, 231U, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T,
    MAX_uint8_T, MAX_uint8_T, 231U, 49U, 0U, 0U, 0U, 18U, 229U, MAX_uint8_T,
    190U, 62U, 10U, 10U, 62U, 190U, MAX_uint8_T, 228U, 18U, 0U, 0U, 130U,
    MAX_uint8_T, 188U, 3U, 0U, 0U, 0U, 0U, 3U, 188U, MAX_uint8_T, 129U, 0U, 0U,
    213U, MAX_uint8_T, 53U, 0U, 0U, 0U, 0U, 0U, 0U, 53U, MAX_uint8_T, 212U, 0U,
    3U, 254U, 244U, 1U, 0U, 0U, 0U, 0U, 0U, 0U, 1U, 244U, 254U, 2U, 15U,
    MAX_uint8_T, 228U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 228U, MAX_uint8_T, 14U,
    3U, 253U, 244U, 1U, 0U, 0U, 0U, 0U, 0U, 0U, 2U, 245U, 253U, 2U, 0U, 211U,
    MAX_uint8_T, 55U, 0U, 0U, 0U, 0U, 0U, 0U, 55U, MAX_uint8_T, 211U, 0U, 0U,
    128U, MAX_uint8_T, 188U, 3U, 0U, 0U, 0U, 0U, 3U, 188U, MAX_uint8_T, 129U, 0U,
    0U, 17U, 228U, MAX_uint8_T, 188U, 60U, 9U, 10U, 63U, 191U, MAX_uint8_T, 230U,
    19U, 0U, 0U, 0U, 50U, 233U, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T,
    MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, 232U, 52U, 0U, 0U, 0U, 0U, 0U, 20U,
    136U, 214U, 246U, 244U, 210U, 133U, 20U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 128U,
    MAX_uint8_T, 4U, 8U, MAX_uint8_T, 128U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 128U,
    MAX_uint8_T, 4U, 8U, MAX_uint8_T, 128U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
    0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 19U, 133U, 210U, 244U, 244U,
    210U, 133U, 19U, 0U, 0U, 0U, 0U, 0U, 50U, 231U, MAX_uint8_T, MAX_uint8_T,
    MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, 231U, 49U, 0U, 0U, 0U,
    18U, 229U, MAX_uint8_T, 190U, 62U, 10U, 10U, 62U, 190U, MAX_uint8_T, 228U,
    18U, 0U, 0U, 130U, MAX_uint8_T, 188U, 3U, 0U, 0U, 0U, 0U, 3U, 188U,
    MAX_uint8_T, 129U, 0U, 0U, 213U, MAX_uint8_T, 53U, 0U, 0U, 0U, 0U, 0U, 0U,
    53U, MAX_uint8_T, 212U, 0U, 3U, 254U, 244U, 1U, 0U, 0U, 0U, 0U, 0U, 0U, 1U,
    244U, 254U, 2U, 15U, MAX_uint8_T, 228U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 228U,
    MAX_uint8_T, 14U, 3U, 253U, 244U, 1U, 0U, 0U, 0U, 0U, 0U, 0U, 2U, 245U, 253U,
    2U, 0U, 211U, MAX_uint8_T, 55U, 0U, 0U, 0U, 0U, 0U, 0U, 55U, MAX_uint8_T,
    211U, 0U, 0U, 128U, MAX_uint8_T, 188U, 3U, 0U, 0U, 0U, 0U, 3U, 188U,
    MAX_uint8_T, 129U, 0U, 0U, 17U, 228U, MAX_uint8_T, 188U, 60U, 9U, 10U, 63U,
    191U, MAX_uint8_T, 230U, 19U, 0U, 0U, 0U, 50U, 233U, MAX_uint8_T,
    MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, 232U, 52U,
    0U, 0U, 0U, 0U, 0U, 20U, 136U, 214U, 246U, 244U, 210U, 133U, 20U, 0U, 0U, 0U,
    0U, 176U, 97U, 0U, 0U, 0U, 0U, 0U, 0U, 40U, 207U, 26U, 0U, 44U, 234U, 97U,
    0U, 0U, 0U, 0U, 40U, 230U, 105U, 0U, 0U, 0U, 51U, 239U, 97U, 0U, 0U, 40U,
    231U, 116U, 0U, 0U, 0U, 0U, 0U, 60U, 243U, 97U, 40U, 231U, 129U, 0U, 0U, 0U,
    0U, 0U, 0U, 0U, 69U, 246U, 237U, 141U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 69U,
    246U, 237U, 140U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 60U, 243U, 97U, 41U, 231U,
    129U, 0U, 0U, 0U, 0U, 0U, 51U, 239U, 97U, 0U, 0U, 41U, 231U, 116U, 0U, 0U,
    0U, 43U, 234U, 97U, 0U, 0U, 0U, 0U, 41U, 231U, 104U, 0U, 0U, 176U, 97U, 0U,
    0U, 0U, 0U, 0U, 0U, 41U, 207U, 26U, 0U, 0U, 0U, 18U, 132U, 210U, 244U, 247U,
    212U, 134U, 39U, 221U, 148U, 0U, 0U, 0U, 47U, 230U, MAX_uint8_T, MAX_uint8_T,
    MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, 198U, 5U,
    0U, 0U, 17U, 227U, MAX_uint8_T, 188U, 61U, 10U, 18U, 68U, 202U, MAX_uint8_T,
    228U, 17U, 0U, 0U, 128U, MAX_uint8_T, 185U, 3U, 0U, 0U, 0U, 28U, 225U, 235U,
    MAX_uint8_T, 126U, 0U, 0U, 212U, MAX_uint8_T, 52U, 0U, 0U, 0U, 20U, 215U,
    154U, 64U, MAX_uint8_T, 209U, 0U, 3U, 254U, 244U, 1U, 0U, 0U, 12U, 202U,
    171U, 2U, 3U, 251U, 253U, 2U, 15U, MAX_uint8_T, 229U, 0U, 0U, 7U, 188U, 185U,
    6U, 0U, 0U, 230U, MAX_uint8_T, 15U, 3U, 254U, 251U, 2U, 3U, 172U, 199U, 11U,
    0U, 0U, 2U, 244U, 253U, 2U, 0U, 212U, MAX_uint8_T, 64U, 155U, 211U, 17U, 0U,
    0U, 0U, 55U, MAX_uint8_T, 211U, 0U, 0U, 127U, MAX_uint8_T, 235U, 221U, 25U,
    0U, 0U, 0U, 3U, 188U, MAX_uint8_T, 128U, 0U, 0U, 17U, 229U, MAX_uint8_T,
    201U, 68U, 18U, 10U, 61U, 190U, MAX_uint8_T, 228U, 18U, 0U, 0U, 5U, 198U,
    MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T,
    MAX_uint8_T, 231U, 50U, 0U, 0U, 0U, 149U, 221U, 40U, 137U, 213U, 248U, 245U,
    212U, 134U, 19U, 0U, 0U, 0U, 0U, 0U, 30U, 225U, 215U, 10U, 0U, 0U, 0U, 0U,
    0U, 0U, 0U, 37U, 232U, 151U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 46U, 238U, 77U,
    0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 108U, MAX_uint8_T, 108U,
    0U, 0U, 0U, 0U, 0U, 180U, 232U, 108U, MAX_uint8_T, 108U, 0U, 0U, 0U, 0U, 0U,
    180U, 232U, 108U, MAX_uint8_T, 108U, 0U, 0U, 0U, 0U, 0U, 180U, 232U, 108U,
    MAX_uint8_T, 108U, 0U, 0U, 0U, 0U, 0U, 180U, 232U, 108U, MAX_uint8_T, 108U,
    0U, 0U, 0U, 0U, 0U, 180U, 232U, 108U, MAX_uint8_T, 108U, 0U, 0U, 0U, 0U, 0U,
    180U, 232U, 108U, MAX_uint8_T, 108U, 0U, 0U, 0U, 0U, 0U, 180U, 232U, 108U,
    MAX_uint8_T, 110U, 0U, 0U, 0U, 0U, 0U, 181U, 232U, 100U, MAX_uint8_T, 135U,
    0U, 0U, 0U, 0U, 0U, 195U, 224U, 72U, MAX_uint8_T, 203U, 0U, 0U, 0U, 0U, 10U,
    242U, 200U, 14U, 244U, MAX_uint8_T, 151U, 36U, 9U, 42U, 178U, MAX_uint8_T,
    137U, 0U, 117U, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T,
    MAX_uint8_T, MAX_uint8_T, 236U, 27U, 0U, 0U, 87U, 194U, 241U, 251U, 230U,
    164U, 36U, 0U, 0U, 0U, 0U, 0U, 0U, 52U, 250U, 175U, 3U, 0U, 0U, 0U, 0U, 0U,
    10U, 217U, 187U, 7U, 0U, 0U, 0U, 0U, 0U, 0U, 153U, 197U, 11U, 0U, 0U, 0U, 0U,
    0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 108U, MAX_uint8_T, 108U, 0U, 0U, 0U, 0U,
    0U, 180U, 232U, 108U, MAX_uint8_T, 108U, 0U, 0U, 0U, 0U, 0U, 180U, 232U,
    108U, MAX_uint8_T, 108U, 0U, 0U, 0U, 0U, 0U, 180U, 232U, 108U, MAX_uint8_T,
    108U, 0U, 0U, 0U, 0U, 0U, 180U, 232U, 108U, MAX_uint8_T, 108U, 0U, 0U, 0U,
    0U, 0U, 180U, 232U, 108U, MAX_uint8_T, 108U, 0U, 0U, 0U, 0U, 0U, 180U, 232U,
    108U, MAX_uint8_T, 108U, 0U, 0U, 0U, 0U, 0U, 180U, 232U, 108U, MAX_uint8_T,
    110U, 0U, 0U, 0U, 0U, 0U, 181U, 232U, 100U, MAX_uint8_T, 135U, 0U, 0U, 0U,
    0U, 0U, 195U, 224U, 72U, MAX_uint8_T, 203U, 0U, 0U, 0U, 0U, 10U, 242U, 200U,
    14U, 244U, MAX_uint8_T, 151U, 36U, 9U, 42U, 178U, MAX_uint8_T, 137U, 0U,
    117U, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T,
    MAX_uint8_T, 236U, 27U, 0U, 0U, 87U, 194U, 241U, 251U, 230U, 164U, 36U, 0U,
    0U, 0U, 0U, 16U, 226U, MAX_uint8_T, 169U, 0U, 0U, 0U, 0U, 0U, 0U, 168U, 218U,
    87U, 249U, 95U, 0U, 0U, 0U, 0U, 93U, 230U, 32U, 0U, 86U, 240U, 34U, 0U, 0U,
    0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 108U, MAX_uint8_T, 108U, 0U, 0U, 0U, 0U,
    0U, 180U, 232U, 108U, MAX_uint8_T, 108U, 0U, 0U, 0U, 0U, 0U, 180U, 232U,
    108U, MAX_uint8_T, 108U, 0U, 0U, 0U, 0U, 0U, 180U, 232U, 108U, MAX_uint8_T,
    108U, 0U, 0U, 0U, 0U, 0U, 180U, 232U, 108U, MAX_uint8_T, 108U, 0U, 0U, 0U,
    0U, 0U, 180U, 232U, 108U, MAX_uint8_T, 108U, 0U, 0U, 0U, 0U, 0U, 180U, 232U,
    108U, MAX_uint8_T, 108U, 0U, 0U, 0U, 0U, 0U, 180U, 232U, 108U, MAX_uint8_T,
    110U, 0U, 0U, 0U, 0U, 0U, 181U, 232U, 100U, MAX_uint8_T, 135U, 0U, 0U, 0U,
    0U, 0U, 195U, 224U, 72U, MAX_uint8_T, 203U, 0U, 0U, 0U, 0U, 10U, 242U, 200U,
    14U, 244U, MAX_uint8_T, 151U, 36U, 9U, 42U, 178U, MAX_uint8_T, 137U, 0U,
    117U, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T,
    MAX_uint8_T, 236U, 27U, 0U, 0U, 87U, 194U, 241U, 251U, 230U, 164U, 36U, 0U,
    0U, 0U, 64U, MAX_uint8_T, 68U, 0U, 200U, 192U, 0U, 0U, 0U, 0U, 64U,
    MAX_uint8_T, 68U, 0U, 200U, 192U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
    0U, 108U, MAX_uint8_T, 108U, 0U, 0U, 0U, 0U, 0U, 180U, 232U, 108U,
    MAX_uint8_T, 108U, 0U, 0U, 0U, 0U, 0U, 180U, 232U, 108U, MAX_uint8_T, 108U,
    0U, 0U, 0U, 0U, 0U, 180U, 232U, 108U, MAX_uint8_T, 108U, 0U, 0U, 0U, 0U, 0U,
    180U, 232U, 108U, MAX_uint8_T, 108U, 0U, 0U, 0U, 0U, 0U, 180U, 232U, 108U,
    MAX_uint8_T, 108U, 0U, 0U, 0U, 0U, 0U, 180U, 232U, 108U, MAX_uint8_T, 108U,
    0U, 0U, 0U, 0U, 0U, 180U, 232U, 108U, MAX_uint8_T, 110U, 0U, 0U, 0U, 0U, 0U,
    181U, 232U, 100U, MAX_uint8_T, 135U, 0U, 0U, 0U, 0U, 0U, 195U, 224U, 72U,
    MAX_uint8_T, 203U, 0U, 0U, 0U, 0U, 10U, 242U, 200U, 14U, 244U, MAX_uint8_T,
    151U, 36U, 9U, 42U, 178U, MAX_uint8_T, 137U, 0U, 117U, MAX_uint8_T,
    MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, 236U, 27U,
    0U, 0U, 87U, 194U, 241U, 251U, 230U, 164U, 36U, 0U, 0U, 0U, 0U, 0U, 0U, 11U,
    219U, 223U, 27U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 156U, 230U, 35U, 0U, 0U, 0U, 0U,
    0U, 0U, 0U, 81U, 236U, 43U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
    0U, 0U, 0U, 143U, MAX_uint8_T, 141U, 0U, 0U, 0U, 0U, 0U, 25U, 241U, 172U,
    16U, 236U, 250U, 38U, 0U, 0U, 0U, 0U, 169U, 243U, 28U, 0U, 107U, MAX_uint8_T,
    179U, 0U, 0U, 0U, 70U, MAX_uint8_T, 114U, 0U, 0U, 3U, 212U, MAX_uint8_T, 70U,
    0U, 8U, 220U, 209U, 4U, 0U, 0U, 0U, 70U, MAX_uint8_T, 213U, 4U, 131U, 254U,
    57U, 0U, 0U, 0U, 0U, 0U, 180U, MAX_uint8_T, 145U, 249U, 153U, 0U, 0U, 0U, 0U,
    0U, 0U, 39U, 250U, MAX_uint8_T, 234U, 18U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 156U,
    MAX_uint8_T, 111U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 132U, MAX_uint8_T, 88U,
    0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 132U, MAX_uint8_T, 88U, 0U, 0U, 0U, 0U, 0U,
    0U, 0U, 0U, 132U, MAX_uint8_T, 88U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 132U,
    MAX_uint8_T, 88U, 0U, 0U, 0U, 0U, 84U, MAX_uint8_T, 128U, 0U, 0U, 0U, 0U, 0U,
    0U, 84U, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, 251U, 230U, 170U, 44U, 0U,
    84U, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T,
    MAX_uint8_T, 239U, 26U, 84U, MAX_uint8_T, 128U, 1U, 20U, 78U, 231U,
    MAX_uint8_T, 119U, 84U, MAX_uint8_T, 128U, 0U, 0U, 0U, 110U, MAX_uint8_T,
    156U, 84U, MAX_uint8_T, 128U, 0U, 0U, 0U, 97U, MAX_uint8_T, 152U, 84U,
    MAX_uint8_T, 128U, 0U, 0U, 0U, 170U, MAX_uint8_T, 110U, 84U, MAX_uint8_T,
    128U, 5U, 38U, 145U, MAX_uint8_T, 246U, 25U, 84U, MAX_uint8_T, MAX_uint8_T,
    MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, 251U, 90U, 0U, 84U, MAX_uint8_T,
    MAX_uint8_T, 251U, 229U, 167U, 51U, 0U, 0U, 84U, MAX_uint8_T, 128U, 0U, 0U,
    0U, 0U, 0U, 0U, 84U, MAX_uint8_T, 128U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 64U,
    202U, 247U, 241U, 194U, 64U, 0U, 0U, 0U, 7U, 236U, MAX_uint8_T, MAX_uint8_T,
    MAX_uint8_T, MAX_uint8_T, 247U, 13U, 0U, 0U, 51U, MAX_uint8_T, 200U, 22U,
    21U, 198U, 248U, 14U, 0U, 0U, 67U, MAX_uint8_T, 124U, 0U, 0U, 193U, 127U, 0U,
    0U, 0U, 68U, MAX_uint8_T, 120U, 0U, 102U, 184U, 2U, 0U, 0U, 0U, 68U,
    MAX_uint8_T, 120U, 17U, 246U, 132U, 0U, 0U, 0U, 0U, 68U, MAX_uint8_T, 120U,
    13U, 228U, 254U, 121U, 0U, 0U, 0U, 68U, MAX_uint8_T, 120U, 0U, 27U, 206U,
    MAX_uint8_T, 151U, 1U, 0U, 68U, MAX_uint8_T, 120U, 0U, 0U, 13U, 185U,
    MAX_uint8_T, 137U, 0U, 68U, MAX_uint8_T, 120U, 0U, 0U, 0U, 6U, 190U, 251U,
    21U, 68U, MAX_uint8_T, 120U, 73U, 146U, 40U, 20U, 160U, MAX_uint8_T, 46U,
    68U, MAX_uint8_T, 120U, 80U, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T,
    MAX_uint8_T, 229U, 7U, 0U, 0U, 0U, 7U, 126U, 222U, 242U, 190U, 48U, 0U, 0U,
    0U, 80U, 251U, 150U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 92U, 253U, 76U, 0U, 0U,
    0U, 0U, 0U, 0U, 0U, 0U, 105U, 233U, 23U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
    0U, 0U, 0U, 0U, 0U, 3U, 86U, 188U, 235U, 241U, 192U, 50U, 0U, 0U, 0U, 68U,
    MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, 222U, 0U,
    0U, 0U, 63U, 161U, 57U, 12U, 32U, 214U, MAX_uint8_T, 21U, 0U, 0U, 0U, 0U, 0U,
    0U, 0U, 153U, MAX_uint8_T, 32U, 0U, 0U, 0U, 71U, 174U, 227U, 249U,
    MAX_uint8_T, MAX_uint8_T, 32U, 0U, 0U, 114U, MAX_uint8_T, MAX_uint8_T,
    MAX_uint8_T, 254U, MAX_uint8_T, MAX_uint8_T, 32U, 0U, 4U, 242U, 236U, 92U,
    24U, 2U, 152U, MAX_uint8_T, 33U, 0U, 12U, MAX_uint8_T, 211U, 39U, 12U, 60U,
    202U, MAX_uint8_T, 89U, 0U, 0U, 200U, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T,
    MAX_uint8_T, 217U, MAX_uint8_T, MAX_uint8_T, 105U, 0U, 33U, 185U, 244U, 217U,
    106U, 12U, 198U, 240U, 84U, 0U, 0U, 0U, 0U, 0U, 127U, MAX_uint8_T, 100U, 0U,
    0U, 0U, 0U, 0U, 0U, 56U, 251U, 113U, 0U, 0U, 0U, 0U, 0U, 0U, 12U, 220U, 128U,
    0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 3U, 86U, 188U,
    235U, 241U, 192U, 50U, 0U, 0U, 0U, 68U, MAX_uint8_T, MAX_uint8_T,
    MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, 222U, 0U, 0U, 0U, 63U, 161U, 57U, 12U,
    32U, 214U, MAX_uint8_T, 21U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 153U, MAX_uint8_T,
    32U, 0U, 0U, 0U, 71U, 174U, 227U, 249U, MAX_uint8_T, MAX_uint8_T, 32U, 0U,
    0U, 114U, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, 254U, MAX_uint8_T,
    MAX_uint8_T, 32U, 0U, 4U, 242U, 236U, 92U, 24U, 2U, 152U, MAX_uint8_T, 33U,
    0U, 12U, MAX_uint8_T, 211U, 39U, 12U, 60U, 202U, MAX_uint8_T, 89U, 0U, 0U,
    200U, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, 217U, MAX_uint8_T,
    MAX_uint8_T, 105U, 0U, 33U, 185U, 244U, 217U, 106U, 12U, 198U, 240U, 84U, 0U,
    0U, 0U, 75U, MAX_uint8_T, MAX_uint8_T, 82U, 0U, 0U, 0U, 0U, 0U, 22U, 233U,
    151U, 147U, 237U, 26U, 0U, 0U, 0U, 0U, 180U, 173U, 2U, 2U, 169U, 187U, 1U,
    0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 3U, 86U, 188U, 235U, 241U,
    192U, 50U, 0U, 0U, 0U, 68U, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T,
    MAX_uint8_T, MAX_uint8_T, 222U, 0U, 0U, 0U, 63U, 161U, 57U, 12U, 32U, 214U,
    MAX_uint8_T, 21U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 153U, MAX_uint8_T, 32U, 0U, 0U,
    0U, 71U, 174U, 227U, 249U, MAX_uint8_T, MAX_uint8_T, 32U, 0U, 0U, 114U,
    MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, 254U, MAX_uint8_T, MAX_uint8_T, 32U,
    0U, 4U, 242U, 236U, 92U, 24U, 2U, 152U, MAX_uint8_T, 33U, 0U, 12U,
    MAX_uint8_T, 211U, 39U, 12U, 60U, 202U, MAX_uint8_T, 89U, 0U, 0U, 200U,
    MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, 217U, MAX_uint8_T,
    MAX_uint8_T, 105U, 0U, 33U, 185U, 244U, 217U, 106U, 12U, 198U, 240U, 84U, 0U,
    0U, 100U, 242U, 207U, 68U, 92U, 210U, 0U, 0U, 0U, 0U, 219U, 84U, 72U, 208U,
    242U, 93U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 3U, 86U, 188U,
    235U, 241U, 192U, 50U, 0U, 0U, 0U, 68U, MAX_uint8_T, MAX_uint8_T,
    MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, 222U, 0U, 0U, 0U, 63U, 161U, 57U, 12U,
    32U, 214U, MAX_uint8_T, 21U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 153U, MAX_uint8_T,
    32U, 0U, 0U, 0U, 71U, 174U, 227U, 249U, MAX_uint8_T, MAX_uint8_T, 32U, 0U,
    0U, 114U, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, 254U, MAX_uint8_T,
    MAX_uint8_T, 32U, 0U, 4U, 242U, 236U, 92U, 24U, 2U, 152U, MAX_uint8_T, 33U,
    0U, 12U, MAX_uint8_T, 211U, 39U, 12U, 60U, 202U, MAX_uint8_T, 89U, 0U, 0U,
    200U, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, 217U, MAX_uint8_T,
    MAX_uint8_T, 105U, 0U, 33U, 185U, 244U, 217U, 106U, 12U, 198U, 240U, 84U, 0U,
    0U, 128U, MAX_uint8_T, 4U, 8U, MAX_uint8_T, 128U, 0U, 0U, 0U, 0U, 128U,
    MAX_uint8_T, 4U, 8U, MAX_uint8_T, 128U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
    0U, 0U, 0U, 0U, 3U, 86U, 188U, 235U, 241U, 192U, 50U, 0U, 0U, 0U, 68U,
    MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, 222U, 0U,
    0U, 0U, 63U, 161U, 57U, 12U, 32U, 214U, MAX_uint8_T, 21U, 0U, 0U, 0U, 0U, 0U,
    0U, 0U, 153U, MAX_uint8_T, 32U, 0U, 0U, 0U, 71U, 174U, 227U, 249U,
    MAX_uint8_T, MAX_uint8_T, 32U, 0U, 0U, 114U, MAX_uint8_T, MAX_uint8_T,
    MAX_uint8_T, 254U, MAX_uint8_T, MAX_uint8_T, 32U, 0U, 4U, 242U, 236U, 92U,
    24U, 2U, 152U, MAX_uint8_T, 33U, 0U, 12U, MAX_uint8_T, 211U, 39U, 12U, 60U,
    202U, MAX_uint8_T, 89U, 0U, 0U, 200U, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T,
    MAX_uint8_T, 217U, MAX_uint8_T, MAX_uint8_T, 105U, 0U, 33U, 185U, 244U, 217U,
    106U, 12U, 198U, 240U, 84U, 0U, 0U, 0U, 73U, 227U, 228U, 73U, 0U, 0U, 0U, 0U,
    0U, 0U, 206U, 35U, 36U, 206U, 0U, 0U, 0U, 0U, 0U, 0U, 206U, 35U, 35U, 206U,
    0U, 0U, 0U, 0U, 0U, 0U, 76U, 230U, 227U, 73U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
    0U, 0U, 0U, 0U, 0U, 0U, 3U, 86U, 188U, 235U, 241U, 192U, 50U, 0U, 0U, 0U,
    68U, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, 222U,
    0U, 0U, 0U, 63U, 161U, 57U, 12U, 32U, 214U, MAX_uint8_T, 21U, 0U, 0U, 0U, 0U,
    0U, 0U, 0U, 153U, MAX_uint8_T, 32U, 0U, 0U, 0U, 71U, 174U, 227U, 249U,
    MAX_uint8_T, MAX_uint8_T, 32U, 0U, 0U, 114U, MAX_uint8_T, MAX_uint8_T,
    MAX_uint8_T, 254U, MAX_uint8_T, MAX_uint8_T, 32U, 0U, 4U, 242U, 236U, 92U,
    24U, 2U, 152U, MAX_uint8_T, 33U, 0U, 12U, MAX_uint8_T, 211U, 39U, 12U, 60U,
    202U, MAX_uint8_T, 89U, 0U, 0U, 200U, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T,
    MAX_uint8_T, 217U, MAX_uint8_T, MAX_uint8_T, 105U, 0U, 33U, 185U, 244U, 217U,
    106U, 12U, 198U, 240U, 84U, 0U, 3U, 86U, 188U, 235U, 237U, 134U, 2U, 58U,
    206U, 247U, 204U, 61U, 0U, 0U, 0U, 68U, MAX_uint8_T, MAX_uint8_T,
    MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, 157U, 242U, MAX_uint8_T, MAX_uint8_T,
    MAX_uint8_T, 243U, 33U, 0U, 0U, 63U, 161U, 57U, 12U, 33U, 215U, MAX_uint8_T,
    209U, 43U, 11U, 101U, MAX_uint8_T, 143U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 153U,
    MAX_uint8_T, 80U, 0U, 0U, 0U, 222U, 210U, 0U, 0U, 0U, 71U, 174U, 227U, 249U,
    MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T,
    MAX_uint8_T, 247U, 0U, 0U, 114U, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, 254U,
    MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T,
    MAX_uint8_T, MAX_uint8_T, 6U, 4U, 242U, 236U, 92U, 24U, 2U, 152U,
    MAX_uint8_T, 79U, 0U, 0U, 0U, 0U, 0U, 0U, 12U, MAX_uint8_T, 211U, 39U, 20U,
    102U, 225U, 223U, 231U, 84U, 20U, 11U, 62U, 181U, 11U, 0U, 200U, MAX_uint8_T,
    MAX_uint8_T, MAX_uint8_T, 252U, 83U, 60U, 247U, MAX_uint8_T, MAX_uint8_T,
    MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, 12U, 0U, 33U, 185U, 243U, 208U, 70U,
    0U, 0U, 55U, 187U, 242U, 231U, 175U, 64U, 0U, 0U, 0U, 15U, 138U, 220U, 248U,
    215U, 109U, 4U, 0U, 14U, 213U, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T,
    MAX_uint8_T, MAX_uint8_T, 64U, 0U, 138U, MAX_uint8_T, 234U, 87U, 16U, 39U,
    148U, 59U, 0U, 222U, MAX_uint8_T, 79U, 0U, 0U, 0U, 0U, 0U, 2U, 253U, 251U,
    6U, 0U, 0U, 0U, 0U, 0U, 1U, 252U, 250U, 5U, 0U, 0U, 0U, 0U, 0U, 0U, 211U,
    MAX_uint8_T, 80U, 0U, 0U, 0U, 0U, 0U, 0U, 115U, MAX_uint8_T, 235U, 90U, 17U,
    30U, 123U, 82U, 0U, 5U, 193U, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T,
    MAX_uint8_T, MAX_uint8_T, 92U, 0U, 0U, 9U, 133U, 224U, 246U, 204U, 105U, 7U,
    0U, 0U, 0U, 0U, 13U, 190U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 107U, 251U, 169U, 4U,
    0U, 0U, 0U, 0U, 0U, 2U, 64U, MAX_uint8_T, 45U, 0U, 0U, 0U, 0U, 0U, 208U,
    250U, 168U, 3U, 0U, 0U, 0U, 93U, 254U, 134U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
    106U, 252U, 62U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 121U, 225U, 15U, 0U, 0U, 0U, 0U,
    0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 23U, 160U, 236U, 245U, 177U, 27U, 0U, 0U,
    14U, 218U, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, 209U, 3U, 0U,
    135U, MAX_uint8_T, 151U, 21U, 17U, 146U, MAX_uint8_T, 83U, 0U, 220U, 241U,
    4U, 0U, 0U, 12U, MAX_uint8_T, 151U, 2U, 254U, MAX_uint8_T, MAX_uint8_T,
    MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, 183U, 1U, 252U,
    MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T,
    194U, 0U, 210U, 235U, 8U, 0U, 0U, 0U, 0U, 0U, 0U, 114U, MAX_uint8_T, 189U,
    59U, 13U, 22U, 95U, 152U, 0U, 4U, 186U, MAX_uint8_T, MAX_uint8_T,
    MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, 188U, 0U, 0U, 5U, 119U, 214U, 248U,
    221U, 145U, 31U, 0U, 0U, 0U, 0U, 0U, 135U, 254U, 92U, 0U, 0U, 0U, 0U, 0U,
    62U, 252U, 106U, 0U, 0U, 0U, 0U, 0U, 15U, 225U, 120U, 0U, 0U, 0U, 0U, 0U, 0U,
    0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 23U, 160U, 236U, 245U, 177U, 27U, 0U, 0U,
    14U, 218U, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, 209U, 3U, 0U,
    135U, MAX_uint8_T, 151U, 21U, 17U, 146U, MAX_uint8_T, 83U, 0U, 220U, 241U,
    4U, 0U, 0U, 12U, MAX_uint8_T, 151U, 2U, 254U, MAX_uint8_T, MAX_uint8_T,
    MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, 183U, 1U, 252U,
    MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T,
    194U, 0U, 210U, 235U, 8U, 0U, 0U, 0U, 0U, 0U, 0U, 114U, MAX_uint8_T, 189U,
    59U, 13U, 22U, 95U, 152U, 0U, 4U, 186U, MAX_uint8_T, MAX_uint8_T,
    MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, 188U, 0U, 0U, 5U, 119U, 214U, 248U,
    221U, 145U, 31U, 0U, 0U, 0U, 49U, 249U, MAX_uint8_T, 114U, 0U, 0U, 0U, 0U,
    9U, 214U, 179U, 119U, 248U, 46U, 0U, 0U, 0U, 149U, 198U, 9U, 0U, 140U, 212U,
    8U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 23U, 160U, 236U, 245U, 177U,
    27U, 0U, 0U, 14U, 218U, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T,
    209U, 3U, 0U, 135U, MAX_uint8_T, 151U, 21U, 17U, 146U, MAX_uint8_T, 83U, 0U,
    220U, 241U, 4U, 0U, 0U, 12U, MAX_uint8_T, 151U, 2U, 254U, MAX_uint8_T,
    MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, 183U, 1U,
    252U, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T,
    MAX_uint8_T, 194U, 0U, 210U, 235U, 8U, 0U, 0U, 0U, 0U, 0U, 0U, 114U,
    MAX_uint8_T, 189U, 59U, 13U, 22U, 95U, 152U, 0U, 4U, 186U, MAX_uint8_T,
    MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, 188U, 0U, 0U, 5U, 119U,
    214U, 248U, 221U, 145U, 31U, 0U, 0U, 76U, MAX_uint8_T, 56U, 0U, 212U, 180U,
    0U, 0U, 0U, 76U, MAX_uint8_T, 56U, 0U, 212U, 180U, 0U, 0U, 0U, 0U, 0U, 0U,
    0U, 0U, 0U, 0U, 0U, 0U, 23U, 160U, 236U, 245U, 177U, 27U, 0U, 0U, 14U, 218U,
    MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, 209U, 3U, 0U, 135U,
    MAX_uint8_T, 151U, 21U, 17U, 146U, MAX_uint8_T, 83U, 0U, 220U, 241U, 4U, 0U,
    0U, 12U, MAX_uint8_T, 151U, 2U, 254U, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T,
    MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, 183U, 1U, 252U, MAX_uint8_T,
    MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, 194U, 0U,
    210U, 235U, 8U, 0U, 0U, 0U, 0U, 0U, 0U, 114U, MAX_uint8_T, 189U, 59U, 13U,
    22U, 95U, 152U, 0U, 4U, 186U, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T,
    MAX_uint8_T, MAX_uint8_T, 188U, 0U, 0U, 5U, 119U, 214U, 248U, 221U, 145U,
    31U, 6U, 185U, 247U, 42U, 0U, 0U, 10U, 195U, 208U, 6U, 0U, 0U, 15U, 205U,
    141U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 68U, MAX_uint8_T, 120U, 0U, 0U, 68U,
    MAX_uint8_T, 120U, 0U, 0U, 68U, MAX_uint8_T, 120U, 0U, 0U, 68U, MAX_uint8_T,
    120U, 0U, 0U, 68U, MAX_uint8_T, 120U, 0U, 0U, 68U, MAX_uint8_T, 120U, 0U, 0U,
    68U, MAX_uint8_T, 120U, 0U, 0U, 68U, MAX_uint8_T, 120U, 0U, 0U, 68U,
    MAX_uint8_T, 120U, 0U, 0U, 68U, MAX_uint8_T, 120U, 0U, 14U, 224U, 219U, 24U,
    0U, 164U, 226U, 31U, 0U, 89U, 233U, 39U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 68U,
    MAX_uint8_T, 120U, 0U, 0U, 68U, MAX_uint8_T, 120U, 0U, 0U, 68U, MAX_uint8_T,
    120U, 0U, 0U, 68U, MAX_uint8_T, 120U, 0U, 0U, 68U, MAX_uint8_T, 120U, 0U, 0U,
    68U, MAX_uint8_T, 120U, 0U, 0U, 68U, MAX_uint8_T, 120U, 0U, 0U, 68U,
    MAX_uint8_T, 120U, 0U, 0U, 68U, MAX_uint8_T, 120U, 0U, 0U, 68U, MAX_uint8_T,
    120U, 0U, 0U, 0U, 0U, 178U, MAX_uint8_T, 220U, 13U, 0U, 0U, 104U, 245U, 83U,
    224U, 159U, 0U, 39U, 241U, 75U, 0U, 39U, 236U, 85U, 0U, 0U, 0U, 0U, 0U, 0U,
    0U, 0U, 0U, 68U, MAX_uint8_T, 120U, 0U, 0U, 0U, 0U, 68U, MAX_uint8_T, 120U,
    0U, 0U, 0U, 0U, 68U, MAX_uint8_T, 120U, 0U, 0U, 0U, 0U, 68U, MAX_uint8_T,
    120U, 0U, 0U, 0U, 0U, 68U, MAX_uint8_T, 120U, 0U, 0U, 0U, 0U, 68U,
    MAX_uint8_T, 120U, 0U, 0U, 0U, 0U, 68U, MAX_uint8_T, 120U, 0U, 0U, 0U, 0U,
    68U, MAX_uint8_T, 120U, 0U, 0U, 0U, 0U, 68U, MAX_uint8_T, 120U, 0U, 0U, 0U,
    0U, 68U, MAX_uint8_T, 120U, 0U, 0U, 228U, 160U, 0U, 108U, MAX_uint8_T, 28U,
    228U, 160U, 0U, 108U, MAX_uint8_T, 28U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 68U,
    MAX_uint8_T, 120U, 0U, 0U, 0U, 68U, MAX_uint8_T, 120U, 0U, 0U, 0U, 68U,
    MAX_uint8_T, 120U, 0U, 0U, 0U, 68U, MAX_uint8_T, 120U, 0U, 0U, 0U, 68U,
    MAX_uint8_T, 120U, 0U, 0U, 0U, 68U, MAX_uint8_T, 120U, 0U, 0U, 0U, 68U,
    MAX_uint8_T, 120U, 0U, 0U, 0U, 68U, MAX_uint8_T, 120U, 0U, 0U, 0U, 68U,
    MAX_uint8_T, 120U, 0U, 0U, 0U, 68U, MAX_uint8_T, 120U, 0U, 0U, 0U, 0U, 0U,
    0U, 6U, 160U, 96U, 0U, 0U, 0U, 7U, 251U, 242U, 219U, 220U, 205U, 20U, 0U, 0U,
    0U, 7U, 234U, 205U, 250U, 232U, 247U, 108U, 0U, 0U, 0U, 0U, 0U, 142U, 120U,
    3U, 108U, 251U, 138U, 0U, 0U, 0U, 0U, 20U, 153U, 232U, 246U, 240U,
    MAX_uint8_T, 100U, 0U, 0U, 15U, 217U, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T,
    MAX_uint8_T, MAX_uint8_T, 240U, 18U, 0U, 138U, MAX_uint8_T, 204U, 53U, 11U,
    50U, 202U, MAX_uint8_T, 111U, 0U, 224U, 252U, 33U, 0U, 0U, 0U, 39U,
    MAX_uint8_T, 179U, 4U, MAX_uint8_T, 220U, 0U, 0U, 0U, 0U, 0U, 241U, 211U, 3U,
    254U, 223U, 0U, 0U, 0U, 0U, 1U, 247U, 210U, 0U, 220U, 253U, 36U, 0U, 0U, 0U,
    63U, MAX_uint8_T, 174U, 0U, 132U, MAX_uint8_T, 205U, 51U, 11U, 66U, 222U,
    MAX_uint8_T, 89U, 0U, 12U, 212U, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T,
    MAX_uint8_T, MAX_uint8_T, 181U, 1U, 0U, 0U, 17U, 147U, 229U, 250U, 221U,
    127U, 6U, 0U, 0U, 0U, 129U, 244U, 193U, 52U, 123U, 178U, 0U, 0U, 3U, 248U,
    54U, 90U, 219U, 236U, 68U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 68U,
    MAX_uint8_T, 120U, 18U, 163U, 240U, 223U, 97U, 0U, 68U, MAX_uint8_T, 127U,
    205U, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, 254U, 49U, 68U, MAX_uint8_T,
    233U, 213U, 62U, 11U, 143U, MAX_uint8_T, 117U, 68U, MAX_uint8_T, 221U, 18U,
    0U, 0U, 57U, MAX_uint8_T, 136U, 68U, MAX_uint8_T, 120U, 0U, 0U, 0U, 48U,
    MAX_uint8_T, 136U, 68U, MAX_uint8_T, 120U, 0U, 0U, 0U, 48U, MAX_uint8_T,
    136U, 68U, MAX_uint8_T, 120U, 0U, 0U, 0U, 48U, MAX_uint8_T, 136U, 68U,
    MAX_uint8_T, 120U, 0U, 0U, 0U, 48U, MAX_uint8_T, 136U, 68U, MAX_uint8_T,
    120U, 0U, 0U, 0U, 48U, MAX_uint8_T, 136U, 68U, MAX_uint8_T, 120U, 0U, 0U, 0U,
    48U, MAX_uint8_T, 136U, 0U, 0U, 12U, 199U, 239U, 30U, 0U, 0U, 0U, 0U, 0U, 0U,
    0U, 0U, 17U, 209U, 193U, 2U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 23U, 217U, 121U,
    0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 18U,
    145U, 225U, 250U, 228U, 151U, 21U, 0U, 0U, 0U, 14U, 216U, MAX_uint8_T,
    MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, 220U, 18U, 0U, 0U, 136U,
    MAX_uint8_T, 199U, 49U, 10U, 44U, 187U, MAX_uint8_T, 146U, 0U, 0U, 220U,
    253U, 32U, 0U, 0U, 0U, 20U, 246U, 231U, 0U, 1U, 253U, 228U, 0U, 0U, 0U, 0U,
    0U, 208U, MAX_uint8_T, 10U, 2U, 253U, 227U, 0U, 0U, 0U, 0U, 0U, 209U,
    MAX_uint8_T, 10U, 0U, 219U, 253U, 30U, 0U, 0U, 0U, 21U, 246U, 231U, 0U, 0U,
    134U, MAX_uint8_T, 197U, 47U, 9U, 45U, 189U, MAX_uint8_T, 148U, 0U, 0U, 13U,
    214U, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, 222U,
    19U, 0U, 0U, 0U, 19U, 148U, 229U, 250U, 228U, 150U, 22U, 0U, 0U, 0U, 0U, 0U,
    0U, 0U, 24U, 234U, 207U, 15U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 183U, 216U, 21U,
    0U, 0U, 0U, 0U, 0U, 0U, 0U, 109U, 224U, 28U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
    0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 18U, 145U, 225U, 250U, 228U, 151U, 21U,
    0U, 0U, 0U, 14U, 216U, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T,
    MAX_uint8_T, 220U, 18U, 0U, 0U, 136U, MAX_uint8_T, 199U, 49U, 10U, 44U, 187U,
    MAX_uint8_T, 146U, 0U, 0U, 220U, 253U, 32U, 0U, 0U, 0U, 20U, 246U, 231U, 0U,
    1U, 253U, 228U, 0U, 0U, 0U, 0U, 0U, 208U, MAX_uint8_T, 10U, 2U, 253U, 227U,
    0U, 0U, 0U, 0U, 0U, 209U, MAX_uint8_T, 10U, 0U, 219U, 253U, 30U, 0U, 0U, 0U,
    21U, 246U, 231U, 0U, 0U, 134U, MAX_uint8_T, 197U, 47U, 9U, 45U, 189U,
    MAX_uint8_T, 148U, 0U, 0U, 13U, 214U, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T,
    MAX_uint8_T, MAX_uint8_T, 222U, 19U, 0U, 0U, 0U, 19U, 148U, 229U, 250U, 228U,
    150U, 22U, 0U, 0U, 0U, 0U, 0U, 3U, 195U, MAX_uint8_T, 207U, 6U, 0U, 0U, 0U,
    0U, 0U, 0U, 124U, 239U, 81U, 234U, 139U, 0U, 0U, 0U, 0U, 0U, 53U, 242U, 60U,
    0U, 52U, 241U, 67U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
    0U, 18U, 145U, 225U, 250U, 228U, 151U, 21U, 0U, 0U, 0U, 14U, 216U,
    MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, 220U, 18U,
    0U, 0U, 136U, MAX_uint8_T, 199U, 49U, 10U, 44U, 187U, MAX_uint8_T, 146U, 0U,
    0U, 220U, 253U, 32U, 0U, 0U, 0U, 20U, 246U, 231U, 0U, 1U, 253U, 228U, 0U, 0U,
    0U, 0U, 0U, 208U, MAX_uint8_T, 10U, 2U, 253U, 227U, 0U, 0U, 0U, 0U, 0U, 209U,
    MAX_uint8_T, 10U, 0U, 219U, 253U, 30U, 0U, 0U, 0U, 21U, 246U, 231U, 0U, 0U,
    134U, MAX_uint8_T, 197U, 47U, 9U, 45U, 189U, MAX_uint8_T, 148U, 0U, 0U, 13U,
    214U, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, 222U,
    19U, 0U, 0U, 0U, 19U, 148U, 229U, 250U, 228U, 150U, 22U, 0U, 0U, 0U, 0U, 9U,
    197U, 243U, 149U, 26U, 201U, 94U, 0U, 0U, 0U, 0U, 79U, 214U, 25U, 138U, 240U,
    206U, 16U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 18U,
    145U, 225U, 250U, 228U, 151U, 21U, 0U, 0U, 0U, 14U, 216U, MAX_uint8_T,
    MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, 220U, 18U, 0U, 0U, 136U,
    MAX_uint8_T, 199U, 49U, 10U, 44U, 187U, MAX_uint8_T, 146U, 0U, 0U, 220U,
    253U, 32U, 0U, 0U, 0U, 20U, 246U, 231U, 0U, 1U, 253U, 228U, 0U, 0U, 0U, 0U,
    0U, 208U, MAX_uint8_T, 10U, 2U, 253U, 227U, 0U, 0U, 0U, 0U, 0U, 209U,
    MAX_uint8_T, 10U, 0U, 219U, 253U, 30U, 0U, 0U, 0U, 21U, 246U, 231U, 0U, 0U,
    134U, MAX_uint8_T, 197U, 47U, 9U, 45U, 189U, MAX_uint8_T, 148U, 0U, 0U, 13U,
    214U, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, 222U,
    19U, 0U, 0U, 0U, 19U, 148U, 229U, 250U, 228U, 150U, 22U, 0U, 0U, 0U, 0U, 0U,
    248U, 140U, 0U, 128U, MAX_uint8_T, 8U, 0U, 0U, 0U, 0U, 0U, 248U, 140U, 0U,
    128U, MAX_uint8_T, 8U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
    0U, 0U, 18U, 145U, 225U, 250U, 228U, 151U, 21U, 0U, 0U, 0U, 14U, 216U,
    MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, 220U, 18U,
    0U, 0U, 136U, MAX_uint8_T, 199U, 49U, 10U, 44U, 187U, MAX_uint8_T, 146U, 0U,
    0U, 220U, 253U, 32U, 0U, 0U, 0U, 20U, 246U, 231U, 0U, 1U, 253U, 228U, 0U, 0U,
    0U, 0U, 0U, 208U, MAX_uint8_T, 10U, 2U, 253U, 227U, 0U, 0U, 0U, 0U, 0U, 209U,
    MAX_uint8_T, 10U, 0U, 219U, 253U, 30U, 0U, 0U, 0U, 21U, 246U, 231U, 0U, 0U,
    134U, MAX_uint8_T, 197U, 47U, 9U, 45U, 189U, MAX_uint8_T, 148U, 0U, 0U, 13U,
    214U, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, 222U,
    19U, 0U, 0U, 0U, 19U, 148U, 229U, 250U, 228U, 150U, 22U, 0U, 0U, 0U, 0U, 0U,
    0U, 0U, 236U, MAX_uint8_T, 64U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 236U,
    MAX_uint8_T, 64U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
    0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
    0U, 0U, 0U, 0U, 0U, 0U, 12U, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T,
    MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T,
    MAX_uint8_T, 92U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
    0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
    0U, 0U, 0U, 0U, 0U, 0U, 0U, 236U, MAX_uint8_T, 64U, 0U, 0U, 0U, 0U, 0U, 0U,
    0U, 0U, 0U, 236U, MAX_uint8_T, 64U, 0U, 0U, 0U, 0U, 0U, 0U, 18U, 145U, 225U,
    249U, 223U, 146U, 162U, 175U, 0U, 0U, 14U, 216U, MAX_uint8_T, MAX_uint8_T,
    MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, 33U, 0U, 0U, 136U,
    MAX_uint8_T, 198U, 50U, 10U, 52U, 242U, MAX_uint8_T, 143U, 0U, 0U, 219U,
    253U, 31U, 0U, 2U, 167U, 167U, 250U, 229U, 0U, 1U, 253U, 227U, 0U, 1U, 159U,
    166U, 2U, 206U, MAX_uint8_T, 11U, 2U, 253U, 225U, 0U, 150U, 175U, 4U, 0U,
    208U, MAX_uint8_T, 10U, 0U, 220U, 254U, 163U, 182U, 6U, 0U, 19U, 246U, 230U,
    0U, 0U, 134U, MAX_uint8_T, 247U, 58U, 8U, 43U, 187U, MAX_uint8_T, 147U, 0U,
    0U, 22U, 251U, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T,
    MAX_uint8_T, 222U, 19U, 0U, 0U, 163U, 174U, 139U, 221U, 249U, 230U, 153U,
    23U, 0U, 0U, 0U, 12U, 199U, 239U, 30U, 0U, 0U, 0U, 0U, 0U, 0U, 17U, 209U,
    193U, 2U, 0U, 0U, 0U, 0U, 0U, 0U, 23U, 217U, 121U, 0U, 0U, 0U, 0U, 0U, 0U,
    0U, 0U, 0U, 0U, 0U, 0U, 96U, MAX_uint8_T, 92U, 0U, 0U, 0U, 76U, MAX_uint8_T,
    112U, 96U, MAX_uint8_T, 92U, 0U, 0U, 0U, 76U, MAX_uint8_T, 112U, 96U,
    MAX_uint8_T, 92U, 0U, 0U, 0U, 76U, MAX_uint8_T, 112U, 96U, MAX_uint8_T, 92U,
    0U, 0U, 0U, 76U, MAX_uint8_T, 112U, 96U, MAX_uint8_T, 92U, 0U, 0U, 0U, 76U,
    MAX_uint8_T, 112U, 96U, MAX_uint8_T, 92U, 0U, 0U, 0U, 76U, MAX_uint8_T, 112U,
    96U, MAX_uint8_T, 101U, 0U, 0U, 4U, 188U, MAX_uint8_T, 112U, 77U,
    MAX_uint8_T, 181U, 13U, 44U, 187U, 233U, MAX_uint8_T, 112U, 21U, 243U,
    MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, 231U, 101U, MAX_uint8_T, 112U, 0U,
    72U, 213U, 244U, 184U, 37U, 76U, MAX_uint8_T, 112U, 0U, 0U, 0U, 0U, 24U,
    234U, 207U, 15U, 0U, 0U, 0U, 0U, 0U, 183U, 216U, 21U, 0U, 0U, 0U, 0U, 0U,
    109U, 224U, 28U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 96U,
    MAX_uint8_T, 92U, 0U, 0U, 0U, 76U, MAX_uint8_T, 112U, 96U, MAX_uint8_T, 92U,
    0U, 0U, 0U, 76U, MAX_uint8_T, 112U, 96U, MAX_uint8_T, 92U, 0U, 0U, 0U, 76U,
    MAX_uint8_T, 112U, 96U, MAX_uint8_T, 92U, 0U, 0U, 0U, 76U, MAX_uint8_T, 112U,
    96U, MAX_uint8_T, 92U, 0U, 0U, 0U, 76U, MAX_uint8_T, 112U, 96U, MAX_uint8_T,
    92U, 0U, 0U, 0U, 76U, MAX_uint8_T, 112U, 96U, MAX_uint8_T, 101U, 0U, 0U, 4U,
    188U, MAX_uint8_T, 112U, 77U, MAX_uint8_T, 181U, 13U, 44U, 187U, 233U,
    MAX_uint8_T, 112U, 21U, 243U, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, 231U,
    101U, MAX_uint8_T, 112U, 0U, 72U, 213U, 244U, 184U, 37U, 76U, MAX_uint8_T,
    112U, 0U, 0U, 3U, 195U, MAX_uint8_T, 207U, 6U, 0U, 0U, 0U, 0U, 124U, 239U,
    81U, 234U, 139U, 0U, 0U, 0U, 53U, 242U, 60U, 0U, 52U, 241U, 67U, 0U, 0U, 0U,
    0U, 0U, 0U, 0U, 0U, 0U, 0U, 96U, MAX_uint8_T, 92U, 0U, 0U, 0U, 76U,
    MAX_uint8_T, 112U, 96U, MAX_uint8_T, 92U, 0U, 0U, 0U, 76U, MAX_uint8_T, 112U,
    96U, MAX_uint8_T, 92U, 0U, 0U, 0U, 76U, MAX_uint8_T, 112U, 96U, MAX_uint8_T,
    92U, 0U, 0U, 0U, 76U, MAX_uint8_T, 112U, 96U, MAX_uint8_T, 92U, 0U, 0U, 0U,
    76U, MAX_uint8_T, 112U, 96U, MAX_uint8_T, 92U, 0U, 0U, 0U, 76U, MAX_uint8_T,
    112U, 96U, MAX_uint8_T, 101U, 0U, 0U, 4U, 188U, MAX_uint8_T, 112U, 77U,
    MAX_uint8_T, 181U, 13U, 44U, 187U, 233U, MAX_uint8_T, 112U, 21U, 243U,
    MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, 231U, 101U, MAX_uint8_T, 112U, 0U,
    72U, 213U, 244U, 184U, 37U, 76U, MAX_uint8_T, 112U, 0U, 0U, 248U, 140U, 0U,
    128U, MAX_uint8_T, 8U, 0U, 0U, 0U, 248U, 140U, 0U, 128U, MAX_uint8_T, 8U, 0U,
    0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 96U, MAX_uint8_T, 92U, 0U, 0U, 0U, 76U,
    MAX_uint8_T, 112U, 96U, MAX_uint8_T, 92U, 0U, 0U, 0U, 76U, MAX_uint8_T, 112U,
    96U, MAX_uint8_T, 92U, 0U, 0U, 0U, 76U, MAX_uint8_T, 112U, 96U, MAX_uint8_T,
    92U, 0U, 0U, 0U, 76U, MAX_uint8_T, 112U, 96U, MAX_uint8_T, 92U, 0U, 0U, 0U,
    76U, MAX_uint8_T, 112U, 96U, MAX_uint8_T, 92U, 0U, 0U, 0U, 76U, MAX_uint8_T,
    112U, 96U, MAX_uint8_T, 101U, 0U, 0U, 4U, 188U, MAX_uint8_T, 112U, 77U,
    MAX_uint8_T, 181U, 13U, 44U, 187U, 233U, MAX_uint8_T, 112U, 21U, 243U,
    MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, 231U, 101U, MAX_uint8_T, 112U, 0U,
    72U, 213U, 244U, 184U, 37U, 76U, MAX_uint8_T, 112U, 0U, 0U, 0U, 0U, 3U, 199U,
    236U, 42U, 0U, 0U, 0U, 0U, 0U, 0U, 128U, 241U, 52U, 0U, 0U, 0U, 0U, 0U, 0U,
    57U, 242U, 62U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 173U,
    MAX_uint8_T, 49U, 0U, 0U, 0U, 0U, 138U, 254U, 30U, 79U, MAX_uint8_T, 142U,
    0U, 0U, 0U, 4U, 231U, 188U, 0U, 5U, 234U, 232U, 3U, 0U, 0U, 79U, MAX_uint8_T,
    93U, 0U, 0U, 146U, MAX_uint8_T, 72U, 0U, 0U, 178U, 243U, 11U, 0U, 0U, 52U,
    MAX_uint8_T, 165U, 0U, 24U, 251U, 159U, 0U, 0U, 0U, 0U, 212U, 245U, 13U,
    119U, MAX_uint8_T, 64U, 0U, 0U, 0U, 0U, 119U, MAX_uint8_T, 96U, 217U, 223U,
    1U, 0U, 0U, 0U, 0U, 27U, 253U, 228U, MAX_uint8_T, 129U, 0U, 0U, 0U, 0U, 0U,
    0U, 186U, MAX_uint8_T, MAX_uint8_T, 35U, 0U, 0U, 0U, 0U, 0U, 0U, 91U,
    MAX_uint8_T, 195U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 94U, MAX_uint8_T, 100U, 0U,
    0U, 0U, 0U, 0U, 0U, 0U, 193U, 246U, 14U, 0U, 0U, 0U, 0U, 0U, 0U, 36U,
    MAX_uint8_T, 166U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 135U, MAX_uint8_T, 71U, 0U,
    0U, 0U, 0U, 0U, 68U, MAX_uint8_T, 120U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 68U,
    MAX_uint8_T, 120U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 68U, MAX_uint8_T, 120U, 0U,
    0U, 0U, 0U, 0U, 0U, 0U, 68U, MAX_uint8_T, 120U, 50U, 194U, 246U, 225U, 121U,
    1U, 0U, 68U, MAX_uint8_T, 150U, 241U, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T,
    MAX_uint8_T, 117U, 0U, 68U, MAX_uint8_T, 249U, 141U, 34U, 13U, 96U, 253U,
    237U, 11U, 68U, MAX_uint8_T, 136U, 0U, 0U, 0U, 0U, 178U, MAX_uint8_T, 52U,
    68U, MAX_uint8_T, 120U, 0U, 0U, 0U, 0U, 134U, MAX_uint8_T, 82U, 68U,
    MAX_uint8_T, 120U, 0U, 0U, 0U, 0U, 144U, MAX_uint8_T, 78U, 68U, MAX_uint8_T,
    126U, 0U, 0U, 0U, 0U, 201U, MAX_uint8_T, 38U, 68U, MAX_uint8_T, 248U, 119U,
    20U, 22U, 130U, MAX_uint8_T, 209U, 0U, 68U, MAX_uint8_T, 173U, 250U,
    MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, 250U, 64U, 0U, 68U, MAX_uint8_T, 120U,
    72U, 214U, 248U, 201U, 67U, 0U, 0U, 68U, MAX_uint8_T, 120U, 0U, 0U, 0U, 0U,
    0U, 0U, 0U, 68U, MAX_uint8_T, 120U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 68U,
    MAX_uint8_T, 120U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 68U, MAX_uint8_T, 120U, 0U,
    0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 168U, 220U, 0U, 48U, MAX_uint8_T, 88U, 0U,
    0U, 0U, 0U, 168U, 220U, 0U, 48U, MAX_uint8_T, 88U, 0U, 0U, 0U, 0U, 0U, 0U,
    0U, 0U, 0U, 0U, 0U, 0U, 173U, MAX_uint8_T, 49U, 0U, 0U, 0U, 0U, 138U, 254U,
    30U, 79U, MAX_uint8_T, 142U, 0U, 0U, 0U, 4U, 231U, 188U, 0U, 5U, 234U, 232U,
    3U, 0U, 0U, 79U, MAX_uint8_T, 93U, 0U, 0U, 146U, MAX_uint8_T, 72U, 0U, 0U,
    178U, 243U, 11U, 0U, 0U, 52U, MAX_uint8_T, 165U, 0U, 24U, 251U, 159U, 0U, 0U,
    0U, 0U, 212U, 245U, 13U, 119U, MAX_uint8_T, 64U, 0U, 0U, 0U, 0U, 119U,
    MAX_uint8_T, 96U, 217U, 223U, 1U, 0U, 0U, 0U, 0U, 27U, 253U, 228U,
    MAX_uint8_T, 129U, 0U, 0U, 0U, 0U, 0U, 0U, 186U, MAX_uint8_T, MAX_uint8_T,
    35U, 0U, 0U, 0U, 0U, 0U, 0U, 91U, MAX_uint8_T, 195U, 0U, 0U, 0U, 0U, 0U, 0U,
    0U, 94U, MAX_uint8_T, 100U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 193U, 246U, 14U, 0U,
    0U, 0U, 0U, 0U, 0U, 36U, MAX_uint8_T, 166U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 135U,
    MAX_uint8_T, 71U, 0U, 0U, 0U, 0U, 0U };

  static const unsigned char uv11[10664] = { 60U, 96U, 96U, 96U, 96U, 96U, 60U,
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

  static const signed char iv1[261] = { 2, 0, 0, 0, 1, 0, 0, 1, 0, 0, 0, 1, 0, 0,
    1, 1, 1, 1, 1, 1, 2, 1, 1, 0, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 1, 0, 1,
    1, 1, 0, 1, 1, -2, 1, 1, 1, 1, 0, 1, 0, 1, 0, 0, 1, 0, 0, 0, 0, 0, 1, 1, 0,
    0, 0, 3, 0, 1, 0, 0, 0, 0, 0, 1, 1, -2, 1, 1, 1, 1, 0, 1, 0, 1, 1, 0, 1, 0,
    0, 0, 0, 1, 0, 2, 1, 0, 0, 0, 0, 1, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    1, -1, -1, 0, 1, 0, 0, 0, 0, 0, 1, 1, 1, 1, 0, 1, 1, 2, 1, 0, 1, 1, 1, 0, 0,
    3, 3, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 1, 1, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 1, 0, 1, 1, 1, -1, 0, -1, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0,
    0, 0, 0, 0, 4, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2, -1, 0, 0, 0, 1, 1, 0, 1, 1, 1,
    1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 1 };

  static const signed char iv2[261] = { 12, 0, 0, 0, 13, 13, 13, 14, 13, 13, 13,
    13, 13, 13, 11, 2, 6, 2, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 10, 10,
    11, 7, 11, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13,
    13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 0, 14, 10,
    13, 10, 13, 10, 13, 10, 13, 13, 13, 13, 13, 10, 10, 10, 10, 10, 10, 10, 12,
    10, 10, 10, 10, 10, 10, 13, 13, 13, 7, 16, 17, 13, 17, 16, 16, 16, 14, 14,
    14, 13, 13, 15, 10, 14, 14, 14, 13, 14, 14, 14, 13, 13, 14, 14, 14, 13, 13,
    14, 14, 14, 13, 0, 14, 13, 13, 13, 0, 13, 13, 13, 14, 0, 14, 13, 0, 13, 13,
    0, 11, 0, 0, 13, 10, 0, 0, 0, 0, 0, 14, 14, 0, 10, 10, 10, 10, 8, 0, 0, 0, 0,
    8, 8, 0, 0, 17, 16, 16, 0, 0, 0, 0, 0, 0, 0, 0, 11, 0, 13, 0, 0, 11, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 17, 17, 17, 16, 17, 17, 17, 16, 17, 17, 17, 0, 17, 17, 17,
    17, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 13, 13, 14, 17, 14,
    13, 13, 0, 10, 13, 13, 13, 13, 13, 13, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 6,
    14, 6 };

  static const signed char iv3[261] = { 14, 0, 0, 6, 6, 7, 11, 11, 12, 13, 4, 6,
    6, 9, 14, 6, 10, 6, 9, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 6, 6, 14, 14,
    14, 8, 15, 12, 10, 12, 13, 10, 10, 13, 13, 5, 6, 12, 10, 16, 13, 14, 10, 14,
    11, 10, 11, 12, 12, 15, 11, 11, 11, 6, 9, 6, 11, 9, 11, 10, 11, 9, 11, 10, 7,
    11, 11, 5, 5, 11, 5, 17, 11, 11, 11, 11, 7, 9, 7, 11, 9, 14, 11, 9, 10, 6, 7,
    6, 11, 12, 12, 12, 10, 13, 14, 12, 10, 10, 10, 10, 10, 10, 9, 10, 10, 10, 10,
    5, 5, 5, 5, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 0, 8, 11, 11, 11, 0, 11,
    11, 11, 15, 0, 11, 11, 0, 16, 14, 0, 14, 0, 0, 11, 11, 0, 0, 0, 0, 0, 8, 8,
    0, 15, 11, 8, 6, 14, 0, 0, 0, 0, 9, 9, 0, 6, 12, 12, 14, 0, 0, 0, 0, 0, 0, 0,
    0, 14, 0, 9, 0, 0, 11, 0, 0, 0, 0, 0, 0, 0, 0, 0, 12, 10, 12, 10, 10, 5, 5,
    5, 5, 14, 14, 0, 14, 12, 12, 12, 0, 0, 0, 0, 0, 0, 0, 11, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 7, 13, 11, 11, 9, 10, 11, 0, 14, 8, 8, 8, 14, 14, 14, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 10, 9, 6 };

  static const signed char iv5[261] = { 9, 0, 0, 4, 4, 4, 8, 8, 8, 8, 3, 4, 4, 6,
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

  static const signed char iv7[261] = { 1, 0, 0, 0, 1, 0, 0, 1, 0, 0, 0, 0, 0, 0,
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

  static const signed char iv8[261] = { 8, 0, 0, 0, 9, 9, 9, 10, 9, 9, 9, 9, 9,
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

  static const signed char uv3[261] = { 10, 0, 0, 0, 3, 6, 12, 9, 12, 12, 4, 5,
    5, 8, 12, 3, 9, 3, 7, 10, 8, 9, 9, 11, 8, 10, 10, 10, 10, 3, 3, 12, 12, 12,
    8, 15, 13, 9, 12, 12, 9, 8, 12, 11, 3, 6, 11, 9, 13, 11, 14, 9, 15, 11, 9,
    12, 10, 12, 16, 11, 11, 11, 5, 7, 5, 11, 9, 5, 10, 10, 9, 10, 9, 8, 10, 9, 3,
    6, 10, 3, 15, 9, 11, 10, 10, 6, 7, 7, 9, 10, 14, 11, 10, 9, 5, 3, 5, 11, 13,
    13, 12, 9, 11, 14, 10, 10, 10, 10, 10, 10, 10, 9, 9, 9, 9, 9, 5, 5, 7, 6, 9,
    11, 11, 11, 11, 11, 9, 9, 9, 9, 0, 6, 9, 8, 9, 0, 9, 10, 9, 15, 0, 5, 6, 0,
    16, 14, 0, 12, 0, 0, 11, 9, 0, 0, 0, 0, 0, 7, 7, 0, 15, 11, 8, 3, 12, 0, 0,
    0, 0, 9, 9, 0, 0, 13, 13, 14, 0, 0, 0, 0, 0, 0, 0, 0, 12, 0, 10, 0, 0, 11, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 13, 9, 13, 9, 9, 5, 7, 6, 5, 14, 14, 0, 14, 10, 10,
    10, 0, 0, 0, 0, 0, 0, 0, 4, 0, 0, 0, 0, 0, 0, 0, 0, 0, 3, 14, 10, 11, 10, 9,
    10, 0, 12, 6, 6, 6, 14, 14, 14, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 9, 9, 3 };

  static const signed char uv4[261] = { 12, 0, 0, 0, 13, 4, 13, 15, 13, 13, 4,
    16, 16, 7, 11, 5, 1, 2, 16, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 10, 13,
    11, 4, 11, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 16, 13, 13, 13, 13,
    13, 13, 16, 13, 13, 13, 13, 12, 13, 13, 12, 13, 16, 16, 16, 10, 1, 3, 10, 13,
    10, 13, 10, 13, 14, 13, 13, 17, 13, 13, 10, 10, 10, 14, 14, 10, 10, 12, 10,
    10, 10, 10, 14, 10, 16, 16, 16, 3, 16, 17, 17, 17, 16, 16, 16, 14, 14, 14,
    13, 13, 15, 14, 14, 14, 14, 13, 14, 14, 14, 13, 13, 14, 14, 14, 13, 13, 14,
    14, 14, 13, 0, 5, 13, 13, 16, 0, 16, 13, 8, 14, 0, 3, 2, 0, 12, 13, 0, 11, 0,
    0, 13, 13, 0, 0, 0, 0, 0, 7, 7, 0, 10, 10, 14, 13, 5, 0, 0, 0, 0, 7, 7, 0, 0,
    17, 16, 16, 0, 0, 0, 0, 0, 0, 0, 0, 11, 0, 17, 0, 0, 10, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 17, 17, 17, 16, 17, 17, 17, 16, 17, 17, 17, 0, 17, 17, 17, 17, 0, 0, 0,
    0, 0, 0, 0, 4, 0, 0, 0, 0, 0, 0, 0, 0, 0, 16, 13, 14, 16, 18, 12, 17, 0, 10,
    8, 8, 8, 13, 13, 13, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 2 };

  static const signed char uv8[261] = { 7, 0, 0, 0, 2, 4, 8, 6, 8, 8, 3, 4, 4, 6,
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

  static const signed char uv9[261] = { 8, 0, 0, 0, 9, 3, 9, 11, 9, 9, 3, 11, 11,
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

  static const char cv[33] = "                                ";
  static const char cv2[26] = { 'v', 'i', '-', 'o', 'u', 't', 'p', 'u', 't', ',',
    ' ', 'i', 'm', 'x', '2', '1', '9', ' ', '6', '-', '0', '0', '1', '0', '\x00',
    '\x00' };

  static const signed char uv1[16] = { 78, 111, 116, 104, 105, 110, 103, 32, 68,
    101, 116, 101, 99, 116, 101, 100 };

  static const signed char dv[8] = { 10, 50, 19, 40, 11, 95, 27, 48 };

  static const char cv3[3] = { 'c', 'a', 'r' };

  static const unsigned char uv2[3] = { MAX_uint8_T, 0U, 0U };

  static const unsigned char uv5[3] = { MAX_uint8_T, MAX_uint8_T, 0U };

  static unsigned char pln0[921600];
  static unsigned char pln1[921600];
  static unsigned char pln2[921600];
  static unsigned char out[150528];
  static unsigned char tmpRGB[150528];
  static unsigned char varargin_1[50176];
  static unsigned char varargin_2[50176];
  static unsigned char varargin_3[50176];
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
  dim3 xb_block;
  dim3 xb_grid;
  dim3 y_block;
  dim3 y_grid;
  dim3 yb_block;
  dim3 yb_grid;
  emxArray_cell_wrap_7_1024 labelCells;
  emxArray_cell_wrap_7_1024 *gpu_labelCells;
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
  double *gpu_validSampleTime;
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
  float ex;
  int position[4];
  int positionOut[4];
  int (*gpu_position)[4];
  int (*gpu_positionOut)[4];
  int b_b_size[2];
  int b_bboxPred_size[2];
  int b_size[2];
  int b_uv7_size[2];
  int b_x_size[2];
  int bboxPred_size[2];
  int bboxesX1Y1X2Y2_size[2];
  int c_b_size[2];
  int c_bboxPred_size[2];
  int d_bboxPred_size[2];
  int inDims[2];
  int inputBbox_size[2];
  int thresholdedPrediction_size[2];
  int uv11_size[2];
  int uv7_size[2];
  int x_size[2];
  int (*b_gpu_b_size)[2];
  int (*b_gpu_bboxPred_size)[2];
  int (*b_gpu_uv7_size)[2];
  int (*b_gpu_x_size)[2];
  int (*c_gpu_b_size)[2];
  int (*c_gpu_bboxPred_size)[2];
  int (*d_gpu_bboxPred_size)[2];
  int (*gpu_b_size)[2];
  int (*gpu_bboxPred_size)[2];
  int (*gpu_bboxesX1Y1X2Y2_size)[2];
  unsigned int (*gpu_castRed)[2];
  int (*gpu_inputBbox_size)[2];
  int (*gpu_thresholdedPrediction_size)[2];
  int (*gpu_uv11_size)[2];
  int (*gpu_uv7_size)[2];
  int (*gpu_x_size)[2];
  int idx_size[1];
  int iv_size[1];
  int scorePred_size[1];
  int selectedIndex_size[1];
  int v_size[1];
  int x2_size[1];
  int y1_size[1];
  int y2_size[1];
  int (*gpu_idx_size)[1];
  int (*gpu_iv_size)[1];
  int (*gpu_scorePred_size)[1];
  int (*gpu_selectedIndex_size)[1];
  int (*gpu_v_size)[1];
  int camIndex;
  int cameraDevice;
  int count;
  int endR;
  int i;
  int initStatus;
  int penX;
  int qY;
  int startC_gl;
  int *gpu_camIndex;
  int *gpu_cameraDevice;
  int *gpu_i;
  int *gpu_oldIdx;
  short (*gpu_ipColIndices)[5152];
  short (*gpu_ipRowIndices)[2912];
  short (*gpu_aux2)[2560];
  short (*gpu_aux1)[1440];
  short iv9_data[1024];
  short iv_data[1024];
  short (*gpu_iv9_data)[1024];
  short (*gpu_iv_data)[1024];
  short (*b_gpu_ipColIndices)[896];
  short (*b_gpu_ipRowIndices)[896];
  short (*b_gpu_aux1)[448];
  short (*b_gpu_aux2)[448];
  unsigned short (*gpu_uv)[256];
  short dv2[2];
  short (*gpu_dv2)[2];
  unsigned char (*gpu_img)[2764800];
  unsigned char (*gpu_pln0)[921600];
  unsigned char (*gpu_pln1)[921600];
  unsigned char (*gpu_pln2)[921600];
  unsigned char (*gpu_partialResize)[483840];
  unsigned char (*b_gpu_out)[150528];
  unsigned char (*gpu_tmpRGB)[150528];
  unsigned char (*b_gpu_partialResize)[86016];
  unsigned char (*gpu_varargin_1)[50176];
  unsigned char (*gpu_varargin_2)[50176];
  unsigned char (*gpu_varargin_3)[50176];
  unsigned char (*gpu_out)[49152];
  unsigned char (*gpu_uv7)[22591];
  unsigned char (*gpu_uv7_data)[22591];
  unsigned char (*gpu_uv11)[10664];
  unsigned char (*gpu_uv11_data)[10664];
  unsigned char thisGlyphBitmap_data[324];
  unsigned char (*b_gpu_uv7_data)[324];
  unsigned char (*gpu_thisGlyphBitmap_data)[324];
  signed char (*gpu_iv5)[261];
  unsigned char pixCount[224];
  unsigned char (*gpu_pixCount)[224];
  unsigned char b_thisGlyphBitmap_data[144];
  unsigned char (*b_gpu_thisGlyphBitmap_data)[144];
  unsigned char (*b_gpu_uv11_data)[144];
  char (*gpu_cv)[33];
  char cv1[26];
  char (*gpu_cv1)[26];
  char (*gpu_cv2)[26];
  signed char (*gpu_dv)[8];
  unsigned char color[3];
  signed char thisTextU16_data[3];
  unsigned char (*gpu_color)[3];
  char (*gpu_cv3)[3];
  signed char (*gpu_thisCharcodes_1b_data)[3];
  signed char (*gpu_thisTextU16_data)[3];
  unsigned char (*gpu_uv5)[3];
  char (*gpu_v_data)[3];
  signed char (*gpu_x_data)[3];
  signed char num[2];
  unsigned char outVal[2];
  signed char (*gpu_num)[2];
  unsigned char (*gpu_outVal)[2];
  bool b_data[1024];
  bool index_data[1024];
  bool (*gpu_b_data)[1024];
  bool (*gpu_index_data)[1024];
  bool (*b_gpu_x_data)[3];
  bool b_bboxPred_data_dirtyOnGpu;
  bool b_data_dirtyOnCpu;
  bool b_data_dirtyOnGpu;
  bool b_thisGlyphBitmap_data_dirtyOnGpu;
  bool bboxPred_data_dirtyOnCpu;
  bool bboxPred_data_dirtyOnGpu;
  bool bboxesX1Y1X2Y2_data_dirtyOnGpu;
  bool bboxesX1Y1X2Y2_size_dirtyOnCpu;
  bool camIndex_dirtyOnGpu;
  bool classPred_data_dirtyOnCpu;
  bool classPred_data_dirtyOnGpu;
  bool cv3_dirtyOnCpu;
  bool guard1 = false;
  bool index_data_dirtyOnGpu;
  bool isInitialise;
  bool iv5_dirtyOnCpu;
  bool iv9_data_dirtyOnCpu;
  bool iv_data_dirtyOnGpu;
  bool out_dirtyOnCpu;
  bool out_dirtyOnGpu;
  bool scorePred_data_dirtyOnCpu;
  bool scorePred_data_dirtyOnGpu;
  bool scores_data_dirtyOnGpu;
  bool thisGlyphBitmap_data_dirtyOnGpu;
  bool thisTextU16_data_dirtyOnGpu;
  bool thresholdedPrediction_size_dirtyOnCpu;
  bool uv11_dirtyOnCpu;
  bool uv7_dirtyOnCpu;
  bool uv_dirtyOnCpu;
  bool validLaunchParams;
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
  cudaMalloc(&b_gpu_thisGlyphBitmap_data, 144UL);
  cudaMalloc(&b_gpu_uv11_data, 144UL);
  cudaMalloc(&gpu_uv11_size, 8UL);
  cudaMalloc(&gpu_uv11_data, 10664UL);
  cudaMalloc(&gpu_uv11, 10664UL);
  cudaMalloc(&b_gpu_x_data, 3UL);
  cudaMalloc(&b_gpu_x_size, 8UL);
  cudaMalloc(&gpu_x_data, 3UL);
  cudaMalloc(&gpu_iv5, 261UL);
  cudaMalloc(&gpu_thisCharcodes_1b_data, 3UL);
  cudaMalloc(&gpu_x_size, 8UL);
  cudaMalloc(&gpu_thisTextU16_data, 3UL);
  cudaMalloc(&gpu_pixCount, 224UL);
  cudaMalloc(&gpu_tmpRGB, 150528UL);
  cudaMalloc(&gpu_color, 3UL);
  cudaMalloc(&gpu_positionOut, 16UL);
  cudaMalloc(&gpu_position, 16UL);
  cudaMalloc(&gpu_uv5, 3UL);
  cudaMalloc(&gpu_thisGlyphBitmap_data, 324UL);
  cudaMalloc(&b_gpu_uv7_data, 324UL);
  cudaMalloc(&gpu_uv7_size, 8UL);
  cudaMalloc(&gpu_num, 2UL);
  cudaMalloc(&gpu_uv7_data, 22591UL);
  cudaMalloc(&gpu_uv7, 22591UL);
  cudaMalloc(&b_gpu_uv7_size, 8UL);
  cudaMalloc(&gpu_uv, 512UL);
  cudaMalloc(&gpu_i, 4UL);
  cudaMalloc(&gpu_labelCells, 12292UL);
  cudaMalloc(&gpu_v_data, 3UL);
  cudaMalloc(&gpu_cv3, 3UL);
  cudaMalloc(&gpu_v_size, 4UL);
  cudaMalloc(&gpu_oldIdx, 4UL);
  cudaMalloc(&gpu_scores_data, 4096UL);
  cudaMalloc(&d_gpu_bboxPred_size, 8UL);
  cudaMalloc(&gpu_iv9_data, 2048UL);
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
  cudaMalloc(&gpu_validSampleTime, 8UL);
  cudaMalloc(&gpu_camIndex, 4UL);
  cudaMalloc(&gpu_cameraDevice, 4UL);
  cudaMalloc(&gpu_cv1, 26UL);
  cudaMalloc(&gpu_cv2, 26UL);
  cudaMalloc(&gpu_cv, 33UL);
  iv9_data_dirtyOnCpu = false;
  scorePred_data_dirtyOnCpu = false;
  bboxPred_data_dirtyOnCpu = false;
  isInitialise = false;
  classPred_data_dirtyOnCpu = false;
  b_thisGlyphBitmap_data_dirtyOnGpu = false;
  thisTextU16_data_dirtyOnGpu = false;
  thisGlyphBitmap_data_dirtyOnGpu = false;
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
  uv11_dirtyOnCpu = true;
  uv_dirtyOnCpu = true;
  iv5_dirtyOnCpu = true;
  uv7_dirtyOnCpu = true;
  cv3_dirtyOnCpu = true;
  if ((!mynet_not_empty) || (!hwobj_not_empty) || (!cam_not_empty) ||
      (!display_not_empty)) {
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
    display.isInitialized = 0;
    display.matlabCodegenIsDeleted = false;
    display_not_empty = true;
    coder::DeepLearningNetwork_setup(&gobj_3);
    mynet.Network = &gobj_3;
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
  for (endR = 0; endR < 12; endR++) {
    detectFunction_kernel10<<<dim3(1U, 1U, 1U), dim3(224U, 1U, 1U)>>>
      (*gpu_rowWeights, (endR + 1) * 224, *gpu_rowWeightsTotal);
  }

  detectFunction_kernel11<<<dim3(1U, 1U, 1U), dim3(224U, 1U, 1U)>>>
    (*gpu_colWeights, *gpu_colWeightsTotal);
  for (endR = 0; endR < 22; endR++) {
    detectFunction_kernel12<<<dim3(1U, 1U, 1U), dim3(224U, 1U, 1U)>>>
      (*gpu_colWeights, (endR + 1) * 224, *gpu_colWeightsTotal);
  }

  detectFunction_kernel13<<<dim3(945U, 1U, 1U), dim3(512U, 1U, 1U)>>>
    (*gpu_colWeightsTotal, *gpu_colWeights, *gpu_img, *gpu_ipColIndices,
     *gpu_partialResize);
  detectFunction_kernel14<<<dim3(294U, 1U, 1U), dim3(512U, 1U, 1U)>>>
    (*gpu_rowWeightsTotal, *gpu_rowWeights, *gpu_partialResize,
     *gpu_ipRowIndices, *b_gpu_out);
  out_dirtyOnCpu = false;
  out_dirtyOnGpu = true;
  detectFunction_kernel15<<<dim3(1U, 1U, 1U), dim3(448U, 1U, 1U)>>>(*b_gpu_aux2,
    *b_gpu_aux1);
  detectFunction_kernel16<<<dim3(2U, 1U, 1U), dim3(512U, 1U, 1U)>>>(*b_gpu_aux1,
    *b_gpu_ipRowIndices, *b_gpu_rowWeights);
  detectFunction_kernel17<<<dim3(2U, 1U, 1U), dim3(512U, 1U, 1U)>>>(*b_gpu_aux2,
    *b_gpu_ipColIndices, *b_gpu_colWeights);
  detectFunction_kernel18<<<dim3(1U, 1U, 1U), dim3(128U, 1U, 1U)>>>
    (*b_gpu_rowWeights, *b_gpu_rowWeightsTotal);
  for (endR = 0; endR < 6; endR++) {
    detectFunction_kernel19<<<dim3(1U, 1U, 1U), dim3(128U, 1U, 1U)>>>
      (*b_gpu_rowWeights, (endR + 1) << 7, *b_gpu_rowWeightsTotal);
  }

  detectFunction_kernel20<<<dim3(1U, 1U, 1U), dim3(128U, 1U, 1U)>>>
    (*b_gpu_colWeights, *b_gpu_colWeightsTotal);
  for (endR = 0; endR < 6; endR++) {
    detectFunction_kernel21<<<dim3(1U, 1U, 1U), dim3(128U, 1U, 1U)>>>
      (*b_gpu_colWeights, (endR + 1) << 7, *b_gpu_colWeightsTotal);
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
  cudaMemcpy(&b_out[0], c_gpu_out, 196608UL, cudaMemcpyDeviceToHost);
  coder::DeepLearningNetwork_activations(mynet.Network, b_out, tmpFeatureMap);
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
      isInitialise = true;
      camIndex++;
    }
  }

  thresholdedPrediction_size[0] = initStatus;
  thresholdedPrediction_size[1] = 6;
  thresholdedPrediction_size_dirtyOnCpu = true;
  validLaunchParams = mwGetLaunchParameters(static_cast<double>(((iv_size[0] - 1)
    + 1L) * 6L), &grid, &block, 1024U, 65535U);
  if (validLaunchParams) {
    if (isInitialise) {
      cudaMemcpy(gpu_iv_data, &iv_data[0], 2048UL, cudaMemcpyHostToDevice);
      isInitialise = false;
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
    short b_i;
    bool bboxPred_size_dirtyOnCpu;
    qY = iv_size[0] - 1;
    bboxesX1Y1X2Y2_size[0] = iv_size[0];
    bboxesX1Y1X2Y2_size[1] = 4;
    bboxesX1Y1X2Y2_size_dirtyOnCpu = true;
    validLaunchParams = mwGetLaunchParameters(static_cast<double>((qY + 1L) * 4L),
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
      detectFunction_kernel33<<<b_grid, b_block>>>
        (*gpu_thresholdedPrediction_data, *gpu_thresholdedPrediction_size,
         *gpu_bboxesX1Y1X2Y2_size, qY, *gpu_bboxesX1Y1X2Y2_data);
      bboxesX1Y1X2Y2_data_dirtyOnGpu = true;
    }

    qY = bboxesX1Y1X2Y2_size[0] - 1;
    iv_size[0] = bboxesX1Y1X2Y2_size[0];
    validLaunchParams = mwGetLaunchParameters(static_cast<double>(qY + 1L),
      &c_grid, &c_block, 1024U, 65535U);
    if (validLaunchParams) {
      detectFunction_kernel34<<<c_grid, c_block>>>(*gpu_bboxesX1Y1X2Y2_data, qY,
        *gpu_x1_data);
      x1_data_dirtyOnGpu = true;
    }

    qY = bboxesX1Y1X2Y2_size[0] - 1;
    y1_size[0] = bboxesX1Y1X2Y2_size[0];
    validLaunchParams = mwGetLaunchParameters(static_cast<double>(qY + 1L),
      &d_grid, &d_block, 1024U, 65535U);
    if (validLaunchParams) {
      if (bboxesX1Y1X2Y2_size_dirtyOnCpu) {
        cudaMemcpy(gpu_bboxesX1Y1X2Y2_size, &bboxesX1Y1X2Y2_size[0], 8UL,
                   cudaMemcpyHostToDevice);
        bboxesX1Y1X2Y2_size_dirtyOnCpu = false;
      }

      detectFunction_kernel35<<<d_grid, d_block>>>(*gpu_bboxesX1Y1X2Y2_data,
        *gpu_bboxesX1Y1X2Y2_size, qY, *gpu_y1_data);
      y1_data_dirtyOnGpu = true;
    }

    qY = bboxesX1Y1X2Y2_size[0] - 1;
    x2_size[0] = bboxesX1Y1X2Y2_size[0];
    validLaunchParams = mwGetLaunchParameters(static_cast<double>(qY + 1L),
      &e_grid, &e_block, 1024U, 65535U);
    if (validLaunchParams) {
      if (bboxesX1Y1X2Y2_size_dirtyOnCpu) {
        cudaMemcpy(gpu_bboxesX1Y1X2Y2_size, &bboxesX1Y1X2Y2_size[0], 8UL,
                   cudaMemcpyHostToDevice);
        bboxesX1Y1X2Y2_size_dirtyOnCpu = false;
      }

      detectFunction_kernel36<<<e_grid, e_block>>>(*gpu_bboxesX1Y1X2Y2_data,
        *gpu_bboxesX1Y1X2Y2_size, qY, *gpu_x2_data);
      x2_data_dirtyOnGpu = true;
    }

    qY = bboxesX1Y1X2Y2_size[0] - 1;
    y2_size[0] = bboxesX1Y1X2Y2_size[0];
    validLaunchParams = mwGetLaunchParameters(static_cast<double>(qY + 1L),
      &f_grid, &f_block, 1024U, 65535U);
    if (validLaunchParams) {
      if (bboxesX1Y1X2Y2_size_dirtyOnCpu) {
        cudaMemcpy(gpu_bboxesX1Y1X2Y2_size, &bboxesX1Y1X2Y2_size[0], 8UL,
                   cudaMemcpyHostToDevice);
      }

      detectFunction_kernel37<<<f_grid, f_block>>>(*gpu_bboxesX1Y1X2Y2_data,
        *gpu_bboxesX1Y1X2Y2_size, qY, *gpu_y2_data);
      y2_data_dirtyOnGpu = true;
    }

    cameraDevice = bboxesX1Y1X2Y2_size[0];
    validLaunchParams = mwGetLaunchParameters(static_cast<double>((cameraDevice
      - 1) + 1L), &g_grid, &g_block, 1024U, 65535U);
    if (validLaunchParams) {
      cudaMemcpy(gpu_cameraDevice, &cameraDevice, 4UL, cudaMemcpyHostToDevice);
      detectFunction_kernel38<<<g_grid, g_block>>>(gpu_cameraDevice,
        *gpu_x1_data);
      x1_data_dirtyOnGpu = true;
    }

    cameraDevice = bboxesX1Y1X2Y2_size[0];
    validLaunchParams = mwGetLaunchParameters(static_cast<double>((cameraDevice
      - 1) + 1L), &h_grid, &h_block, 1024U, 65535U);
    if (validLaunchParams) {
      cudaMemcpy(gpu_cameraDevice, &cameraDevice, 4UL, cudaMemcpyHostToDevice);
      detectFunction_kernel39<<<h_grid, h_block>>>(gpu_cameraDevice,
        *gpu_y1_data);
      y1_data_dirtyOnGpu = true;
    }

    cameraDevice = bboxesX1Y1X2Y2_size[0];
    validLaunchParams = mwGetLaunchParameters(static_cast<double>((cameraDevice
      - 1) + 1L), &i_grid, &i_block, 1024U, 65535U);
    if (validLaunchParams) {
      cudaMemcpy(gpu_cameraDevice, &cameraDevice, 4UL, cudaMemcpyHostToDevice);
      detectFunction_kernel40<<<i_grid, i_block>>>(gpu_cameraDevice,
        *gpu_x2_data);
      x2_data_dirtyOnGpu = true;
    }

    cameraDevice = bboxesX1Y1X2Y2_size[0];
    validLaunchParams = mwGetLaunchParameters(static_cast<double>((cameraDevice
      - 1) + 1L), &j_grid, &j_block, 1024U, 65535U);
    if (validLaunchParams) {
      cudaMemcpy(gpu_cameraDevice, &cameraDevice, 4UL, cudaMemcpyHostToDevice);
      detectFunction_kernel41<<<j_grid, j_block>>>(gpu_cameraDevice,
        *gpu_y2_data);
      y2_data_dirtyOnGpu = true;
    }

    bboxesX1Y1X2Y2_size[0] = iv_size[0];
    bboxesX1Y1X2Y2_size[1] = 4;
    bboxesX1Y1X2Y2_size_dirtyOnCpu = true;
    validLaunchParams = mwGetLaunchParameters(static_cast<double>((iv_size[0] -
      1) + 1L), &k_grid, &k_block, 1024U, 65535U);
    if (validLaunchParams) {
      detectFunction_kernel42<<<k_grid, k_block>>>(*gpu_x1_data, iv_size[0] - 1,
        *gpu_bboxesX1Y1X2Y2_data);
      bboxesX1Y1X2Y2_data_dirtyOnGpu = true;
    }

    validLaunchParams = mwGetLaunchParameters(static_cast<double>((y1_size[0] -
      1) + 1L), &l_grid, &l_block, 1024U, 65535U);
    if (validLaunchParams) {
      cudaMemcpy(gpu_bboxesX1Y1X2Y2_size, &bboxesX1Y1X2Y2_size[0], 8UL,
                 cudaMemcpyHostToDevice);
      bboxesX1Y1X2Y2_size_dirtyOnCpu = false;
      detectFunction_kernel43<<<l_grid, l_block>>>(*gpu_y1_data,
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

      detectFunction_kernel44<<<m_grid, m_block>>>(*gpu_x2_data,
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

      detectFunction_kernel45<<<n_grid, n_block>>>(*gpu_y2_data,
        *gpu_bboxesX1Y1X2Y2_size, y2_size[0] - 1, *gpu_bboxesX1Y1X2Y2_data);
      bboxesX1Y1X2Y2_data_dirtyOnGpu = true;
    }

    qY = bboxesX1Y1X2Y2_size[0];
    camIndex = bboxesX1Y1X2Y2_size[0];
    cameraDevice = bboxesX1Y1X2Y2_size[0];
    b_bboxPred_size[0] = bboxesX1Y1X2Y2_size[0];
    b_bboxPred_size[1] = 4;
    bboxPred_size_dirtyOnCpu = true;
    validLaunchParams = mwGetLaunchParameters(static_cast<double>
      ((bboxesX1Y1X2Y2_size[0] - 1) + 1L), &o_grid, &o_block, 1024U, 65535U);
    if (validLaunchParams) {
      detectFunction_kernel46<<<o_grid, o_block>>>(*gpu_bboxesX1Y1X2Y2_data,
        bboxesX1Y1X2Y2_size[0] - 1, *gpu_bboxPred_data);
      bboxPred_data_dirtyOnGpu = true;
    }

    validLaunchParams = mwGetLaunchParameters(static_cast<double>((qY - 1) + 1L),
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
      detectFunction_kernel47<<<p_grid, p_block>>>(*gpu_bboxesX1Y1X2Y2_data,
        *gpu_bboxesX1Y1X2Y2_size, *gpu_bboxPred_size, qY - 1, *gpu_bboxPred_data);
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

      detectFunction_kernel48<<<q_grid, q_block>>>(*gpu_bboxesX1Y1X2Y2_data,
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

      detectFunction_kernel49<<<r_grid, r_block>>>(*gpu_bboxesX1Y1X2Y2_data,
        *gpu_bboxesX1Y1X2Y2_size, *gpu_bboxPred_size, cameraDevice - 1,
        *gpu_bboxPred_data);
      bboxPred_data_dirtyOnGpu = true;
    }

    initStatus = b_bboxPred_size[0] << 2;
    validLaunchParams = mwGetLaunchParameters(static_cast<double>((initStatus -
      1) + 1L), &s_grid, &s_block, 1024U, 65535U);
    if (validLaunchParams) {
      detectFunction_kernel50<<<s_grid, s_block>>>(initStatus,
        *gpu_bboxPred_data);
      bboxPred_data_dirtyOnGpu = true;
    }

    qY = b_bboxPred_size[0] - 1;
    iv_size[0] = b_bboxPred_size[0];
    validLaunchParams = mwGetLaunchParameters(static_cast<double>(qY + 1L),
      &t_grid, &t_block, 1024U, 65535U);
    if (validLaunchParams) {
      if (bboxPred_size_dirtyOnCpu) {
        cudaMemcpy(gpu_bboxPred_size, &b_bboxPred_size[0], 8UL,
                   cudaMemcpyHostToDevice);
        bboxPred_size_dirtyOnCpu = false;
      }

      detectFunction_kernel51<<<t_grid, t_block>>>(*gpu_bboxPred_data,
        *gpu_bboxPred_size, qY, *gpu_x1_data);
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

      detectFunction_kernel52<<<u_grid, u_block>>>(*gpu_x1_data,
        *gpu_bboxPred_size, *gpu_iv_size, *gpu_bboxPred_data);
      bboxPred_data_dirtyOnGpu = true;
    }

    qY = b_bboxPred_size[0] - 1;
    iv_size[0] = b_bboxPred_size[0];
    validLaunchParams = mwGetLaunchParameters(static_cast<double>(qY + 1L),
      &v_grid, &v_block, 1024U, 65535U);
    if (validLaunchParams) {
      if (bboxPred_size_dirtyOnCpu) {
        cudaMemcpy(gpu_bboxPred_size, &b_bboxPred_size[0], 8UL,
                   cudaMemcpyHostToDevice);
        bboxPred_size_dirtyOnCpu = false;
      }

      detectFunction_kernel53<<<v_grid, v_block>>>(*gpu_bboxPred_data,
        *gpu_bboxPred_size, qY, *gpu_x1_data);
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

      detectFunction_kernel54<<<w_grid, w_block>>>(*gpu_x1_data,
        *gpu_bboxPred_size, *gpu_iv_size, *gpu_bboxPred_data);
      bboxPred_data_dirtyOnGpu = true;
    }

    count = 0;
    bboxPred_size[0] = b_bboxPred_size[0];
    bboxPred_size[1] = 4;
    bboxesX1Y1X2Y2_size_dirtyOnCpu = true;
    scorePred_size[0] = b_bboxPred_size[0];
    iv_size[0] = b_bboxPred_size[0];
    initStatus = b_bboxPred_size[0];
    for (i = 0; i < initStatus; i++) {
      if (bboxPred_data_dirtyOnGpu) {
        cudaMemcpy(&b_bboxPred_data[0], gpu_bboxPred_data, 32768UL,
                   cudaMemcpyDeviceToHost);
        bboxPred_data_dirtyOnGpu = false;
      }

      if ((b_bboxPred_data[i + b_bboxPred_size[0] * 3] >= 1.0) &&
          (b_bboxPred_data[i + (b_bboxPred_size[0] << 1)] >= 1.0) &&
          (b_bboxPred_data[i + b_bboxPred_size[0] * 3] <= 224.0) &&
          (b_bboxPred_data[i + (b_bboxPred_size[0] << 1)] <= 224.0)) {
        count++;
        camIndex = count - 1;
        if (bboxPred_size_dirtyOnCpu) {
          cudaMemcpy(gpu_bboxPred_size, &b_bboxPred_size[0], 8UL,
                     cudaMemcpyHostToDevice);
          bboxPred_size_dirtyOnCpu = false;
        }

        if (bboxesX1Y1X2Y2_size_dirtyOnCpu) {
          cudaMemcpy(b_gpu_bboxPred_size, &bboxPred_size[0], 8UL,
                     cudaMemcpyHostToDevice);
          bboxesX1Y1X2Y2_size_dirtyOnCpu = false;
        }

        cudaMemcpy(gpu_camIndex, &camIndex, 4UL, cudaMemcpyHostToDevice);
        detectFunction_kernel55<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
          (*gpu_bboxPred_data, *gpu_bboxPred_size, i, *b_gpu_bboxPred_size,
           gpu_camIndex, *b_gpu_bboxPred_data);
        b_bboxPred_data_dirtyOnGpu = true;
        if (thresholdedPrediction_size_dirtyOnCpu) {
          cudaMemcpy(gpu_thresholdedPrediction_size,
                     &thresholdedPrediction_size[0], 8UL, cudaMemcpyHostToDevice);
          thresholdedPrediction_size_dirtyOnCpu = false;
        }

        detectFunction_kernel56<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
          (*gpu_thresholdedPrediction_data, *gpu_thresholdedPrediction_size, i,
           count, *gpu_classPred_data, *gpu_scorePred_data);
        scorePred_data_dirtyOnGpu = true;
        classPred_data_dirtyOnGpu = true;
      }
    }

    b_i = static_cast<short>(bboxPred_size[0]);
    validLaunchParams = mwGetLaunchParameters(static_cast<double>((b_i -
      static_cast<short>(count + 1)) + 1L), &x_grid, &x_block, 1024U, 65535U);
    if (validLaunchParams) {
      if (isInitialise) {
        cudaMemcpy(gpu_iv_data, &iv_data[0], 2048UL, cudaMemcpyHostToDevice);
      }

      detectFunction_kernel57<<<x_grid, x_block>>>(static_cast<short>(count + 1),
        b_i, *gpu_iv_data);
      isInitialise = false;
      iv_data_dirtyOnGpu = true;
    }

    cameraDevice = bboxPred_size[0];
    if (static_cast<short>(bboxPred_size[0] - count) == 1) {
      initStatus = bboxPred_size[0] - 1;
      if (isInitialise) {
        cudaMemcpy(gpu_iv_data, &iv_data[0], 2048UL, cudaMemcpyHostToDevice);
        isInitialise = false;
      }

      if (bboxesX1Y1X2Y2_size_dirtyOnCpu) {
        cudaMemcpy(b_gpu_bboxPred_size, &bboxPred_size[0], 8UL,
                   cudaMemcpyHostToDevice);
        bboxesX1Y1X2Y2_size_dirtyOnCpu = false;
      }

      detectFunction_kernel60<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
        (*b_gpu_bboxPred_size, initStatus, *gpu_iv_data, *b_gpu_bboxPred_data);
    } else {
      b_size[0] = 1;
      b_size[1] = bboxPred_size[0];
      validLaunchParams = mwGetLaunchParameters(static_cast<double>
        ((bboxPred_size[0] - 1) + 1L), &y_grid, &y_block, 1024U, 65535U);
      if (validLaunchParams) {
        if (bboxesX1Y1X2Y2_size_dirtyOnCpu) {
          cudaMemcpy(b_gpu_bboxPred_size, &bboxPred_size[0], 8UL,
                     cudaMemcpyHostToDevice);
          bboxesX1Y1X2Y2_size_dirtyOnCpu = false;
        }

        detectFunction_kernel58<<<y_grid, y_block>>>(*b_gpu_bboxPred_size,
          *gpu_b_data);
        b_data_dirtyOnGpu = true;
      }

      initStatus = static_cast<short>(bboxPred_size[0] - count);
      for (endR = 0; endR < initStatus; endR++) {
        if (b_data_dirtyOnGpu) {
          cudaMemcpy(&b_data[0], gpu_b_data, 1024UL, cudaMemcpyDeviceToHost);
          b_data_dirtyOnGpu = false;
        }

        if (iv_data_dirtyOnGpu) {
          cudaMemcpy(&iv_data[0], gpu_iv_data, 2048UL, cudaMemcpyDeviceToHost);
          iv_data_dirtyOnGpu = false;
        }

        b_data[iv_data[endR] - 1] = true;
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
        detectFunction_kernel59<<<ab_grid, ab_block>>>(*gpu_b_data, initStatus,
          gpu_camIndex);
        camIndex_dirtyOnGpu = true;
      }

      if (camIndex_dirtyOnGpu) {
        cudaMemcpy(&camIndex, gpu_camIndex, 4UL, cudaMemcpyDeviceToHost);
        camIndex_dirtyOnGpu = false;
      }

      initStatus = bboxPred_size[0] - camIndex;
      i = 0;
      for (endR = 0; endR < cameraDevice; endR++) {
        guard1 = false;
        if (endR + 1 > b_size[1]) {
          guard1 = true;
        } else {
          if (b_data_dirtyOnGpu) {
            cudaMemcpy(&b_data[0], gpu_b_data, 1024UL, cudaMemcpyDeviceToHost);
            b_data_dirtyOnGpu = false;
          }

          if (!b_data[endR]) {
            guard1 = true;
          }
        }

        if (guard1) {
          for (startC_gl = 0; startC_gl < 4; startC_gl++) {
            if (b_bboxPred_data_dirtyOnGpu) {
              cudaMemcpy(&bboxPred_data[0], b_gpu_bboxPred_data, 32768UL,
                         cudaMemcpyDeviceToHost);
              b_bboxPred_data_dirtyOnGpu = false;
            }

            bboxPred_data[i + bboxPred_size[0] * startC_gl] = bboxPred_data[endR
              + bboxPred_size[0] * startC_gl];
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
      if (bboxesX1Y1X2Y2_size_dirtyOnCpu) {
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
      detectFunction_kernel61<<<bb_grid, bb_block>>>(*b_gpu_bboxPred_data,
        *b_gpu_bboxPred_size, *c_gpu_bboxPred_size, initStatus,
        *gpu_bboxPred_data);
    }

    bboxPred_size[0] = c_bboxPred_size[0];
    bboxPred_size[1] = 4;
    bboxesX1Y1X2Y2_size_dirtyOnCpu = true;
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

      detectFunction_kernel62<<<cb_grid, cb_block>>>(*gpu_bboxPred_data,
        *c_gpu_bboxPred_size, *b_gpu_bboxPred_data);
    }

    b_i = static_cast<short>(scorePred_size[0]);
    validLaunchParams = mwGetLaunchParameters(static_cast<double>((b_i -
      static_cast<short>(count + 1)) + 1L), &db_grid, &db_block, 1024U, 65535U);
    if (validLaunchParams) {
      if (isInitialise) {
        cudaMemcpy(gpu_iv_data, &iv_data[0], 2048UL, cudaMemcpyHostToDevice);
        isInitialise = false;
      }

      detectFunction_kernel63<<<db_grid, db_block>>>(static_cast<short>(count +
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
      detectFunction_kernel64<<<eb_grid, eb_block>>>(*gpu_scorePred_size,
        *gpu_b_data);
      b_data_dirtyOnGpu = true;
    }

    initStatus = static_cast<short>(scorePred_size[0] - count);
    for (endR = 0; endR < initStatus; endR++) {
      if (b_data_dirtyOnGpu) {
        cudaMemcpy(&b_data[0], gpu_b_data, 1024UL, cudaMemcpyDeviceToHost);
        b_data_dirtyOnGpu = false;
      }

      if (iv_data_dirtyOnGpu) {
        cudaMemcpy(&iv_data[0], gpu_iv_data, 2048UL, cudaMemcpyDeviceToHost);
        iv_data_dirtyOnGpu = false;
      }

      b_data[iv_data[endR] - 1] = true;
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
      detectFunction_kernel65<<<fb_grid, fb_block>>>(*gpu_b_data, initStatus,
        gpu_camIndex);
      cudaMemcpy(&camIndex, gpu_camIndex, 4UL, cudaMemcpyDeviceToHost);
      camIndex_dirtyOnGpu = false;
    }

    initStatus = scorePred_size[0] - camIndex;
    camIndex = -1;
    for (endR = 0; endR < cameraDevice; endR++) {
      guard1 = false;
      if (endR + 1 > b_b_size[1]) {
        guard1 = true;
      } else {
        if (b_data_dirtyOnGpu) {
          cudaMemcpy(&b_data[0], gpu_b_data, 1024UL, cudaMemcpyDeviceToHost);
          b_data_dirtyOnGpu = false;
        }

        if (!b_data[endR]) {
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

        scorePred_data[camIndex] = scorePred_data[endR];
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
      if (isInitialise) {
        cudaMemcpy(gpu_iv_data, &iv_data[0], 2048UL, cudaMemcpyHostToDevice);
      }

      detectFunction_kernel66<<<gb_grid, gb_block>>>(static_cast<short>(count +
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
      detectFunction_kernel67<<<hb_grid, hb_block>>>(*gpu_iv_size, *gpu_b_data);
      b_data_dirtyOnGpu = true;
    }

    initStatus = static_cast<short>(iv_size[0] - count);
    for (endR = 0; endR < initStatus; endR++) {
      if (b_data_dirtyOnGpu) {
        cudaMemcpy(&b_data[0], gpu_b_data, 1024UL, cudaMemcpyDeviceToHost);
        b_data_dirtyOnGpu = false;
      }

      if (iv_data_dirtyOnGpu) {
        cudaMemcpy(&iv_data[0], gpu_iv_data, 2048UL, cudaMemcpyDeviceToHost);
        iv_data_dirtyOnGpu = false;
      }

      b_data[iv_data[endR] - 1] = true;
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
      detectFunction_kernel68<<<ib_grid, ib_block>>>(*gpu_b_data, initStatus,
        gpu_camIndex);
      camIndex_dirtyOnGpu = true;
    }

    if (camIndex_dirtyOnGpu) {
      cudaMemcpy(&camIndex, gpu_camIndex, 4UL, cudaMemcpyDeviceToHost);
    }

    initStatus = iv_size[0] - camIndex;
    camIndex = -1;
    for (endR = 0; endR < cameraDevice; endR++) {
      guard1 = false;
      if (endR + 1 > c_b_size[1]) {
        guard1 = true;
      } else {
        if (b_data_dirtyOnGpu) {
          cudaMemcpy(&b_data[0], gpu_b_data, 1024UL, cudaMemcpyDeviceToHost);
          b_data_dirtyOnGpu = false;
        }

        if (!b_data[endR]) {
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

        classPred_data[camIndex] = classPred_data[endR];
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
        detectFunction_kernel83<<<jb_grid, jb_block>>>(*gpu_scorePred_data,
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
        detectFunction_kernel69<<<jb_grid, jb_block>>>(*gpu_classPred_data,
          *gpu_iv_size, *gpu_y1_data);
        y1_data_dirtyOnGpu = true;
      }

      idx_size[0] = scorePred_size[0];
      camIndex_dirtyOnGpu = true;
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
          detectFunction_kernel70<<<kb_grid, kb_block>>>(*gpu_scorePred_data,
            *gpu_scorePred_size, *gpu_classPred_data);
        }

        dv2[0] = static_cast<short>(scorePred_size[0]);
        idx_size[0] = static_cast<short>(scorePred_size[0]);
        validLaunchParams = mwGetLaunchParameters(static_cast<double>((dv2[0] -
          1) + 1L), &lb_grid, &lb_block, 1024U, 65535U);
        if (validLaunchParams) {
          cudaMemcpy(gpu_dv2, &dv2[0], 4UL, cudaMemcpyHostToDevice);
          detectFunction_kernel71<<<lb_grid, lb_block>>>(*gpu_dv2, *gpu_idx_data);
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
      classPred_data_dirtyOnGpu = true;
      validLaunchParams = mwGetLaunchParameters(static_cast<double>(((idx_size[0]
        - 1) + 1L) * 4L), &mb_grid, &mb_block, 1024U, 65535U);
      if (validLaunchParams) {
        cudaMemcpy(b_gpu_bboxPred_size, &bboxPred_size[0], 8UL,
                   cudaMemcpyHostToDevice);
        bboxesX1Y1X2Y2_size_dirtyOnCpu = false;
        if (bboxPred_data_dirtyOnCpu) {
          cudaMemcpy(b_gpu_bboxPred_data, &bboxPred_data[0], 32768UL,
                     cudaMemcpyHostToDevice);
          bboxPred_data_dirtyOnCpu = false;
        }

        cudaMemcpy(gpu_inputBbox_size, &inputBbox_size[0], 8UL,
                   cudaMemcpyHostToDevice);
        classPred_data_dirtyOnGpu = false;
        cudaMemcpy(gpu_idx_size, &idx_size[0], 4UL, cudaMemcpyHostToDevice);
        camIndex_dirtyOnGpu = false;
        detectFunction_kernel72<<<mb_grid, mb_block>>>(*b_gpu_bboxPred_data,
          *b_gpu_bboxPred_size, *gpu_idx_data, *gpu_inputBbox_size,
          *gpu_idx_size, *gpu_bboxesX1Y1X2Y2_data);
        bboxesX1Y1X2Y2_data_dirtyOnGpu = true;
      }

      iv_size[0] = idx_size[0];
      validLaunchParams = mwGetLaunchParameters(static_cast<double>((idx_size[0]
        - 1) + 1L), &nb_grid, &nb_block, 1024U, 65535U);
      if (validLaunchParams) {
        if (camIndex_dirtyOnGpu) {
          cudaMemcpy(gpu_idx_size, &idx_size[0], 4UL, cudaMemcpyHostToDevice);
          camIndex_dirtyOnGpu = false;
        }

        detectFunction_kernel73<<<nb_grid, nb_block>>>(*gpu_y1_data,
          *gpu_idx_data, *gpu_idx_size, *gpu_x1_data);
        x1_data_dirtyOnGpu = true;
      }

      validLaunchParams = mwGetLaunchParameters(static_cast<double>((iv_size[0]
        - 1) + 1L), &ob_grid, &ob_block, 1024U, 65535U);
      if (validLaunchParams) {
        cudaMemcpy(gpu_iv_size, &iv_size[0], 4UL, cudaMemcpyHostToDevice);
        detectFunction_kernel74<<<ob_grid, ob_block>>>(*gpu_x1_data,
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

        if (camIndex_dirtyOnGpu) {
          cudaMemcpy(gpu_idx_size, &idx_size[0], 4UL, cudaMemcpyHostToDevice);
        }

        detectFunction_kernel75<<<pb_grid, pb_block>>>(*gpu_idx_size,
          *gpu_b_data);
        b_data_dirtyOnGpu = true;
      }

      qY = idx_size[0] - 1;
      validLaunchParams = mwGetLaunchParameters(static_cast<double>(qY + 1L),
        &qb_grid, &qb_block, 1024U, 65535U);
      if (validLaunchParams) {
        if (classPred_data_dirtyOnGpu) {
          cudaMemcpy(gpu_inputBbox_size, &inputBbox_size[0], 8UL,
                     cudaMemcpyHostToDevice);
          classPred_data_dirtyOnGpu = false;
        }

        detectFunction_kernel76<<<qb_grid, qb_block>>>(*gpu_bboxesX1Y1X2Y2_data,
          *gpu_inputBbox_size, qY, *gpu_x1_data);
        x1_data_dirtyOnGpu = true;
      }

      qY = idx_size[0] - 1;
      validLaunchParams = mwGetLaunchParameters(static_cast<double>(qY + 1L),
        &rb_grid, &rb_block, 1024U, 65535U);
      if (validLaunchParams) {
        if (classPred_data_dirtyOnGpu) {
          cudaMemcpy(gpu_inputBbox_size, &inputBbox_size[0], 8UL,
                     cudaMemcpyHostToDevice);
          classPred_data_dirtyOnGpu = false;
        }

        detectFunction_kernel77<<<rb_grid, rb_block>>>(*gpu_inputBbox_size,
          *gpu_bboxesX1Y1X2Y2_data, qY, *gpu_x2_data);
        x2_data_dirtyOnGpu = true;
      }

      qY = idx_size[0] - 1;
      validLaunchParams = mwGetLaunchParameters(static_cast<double>(qY + 1L),
        &sb_grid, &sb_block, 1024U, 65535U);
      if (validLaunchParams) {
        if (classPred_data_dirtyOnGpu) {
          cudaMemcpy(gpu_inputBbox_size, &inputBbox_size[0], 8UL,
                     cudaMemcpyHostToDevice);
        }

        detectFunction_kernel78<<<sb_grid, sb_block>>>(*gpu_bboxesX1Y1X2Y2_data,
          *gpu_inputBbox_size, qY, *gpu_y2_data);
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
            for (startC_gl = 0; startC_gl <= cameraDevice; startC_gl++) {
              endR = (i + startC_gl) + 1;
              if (b_data[endR] && (!(y1_data[endR] != y1_data[i]))) {
                double maxval;
                double width;
                if (x2_data_dirtyOnGpu) {
                  cudaMemcpy(&x2_data[0], gpu_x2_data, 8192UL,
                             cudaMemcpyDeviceToHost);
                  x2_data_dirtyOnGpu = false;
                }

                if ((x2_data[i] < x2_data[endR]) || rtIsNaN(x2_data[endR])) {
                  validSampleTime = x2_data[i];
                } else {
                  validSampleTime = x2_data[endR];
                }

                if (bboxesX1Y1X2Y2_data_dirtyOnGpu) {
                  cudaMemcpy(&bboxesX1Y1X2Y2_data[0], gpu_bboxesX1Y1X2Y2_data,
                             32768UL, cudaMemcpyDeviceToHost);
                  bboxesX1Y1X2Y2_data_dirtyOnGpu = false;
                }

                if ((bboxesX1Y1X2Y2_data[i] > bboxesX1Y1X2Y2_data[endR]) ||
                    rtIsNaN(bboxesX1Y1X2Y2_data[endR])) {
                  maxval = bboxesX1Y1X2Y2_data[i];
                } else {
                  maxval = bboxesX1Y1X2Y2_data[endR];
                }

                width = validSampleTime - maxval;
                if (!(width <= 0.0)) {
                  if (y2_data_dirtyOnGpu) {
                    cudaMemcpy(&y2_data[0], gpu_y2_data, 8192UL,
                               cudaMemcpyDeviceToHost);
                    y2_data_dirtyOnGpu = false;
                  }

                  if ((y2_data[i] < y2_data[endR]) || rtIsNaN(y2_data[endR])) {
                    validSampleTime = y2_data[i];
                  } else {
                    validSampleTime = y2_data[endR];
                  }

                  if ((bboxesX1Y1X2Y2_data[i + inputBbox_size[0]] >
                       bboxesX1Y1X2Y2_data[endR + inputBbox_size[0]]) || rtIsNaN
                      (bboxesX1Y1X2Y2_data[endR + inputBbox_size[0]])) {
                    maxval = bboxesX1Y1X2Y2_data[i + inputBbox_size[0]];
                  } else {
                    maxval = bboxesX1Y1X2Y2_data[endR + inputBbox_size[0]];
                  }

                  validSampleTime -= maxval;
                  if (!(validSampleTime <= 0.0)) {
                    validSampleTime *= width;
                    if (x1_data_dirtyOnGpu) {
                      cudaMemcpy(&x1_data[0], gpu_x1_data, 8192UL,
                                 cudaMemcpyDeviceToHost);
                      x1_data_dirtyOnGpu = false;
                    }

                    if (validSampleTime / ((x1_data[i] + x1_data[endR]) -
                                           validSampleTime) > 0.5) {
                      b_data[endR] = false;
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
        detectFunction_kernel79<<<tb_grid, tb_block>>>(gpu_camIndex,
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
        detectFunction_kernel80<<<ub_grid, ub_block>>>(*gpu_b_data,
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
          iv9_data[camIndex] = static_cast<short>(i + 1);
          iv9_data_dirtyOnCpu = true;
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
        if (bboxesX1Y1X2Y2_size_dirtyOnCpu) {
          cudaMemcpy(b_gpu_bboxPred_size, &bboxPred_size[0], 8UL,
                     cudaMemcpyHostToDevice);
        }

        if (bboxPred_data_dirtyOnCpu) {
          cudaMemcpy(b_gpu_bboxPred_data, &bboxPred_data[0], 32768UL,
                     cudaMemcpyHostToDevice);
          bboxPred_data_dirtyOnCpu = false;
        }

        if (iv9_data_dirtyOnCpu) {
          cudaMemcpy(gpu_iv9_data, &iv9_data[0], 2048UL, cudaMemcpyHostToDevice);
        }

        cudaMemcpy(d_gpu_bboxPred_size, &d_bboxPred_size[0], 8UL,
                   cudaMemcpyHostToDevice);
        bboxPred_size_dirtyOnCpu = false;
        detectFunction_kernel81<<<vb_grid, vb_block>>>(*b_gpu_bboxPred_data,
          *b_gpu_bboxPred_size, *gpu_iv9_data, *d_gpu_bboxPred_size,
          *gpu_iv_size, *gpu_bboxPred_data);
      }

      bboxPred_size[0] = d_bboxPred_size[0];
      bboxPred_size[1] = 4;
      validLaunchParams = mwGetLaunchParameters(static_cast<double>
        ((d_bboxPred_size[0] * 4 - 1) + 1L), &wb_grid, &wb_block, 1024U, 65535U);
      if (validLaunchParams) {
        if (bboxPred_data_dirtyOnCpu) {
          cudaMemcpy(b_gpu_bboxPred_data, &bboxPred_data[0], 32768UL,
                     cudaMemcpyHostToDevice);
          bboxPred_data_dirtyOnCpu = false;
        }

        if (bboxPred_size_dirtyOnCpu) {
          cudaMemcpy(d_gpu_bboxPred_size, &d_bboxPred_size[0], 8UL,
                     cudaMemcpyHostToDevice);
        }

        detectFunction_kernel82<<<wb_grid, wb_block>>>(*gpu_bboxPred_data,
          *d_gpu_bboxPred_size, *b_gpu_bboxPred_data);
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

    labelCells.size[0] = bboxPred_size[0];
    bboxPred_data_dirtyOnGpu = false;
    b_bboxPred_data_dirtyOnGpu = true;
    initStatus = bboxPred_size[0];
    for (i = 0; i < initStatus; i++) {
      cameraDevice = 0;
      for (qY = 0; qY < 3; qY++) {
        cameraDevice = qY + 1;
      }

      cudaMemcpy(gpu_oldIdx, &cameraDevice, 4UL, cudaMemcpyHostToDevice);
      detectFunction_kernel84<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
        (gpu_oldIdx, *gpu_v_size);
      if (cv3_dirtyOnCpu) {
        cudaMemcpy(gpu_cv3, (void *)&cv3[0], 3UL, cudaMemcpyHostToDevice);
        cv3_dirtyOnCpu = false;
      }

      detectFunction_kernel85<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>(*gpu_cv3,
        0, *gpu_v_data);
      if (bboxPred_data_dirtyOnGpu) {
        cudaMemcpy(&labelCells, gpu_labelCells, 12292UL, cudaMemcpyDeviceToHost);
        bboxPred_data_dirtyOnGpu = false;
      }

      labelCells.data[i].f1.size[0] = 1;
      labelCells.data[i].f1.size[1] = cameraDevice;
      b_bboxPred_data_dirtyOnGpu = true;
      cudaMemcpy(&v_size[0], gpu_v_size, 4UL, cudaMemcpyDeviceToHost);
      validLaunchParams = mwGetLaunchParameters(static_cast<double>((v_size[0] -
        1) + 1L), &xb_grid, &xb_block, 1024U, 65535U);
      if (validLaunchParams) {
        cudaMemcpy(gpu_labelCells, &labelCells, 12292UL, cudaMemcpyHostToDevice);
        cudaMemcpy(gpu_i, &i, 4UL, cudaMemcpyHostToDevice);
        detectFunction_kernel86<<<xb_grid, xb_block>>>(*gpu_v_data, gpu_i,
          *gpu_v_size, gpu_labelCells);
        b_bboxPred_data_dirtyOnGpu = false;
        bboxPred_data_dirtyOnGpu = true;
      }
    }
  } else {
    bboxPred_size[0] = 0;
    bboxPred_size[1] = 4;
    iv_size[0] = 0;
    detectFunction_kernel32<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
      (gpu_labelCells);
    b_bboxPred_data_dirtyOnGpu = false;
    bboxPred_data_dirtyOnGpu = true;
  }

  //  first output will be discarded
  camIndex = 1;
  iv_data_dirtyOnGpu = true;
  initStatus = iv_size[0];
  if (scores_data_dirtyOnGpu) {
    cudaMemcpy(&scores_data[0], gpu_scores_data, 4096UL, cudaMemcpyDeviceToHost);
  }

  ex = scores_data[0];
  for (i = 0; i <= initStatus - 2; i++) {
    if (rtIsNaNF(scores_data[i + 1])) {
      isInitialise = false;
    } else if (rtIsNaNF(ex)) {
      isInitialise = true;
    } else {
      isInitialise = (ex < scores_data[i + 1]);
    }

    if (isInitialise) {
      ex = scores_data[i + 1];
      camIndex = i + 2;
    }
  }

  //  annotate detecttions in the image
  if (bboxPred_size[0] != 0) {
    void* colPtr;
    void* posPtr;
    void* ptrObj;
    int c;
    cudaMemcpy(b_gpu_bboxPred_size, &bboxPred_size[0], 8UL,
               cudaMemcpyHostToDevice);
    if (bboxPred_data_dirtyOnCpu) {
      cudaMemcpy(b_gpu_bboxPred_data, &bboxPred_data[0], 32768UL,
                 cudaMemcpyHostToDevice);
    }

    detectFunction_kernel90<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
      (*b_gpu_bboxPred_data, *b_gpu_bboxPred_size, camIndex - 1, *gpu_position);
    detectFunction_kernel91<<<dim3(294U, 1U, 1U), dim3(512U, 1U, 1U)>>>
      (*b_gpu_out, *gpu_tmpRGB);
    classPred_data_dirtyOnGpu = true;
    cudaMemcpy(gpu_uv5, (void *)&uv5[0], 3UL, cudaMemcpyHostToDevice);
    detectFunction_kernel92<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>(*gpu_uv5,
      *gpu_color);
    detectFunction_kernel93<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
      (*gpu_position, *gpu_positionOut);
    detectFunction_kernel94<<<dim3(294U, 1U, 1U), dim3(512U, 1U, 1U)>>>
      (*b_gpu_out);
    detectFunction_kernel95<<<dim3(1U, 1U, 1U), dim3(224U, 1U, 1U)>>>
      (*gpu_pixCount);
    camIndex_dirtyOnGpu = true;
    ptrObj = NULL;
    constructDrawBaseObjectShape(&ptrObj);
    posPtr = NULL;
    cudaMemcpy(&positionOut[0], gpu_positionOut, 16UL, cudaMemcpyDeviceToHost);
    getPositionDataPointer(&posPtr, &positionOut[0], 1U, 4U);
    colPtr = NULL;
    cudaMemcpy(&color[0], gpu_color, 3UL, cudaMemcpyDeviceToHost);
    getColorDataPointer_uint8(&colPtr, &color[0], 1U, 3U);
    for (i = 0; i < 2; i++) {
      isInitialise = initialiseDrawbaseShape(ptrObj, static_cast<short>(i), 1);
      if (!isInitialise) {
        if (out_dirtyOnGpu) {
          cudaMemcpy(&out[0], b_gpu_out, 150528UL, cudaMemcpyDeviceToHost);
          out_dirtyOnGpu = false;
        }

        if (classPred_data_dirtyOnGpu) {
          cudaMemcpy(&tmpRGB[0], gpu_tmpRGB, 150528UL, cudaMemcpyDeviceToHost);
          classPred_data_dirtyOnGpu = false;
        }

        if (camIndex_dirtyOnGpu) {
          cudaMemcpy(&pixCount[0], gpu_pixCount, 224UL, cudaMemcpyDeviceToHost);
          camIndex_dirtyOnGpu = false;
        }

        instantiateDrawBaseShape_uint8(ptrObj, &out[0], &tmpRGB[0], posPtr,
          colPtr, 0.6, 1, 1, true, 224, 224, 3, 2, 1, 4, 1, false, bv1[i],
          &pixCount[0], static_cast<short>(i));
        out_dirtyOnCpu = true;
      }
    }

    mDrawShapes(ptrObj, false, true, 1, 1, 224, 224);
    deallocateMemoryShape(ptrObj);
    deletePositionDataPointer(posPtr);
    deleteColorDataPointer_uint8(colPtr);
    cudaMemcpy(gpu_positionOut, &positionOut[0], 16UL, cudaMemcpyHostToDevice);
    detectFunction_kernel96<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
      (*gpu_position, *gpu_positionOut);
    cudaMemcpy(&position[0], gpu_position, 16UL, cudaMemcpyDeviceToHost);
    if (position[1] < -2147483647) {
      c = MIN_int32_T;
    } else {
      c = position[1] - 1;
    }

    detectFunction_kernel97<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>(c,
      *gpu_positionOut);
    cudaMemcpy(gpu_color, &color[0], 3UL, cudaMemcpyHostToDevice);
    detectFunction_kernel98<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>(*gpu_uv5,
      *gpu_color);
    bboxesX1Y1X2Y2_size_dirtyOnCpu = true;
    uv7_size[0] = 1;
    if (bboxPred_data_dirtyOnGpu) {
      cudaMemcpy(&labelCells, gpu_labelCells, 12292UL, cudaMemcpyDeviceToHost);
    }

    uv7_size[1] = labelCells.data[camIndex - 1].f1.size[1];
    validLaunchParams = mwGetLaunchParameters(static_cast<double>
      ((labelCells.data[camIndex - 1].f1.size[1] - 1) + 1L), &yb_grid, &yb_block,
      1024U, 65535U);
    if (validLaunchParams) {
      if (b_bboxPred_data_dirtyOnGpu) {
        cudaMemcpy(gpu_labelCells, &labelCells, 12292UL, cudaMemcpyHostToDevice);
      }

      cudaMemcpy(gpu_camIndex, &camIndex, 4UL, cudaMemcpyHostToDevice);
      iv_data_dirtyOnGpu = false;
      detectFunction_kernel99<<<yb_grid, yb_block>>>(gpu_labelCells,
        gpu_camIndex, *gpu_thisTextU16_data);
      thisTextU16_data_dirtyOnGpu = true;
    }

    if (uv7_size[1] != 0) {
      int b_c;
      int b_r;
      int endC_gl;
      int i22;
      int r;
      unsigned char tmp11;
      b_uv7_size[0] = 1;
      b_uv7_size[1] = uv7_size[1];
      classPred_data_dirtyOnGpu = true;
      validLaunchParams = mwGetLaunchParameters(static_cast<double>((uv7_size[1]
        - 1) + 1L), &ac_grid, &ac_block, 1024U, 65535U);
      if (validLaunchParams) {
        cudaMemcpy(b_gpu_uv7_size, &uv7_size[0], 8UL, cudaMemcpyHostToDevice);
        detectFunction_kernel100<<<ac_grid, ac_block>>>(*gpu_thisTextU16_data,
          *b_gpu_uv7_size, *gpu_thisCharcodes_1b_data);
      }

      x_size[0] = 1;
      x_size[1] = b_uv7_size[1];
      validLaunchParams = mwGetLaunchParameters(static_cast<double>((b_uv7_size
        [1] - 1) + 1L), &bc_grid, &bc_block, 1024U, 65535U);
      if (validLaunchParams) {
        cudaMemcpy(gpu_uv7_size, &b_uv7_size[0], 8UL, cudaMemcpyHostToDevice);
        classPred_data_dirtyOnGpu = false;
        cudaMemcpy(gpu_iv5, (void *)&iv5[0], 261UL, cudaMemcpyHostToDevice);
        iv5_dirtyOnCpu = false;
        cudaMemcpy(gpu_uv, (void *)&uv[0], 512UL, cudaMemcpyHostToDevice);
        uv_dirtyOnCpu = false;
        detectFunction_kernel101<<<bc_grid, bc_block>>>(*gpu_iv5, *gpu_uv,
          *gpu_thisCharcodes_1b_data, *gpu_uv7_size, *gpu_x_data);
      }

      if (iv5_dirtyOnCpu) {
        cudaMemcpy(gpu_iv5, (void *)&iv5[0], 261UL, cudaMemcpyHostToDevice);
      }

      if (uv_dirtyOnCpu) {
        cudaMemcpy(gpu_uv, (void *)&uv[0], 512UL, cudaMemcpyHostToDevice);
      }

      cudaMemcpy(gpu_validSampleTime, &validSampleTime, 8UL,
                 cudaMemcpyHostToDevice);
      detectFunction_kernel102<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>(*gpu_iv5,
        *gpu_uv, *gpu_thisCharcodes_1b_data, gpu_validSampleTime);
      validLaunchParams = mwGetLaunchParameters(static_cast<double>((b_uv7_size
        [1] - 2) + 1L), &cc_grid, &cc_block, 1024U, 65535U);
      if (validLaunchParams) {
        cudaMemcpy(gpu_x_size, &x_size[0], 8UL, cudaMemcpyHostToDevice);
        detectFunction_kernel103<<<cc_grid, cc_block>>>(*gpu_x_data, b_uv7_size
          [1], gpu_validSampleTime);
      }

      b_x_size[0] = 1;
      b_x_size[1] = b_uv7_size[1];
      validLaunchParams = mwGetLaunchParameters(static_cast<double>((b_uv7_size
        [1] - 1) + 1L), &dc_grid, &dc_block, 1024U, 65535U);
      if (validLaunchParams) {
        if (classPred_data_dirtyOnGpu) {
          cudaMemcpy(gpu_uv7_size, &b_uv7_size[0], 8UL, cudaMemcpyHostToDevice);
        }

        detectFunction_kernel104<<<dc_grid, dc_block>>>(*gpu_uv,
          *gpu_thisCharcodes_1b_data, *gpu_uv7_size, *b_gpu_x_data);
      }

      initStatus = b_x_size[1];
      if (iv_data_dirtyOnGpu) {
        cudaMemcpy(gpu_camIndex, &camIndex, 4UL, cudaMemcpyHostToDevice);
      }

      detectFunction_kernel105<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
        (*b_gpu_x_data, gpu_camIndex);
      validLaunchParams = mwGetLaunchParameters(static_cast<double>((b_x_size[1]
        - 2) + 1L), &ec_grid, &ec_block, 1024U, 65535U);
      if (validLaunchParams) {
        cudaMemcpy(b_gpu_x_size, &b_x_size[0], 8UL, cudaMemcpyHostToDevice);
        detectFunction_kernel106<<<ec_grid, ec_block>>>(*b_gpu_x_data,
          initStatus, gpu_camIndex);
      }

      cudaMemcpy(&validSampleTime, gpu_validSampleTime, 8UL,
                 cudaMemcpyDeviceToHost);
      if (validSampleTime < 2.147483648E+9) {
        initStatus = static_cast<int>(validSampleTime);
      } else {
        initStatus = MAX_int32_T;
      }

      cudaMemcpy(&camIndex, gpu_camIndex, 4UL, cudaMemcpyDeviceToHost);
      validSampleTime = static_cast<double>(camIndex) * 4.0;
      if (validSampleTime < 2.147483648E+9) {
        if (validSampleTime >= -2.147483648E+9) {
          cameraDevice = static_cast<int>(validSampleTime);
        } else {
          cameraDevice = MIN_int32_T;
        }
      } else {
        cameraDevice = MAX_int32_T;
      }

      if ((initStatus > 0) && (cameraDevice > MAX_int32_T - initStatus)) {
        camIndex = MAX_int32_T;
      } else {
        camIndex = initStatus + cameraDevice;
      }

      if (camIndex <= position[2]) {
        camIndex = position[2];
      }

      if (camIndex > position[2]) {
        if (camIndex > 2147483639) {
          camIndex = MAX_int32_T;
        } else {
          camIndex += 8;
        }
      }

      cudaMemcpy(&positionOut[0], gpu_positionOut, 16UL, cudaMemcpyDeviceToHost);
      if (positionOut[1] < -2147483625) {
        c = MIN_int32_T;
      } else {
        c = positionOut[1] - 23;
      }

      endC_gl = c + 1;
      count = positionOut[0];
      if ((position[2] > 0) && (position[3] > 0)) {
        if (c + 1 > 2147483624) {
          qY = MAX_int32_T;
        } else {
          qY = c + 24;
        }

        guard1 = false;
        if (positionOut[0] <= 224) {
          if ((positionOut[0] < 0) && (position[2] < MIN_int32_T - positionOut[0]))
          {
            initStatus = MIN_int32_T;
          } else if ((positionOut[0] > 0) && (position[2] > MAX_int32_T
                      - positionOut[0])) {
            initStatus = MAX_int32_T;
          } else {
            initStatus = positionOut[0] + position[2];
          }

          if ((initStatus - 1 >= 1) && (qY <= 224)) {
            if ((qY < 0) && (position[3] < MIN_int32_T - qY)) {
              qY = MIN_int32_T;
            } else if ((qY > 0) && (position[3] > MAX_int32_T - qY)) {
              qY = MAX_int32_T;
            } else {
              qY += position[3];
            }

            if (qY - 1 >= 1) {
              if (c + 1 < 1) {
                if ((c + 24 < 0) && (position[3] < 2147483624 - c)) {
                  c = MIN_int32_T;
                } else if ((c + 24 > 0) && (position[3] > 2147483623 - c)) {
                  c = MAX_int32_T;
                } else {
                  c = (c + position[3]) + 24;
                }

                if (c >= 1) {
                  if ((positionOut[1] < 0) && (position[3] < MIN_int32_T
                       - positionOut[1])) {
                    c = MIN_int32_T;
                  } else if ((positionOut[1] > 0) && (position[3] > MAX_int32_T
                              - positionOut[1])) {
                    c = MAX_int32_T;
                  } else {
                    c = positionOut[1] + position[3];
                  }

                  if (c > 2147483646) {
                    endC_gl = MAX_int32_T;
                  } else {
                    endC_gl = c + 1;
                  }
                }
              }

              if ((positionOut[0] < 0) && (camIndex < MIN_int32_T - positionOut
                   [0])) {
                c = MIN_int32_T;
              } else if ((positionOut[0] > 0) && (camIndex > MAX_int32_T
                          - positionOut[0])) {
                c = MAX_int32_T;
              } else {
                c = positionOut[0] + camIndex;
              }

              if (static_cast<double>(c) - 224.0 >= -2.147483648E+9) {
                cameraDevice = c - 224;
              } else {
                cameraDevice = MIN_int32_T;
              }

              if (cameraDevice > 0) {
                if ((positionOut[0] >= 0) && (cameraDevice < positionOut[0] -
                     MAX_int32_T)) {
                  c = MAX_int32_T;
                } else if ((positionOut[0] < 0) && (cameraDevice > positionOut[0]
                            - MIN_int32_T)) {
                  c = MIN_int32_T;
                } else {
                  c = positionOut[0] - cameraDevice;
                }

                count = c + 1;
              }

              if (count < 1) {
                if ((positionOut[0] < 0) && (position[2] < MIN_int32_T
                     - positionOut[0])) {
                  c = MIN_int32_T;
                } else if ((positionOut[0] > 0) && (position[2] > MAX_int32_T
                            - positionOut[0])) {
                  c = MAX_int32_T;
                } else {
                  c = positionOut[0] + position[2];
                }

                if (c >= 1) {
                  count = 1;
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
          endC_gl = -32767;
          count = -32767;
        }
      }

      cameraDevice = endC_gl;
      if (endC_gl > 2147483624) {
        c = MAX_int32_T;
      } else {
        c = endC_gl + 23;
      }

      endR = c - 1;
      startC_gl = count;
      if ((count < 0) && (camIndex < MIN_int32_T - count)) {
        qY = MIN_int32_T;
      } else if ((count > 0) && (camIndex > MAX_int32_T - count)) {
        qY = MAX_int32_T;
      } else {
        qY = count + camIndex;
      }

      if (qY < -2147483647) {
        initStatus = MIN_int32_T;
      } else {
        initStatus = qY - 1;
      }

      if ((endC_gl <= 224) && (c - 1 >= 1) && (count <= 224) && (initStatus >= 1))
      {
        if (endC_gl < 1) {
          cameraDevice = 1;
        }

        if (c - 1 > 224) {
          endR = 224;
        }

        if (count < 1) {
          startC_gl = 1;
        }

        if (initStatus > 224) {
          initStatus = 224;
        }

        for (i = 0; i < 3; i++) {
          for (c = 0; c <= initStatus - startC_gl; c++) {
            b_c = (startC_gl + c) - 1;
            for (r = 0; r <= endR - cameraDevice; r++) {
              unsigned char tmp22;
              b_r = (cameraDevice + r) - 1;
              if (bboxesX1Y1X2Y2_size_dirtyOnCpu) {
                cudaMemcpy(&color[0], gpu_color, 3UL, cudaMemcpyDeviceToHost);
                bboxesX1Y1X2Y2_size_dirtyOnCpu = false;
              }

              tmp11 = (uint8_T)(0.6 * static_cast<double>(color[i]) + 0.5);
              if (out_dirtyOnGpu) {
                cudaMemcpy(&out[0], b_gpu_out, 150528UL, cudaMemcpyDeviceToHost);
                out_dirtyOnGpu = false;
              }

              tmp22 = (uint8_T)(0.4 * static_cast<double>(out[(b_r + 224 * b_c)
                + 50176 * i]) + 0.5);
              qY = static_cast<int>(static_cast<unsigned int>(tmp11) + tmp22);
              if (static_cast<unsigned int>(qY) > 255U) {
                qY = 255;
              }

              out[(b_r + 224 * b_c) + 50176 * i] = static_cast<unsigned char>(qY);
              out_dirtyOnCpu = true;
            }
          }
        }
      }

      if (count > 2147483643) {
        penX = MAX_int32_T;
      } else {
        penX = count + 4;
      }

      if (endC_gl > 2147483643) {
        c = MAX_int32_T;
      } else {
        c = endC_gl + 4;
      }

      if (c > 2147483635) {
        count = MAX_int32_T;
      } else {
        count = c + 12;
      }

      i22 = uv7_size[1];
      for (i = 0; i < i22; i++) {
        if (thisTextU16_data_dirtyOnGpu) {
          cudaMemcpy(&thisTextU16_data[0], gpu_thisTextU16_data, 3UL,
                     cudaMemcpyDeviceToHost);
          thisTextU16_data_dirtyOnGpu = false;
        }

        if (uv[thisTextU16_data[i]] == 0) {
          if (penX > 2147483643) {
            penX = MAX_int32_T;
          } else {
            penX += 4;
          }
        } else {
          int endC_im;
          int endR_im;
          int startC_im;
          int yy;
          cameraDevice = iv7[uv[thisTextU16_data[i]]];
          if ((penX < 0) && (cameraDevice < MIN_int32_T - penX)) {
            startC_im = MIN_int32_T;
          } else if ((penX > 0) && (cameraDevice > MAX_int32_T - penX)) {
            startC_im = MAX_int32_T;
          } else {
            startC_im = penX + cameraDevice;
          }

          yy = count - iv8[uv[thisTextU16_data[i]]];
          cameraDevice = uv9[uv[thisTextU16_data[i]]];
          if ((yy < 0) && (cameraDevice < MIN_int32_T - yy)) {
            c = MIN_int32_T;
          } else if ((yy > 0) && (cameraDevice > MAX_int32_T - yy)) {
            c = MAX_int32_T;
          } else {
            c = yy + cameraDevice;
          }

          endR_im = c - 1;
          cameraDevice = uv8[uv[thisTextU16_data[i]]];
          if ((startC_im < 0) && (cameraDevice < MIN_int32_T - startC_im)) {
            qY = MIN_int32_T;
          } else if ((startC_im > 0) && (cameraDevice > MAX_int32_T - startC_im))
          {
            qY = MAX_int32_T;
          } else {
            qY = startC_im + cameraDevice;
          }

          endC_im = qY - 1;
          if ((yy <= 224) && (c - 1 >= 1) && (startC_im <= 224) && (qY - 1 >= 1))
          {
            signed char num_idx_0;
            signed char num_idx_1;
            cameraDevice = 1;
            startC_gl = 1;
            endR = uv9[uv[thisTextU16_data[i]]];
            endC_gl = uv8[uv[thisTextU16_data[i]]];
            if (yy < 1) {
              cameraDevice = 2 - yy;
              yy = 1;
            }

            if (c - 1 > 224) {
              endR = (uv9[uv[thisTextU16_data[i]]] - c) + 225;
              endR_im = 224;
            }

            if (startC_im < 1) {
              if (-startC_im > 2147483645) {
                startC_gl = MAX_int32_T;
              } else {
                startC_gl = 2 - startC_im;
              }

              startC_im = 1;
            }

            if (qY - 1 > 224) {
              endC_gl = (uv8[uv[thisTextU16_data[i]]] - qY) + 225;
              endC_im = 224;
            }

            initStatus = static_cast<int>(static_cast<unsigned int>
              (uv10[uv[thisTextU16_data[i]]]) + uv8[uv[thisTextU16_data[i]]] *
              uv9[uv[thisTextU16_data[i]]]);
            if (uv10[uv[thisTextU16_data[i]]] + 1U > static_cast<unsigned int>
                (initStatus)) {
              camIndex = 0;
              initStatus = -1;
            } else {
              camIndex = uv10[uv[thisTextU16_data[i]]];
              initStatus--;
            }

            num_idx_0 = uv8[uv[thisTextU16_data[i]]];
            num_idx_1 = uv9[uv[thisTextU16_data[i]]];
            validLaunchParams = mwGetLaunchParameters(static_cast<double>
              ((initStatus - camIndex) + 1L), &fc_grid, &fc_block, 1024U, 65535U);
            if (validLaunchParams) {
              cudaMemcpy(gpu_camIndex, &camIndex, 4UL, cudaMemcpyHostToDevice);
              if (uv11_dirtyOnCpu) {
                cudaMemcpy(gpu_uv11, (void *)&uv11[0], 10664UL,
                           cudaMemcpyHostToDevice);
                uv11_dirtyOnCpu = false;
              }

              detectFunction_kernel107<<<fc_grid, fc_block>>>(*gpu_uv11,
                gpu_camIndex, initStatus, *gpu_uv11_data);
            }

            num[0] = num_idx_0;
            num[1] = num_idx_1;
            uv11_size[0] = num_idx_1;
            uv11_size[1] = num_idx_0;
            validLaunchParams = mwGetLaunchParameters(static_cast<double>(((num
              [1] - 1) + 1L) * ((num[0] - 1) + 1L)), &gc_grid, &gc_block, 1024U,
              65535U);
            if (validLaunchParams) {
              cudaMemcpy(gpu_num, &num[0], 2UL, cudaMemcpyHostToDevice);
              cudaMemcpy(gpu_uv11_size, &uv11_size[0], 8UL,
                         cudaMemcpyHostToDevice);
              detectFunction_kernel108<<<gc_grid, gc_block>>>(*gpu_uv11_data,
                *gpu_uv11_size, *gpu_num, *b_gpu_uv11_data);
            }

            num[0] = num_idx_1;
            num[1] = num_idx_0;
            validLaunchParams = mwGetLaunchParameters(static_cast<double>((num[0]
              * num[1] - 1) + 1L), &hc_grid, &hc_block, 1024U, 65535U);
            if (validLaunchParams) {
              cudaMemcpy(gpu_num, &num[0], 2UL, cudaMemcpyHostToDevice);
              detectFunction_kernel109<<<hc_grid, hc_block>>>(*b_gpu_uv11_data, *
                gpu_num, *b_gpu_thisGlyphBitmap_data);
              b_thisGlyphBitmap_data_dirtyOnGpu = true;
            }

            if (cameraDevice > endR) {
              endR = 1;
            } else {
              endR = cameraDevice;
            }

            if (startC_gl > endC_gl) {
              initStatus = -1;
            } else {
              initStatus = startC_gl - 2;
            }

            for (camIndex = 0; camIndex < 3; camIndex++) {
              validSampleTime = 1.0;
              for (c = 0; c <= endC_im - startC_im; c++) {
                b_c = (startC_im + c) - 1;
                cameraDevice = 0;
                for (r = 0; r <= endR_im - yy; r++) {
                  b_r = (yy + r) - 1;
                  if (b_thisGlyphBitmap_data_dirtyOnGpu) {
                    cudaMemcpy(&b_thisGlyphBitmap_data[0],
                               b_gpu_thisGlyphBitmap_data, 144UL,
                               cudaMemcpyDeviceToHost);
                    b_thisGlyphBitmap_data_dirtyOnGpu = false;
                  }

                  tmp11 = b_thisGlyphBitmap_data[((endR + cameraDevice) +
                    num_idx_1 * (initStatus + static_cast<int>(validSampleTime)))
                    - 1];
                  if (tmp11 == 255) {
                    if (out_dirtyOnGpu) {
                      cudaMemcpy(&out[0], b_gpu_out, 150528UL,
                                 cudaMemcpyDeviceToHost);
                      out_dirtyOnGpu = false;
                    }

                    out[(b_r + 224 * b_c) + 50176 * camIndex] = 0U;
                    out_dirtyOnCpu = true;
                  } else {
                    if (tmp11 != 0) {
                      unsigned short b_x;
                      unsigned short tmp3;
                      if (out_dirtyOnGpu) {
                        cudaMemcpy(&out[0], b_gpu_out, 150528UL,
                                   cudaMemcpyDeviceToHost);
                        out_dirtyOnGpu = false;
                      }

                      b_x = static_cast<unsigned short>(out[(b_r + 224 * b_c) +
                        50176 * camIndex] * (255 - tmp11));
                      tmp3 = static_cast<unsigned short>(b_x / 255U);
                      b_x = static_cast<unsigned short>(static_cast<unsigned int>
                        (b_x) - tmp3 * 255);
                      if ((b_x > 0) && (b_x >= 128)) {
                        tmp3 = static_cast<unsigned short>(tmp3 + 1);
                      }

                      out[(b_r + 224 * b_c) + 50176 * camIndex] = static_cast<
                        unsigned char>(tmp3);
                      out_dirtyOnCpu = true;
                    }
                  }

                  cameraDevice++;
                }

                validSampleTime++;
              }
            }
          }

          cudaMemcpy(gpu_cameraDevice, &cameraDevice, 4UL,
                     cudaMemcpyHostToDevice);
          detectFunction_kernel110<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
            (*gpu_iv5, *gpu_uv, *gpu_thisTextU16_data, i, gpu_cameraDevice);
          cudaMemcpy(&cameraDevice, gpu_cameraDevice, 4UL,
                     cudaMemcpyDeviceToHost);
          if ((penX < 0) && (cameraDevice < MIN_int32_T - penX)) {
            penX = MIN_int32_T;
          } else if ((penX > 0) && (cameraDevice > MAX_int32_T - penX)) {
            penX = MAX_int32_T;
          } else {
            penX += cameraDevice;
          }
        }
      }
    }
  } else {
    int c;
    int r;
    unsigned char tmp11;

    //      % rot
    for (i = 0; i < 3; i++) {
      for (c = 0; c < 164; c++) {
        for (r = 0; r < 34; r++) {
          unsigned char tmp22;
          tmp11 = (uint8_T)(0.4 * static_cast<double>(uv2[i]) + 0.5);
          if (out_dirtyOnGpu) {
            cudaMemcpy(&out[0], b_gpu_out, 150528UL, cudaMemcpyDeviceToHost);
            out_dirtyOnGpu = false;
          }

          tmp22 = (uint8_T)(0.6 * static_cast<double>(out[((r + 224 * (c + 9)) +
            50176 * i) + 19]) + 0.5);
          qY = static_cast<int>(static_cast<unsigned int>(tmp11) + tmp22);
          if (static_cast<unsigned int>(qY) > 255U) {
            qY = 255;
          }

          out[((r + 224 * (c + 9)) + 50176 * i) + 19] = static_cast<unsigned
            char>(qY);
          out_dirtyOnCpu = true;
        }
      }
    }

    penX = 16;
    for (i = 0; i < 16; i++) {
      if (uv[uv1[i]] == 0) {
        if (penX > 2147483641) {
          penX = MAX_int32_T;
        } else {
          penX += 6;
        }
      } else {
        int endC_im;
        int endR_im;
        int startC_im;
        int yy;
        cameraDevice = iv1[uv[uv1[i]]];
        if ((penX < 0) && (cameraDevice < MIN_int32_T - penX)) {
          startC_im = MIN_int32_T;
        } else if ((penX > 0) && (cameraDevice > MAX_int32_T - penX)) {
          startC_im = MAX_int32_T;
        } else {
          startC_im = penX + cameraDevice;
        }

        yy = 1 - iv2[uv[uv1[i]]];
        endR_im = uv4[uv[uv1[i]]] - iv2[uv[uv1[i]]];
        cameraDevice = uv3[uv[uv1[i]]];
        if ((startC_im < 0) && (cameraDevice < MIN_int32_T - startC_im)) {
          c = MIN_int32_T;
        } else if ((startC_im > 0) && (cameraDevice > MAX_int32_T - startC_im))
        {
          c = MAX_int32_T;
        } else {
          c = startC_im + cameraDevice;
        }

        if (c < -2147483647) {
          endC_im = MIN_int32_T;
        } else {
          endC_im = c - 1;
        }

        if ((startC_im <= 224) && (endC_im >= 1)) {
          int endC_gl;
          signed char num_idx_0;
          signed char num_idx_1;
          startC_gl = 1;
          endC_gl = uv3[uv[uv1[i]]];
          if (startC_im < 1) {
            if (startC_im <= MIN_int32_T) {
              initStatus = MAX_int32_T;
            } else {
              initStatus = -startC_im;
            }

            if (initStatus > 2147483645) {
              startC_gl = MAX_int32_T;
            } else {
              startC_gl = initStatus + 2;
            }

            startC_im = 1;
          }

          if (endC_im > 224) {
            endC_gl = (uv3[uv[uv1[i]]] - endC_im) + 224;
            endC_im = 224;
          }

          initStatus = static_cast<int>(static_cast<unsigned int>(uv6[uv[uv1[i]]])
            + uv3[uv[uv1[i]]] * uv4[uv[uv1[i]]]);
          if (uv6[uv[uv1[i]]] + 1U > static_cast<unsigned int>(initStatus)) {
            camIndex = 0;
            initStatus = -1;
          } else {
            camIndex = uv6[uv[uv1[i]]];
            initStatus--;
          }

          num_idx_0 = uv3[uv[uv1[i]]];
          num_idx_1 = uv4[uv[uv1[i]]];
          validLaunchParams = mwGetLaunchParameters(static_cast<double>
            ((initStatus - camIndex) + 1L), &yb_grid, &yb_block, 1024U, 65535U);
          if (validLaunchParams) {
            cudaMemcpy(gpu_camIndex, &camIndex, 4UL, cudaMemcpyHostToDevice);
            if (uv7_dirtyOnCpu) {
              cudaMemcpy(gpu_uv7, (void *)&uv7[0], 22591UL,
                         cudaMemcpyHostToDevice);
              uv7_dirtyOnCpu = false;
            }

            detectFunction_kernel87<<<yb_grid, yb_block>>>(*gpu_uv7,
              gpu_camIndex, initStatus, *gpu_uv7_data);
          }

          num[0] = num_idx_0;
          num[1] = num_idx_1;
          b_uv7_size[0] = num_idx_1;
          b_uv7_size[1] = num_idx_0;
          validLaunchParams = mwGetLaunchParameters(static_cast<double>(((num[1]
            - 1) + 1L) * ((num[0] - 1) + 1L)), &ac_grid, &ac_block, 1024U,
            65535U);
          if (validLaunchParams) {
            cudaMemcpy(gpu_uv7_size, &b_uv7_size[0], 8UL, cudaMemcpyHostToDevice);
            cudaMemcpy(gpu_num, &num[0], 2UL, cudaMemcpyHostToDevice);
            detectFunction_kernel88<<<ac_grid, ac_block>>>(*gpu_uv7_data,
              *gpu_uv7_size, *gpu_num, *b_gpu_uv7_data);
          }

          num[0] = num_idx_1;
          num[1] = num_idx_0;
          validLaunchParams = mwGetLaunchParameters(static_cast<double>((num[0] *
            num[1] - 1) + 1L), &bc_grid, &bc_block, 1024U, 65535U);
          if (validLaunchParams) {
            cudaMemcpy(gpu_num, &num[0], 2UL, cudaMemcpyHostToDevice);
            detectFunction_kernel89<<<bc_grid, bc_block>>>(*b_gpu_uv7_data,
              *gpu_num, *gpu_thisGlyphBitmap_data);
            thisGlyphBitmap_data_dirtyOnGpu = true;
          }

          if (startC_gl > endC_gl) {
            initStatus = -1;
          } else {
            initStatus = startC_gl - 2;
          }

          for (camIndex = 0; camIndex < 3; camIndex++) {
            validSampleTime = 1.0;
            for (c = 0; c <= endC_im - startC_im; c++) {
              int b_c;
              b_c = (startC_im + c) - 1;
              cameraDevice = 0;
              for (r = 0; r <= endR_im - yy; r++) {
                int b_r;
                b_r = (yy + r) + 42;
                if (thisGlyphBitmap_data_dirtyOnGpu) {
                  cudaMemcpy(&thisGlyphBitmap_data[0], gpu_thisGlyphBitmap_data,
                             324UL, cudaMemcpyDeviceToHost);
                  thisGlyphBitmap_data_dirtyOnGpu = false;
                }

                tmp11 = thisGlyphBitmap_data[cameraDevice + num_idx_1 *
                  (initStatus + static_cast<int>(validSampleTime))];
                if (tmp11 == 255) {
                  if (out_dirtyOnGpu) {
                    cudaMemcpy(&out[0], b_gpu_out, 150528UL,
                               cudaMemcpyDeviceToHost);
                    out_dirtyOnGpu = false;
                  }

                  out[(b_r + 224 * b_c) + 50176 * camIndex] = MAX_uint8_T;
                  out_dirtyOnCpu = true;
                } else {
                  if (tmp11 != 0) {
                    unsigned int outputVar;
                    unsigned short b_x;
                    unsigned short tmp3;
                    if (out_dirtyOnGpu) {
                      cudaMemcpy(&out[0], b_gpu_out, 150528UL,
                                 cudaMemcpyDeviceToHost);
                      out_dirtyOnGpu = false;
                    }

                    outputVar = static_cast<unsigned int>(out[(b_r + 224 * b_c)
                      + 50176 * camIndex] * (255 - tmp11)) + 255 * tmp11;
                    if (outputVar > 65535U) {
                      outputVar = 65535U;
                    }

                    tmp3 = static_cast<unsigned short>(outputVar / 255U);
                    b_x = static_cast<unsigned short>(outputVar - tmp3 * 255);
                    if ((b_x > 0) && (b_x >= 128)) {
                      tmp3 = static_cast<unsigned short>(tmp3 + 1);
                    }

                    if (tmp3 > 255) {
                      tmp3 = 255U;
                    }

                    out[(b_r + 224 * b_c) + 50176 * camIndex] = static_cast<
                      unsigned char>(tmp3);
                    out_dirtyOnCpu = true;
                  }
                }

                cameraDevice++;
              }

              validSampleTime++;
            }
          }
        }

        cameraDevice = iv3[uv[uv1[i]]];
        if ((penX < 0) && (cameraDevice < MIN_int32_T - penX)) {
          penX = MIN_int32_T;
        } else if ((penX > 0) && (cameraDevice > MAX_int32_T - penX)) {
          penX = MAX_int32_T;
        } else {
          penX += cameraDevice;
        }
      }
    }

    //  weiss
  }

  if (out_dirtyOnCpu) {
    cudaMemcpy(b_gpu_out, &out[0], 150528UL, cudaMemcpyHostToDevice);
  }

  detectFunction_kernel111<<<dim3(98U, 1U, 1U), dim3(512U, 1U, 1U)>>>(*b_gpu_out,
    *gpu_varargin_2, *gpu_varargin_1);
  detectFunction_kernel112<<<dim3(98U, 1U, 1U), dim3(512U, 1U, 1U)>>>(*b_gpu_out,
    *gpu_varargin_3);
  if (display.isInitialized != 1) {
    display.isSetupComplete = false;
    display.isInitialized = 1;
    display.PixelFormatEnum = 1;
    MW_SDL_videoDisplayInit(display.PixelFormatEnum, 1, 1, 224.0, 224.0);
    display.isSetupComplete = true;
  }

  cudaMemcpy(&varargin_2[0], gpu_varargin_2, 50176UL, cudaMemcpyDeviceToHost);
  cudaMemcpy(&varargin_1[0], gpu_varargin_1, 50176UL, cudaMemcpyDeviceToHost);
  cudaMemcpy(&varargin_3[0], gpu_varargin_3, 50176UL, cudaMemcpyDeviceToHost);
  MW_SDL_videoDisplayOutput(&varargin_1[0], &varargin_2[0], &varargin_3[0]);
  cudaFree(*gpu_cv);
  cudaFree(*gpu_cv2);
  cudaFree(*gpu_cv1);
  cudaFree(gpu_cameraDevice);
  cudaFree(gpu_camIndex);
  cudaFree(gpu_validSampleTime);
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
  cudaFree(*gpu_iv9_data);
  cudaFree(*d_gpu_bboxPred_size);
  cudaFree(*gpu_scores_data);
  cudaFree(gpu_oldIdx);
  cudaFree(*gpu_v_size);
  cudaFree(*gpu_cv3);
  cudaFree(*gpu_v_data);
  cudaFree(gpu_labelCells);
  cudaFree(gpu_i);
  cudaFree(*gpu_uv);
  cudaFree(*b_gpu_uv7_size);
  cudaFree(*gpu_uv7);
  cudaFree(*gpu_uv7_data);
  cudaFree(*gpu_num);
  cudaFree(*gpu_uv7_size);
  cudaFree(*b_gpu_uv7_data);
  cudaFree(*gpu_thisGlyphBitmap_data);
  cudaFree(*gpu_uv5);
  cudaFree(*gpu_position);
  cudaFree(*gpu_positionOut);
  cudaFree(*gpu_color);
  cudaFree(*gpu_tmpRGB);
  cudaFree(*gpu_pixCount);
  cudaFree(*gpu_thisTextU16_data);
  cudaFree(*gpu_x_size);
  cudaFree(*gpu_thisCharcodes_1b_data);
  cudaFree(*gpu_iv5);
  cudaFree(*gpu_x_data);
  cudaFree(*b_gpu_x_size);
  cudaFree(*b_gpu_x_data);
  cudaFree(*gpu_uv11);
  cudaFree(*gpu_uv11_data);
  cudaFree(*gpu_uv11_size);
  cudaFree(*b_gpu_uv11_data);
  cudaFree(*b_gpu_thisGlyphBitmap_data);
  cudaFree(*gpu_varargin_2);
  cudaFree(*gpu_varargin_1);
  cudaFree(*gpu_varargin_3);
}

//
// Arguments    : void
// Return Type  : void
//
void detectFunction_free()
{
  if (!display.matlabCodegenIsDeleted) {
    display.matlabCodegenIsDeleted = true;
    if (display.isInitialized == 1) {
      display.isInitialized = 2;
      if (display.isSetupComplete) {
        MW_SDL_videoDisplayTerminate(0, 0);
      }
    }
  }

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
  display_not_empty = false;
  cam_not_empty = false;
  hwobj_not_empty = false;
  mynet_not_empty = false;
  cam.matlabCodegenIsDeleted = true;
  display.matlabCodegenIsDeleted = true;
}

//
// File trailer for detectFunction.cu
//
// [EOF]
//
