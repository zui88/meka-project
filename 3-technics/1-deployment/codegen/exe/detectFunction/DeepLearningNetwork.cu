//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: DeepLearningNetwork.cu
//
// GPU Coder version                    : 2.0
// CUDA/C/C++ source code generated on  : 08-Mar-2021 19:56:17
//

// Include Files
#include "DeepLearningNetwork.h"
#include "detectFunction_internal_types.h"
#include "rt_nonfinite.h"
#include "MWConcatenationLayer.hpp"
#include "MWConvLayer.hpp"
#include "MWElementwiseAffineLayer.hpp"
#include "MWExponentialLayer.hpp"
#include "MWFusedConvReLULayer.hpp"
#include "MWSigmoidLayer.hpp"
#include "MWTargetNetworkImpl.hpp"
#include "MWYoloExtractionLayer.hpp"
#include "MWYoloSoftmaxLayer.hpp"
#include "cnn_api.hpp"
#include <cstdio>

const char *errorString =
  "Abnormal termination due to: %s.\nError in %s (line %d).";

// Function Declarations
static void checkCleanupCudaError(cudaError_t errCode, const char *file,
  unsigned int line);

// Function Definitions
//
// Arguments    : void
// Return Type  : void
//
void yoloNetwork0_0::allocate()
{
  this->targetImpl->allocate(262144, 4);
  for (int idx = 0; idx < 57; idx++) {
    this->layers[idx]->allocate();
  }

  (static_cast<MWTensor<float> *>(this->inputTensors[0]))->setData(this->layers
    [0]->getLayerOutput(0));
}

//
// Arguments    : void
// Return Type  : void
//
void yoloNetwork0_0::cleanup()
{
  this->deallocate();
  for (int idx = 0; idx < 57; idx++) {
    this->layers[idx]->cleanup();
  }

  if (this->targetImpl) {
    this->targetImpl->cleanup();
  }
}

//
// Arguments    : void
// Return Type  : void
//
void yoloNetwork0_0::deallocate()
{
  this->targetImpl->deallocate();
  for (int idx = 0; idx < 57; idx++) {
    this->layers[idx]->deallocate();
  }
}

//
// Arguments    : void
// Return Type  : void
//
void yoloNetwork0_0::postsetup()
{
  this->targetImpl->postSetup(this->layers, this->numLayers);
}

//
// Arguments    : void
// Return Type  : void
//
void yoloNetwork0_0::resetState()
{
}

//
// Arguments    : void
// Return Type  : void
//
void yoloNetwork0_0::setSize()
{
  for (int idx = 0; idx < 57; idx++) {
    this->layers[idx]->propagateSize();
  }

  this->allocate();
  this->postsetup();
}

//
// Arguments    : void
// Return Type  : void
//
void yoloNetwork0_0::setup()
{
  if (this->isInitialized) {
    this->resetState();
  } else {
    this->isInitialized = true;
    this->targetImpl->preSetup();
    this->targetImpl->setAutoTune(true);
    (static_cast<MWInputLayer *>(this->layers[0]))->createInputLayer
      (this->targetImpl, this->inputTensors[0], 128, 128, 3, 0, "", 0);
    (static_cast<MWElementwiseAffineLayer *>(this->layers[1]))
      ->createElementwiseAffineLayer(this->targetImpl, this->layers[0]
      ->getOutputTensor(0), 1, 1, 3, 1, 1, 3, false, 1, 1,
      "./codegen/exe/detectFunction/cnn_yoloNetwork0_0_input_1_scale.bin",
      "./codegen/exe/detectFunction/cnn_yoloNetwork0_0_input_1_offset.bin", 0);
    (static_cast<MWFusedConvReLULayer *>(this->layers[2]))
      ->createFusedConvReLULayer(this->targetImpl, 1, this->layers[1]
      ->getOutputTensor(0), 7, 7, 3, 64, 2, 2, 3, 3, 3, 3, 1, 1, 1,
      "./codegen/exe/detectFunction/cnn_yoloNetwork0_0_conv1_w.bin",
      "./codegen/exe/detectFunction/cnn_yoloNetwork0_0_conv1_b.bin", 1);
    (static_cast<MWMaxPoolingLayer *>(this->layers[3]))->createMaxPoolingLayer
      (this->targetImpl, this->layers[2]->getOutputTensor(0), 3, 3, 2, 2, 1, 1,
       1, 1, 0, 1, 0);
    (static_cast<MWFusedConvReLULayer *>(this->layers[4]))
      ->createFusedConvReLULayer(this->targetImpl, 1, this->layers[3]
      ->getOutputTensor(0), 1, 1, 64, 64, 1, 1, 0, 0, 0, 0, 1, 1, 1,
      "./codegen/exe/detectFunction/cnn_yoloNetwork0_0_res2a_branch2a_w.bin",
      "./codegen/exe/detectFunction/cnn_yoloNetwork0_0_res2a_branch2a_b.bin", 1);
    (static_cast<MWFusedConvReLULayer *>(this->layers[5]))
      ->createFusedConvReLULayer(this->targetImpl, 1, this->layers[4]
      ->getOutputTensor(0), 3, 3, 64, 64, 1, 1, 1, 1, 1, 1, 1, 1, 1,
      "./codegen/exe/detectFunction/cnn_yoloNetwork0_0_res2a_branch2b_w.bin",
      "./codegen/exe/detectFunction/cnn_yoloNetwork0_0_res2a_branch2b_b.bin", 2);
    (static_cast<MWConvLayer *>(this->layers[6]))->createConvLayer
      (this->targetImpl, this->layers[3]->getOutputTensor(0), 1, 1, 64, 256, 1,
       1, 0, 0, 0, 0, 1, 1, 1,
       "./codegen/exe/detectFunction/cnn_yoloNetwork0_0_res2a_branch1_w.bin",
       "./codegen/exe/detectFunction/cnn_yoloNetwork0_0_res2a_branch1_b.bin", 1);
    (static_cast<MWFusedConvReLULayer *>(this->layers[7]))
      ->createFusedConvReLULayer(this->targetImpl, 2, this->layers[5]
      ->getOutputTensor(0), this->layers[6]->getOutputTensor(0), 1, 1, 64, 256,
      1, 1, 0, 0, 0, 0, 1, 1, 1,
      "./codegen/exe/detectFunction/cnn_yoloNetwork0_0_res2a_branch2c_w.bin",
      "./codegen/exe/detectFunction/cnn_yoloNetwork0_0_res2a_branch2c_b.bin", 1);
    (static_cast<MWFusedConvReLULayer *>(this->layers[8]))
      ->createFusedConvReLULayer(this->targetImpl, 1, this->layers[7]
      ->getOutputTensor(0), 1, 1, 256, 64, 1, 1, 0, 0, 0, 0, 1, 1, 1,
      "./codegen/exe/detectFunction/cnn_yoloNetwork0_0_res2b_branch2a_w.bin",
      "./codegen/exe/detectFunction/cnn_yoloNetwork0_0_res2b_branch2a_b.bin", 0);
    (static_cast<MWFusedConvReLULayer *>(this->layers[9]))
      ->createFusedConvReLULayer(this->targetImpl, 1, this->layers[8]
      ->getOutputTensor(0), 3, 3, 64, 64, 1, 1, 1, 1, 1, 1, 1, 1, 1,
      "./codegen/exe/detectFunction/cnn_yoloNetwork0_0_res2b_branch2b_w.bin",
      "./codegen/exe/detectFunction/cnn_yoloNetwork0_0_res2b_branch2b_b.bin", 2);
    (static_cast<MWFusedConvReLULayer *>(this->layers[10]))
      ->createFusedConvReLULayer(this->targetImpl, 2, this->layers[9]
      ->getOutputTensor(0), this->layers[7]->getOutputTensor(0), 1, 1, 64, 256,
      1, 1, 0, 0, 0, 0, 1, 1, 1,
      "./codegen/exe/detectFunction/cnn_yoloNetwork0_0_res2b_branch2c_w.bin",
      "./codegen/exe/detectFunction/cnn_yoloNetwork0_0_res2b_branch2c_b.bin", 1);
    (static_cast<MWFusedConvReLULayer *>(this->layers[11]))
      ->createFusedConvReLULayer(this->targetImpl, 1, this->layers[10]
      ->getOutputTensor(0), 1, 1, 256, 64, 1, 1, 0, 0, 0, 0, 1, 1, 1,
      "./codegen/exe/detectFunction/cnn_yoloNetwork0_0_res2c_branch2a_w.bin",
      "./codegen/exe/detectFunction/cnn_yoloNetwork0_0_res2c_branch2a_b.bin", 0);
    (static_cast<MWFusedConvReLULayer *>(this->layers[12]))
      ->createFusedConvReLULayer(this->targetImpl, 1, this->layers[11]
      ->getOutputTensor(0), 3, 3, 64, 64, 2, 2, 1, 1, 1, 1, 1, 1, 1,
      "./codegen/exe/detectFunction/cnn_yoloNetwork0_0_res2c_branch2b_w.bin",
      "./codegen/exe/detectFunction/cnn_yoloNetwork0_0_res2c_branch2b_b.bin", 2);
    (static_cast<MWMaxPoolingLayer *>(this->layers[13]))->createMaxPoolingLayer
      (this->targetImpl, this->layers[10]->getOutputTensor(0), 1, 1, 2, 2, 0, 0,
       0, 0, 0, 1, 0);
    (static_cast<MWFusedConvReLULayer *>(this->layers[14]))
      ->createFusedConvReLULayer(this->targetImpl, 2, this->layers[12]
      ->getOutputTensor(0), this->layers[13]->getOutputTensor(0), 1, 1, 64, 256,
      1, 1, 0, 0, 0, 0, 1, 1, 1,
      "./codegen/exe/detectFunction/cnn_yoloNetwork0_0_res2c_branch2c_w.bin",
      "./codegen/exe/detectFunction/cnn_yoloNetwork0_0_res2c_branch2c_b.bin", 0);
    (static_cast<MWFusedConvReLULayer *>(this->layers[15]))
      ->createFusedConvReLULayer(this->targetImpl, 1, this->layers[14]
      ->getOutputTensor(0), 1, 1, 256, 128, 1, 1, 0, 0, 0, 0, 1, 1, 1,
      "./codegen/exe/detectFunction/cnn_yoloNetwork0_0_res3a_branch2a_w.bin",
      "./codegen/exe/detectFunction/cnn_yoloNetwork0_0_res3a_branch2a_b.bin", 1);
    (static_cast<MWFusedConvReLULayer *>(this->layers[16]))
      ->createFusedConvReLULayer(this->targetImpl, 1, this->layers[15]
      ->getOutputTensor(0), 3, 3, 128, 128, 1, 1, 1, 1, 1, 1, 1, 1, 1,
      "./codegen/exe/detectFunction/cnn_yoloNetwork0_0_res3a_branch2b_w.bin",
      "./codegen/exe/detectFunction/cnn_yoloNetwork0_0_res3a_branch2b_b.bin", 2);
    (static_cast<MWConvLayer *>(this->layers[17]))->createConvLayer
      (this->targetImpl, this->layers[14]->getOutputTensor(0), 1, 1, 256, 512, 1,
       1, 0, 0, 0, 0, 1, 1, 1,
       "./codegen/exe/detectFunction/cnn_yoloNetwork0_0_res3a_branch1_w.bin",
       "./codegen/exe/detectFunction/cnn_yoloNetwork0_0_res3a_branch1_b.bin", 1);
    (static_cast<MWFusedConvReLULayer *>(this->layers[18]))
      ->createFusedConvReLULayer(this->targetImpl, 2, this->layers[16]
      ->getOutputTensor(0), this->layers[17]->getOutputTensor(0), 1, 1, 128, 512,
      1, 1, 0, 0, 0, 0, 1, 1, 1,
      "./codegen/exe/detectFunction/cnn_yoloNetwork0_0_res3a_branch2c_w.bin",
      "./codegen/exe/detectFunction/cnn_yoloNetwork0_0_res3a_branch2c_b.bin", 1);
    (static_cast<MWFusedConvReLULayer *>(this->layers[19]))
      ->createFusedConvReLULayer(this->targetImpl, 1, this->layers[18]
      ->getOutputTensor(0), 1, 1, 512, 128, 1, 1, 0, 0, 0, 0, 1, 1, 1,
      "./codegen/exe/detectFunction/cnn_yoloNetwork0_0_res3b_branch2a_w.bin",
      "./codegen/exe/detectFunction/cnn_yoloNetwork0_0_res3b_branch2a_b.bin", 0);
    (static_cast<MWFusedConvReLULayer *>(this->layers[20]))
      ->createFusedConvReLULayer(this->targetImpl, 1, this->layers[19]
      ->getOutputTensor(0), 3, 3, 128, 128, 1, 1, 1, 1, 1, 1, 1, 1, 1,
      "./codegen/exe/detectFunction/cnn_yoloNetwork0_0_res3b_branch2b_w.bin",
      "./codegen/exe/detectFunction/cnn_yoloNetwork0_0_res3b_branch2b_b.bin", 2);
    (static_cast<MWFusedConvReLULayer *>(this->layers[21]))
      ->createFusedConvReLULayer(this->targetImpl, 2, this->layers[20]
      ->getOutputTensor(0), this->layers[18]->getOutputTensor(0), 1, 1, 128, 512,
      1, 1, 0, 0, 0, 0, 1, 1, 1,
      "./codegen/exe/detectFunction/cnn_yoloNetwork0_0_res3b_branch2c_w.bin",
      "./codegen/exe/detectFunction/cnn_yoloNetwork0_0_res3b_branch2c_b.bin", 1);
    (static_cast<MWFusedConvReLULayer *>(this->layers[22]))
      ->createFusedConvReLULayer(this->targetImpl, 1, this->layers[21]
      ->getOutputTensor(0), 1, 1, 512, 128, 1, 1, 0, 0, 0, 0, 1, 1, 1,
      "./codegen/exe/detectFunction/cnn_yoloNetwork0_0_res3c_branch2a_w.bin",
      "./codegen/exe/detectFunction/cnn_yoloNetwork0_0_res3c_branch2a_b.bin", 0);
    (static_cast<MWFusedConvReLULayer *>(this->layers[23]))
      ->createFusedConvReLULayer(this->targetImpl, 1, this->layers[22]
      ->getOutputTensor(0), 3, 3, 128, 128, 1, 1, 1, 1, 1, 1, 1, 1, 1,
      "./codegen/exe/detectFunction/cnn_yoloNetwork0_0_res3c_branch2b_w.bin",
      "./codegen/exe/detectFunction/cnn_yoloNetwork0_0_res3c_branch2b_b.bin", 2);
    (static_cast<MWFusedConvReLULayer *>(this->layers[24]))
      ->createFusedConvReLULayer(this->targetImpl, 2, this->layers[23]
      ->getOutputTensor(0), this->layers[21]->getOutputTensor(0), 1, 1, 128, 512,
      1, 1, 0, 0, 0, 0, 1, 1, 1,
      "./codegen/exe/detectFunction/cnn_yoloNetwork0_0_res3c_branch2c_w.bin",
      "./codegen/exe/detectFunction/cnn_yoloNetwork0_0_res3c_branch2c_b.bin", 1);
    (static_cast<MWFusedConvReLULayer *>(this->layers[25]))
      ->createFusedConvReLULayer(this->targetImpl, 1, this->layers[24]
      ->getOutputTensor(0), 1, 1, 512, 128, 1, 1, 0, 0, 0, 0, 1, 1, 1,
      "./codegen/exe/detectFunction/cnn_yoloNetwork0_0_res3d_branch2a_w.bin",
      "./codegen/exe/detectFunction/cnn_yoloNetwork0_0_res3d_branch2a_b.bin", 0);
    (static_cast<MWFusedConvReLULayer *>(this->layers[26]))
      ->createFusedConvReLULayer(this->targetImpl, 1, this->layers[25]
      ->getOutputTensor(0), 3, 3, 128, 128, 2, 2, 1, 1, 1, 1, 1, 1, 1,
      "./codegen/exe/detectFunction/cnn_yoloNetwork0_0_res3d_branch2b_w.bin",
      "./codegen/exe/detectFunction/cnn_yoloNetwork0_0_res3d_branch2b_b.bin", 2);
    (static_cast<MWMaxPoolingLayer *>(this->layers[27]))->createMaxPoolingLayer
      (this->targetImpl, this->layers[24]->getOutputTensor(0), 1, 1, 2, 2, 0, 0,
       0, 0, 0, 1, 0);
    (static_cast<MWFusedConvReLULayer *>(this->layers[28]))
      ->createFusedConvReLULayer(this->targetImpl, 2, this->layers[26]
      ->getOutputTensor(0), this->layers[27]->getOutputTensor(0), 1, 1, 128, 512,
      1, 1, 0, 0, 0, 0, 1, 1, 1,
      "./codegen/exe/detectFunction/cnn_yoloNetwork0_0_res3d_branch2c_w.bin",
      "./codegen/exe/detectFunction/cnn_yoloNetwork0_0_res3d_branch2c_b.bin", 0);
    (static_cast<MWFusedConvReLULayer *>(this->layers[29]))
      ->createFusedConvReLULayer(this->targetImpl, 1, this->layers[28]
      ->getOutputTensor(0), 1, 1, 512, 256, 1, 1, 0, 0, 0, 0, 1, 1, 1,
      "./codegen/exe/detectFunction/cnn_yoloNetwork0_0_res4a_branch2a_w.bin",
      "./codegen/exe/detectFunction/cnn_yoloNetwork0_0_res4a_branch2a_b.bin", 1);
    (static_cast<MWFusedConvReLULayer *>(this->layers[30]))
      ->createFusedConvReLULayer(this->targetImpl, 1, this->layers[29]
      ->getOutputTensor(0), 3, 3, 256, 256, 1, 1, 1, 1, 1, 1, 1, 1, 1,
      "./codegen/exe/detectFunction/cnn_yoloNetwork0_0_res4a_branch2b_w.bin",
      "./codegen/exe/detectFunction/cnn_yoloNetwork0_0_res4a_branch2b_b.bin", 2);
    (static_cast<MWConvLayer *>(this->layers[31]))->createConvLayer
      (this->targetImpl, this->layers[28]->getOutputTensor(0), 1, 1, 512, 1024,
       1, 1, 0, 0, 0, 0, 1, 1, 1,
       "./codegen/exe/detectFunction/cnn_yoloNetwork0_0_res4a_branch1_w.bin",
       "./codegen/exe/detectFunction/cnn_yoloNetwork0_0_res4a_branch1_b.bin", 1);
    (static_cast<MWFusedConvReLULayer *>(this->layers[32]))
      ->createFusedConvReLULayer(this->targetImpl, 2, this->layers[30]
      ->getOutputTensor(0), this->layers[31]->getOutputTensor(0), 1, 1, 256,
      1024, 1, 1, 0, 0, 0, 0, 1, 1, 1,
      "./codegen/exe/detectFunction/cnn_yoloNetwork0_0_res4a_branch2c_w.bin",
      "./codegen/exe/detectFunction/cnn_yoloNetwork0_0_res4a_branch2c_b.bin", 1);
    (static_cast<MWFusedConvReLULayer *>(this->layers[33]))
      ->createFusedConvReLULayer(this->targetImpl, 1, this->layers[32]
      ->getOutputTensor(0), 1, 1, 1024, 256, 1, 1, 0, 0, 0, 0, 1, 1, 1,
      "./codegen/exe/detectFunction/cnn_yoloNetwork0_0_res4b_branch2a_w.bin",
      "./codegen/exe/detectFunction/cnn_yoloNetwork0_0_res4b_branch2a_b.bin", 0);
    (static_cast<MWFusedConvReLULayer *>(this->layers[34]))
      ->createFusedConvReLULayer(this->targetImpl, 1, this->layers[33]
      ->getOutputTensor(0), 3, 3, 256, 256, 1, 1, 1, 1, 1, 1, 1, 1, 1,
      "./codegen/exe/detectFunction/cnn_yoloNetwork0_0_res4b_branch2b_w.bin",
      "./codegen/exe/detectFunction/cnn_yoloNetwork0_0_res4b_branch2b_b.bin", 2);
    (static_cast<MWFusedConvReLULayer *>(this->layers[35]))
      ->createFusedConvReLULayer(this->targetImpl, 2, this->layers[34]
      ->getOutputTensor(0), this->layers[32]->getOutputTensor(0), 1, 1, 256,
      1024, 1, 1, 0, 0, 0, 0, 1, 1, 1,
      "./codegen/exe/detectFunction/cnn_yoloNetwork0_0_res4b_branch2c_w.bin",
      "./codegen/exe/detectFunction/cnn_yoloNetwork0_0_res4b_branch2c_b.bin", 1);
    (static_cast<MWFusedConvReLULayer *>(this->layers[36]))
      ->createFusedConvReLULayer(this->targetImpl, 1, this->layers[35]
      ->getOutputTensor(0), 1, 1, 1024, 256, 1, 1, 0, 0, 0, 0, 1, 1, 1,
      "./codegen/exe/detectFunction/cnn_yoloNetwork0_0_res4c_branch2a_w.bin",
      "./codegen/exe/detectFunction/cnn_yoloNetwork0_0_res4c_branch2a_b.bin", 0);
    (static_cast<MWFusedConvReLULayer *>(this->layers[37]))
      ->createFusedConvReLULayer(this->targetImpl, 1, this->layers[36]
      ->getOutputTensor(0), 3, 3, 256, 256, 1, 1, 1, 1, 1, 1, 1, 1, 1,
      "./codegen/exe/detectFunction/cnn_yoloNetwork0_0_res4c_branch2b_w.bin",
      "./codegen/exe/detectFunction/cnn_yoloNetwork0_0_res4c_branch2b_b.bin", 2);
    (static_cast<MWFusedConvReLULayer *>(this->layers[38]))
      ->createFusedConvReLULayer(this->targetImpl, 2, this->layers[37]
      ->getOutputTensor(0), this->layers[35]->getOutputTensor(0), 1, 1, 256,
      1024, 1, 1, 0, 0, 0, 0, 1, 1, 1,
      "./codegen/exe/detectFunction/cnn_yoloNetwork0_0_res4c_branch2c_w.bin",
      "./codegen/exe/detectFunction/cnn_yoloNetwork0_0_res4c_branch2c_b.bin", 1);
    (static_cast<MWFusedConvReLULayer *>(this->layers[39]))
      ->createFusedConvReLULayer(this->targetImpl, 1, this->layers[38]
      ->getOutputTensor(0), 1, 1, 1024, 256, 1, 1, 0, 0, 0, 0, 1, 1, 1,
      "./codegen/exe/detectFunction/cnn_yoloNetwork0_0_res4d_branch2a_w.bin",
      "./codegen/exe/detectFunction/cnn_yoloNetwork0_0_res4d_branch2a_b.bin", 0);
    (static_cast<MWFusedConvReLULayer *>(this->layers[40]))
      ->createFusedConvReLULayer(this->targetImpl, 1, this->layers[39]
      ->getOutputTensor(0), 3, 3, 256, 256, 1, 1, 1, 1, 1, 1, 1, 1, 1,
      "./codegen/exe/detectFunction/cnn_yoloNetwork0_0_res4d_branch2b_w.bin",
      "./codegen/exe/detectFunction/cnn_yoloNetwork0_0_res4d_branch2b_b.bin", 2);
    (static_cast<MWFusedConvReLULayer *>(this->layers[41]))
      ->createFusedConvReLULayer(this->targetImpl, 2, this->layers[40]
      ->getOutputTensor(0), this->layers[38]->getOutputTensor(0), 1, 1, 256,
      1024, 1, 1, 0, 0, 0, 0, 1, 1, 1,
      "./codegen/exe/detectFunction/cnn_yoloNetwork0_0_res4d_branch2c_w.bin",
      "./codegen/exe/detectFunction/cnn_yoloNetwork0_0_res4d_branch2c_b.bin", 1);
    (static_cast<MWFusedConvReLULayer *>(this->layers[42]))
      ->createFusedConvReLULayer(this->targetImpl, 1, this->layers[41]
      ->getOutputTensor(0), 1, 1, 1024, 256, 1, 1, 0, 0, 0, 0, 1, 1, 1,
      "./codegen/exe/detectFunction/cnn_yoloNetwork0_0_res4e_branch2a_w.bin",
      "./codegen/exe/detectFunction/cnn_yoloNetwork0_0_res4e_branch2a_b.bin", 0);
    (static_cast<MWFusedConvReLULayer *>(this->layers[43]))
      ->createFusedConvReLULayer(this->targetImpl, 1, this->layers[42]
      ->getOutputTensor(0), 3, 3, 256, 256, 1, 1, 1, 1, 1, 1, 1, 1, 1,
      "./codegen/exe/detectFunction/cnn_yoloNetwork0_0_res4e_branch2b_w.bin",
      "./codegen/exe/detectFunction/cnn_yoloNetwork0_0_res4e_branch2b_b.bin", 2);
    (static_cast<MWFusedConvReLULayer *>(this->layers[44]))
      ->createFusedConvReLULayer(this->targetImpl, 2, this->layers[43]
      ->getOutputTensor(0), this->layers[41]->getOutputTensor(0), 1, 1, 256,
      1024, 1, 1, 0, 0, 0, 0, 1, 1, 1,
      "./codegen/exe/detectFunction/cnn_yoloNetwork0_0_res4e_branch2c_w.bin",
      "./codegen/exe/detectFunction/cnn_yoloNetwork0_0_res4e_branch2c_b.bin", 1);
    (static_cast<MWFusedConvReLULayer *>(this->layers[45]))
      ->createFusedConvReLULayer(this->targetImpl, 1, this->layers[44]
      ->getOutputTensor(0), 1, 1, 1024, 256, 1, 1, 0, 0, 0, 0, 1, 1, 1,
      "./codegen/exe/detectFunction/cnn_yoloNetwork0_0_res4f_branch2a_w.bin",
      "./codegen/exe/detectFunction/cnn_yoloNetwork0_0_res4f_branch2a_b.bin", 0);
    (static_cast<MWFusedConvReLULayer *>(this->layers[46]))
      ->createFusedConvReLULayer(this->targetImpl, 1, this->layers[45]
      ->getOutputTensor(0), 3, 3, 256, 256, 1, 1, 1, 1, 1, 1, 1, 1, 1,
      "./codegen/exe/detectFunction/cnn_yoloNetwork0_0_res4f_branch2b_w.bin",
      "./codegen/exe/detectFunction/cnn_yoloNetwork0_0_res4f_branch2b_b.bin", 2);
    (static_cast<MWFusedConvReLULayer *>(this->layers[47]))
      ->createFusedConvReLULayer(this->targetImpl, 2, this->layers[46]
      ->getOutputTensor(0), this->layers[44]->getOutputTensor(0), 1, 1, 256,
      1024, 1, 1, 0, 0, 0, 0, 1, 1, 1,
      "./codegen/exe/detectFunction/cnn_yoloNetwork0_0_res4f_branch2c_w.bin",
      "./codegen/exe/detectFunction/cnn_yoloNetwork0_0_res4f_branch2c_b.bin", 1);
    (static_cast<MWFusedConvReLULayer *>(this->layers[48]))
      ->createFusedConvReLULayer(this->targetImpl, 1, this->layers[47]
      ->getOutputTensor(0), 3, 3, 1024, 1024, 1, 1, 1, 1, 1, 1, 1, 1, 1,
      "./codegen/exe/detectFunction/cnn_yoloNetwork0_0_yolov2Conv1_w.bin",
      "./codegen/exe/detectFunction/cnn_yoloNetwork0_0_yolov2Conv1_b.bin", 0);
    (static_cast<MWFusedConvReLULayer *>(this->layers[49]))
      ->createFusedConvReLULayer(this->targetImpl, 1, this->layers[48]
      ->getOutputTensor(0), 3, 3, 1024, 1024, 1, 1, 1, 1, 1, 1, 1, 1, 1,
      "./codegen/exe/detectFunction/cnn_yoloNetwork0_0_yolov2Conv2_w.bin",
      "./codegen/exe/detectFunction/cnn_yoloNetwork0_0_yolov2Conv2_b.bin", 1);
    (static_cast<MWConvLayer *>(this->layers[50]))->createConvLayer
      (this->targetImpl, this->layers[49]->getOutputTensor(0), 1, 1, 1024, 48, 1,
       1, 0, 0, 0, 0, 1, 1, 1,
       "./codegen/exe/detectFunction/cnn_yoloNetwork0_0_yolov2ClassConv_w.bin",
       "./codegen/exe/detectFunction/cnn_yoloNetwork0_0_yolov2ClassConv_b.bin",
       0);
    (static_cast<MWYoloExtractionLayer *>(this->layers[51]))
      ->createYoloExtractionLayer(this->targetImpl, this->layers[50]
      ->getOutputTensor(0), 8, 1, 2, 3);
    (static_cast<MWSigmoidLayer *>(this->layers[52]))->createSigmoidLayer
      (this->targetImpl, this->layers[51]->getOutputTensor(0), 0);
    (static_cast<MWExponentialLayer *>(this->layers[53]))
      ->createExponentialLayer(this->targetImpl, this->layers[51]
      ->getOutputTensor(1), 1);
    (static_cast<MWYoloSoftmaxLayer *>(this->layers[54]))
      ->createYoloSoftmaxLayer(this->targetImpl, this->layers[51]
      ->getOutputTensor(2), 8, 2);
    (static_cast<MWConcatenationLayer *>(this->layers[55]))
      ->createConcatenationLayer(this->targetImpl, 3, this->layers[52]
      ->getOutputTensor(0), this->layers[53]->getOutputTensor(0), this->layers
      [54]->getOutputTensor(0), 3, 3);
    (static_cast<MWOutputLayer *>(this->layers[56]))->createOutputLayer
      (this->targetImpl, this->layers[55]->getOutputTensor(0), 3);
    this->outputTensors[0] = this->layers[56]->getOutputTensor(0);
    this->setSize();
  }
}

//
// Arguments    : cudaError_t errCode
//                const char *file
//                unsigned int line
// Return Type  : void
//
static void checkCleanupCudaError(cudaError_t errCode, const char *file,
  unsigned int line)
{
  if ((errCode != cudaSuccess) && (errCode != cudaErrorCudartUnloading)) {
    printf(errorString, cudaGetErrorString(errCode), file, line);
  }
}

//
// Arguments    : int layerIdx
// Return Type  : void
//
void yoloNetwork0_0::activations(int layerIdx)
{
  for (int idx = 0; idx <= layerIdx; idx++) {
    this->layers[idx]->predict();
  }
}

//
// Arguments    : void
// Return Type  : int
//
int yoloNetwork0_0::getBatchSize()
{
  return this->inputTensors[0]->getBatchSize();
}

//
// Arguments    : int b_index
// Return Type  : float *
//
float *yoloNetwork0_0::getInputDataPointer(int b_index)
{
  return (static_cast<MWTensor<float> *>(this->inputTensors[b_index]))->getData();
}

//
// Arguments    : void
// Return Type  : float *
//
float *yoloNetwork0_0::getInputDataPointer()
{
  return (static_cast<MWTensor<float> *>(this->inputTensors[0]))->getData();
}

//
// Arguments    : int layerIndex
//                int portIndex
// Return Type  : float *
//
float *yoloNetwork0_0::getLayerOutput(int layerIndex, int portIndex)
{
  return this->layers[layerIndex]->getLayerOutput(portIndex);
}

//
// Arguments    : int b_index
// Return Type  : float *
//
float *yoloNetwork0_0::getOutputDataPointer(int b_index)
{
  return (static_cast<MWTensor<float> *>(this->outputTensors[b_index]))->getData
    ();
}

//
// Arguments    : void
// Return Type  : float *
//
float *yoloNetwork0_0::getOutputDataPointer()
{
  return (static_cast<MWTensor<float> *>(this->outputTensors[0]))->getData();
}

//
// Arguments    : void
// Return Type  : void
//
void yoloNetwork0_0::predict()
{
  for (int idx = 0; idx < 57; idx++) {
    this->layers[idx]->predict();
  }
}

//
// Arguments    : void
// Return Type  : void
//
yoloNetwork0_0::yoloNetwork0_0()
{
  this->numLayers = 57;
  this->isInitialized = false;
  this->targetImpl = 0;
  this->layers[0] = new MWInputLayer;
  this->layers[0]->setName("input_1");
  this->layers[1] = new MWElementwiseAffineLayer;
  this->layers[1]->setName("input_1_normalization");
  this->layers[1]->setInPlaceIndex(0, 0);
  this->layers[2] = new MWFusedConvReLULayer;
  this->layers[2]->setName("conv1_activation_1_relu");
  this->layers[3] = new MWMaxPoolingLayer;
  this->layers[3]->setName("max_pooling2d_1");
  this->layers[4] = new MWFusedConvReLULayer;
  this->layers[4]->setName("res2a_branch2a_activation_2_relu");
  this->layers[5] = new MWFusedConvReLULayer;
  this->layers[5]->setName("res2a_branch2b_activation_3_relu");
  this->layers[6] = new MWConvLayer;
  this->layers[6]->setName("res2a_branch1");
  this->layers[7] = new MWFusedConvReLULayer;
  this->layers[7]->setName("res2a_branch2c_activation_4_relu");
  this->layers[7]->setInPlaceIndex(0, 1);
  this->layers[8] = new MWFusedConvReLULayer;
  this->layers[8]->setName("res2b_branch2a_activation_5_relu");
  this->layers[9] = new MWFusedConvReLULayer;
  this->layers[9]->setName("res2b_branch2b_activation_6_relu");
  this->layers[10] = new MWFusedConvReLULayer;
  this->layers[10]->setName("res2b_branch2c_activation_7_relu");
  this->layers[10]->setInPlaceIndex(0, 1);
  this->layers[11] = new MWFusedConvReLULayer;
  this->layers[11]->setName("res2c_branch2a_activation_8_relu");
  this->layers[12] = new MWFusedConvReLULayer;
  this->layers[12]->setName("res2c_branch2b_activation_9_relu");
  this->layers[13] = new MWMaxPoolingLayer;
  this->layers[13]->setName("downsample_add_3");
  this->layers[14] = new MWFusedConvReLULayer;
  this->layers[14]->setName("res2c_branch2c_activation_10_relu");
  this->layers[14]->setInPlaceIndex(0, 1);
  this->layers[15] = new MWFusedConvReLULayer;
  this->layers[15]->setName("res3a_branch2a_activation_11_relu");
  this->layers[16] = new MWFusedConvReLULayer;
  this->layers[16]->setName("res3a_branch2b_activation_12_relu");
  this->layers[17] = new MWConvLayer;
  this->layers[17]->setName("res3a_branch1");
  this->layers[18] = new MWFusedConvReLULayer;
  this->layers[18]->setName("res3a_branch2c_activation_13_relu");
  this->layers[18]->setInPlaceIndex(0, 1);
  this->layers[19] = new MWFusedConvReLULayer;
  this->layers[19]->setName("res3b_branch2a_activation_14_relu");
  this->layers[20] = new MWFusedConvReLULayer;
  this->layers[20]->setName("res3b_branch2b_activation_15_relu");
  this->layers[21] = new MWFusedConvReLULayer;
  this->layers[21]->setName("res3b_branch2c_activation_16_relu");
  this->layers[21]->setInPlaceIndex(0, 1);
  this->layers[22] = new MWFusedConvReLULayer;
  this->layers[22]->setName("res3c_branch2a_activation_17_relu");
  this->layers[23] = new MWFusedConvReLULayer;
  this->layers[23]->setName("res3c_branch2b_activation_18_relu");
  this->layers[24] = new MWFusedConvReLULayer;
  this->layers[24]->setName("res3c_branch2c_activation_19_relu");
  this->layers[24]->setInPlaceIndex(0, 1);
  this->layers[25] = new MWFusedConvReLULayer;
  this->layers[25]->setName("res3d_branch2a_activation_20_relu");
  this->layers[26] = new MWFusedConvReLULayer;
  this->layers[26]->setName("res3d_branch2b_activation_21_relu");
  this->layers[27] = new MWMaxPoolingLayer;
  this->layers[27]->setName("downsample_add_7");
  this->layers[28] = new MWFusedConvReLULayer;
  this->layers[28]->setName("res3d_branch2c_activation_22_relu");
  this->layers[28]->setInPlaceIndex(0, 1);
  this->layers[29] = new MWFusedConvReLULayer;
  this->layers[29]->setName("res4a_branch2a_activation_23_relu");
  this->layers[30] = new MWFusedConvReLULayer;
  this->layers[30]->setName("res4a_branch2b_activation_24_relu");
  this->layers[31] = new MWConvLayer;
  this->layers[31]->setName("res4a_branch1");
  this->layers[32] = new MWFusedConvReLULayer;
  this->layers[32]->setName("res4a_branch2c_activation_25_relu");
  this->layers[32]->setInPlaceIndex(0, 1);
  this->layers[33] = new MWFusedConvReLULayer;
  this->layers[33]->setName("res4b_branch2a_activation_26_relu");
  this->layers[34] = new MWFusedConvReLULayer;
  this->layers[34]->setName("res4b_branch2b_activation_27_relu");
  this->layers[35] = new MWFusedConvReLULayer;
  this->layers[35]->setName("res4b_branch2c_activation_28_relu");
  this->layers[35]->setInPlaceIndex(0, 1);
  this->layers[36] = new MWFusedConvReLULayer;
  this->layers[36]->setName("res4c_branch2a_activation_29_relu");
  this->layers[37] = new MWFusedConvReLULayer;
  this->layers[37]->setName("res4c_branch2b_activation_30_relu");
  this->layers[38] = new MWFusedConvReLULayer;
  this->layers[38]->setName("res4c_branch2c_activation_31_relu");
  this->layers[38]->setInPlaceIndex(0, 1);
  this->layers[39] = new MWFusedConvReLULayer;
  this->layers[39]->setName("res4d_branch2a_activation_32_relu");
  this->layers[40] = new MWFusedConvReLULayer;
  this->layers[40]->setName("res4d_branch2b_activation_33_relu");
  this->layers[41] = new MWFusedConvReLULayer;
  this->layers[41]->setName("res4d_branch2c_activation_34_relu");
  this->layers[41]->setInPlaceIndex(0, 1);
  this->layers[42] = new MWFusedConvReLULayer;
  this->layers[42]->setName("res4e_branch2a_activation_35_relu");
  this->layers[43] = new MWFusedConvReLULayer;
  this->layers[43]->setName("res4e_branch2b_activation_36_relu");
  this->layers[44] = new MWFusedConvReLULayer;
  this->layers[44]->setName("res4e_branch2c_activation_37_relu");
  this->layers[44]->setInPlaceIndex(0, 1);
  this->layers[45] = new MWFusedConvReLULayer;
  this->layers[45]->setName("res4f_branch2a_activation_38_relu");
  this->layers[46] = new MWFusedConvReLULayer;
  this->layers[46]->setName("res4f_branch2b_activation_39_relu");
  this->layers[47] = new MWFusedConvReLULayer;
  this->layers[47]->setName("res4f_branch2c_activation_40_relu");
  this->layers[47]->setInPlaceIndex(0, 1);
  this->layers[48] = new MWFusedConvReLULayer;
  this->layers[48]->setName("yolov2Conv1_yolov2Relu1");
  this->layers[49] = new MWFusedConvReLULayer;
  this->layers[49]->setName("yolov2Conv2_yolov2Relu2");
  this->layers[50] = new MWConvLayer;
  this->layers[50]->setName("yolov2ClassConv");
  this->layers[51] = new MWYoloExtractionLayer;
  this->layers[51]->setName("YOLOv2ExtractionLayer");
  this->layers[52] = new MWSigmoidLayer;
  this->layers[52]->setName("YOLOSigmoidLayer");
  this->layers[53] = new MWExponentialLayer;
  this->layers[53]->setName("YOLOv2ExponentialLayer");
  this->layers[54] = new MWYoloSoftmaxLayer;
  this->layers[54]->setName("YOLOv2SoftmaxLayer");
  this->layers[55] = new MWConcatenationLayer;
  this->layers[55]->setName("YOLOv2ConcatenationLayer");
  this->layers[56] = new MWOutputLayer;
  this->layers[56]->setName("yolov2OutputLayer");
  this->layers[56]->setInPlaceIndex(0, 0);
  this->targetImpl = new MWTargetNetworkImpl;
  this->inputTensors[0] = new MWTensor<float>;
  this->inputTensors[0]->setHeight(128);
  this->inputTensors[0]->setWidth(128);
  this->inputTensors[0]->setChannels(3);
  this->inputTensors[0]->setBatchSize(1);
  this->inputTensors[0]->setSequenceLength(1);
}

//
// Arguments    : void
// Return Type  : void
//
yoloNetwork0_0::~yoloNetwork0_0()
{
  this->cleanup();
  checkCleanupCudaError(cudaGetLastError(), __FILE__, __LINE__);
  for (int idx = 0; idx < 57; idx++) {
    delete this->layers[idx];
  }

  if (this->targetImpl) {
    delete this->targetImpl;
  }

  delete this->inputTensors[0];
}

//
// Arguments    : yoloNetwork0_0 *obj
// Return Type  : void
//
namespace coder
{
  void DeepLearningNetwork_setup(yoloNetwork0_0 *obj)
  {
    obj->setup();
  }
}

//
// File trailer for DeepLearningNetwork.cu
//
// [EOF]
//
