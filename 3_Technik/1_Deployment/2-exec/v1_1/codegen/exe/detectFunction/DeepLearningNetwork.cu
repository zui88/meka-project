//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: DeepLearningNetwork.cu
//
// GPU Coder version                    : 2.0
// CUDA/C/C++ source code generated on  : 12-Mar-2021 11:18:37
//

// Include Files
#include "DeepLearningNetwork.h"
#include "detectFunction_internal_types.h"
#include "rt_nonfinite.h"
#include "MWConcatenationLayer.hpp"
#include "MWConvLayer.hpp"
#include "MWExponentialLayer.hpp"
#include "MWFusedConvReLULayer.hpp"
#include "MWSigmoidLayer.hpp"
#include "MWTargetNetworkImpl.hpp"
#include "MWYoloExtractionLayer.hpp"
#include "MWYoloSoftmaxLayer.hpp"
#include "cnn_api.hpp"
#include <cstdio>

// Custom Source Code
#include "main.h"

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
  for (int idx = 0; idx < 17; idx++) {
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
  for (int idx = 0; idx < 17; idx++) {
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
  for (int idx = 0; idx < 17; idx++) {
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
  for (int idx = 0; idx < 17; idx++) {
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
    (static_cast<MWFusedConvReLULayer *>(this->layers[1]))
      ->createFusedConvReLULayer(this->targetImpl, 1, this->layers[0]
      ->getOutputTensor(0), 3, 3, 3, 16, 1, 1, 1, 1, 1, 1, 1, 1, 1,
      "./codegen/exe/detectFunction/cnn_yoloNetwork0_0_conv_1_w.bin",
      "./codegen/exe/detectFunction/cnn_yoloNetwork0_0_conv_1_b.bin", 1);
    (static_cast<MWMaxPoolingLayer *>(this->layers[2]))->createMaxPoolingLayer
      (this->targetImpl, this->layers[1]->getOutputTensor(0), 2, 2, 2, 2, 0, 0,
       0, 0, 0, 1, 0);
    (static_cast<MWFusedConvReLULayer *>(this->layers[3]))
      ->createFusedConvReLULayer(this->targetImpl, 1, this->layers[2]
      ->getOutputTensor(0), 3, 3, 16, 32, 1, 1, 1, 1, 1, 1, 1, 1, 1,
      "./codegen/exe/detectFunction/cnn_yoloNetwork0_0_conv_2_w.bin",
      "./codegen/exe/detectFunction/cnn_yoloNetwork0_0_conv_2_b.bin", 1);
    (static_cast<MWMaxPoolingLayer *>(this->layers[4]))->createMaxPoolingLayer
      (this->targetImpl, this->layers[3]->getOutputTensor(0), 2, 2, 2, 2, 0, 0,
       0, 0, 0, 1, 0);
    (static_cast<MWFusedConvReLULayer *>(this->layers[5]))
      ->createFusedConvReLULayer(this->targetImpl, 1, this->layers[4]
      ->getOutputTensor(0), 3, 3, 32, 64, 1, 1, 1, 1, 1, 1, 1, 1, 1,
      "./codegen/exe/detectFunction/cnn_yoloNetwork0_0_conv_3_w.bin",
      "./codegen/exe/detectFunction/cnn_yoloNetwork0_0_conv_3_b.bin", 1);
    (static_cast<MWMaxPoolingLayer *>(this->layers[6]))->createMaxPoolingLayer
      (this->targetImpl, this->layers[5]->getOutputTensor(0), 2, 2, 2, 2, 0, 0,
       0, 0, 0, 1, 0);
    (static_cast<MWFusedConvReLULayer *>(this->layers[7]))
      ->createFusedConvReLULayer(this->targetImpl, 1, this->layers[6]
      ->getOutputTensor(0), 3, 3, 64, 128, 1, 1, 1, 1, 1, 1, 1, 1, 1,
      "./codegen/exe/detectFunction/cnn_yoloNetwork0_0_conv_4_w.bin",
      "./codegen/exe/detectFunction/cnn_yoloNetwork0_0_conv_4_b.bin", 1);
    (static_cast<MWFusedConvReLULayer *>(this->layers[8]))
      ->createFusedConvReLULayer(this->targetImpl, 1, this->layers[7]
      ->getOutputTensor(0), 3, 3, 128, 128, 1, 1, 1, 1, 1, 1, 1, 1, 1,
      "./codegen/exe/detectFunction/cnn_yoloNetwork0_0_yolov2Conv1_w.bin",
      "./codegen/exe/detectFunction/cnn_yoloNetwork0_0_yolov2Conv1_b.bin", 0);
    (static_cast<MWFusedConvReLULayer *>(this->layers[9]))
      ->createFusedConvReLULayer(this->targetImpl, 1, this->layers[8]
      ->getOutputTensor(0), 3, 3, 128, 128, 1, 1, 1, 1, 1, 1, 1, 1, 1,
      "./codegen/exe/detectFunction/cnn_yoloNetwork0_0_yolov2Conv2_w.bin",
      "./codegen/exe/detectFunction/cnn_yoloNetwork0_0_yolov2Conv2_b.bin", 1);
    (static_cast<MWConvLayer *>(this->layers[10]))->createConvLayer
      (this->targetImpl, this->layers[9]->getOutputTensor(0), 1, 1, 128, 24, 1,
       1, 0, 0, 0, 0, 1, 1, 1,
       "./codegen/exe/detectFunction/cnn_yoloNetwork0_0_yolov2ClassConv_w.bin",
       "./codegen/exe/detectFunction/cnn_yoloNetwork0_0_yolov2ClassConv_b.bin",
       0);
    (static_cast<MWYoloExtractionLayer *>(this->layers[11]))
      ->createYoloExtractionLayer(this->targetImpl, this->layers[10]
      ->getOutputTensor(0), 4, 1, 2, 3);
    (static_cast<MWSigmoidLayer *>(this->layers[12]))->createSigmoidLayer
      (this->targetImpl, this->layers[11]->getOutputTensor(0), 0);
    (static_cast<MWExponentialLayer *>(this->layers[13]))
      ->createExponentialLayer(this->targetImpl, this->layers[11]
      ->getOutputTensor(1), 1);
    (static_cast<MWYoloSoftmaxLayer *>(this->layers[14]))
      ->createYoloSoftmaxLayer(this->targetImpl, this->layers[11]
      ->getOutputTensor(2), 4, 2);
    (static_cast<MWConcatenationLayer *>(this->layers[15]))
      ->createConcatenationLayer(this->targetImpl, 3, this->layers[12]
      ->getOutputTensor(0), this->layers[13]->getOutputTensor(0), this->layers
      [14]->getOutputTensor(0), 3, 3);
    (static_cast<MWOutputLayer *>(this->layers[16]))->createOutputLayer
      (this->targetImpl, this->layers[15]->getOutputTensor(0), 3);
    this->outputTensors[0] = this->layers[16]->getOutputTensor(0);
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
  for (int idx = 0; idx < 17; idx++) {
    this->layers[idx]->predict();
  }
}

//
// Arguments    : void
// Return Type  : void
//
yoloNetwork0_0::yoloNetwork0_0()
{
  this->numLayers = 17;
  this->isInitialized = false;
  this->targetImpl = 0;
  this->layers[0] = new MWInputLayer;
  this->layers[0]->setName("input");
  this->layers[1] = new MWFusedConvReLULayer;
  this->layers[1]->setName("conv_1_relu_1");
  this->layers[2] = new MWMaxPoolingLayer;
  this->layers[2]->setName("maxpool1");
  this->layers[3] = new MWFusedConvReLULayer;
  this->layers[3]->setName("conv_2_relu_2");
  this->layers[4] = new MWMaxPoolingLayer;
  this->layers[4]->setName("maxpool2");
  this->layers[5] = new MWFusedConvReLULayer;
  this->layers[5]->setName("conv_3_relu_3");
  this->layers[6] = new MWMaxPoolingLayer;
  this->layers[6]->setName("maxpool3");
  this->layers[7] = new MWFusedConvReLULayer;
  this->layers[7]->setName("conv_4_relu_4");
  this->layers[8] = new MWFusedConvReLULayer;
  this->layers[8]->setName("yolov2Conv1_yolov2Relu1");
  this->layers[9] = new MWFusedConvReLULayer;
  this->layers[9]->setName("yolov2Conv2_yolov2Relu2");
  this->layers[10] = new MWConvLayer;
  this->layers[10]->setName("yolov2ClassConv");
  this->layers[11] = new MWYoloExtractionLayer;
  this->layers[11]->setName("YOLOv2ExtractionLayer");
  this->layers[12] = new MWSigmoidLayer;
  this->layers[12]->setName("YOLOSigmoidLayer");
  this->layers[13] = new MWExponentialLayer;
  this->layers[13]->setName("YOLOv2ExponentialLayer");
  this->layers[14] = new MWYoloSoftmaxLayer;
  this->layers[14]->setName("YOLOv2SoftmaxLayer");
  this->layers[15] = new MWConcatenationLayer;
  this->layers[15]->setName("YOLOv2ConcatenationLayer");
  this->layers[16] = new MWOutputLayer;
  this->layers[16]->setName("yolov2OutputLayer");
  this->layers[16]->setInPlaceIndex(0, 0);
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
  for (int idx = 0; idx < 17; idx++) {
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
