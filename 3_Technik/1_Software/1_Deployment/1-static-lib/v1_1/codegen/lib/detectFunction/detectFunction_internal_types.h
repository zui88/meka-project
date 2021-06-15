//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: detectFunction_internal_types.h
//
// GPU Coder version                    : 2.0
// CUDA/C/C++ source code generated on  : 12-Mar-2021 17:27:33
//
#ifndef DETECTFUNCTION_INTERNAL_TYPES_H
#define DETECTFUNCTION_INTERNAL_TYPES_H

// Include Files
#include "detectFunction_types.h"
#include "rtwtypes.h"
#include "MWTargetNetworkImpl.hpp"
#include "cnn_api.hpp"

// Type Definitions
class yoloNetwork0_0
{
 public:
  yoloNetwork0_0();
  void setSize();
  void resetState();
  void setup();
  void predict();
  void activations(int layerIdx);
  void cleanup();
  float *getLayerOutput(int layerIndex, int portIndex);
  float *getInputDataPointer(int b_index);
  float *getInputDataPointer();
  float *getOutputDataPointer(int b_index);
  float *getOutputDataPointer();
  int getBatchSize();
  ~yoloNetwork0_0();
 private:
  void allocate();
  void postsetup();
  void deallocate();
 public:
  int numLayers;
  MWCNNLayer *layers[17];
 private:
  bool isInitialized;
  MWTensorBase *inputTensors[1];
  MWTensorBase *outputTensors[1];
  MWTargetNetworkImpl *targetImpl;
};

#endif

//
// File trailer for detectFunction_internal_types.h
//
// [EOF]
//
