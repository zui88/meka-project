/* Copyright 2018 The MathWorks, Inc. */

#ifndef __YOLO_SOFTMAX_LAYER_HPP
#define __YOLO_SOFTMAX_LAYER_HPP

#include "cnn_api.hpp"

/**
 *  Codegen class for YoloSoftmaxLayer
 **/
class MWYoloSoftmaxLayer : public MWCNNLayer {
  public:
    MWYoloSoftmaxLayer();
    ~MWYoloSoftmaxLayer();

    void createYoloSoftmaxLayer(MWTargetNetworkImpl*, MWTensorBase*, int, int);
    void propagateSize();
};

#endif
