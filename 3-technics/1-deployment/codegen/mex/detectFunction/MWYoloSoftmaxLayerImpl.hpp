/* Copyright 2018 The MathWorks, Inc. */

#ifndef YOLO_SOFTMAX_LAYER_IMPL_HPP
#define YOLO_SOFTMAX_LAYER_IMPL_HPP

#include "MWCNNLayerImpl.hpp"

/**
 *  Codegen class for Keras Flatten Layer
 **/
class MWCNNLayer;
class MWTargetNetworkImpl;
class MWYoloSoftmaxLayerImpl : public MWCNNLayerImpl {
  public:
    MWYoloSoftmaxLayerImpl(MWCNNLayer*, MWTargetNetworkImpl*, int);
    ~MWYoloSoftmaxLayerImpl();

    void predict();
    void cleanup();
    void propagateSize();

  private:
    cudnnTensorDescriptor_t* tCfVGVGaqfGdJypAKQqq;
    int dkLDkRwCBjeybwDHbKiE;
};
#endif
