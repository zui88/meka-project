/* Copyright 2018 The MathWorks, Inc. */

#ifndef __EXPONENTIAL_LAYER_HPP
#define __EXPONENTIAL_LAYER_HPP

#include "cnn_api.hpp"

/**
  *  Codegen class for Exponential Layer 
**/
class MWExponentialLayer : public MWCNNLayer
{
  public:
    MWExponentialLayer();
    ~MWExponentialLayer();

    void createExponentialLayer(MWTargetNetworkImpl*, MWTensorBase*, int);
    void propagateSize();
};

#endif
