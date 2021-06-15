/* Copyright 2019 The MathWorks, Inc. */

#ifndef __CONCATENTION_LAYER_HPP
#define __CONCATENTION_LAYER_HPP

#include "cnn_api.hpp"

/**
 *  Codegen class for Concatenation Layer
 *  Concatenation layer
 **/
class MWTargetNetworkImpl;
class MWConcatenationLayer : public MWCNNLayer
{
  public:
    MWConcatenationLayer();
    ~MWConcatenationLayer();

    void createConcatenationLayer(MWTargetNetworkImpl*, int numInputs, ...);
    void propagateSize();

  public:
    int dimension;
};

#endif
