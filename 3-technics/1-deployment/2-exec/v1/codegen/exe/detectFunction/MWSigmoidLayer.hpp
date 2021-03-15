/* Copyright 2018 The MathWorks, Inc. */

// Target Agnostic header for Keras' Sigmoid Layer
#ifndef SIGMOID_LAYER_HPP
#define SIGMOID_LAYER_HPP

#include "cnn_api.hpp"

/**
  * Codegen class for Keras Sigmoid Layer
**/
class MWSigmoidLayer : public MWCNNLayer
{
  public:
    MWSigmoidLayer();
    ~MWSigmoidLayer();

    /** Create a new Sigmoid Layer */
    void createSigmoidLayer(MWTargetNetworkImpl*, MWTensorBase*, int);
    void propagateSize();
};
#endif
