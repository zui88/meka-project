/* Copyright 2019 The MathWorks, Inc. */

#ifndef __CONCAT_LAYER_IMPL_HPP
#define __CONCAT_LAYER_IMPL_HPP

#include "MWCNNLayerImpl.hpp"

/**
  *  Codegen class for Concat Layer
  *  Concatenates inputs along given dimension
**/
class MWCNNLayer;
class MWTargetNetworkImpl;
class MWConcatenationLayerImpl : public MWCNNLayerImpl
{   
  public:
    MWConcatenationLayerImpl(MWCNNLayer*, MWTargetNetworkImpl*);
    ~MWConcatenationLayerImpl();

    void predict();
    void cleanup();
    void propagateSize();

    void prepareKernelInputs(int& KHClOltUSuqFVVErSxVb, int& tGsvtyAVkrDznETdweDC, int eVAFqeShtGZAZluKdMvQ);
};

#endif
