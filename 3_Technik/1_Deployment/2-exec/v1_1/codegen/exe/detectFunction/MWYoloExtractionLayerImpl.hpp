/* Copyright 2018 The MathWorks, Inc. */

#ifndef __YOLO_EXTRACTION_IMPL_HPP
#define __YOLO_EXTRACTION_IMPL_HPP

#include "MWCNNLayerImpl.hpp"
#include <vector>


/**
 *  Codegen class for Yolo Exactraction layer
 **/
class MWCNNLayer;
class MWTargetNetworkImpl;
class MWYoloExtractionLayerImpl : public MWCNNLayerImpl {
  public:
    MWYoloExtractionLayerImpl(MWCNNLayer*,
                              MWTargetNetworkImpl*,
                              int);
    ~MWYoloExtractionLayerImpl();

    void predict();
    void cleanup();
    void propagateSize();

  public:
    int dkLDkRwCBjeybwDHbKiE;

  private:
    int fSbUUBgjKRbNXrHrlOLo;
};



#endif
