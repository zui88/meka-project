/* Copyright 2018 The MathWorks, Inc. */

#ifndef __YOLO_EXTRACTION_LAYER_HPP
#define __YOLO_EXTRACTION_LAYER_HPP

#include "cnn_api.hpp"

/**
 *  Codegen class for YoloExtractionLayer
 **/
class MWYoloExtractionLayer : public MWCNNLayer {
  public:
    MWYoloExtractionLayer();
    ~MWYoloExtractionLayer();

    void createYoloExtractionLayer(MWTargetNetworkImpl*, MWTensorBase*, int, int, int, int);
    void propagateSize();

  private:
    int numberAnchors;
};

#endif
