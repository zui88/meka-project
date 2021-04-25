/* Copyright 2018 The MathWorks, Inc. */

#include "MWYoloSoftmaxLayer.hpp"
#include "MWYoloSoftmaxLayerImpl.hpp"
#include "MWTargetNetworkImpl.hpp"

#include <cassert>

MWYoloSoftmaxLayer::MWYoloSoftmaxLayer() {
}

MWYoloSoftmaxLayer::~MWYoloSoftmaxLayer() {
}

void MWYoloSoftmaxLayer::createYoloSoftmaxLayer(MWTargetNetworkImpl* ntwk_impl,
                                                MWTensorBase* Input,
                                                int numAnchors,
                                                int outbufIdx)
{
#if defined(MW_TARGET_TYPE_CUDNN) || defined(MW_TARGET_TYPE_MKLDNN) || defined(MW_TARGET_TYPE_ARMNEON)
    setInputTensor(Input, 0);
    allocateOutputTensor<float>(-1, -1, -1, -1, -1, NULL);

    getOutputTensor(0)->setopBufIndex(outbufIdx);
    
    assert(numAnchors >= 0);

    m_impl = new MWYoloSoftmaxLayerImpl(this, ntwk_impl, numAnchors);
    
#else
    setInputTensor(Input, 0);

    int outputHeight = getInputTensor(0)->getHeight();
    int outputWidth = getInputTensor(0)->getWidth();
    int BatchSize = getInputTensor()->getBatchSize();
    int seqLength = getInputTensor()->getSequenceLength();
    int numChannels = getInputTensor(0)->getChannels();

    // assert truth
    assert((BatchSize >= 0) && (numChannels >= 0) && (numAnchors >= 0));

    allocateOutputTensor<float>(outputHeight, outputWidth, numChannels, BatchSize, seqLength, NULL, 0);

    assert(getOutputTensor()->getSequenceLength() == 1);

    m_impl = new MWYoloSoftmaxLayerImpl(this, ntwk_impl, numAnchors, outbufIdx);
#endif
}

void MWYoloSoftmaxLayer::propagateSize()
{
#if defined(MW_TARGET_TYPE_CUDNN) || defined(MW_TARGET_TYPE_MKLDNN) || defined(MW_TARGET_TYPE_ARMNEON)
    assert(getInputTensor()->getChannels() >= 0);
    assert(getInputTensor()->getBatchSize() >= 0);
    assert(getInputTensor()->getSequenceLength() == 1);
    
    resizeOutputTensor(getInputTensor()->getHeight(),
                       getInputTensor()->getWidth(),
                       getInputTensor()->getChannels(),
                       getInputTensor()->getBatchSize(),
                       getInputTensor()->getSequenceLength());

    m_impl->propagateSize();
#endif
}
