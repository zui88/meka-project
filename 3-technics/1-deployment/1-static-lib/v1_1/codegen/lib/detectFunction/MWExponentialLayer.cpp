/* Copyright 2018 The MathWorks, Inc. */

#include "MWExponentialLayer.hpp"
#include "MWExponentialLayerImpl.hpp"
#include "MWTargetNetworkImpl.hpp"

MWExponentialLayer::MWExponentialLayer() {
}

MWExponentialLayer::~MWExponentialLayer() {
}

void MWExponentialLayer::createExponentialLayer(MWTargetNetworkImpl* ntwk_impl,
                                                MWTensorBase* Input,
                                                int outbufIdx)
{
#if defined(MW_TARGET_TYPE_CUDNN) || defined(MW_TARGET_TYPE_MKLDNN) || defined(MW_TARGET_TYPE_ARMNEON)
    setInputTensor(Input, 0);
    allocateOutputTensor<float>(-1, -1, -1, -1, -1, NULL);

    getOutputTensor(0)->setopBufIndex(outbufIdx);

    m_impl = new MWExponentialLayerImpl(this, ntwk_impl);

#else
    setInputTensor(Input, 0);

    // Compute height and width of output
    int outH = getInputTensor(0)->getHeight();
    int outW = getInputTensor(0)->getWidth();
    int numOutputFeatures = getInputTensor()->getChannels();
    int BatchSize = getInputTensor()->getBatchSize();
    int seqLength = getInputTensor()->getSequenceLength();
    allocateOutputTensor<float>(outH, outW, numOutputFeatures, BatchSize, seqLength, NULL, 0);

    m_impl = new MWExponentialLayerImpl(this, ntwk_impl, outbufIdx);
#endif
}

void MWExponentialLayer::propagateSize()
{
#if defined(MW_TARGET_TYPE_CUDNN) || defined(MW_TARGET_TYPE_MKLDNN) || defined(MW_TARGET_TYPE_ARMNEON)
    resizeOutputTensor(getInputTensor()->getHeight(),
                       getInputTensor()->getWidth(),
                       getInputTensor()->getChannels(),
                       getInputTensor()->getBatchSize(),
                       getInputTensor()->getSequenceLength());

    m_impl->propagateSize();
#endif
}
