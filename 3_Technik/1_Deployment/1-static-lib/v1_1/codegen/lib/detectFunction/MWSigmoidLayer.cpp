/* Copyright 2018-2019 The MathWorks, Inc. */

// Target Agnostic implementation for Keras's Sigmoid Layer
#include "MWSigmoidLayer.hpp"
#include "MWSigmoidLayerImpl.hpp"
#include "MWTargetNetworkImpl.hpp"

MWSigmoidLayer::MWSigmoidLayer()
{
}

MWSigmoidLayer::~MWSigmoidLayer()
{
}

void MWSigmoidLayer::createSigmoidLayer(MWTargetNetworkImpl* ntwk_impl,
                                        MWTensorBase* m_in,
                                        int outbufIdx)
{
#if defined(MW_TARGET_TYPE_CUDNN) || defined(MW_TARGET_TYPE_MKLDNN) || defined(MW_TARGET_TYPE_ARMNEON)
    setInputTensor(m_in);
    allocateOutputTensor<float>(-1, -1, -1, -1, -1, NULL);

    getOutputTensor(0)->setopBufIndex(outbufIdx);

    m_impl = new MWSigmoidLayerImpl(this, ntwk_impl);
    
#else
    setInputTensor(m_in);
    allocateOutputTensor<float>(getInputTensor()->getHeight(),
                                getInputTensor()->getWidth(),
                                getInputTensor()->getChannels(),
                                getInputTensor()->getBatchSize(),
                                getInputTensor()->getSequenceLength(),
                                NULL);

    m_impl = new MWSigmoidLayerImpl(this, ntwk_impl, outbufIdx);
#endif
}

void MWSigmoidLayer::propagateSize()
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
