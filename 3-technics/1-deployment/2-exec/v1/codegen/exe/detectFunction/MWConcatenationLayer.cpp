/* Copyright 2019 The MathWorks, Inc. */

#include "MWConcatenationLayer.hpp"
#include "MWConcatenationLayerImpl.hpp"
#include "MWTargetNetworkImpl.hpp"

#include <stdarg.h>
#include <cassert>

MWConcatenationLayer::MWConcatenationLayer()
{
}

MWConcatenationLayer::~MWConcatenationLayer()
{    
}

void MWConcatenationLayer::createConcatenationLayer(MWTargetNetworkImpl* ntwk_impl, int numInputs, ...)
{
#if defined(MW_TARGET_TYPE_CUDNN) || defined(MW_TARGET_TYPE_MKLDNN) || defined(MW_TARGET_TYPE_ARMNEON)
    va_list args;
    va_start(args, numInputs);
   
    for(int i = 0; i < numInputs; i++)
    {
        MWTensorBase* inputTensor = va_arg(args, MWTensorBase*);
        setInputTensor(inputTensor, i);
    }

    dimension = va_arg(args, int);
    assert((dimension == 1 || dimension == 2) || dimension == 3);

    allocateOutputTensor<float>(-1, -1, -1, -1, -1, NULL);

    int outbufIdx = va_arg(args, int);
    getOutputTensor(0)->setopBufIndex(outbufIdx);

    m_impl = new MWConcatenationLayerImpl(this, ntwk_impl);
#else
        va_list args;
    va_start(args, numInputs);
   
    for(int i = 0; i < numInputs; i++)
    {
        MWTensorBase* inputTensor = va_arg(args, MWTensorBase*);
        setInputTensor(inputTensor, i);
    }

    dimension = va_arg(args, int);
    assert((dimension == 1 || dimension == 2) || dimension == 3);

    // allocate output tensor
    MWTensorBase* firstInputTensor = getInputTensor(0);
    int height = firstInputTensor->getHeight();
    int width = firstInputTensor->getWidth();
    int numChannels = firstInputTensor->getChannels();
    int batchSize = firstInputTensor->getBatchSize();
    int sequenceLength = firstInputTensor->getSequenceLength();
    for(int k = 1; k < getNumInputs(); k++)
    {
        MWTensorBase* ipTensor = getInputTensor(k);

        // Determine along which dimension concatenation will take place. Check that all input tensors match in size in all dimensions except the concatenation dimension.
        assert(batchSize == ipTensor->getBatchSize());
        assert(sequenceLength == ipTensor->getSequenceLength());

        switch (dimension)
        {
        case 1:
        {
            assert(getInputTensor(0)->getWidth() == ipTensor->getWidth());
            assert(getInputTensor(0)->getChannels() == ipTensor->getChannels());
            height += (int)getInputTensor(k)->getHeight();
        }
        break;

        case 2:
        {
            assert(getInputTensor(0)->getHeight() == ipTensor->getHeight());
            assert(getInputTensor(0)->getChannels() == ipTensor->getChannels());      
            width += (int)getInputTensor(k)->getWidth();
        }
        break;

        case 3:
        {
            assert(getInputTensor(0)->getHeight() == ipTensor->getHeight());
            assert(getInputTensor(0)->getWidth() == ipTensor->getWidth());
            numChannels += (int)getInputTensor(k)->getChannels();
        }
        break;

        default:
            assert((dimension == 1 || dimension == 2) || dimension == 3);
        }
    }

    allocateOutputTensor<float>(height,
                                width,
                                numChannels,
                                batchSize,
                                sequenceLength, NULL);

    int outbufIdx = va_arg(args, int);
    getOutputTensor(0)->setopBufIndex(outbufIdx);

    m_impl = new MWConcatenationLayerImpl(this, ntwk_impl);
#endif
}

void MWConcatenationLayer::propagateSize()
{
#if defined(MW_TARGET_TYPE_CUDNN) || defined(MW_TARGET_TYPE_MKLDNN) || defined(MW_TARGET_TYPE_ARMNEON)
    MWTensorBase* firstInputTensor = getInputTensor(0);
    int height = firstInputTensor->getHeight();
    int width = firstInputTensor->getWidth();
    int numChannels = firstInputTensor->getChannels();
    int batchSize = firstInputTensor->getBatchSize();
    int sequenceLength = firstInputTensor->getSequenceLength();
    for(int k = 1; k < getNumInputs(); k++)
    {
        MWTensorBase* ipTensor = getInputTensor(k);

        // Determine along which dimension concatenation will take place. Check that all input tensors match in size in all dimensions except the concatenation dimension.
        assert(batchSize == ipTensor->getBatchSize());
        assert(sequenceLength == ipTensor->getSequenceLength());

        switch (dimension)
        {
        case 1:
        {
            assert(getInputTensor(0)->getWidth() == ipTensor->getWidth());
            assert(getInputTensor(0)->getChannels() == ipTensor->getChannels());
            height += (int)getInputTensor(k)->getHeight();
        }
        break;

        case 2:
        {
            assert(getInputTensor(0)->getHeight() == ipTensor->getHeight());
            assert(getInputTensor(0)->getChannels() == ipTensor->getChannels());      
            width += (int)getInputTensor(k)->getWidth();
        }
        break;

        case 3:
        {
            assert(getInputTensor(0)->getHeight() == ipTensor->getHeight());
            assert(getInputTensor(0)->getWidth() == ipTensor->getWidth());
            numChannels += (int)getInputTensor(k)->getChannels();
        }
        break;

        default:
            assert((dimension == 1 || dimension == 2) || dimension == 3);
        }
    }

    resizeOutputTensor(height,
                       width,
                       numChannels,
                       batchSize,
                       sequenceLength);

    m_impl->propagateSize();
#endif
}
