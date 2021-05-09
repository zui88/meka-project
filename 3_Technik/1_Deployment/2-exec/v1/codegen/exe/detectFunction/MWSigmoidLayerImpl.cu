#include "MWSigmoidLayer.hpp"
#include "MWSigmoidLayerImpl.hpp"
#include "MWTargetNetworkImpl.hpp"
#include <stdarg.h>
#include <cassert>
 MWSigmoidLayerImpl::MWSigmoidLayerImpl(MWCNNLayer* layer, MWTargetNetworkImpl* 
ntwk_impl) : MWCNNLayerImpl(layer, ntwk_impl)  { 
CUDNN_CALL(cudnnCreateActivationDescriptor(&rkzbRnJPJHmyWmkoOrFj)); 
createAndAddDescriptor(getLayer()->getOutputTensor()->getSourcePortIndex()); } 
MWSigmoidLayerImpl::~MWSigmoidLayerImpl() { } void 
MWSigmoidLayerImpl::propagateSize() { MWTensorBase* opTensor = 
getLayer()->getOutputTensor(0); cudnnTensorDescriptor_t* desc = 
getDescriptor(opTensor->getSourcePortIndex()); assert(desc); 
setDescriptor<float>(*desc, static_cast<MWTensor<float>*>(opTensor)); 
CUDNN_CALL(cudnnSetActivationDescriptor(rkzbRnJPJHmyWmkoOrFj, 
CUDNN_ACTIVATION_SIGMOID,  CUDNN_NOT_PROPAGATE_NAN, 0));  } void 
MWSigmoidLayerImpl::predict() { MWSigmoidLayer* SigmoidLayer = 
static_cast<MWSigmoidLayer*>(getLayer()); MWTensorBase* ipTensorBase = 
SigmoidLayer->getInputTensor(0);  MWTensorBase* opTensorBase = 
SigmoidLayer->getOutputTensor(0); MWTensor<float>* ipTensor = 
static_cast<MWTensor<float>*>(ipTensorBase); MWTensor<float>* opTensor = 
static_cast<MWTensor<float>*>(opTensorBase); cudnnTensorDescriptor_t* desc = 
getDescriptor(opTensor->getSourcePortIndex()); assert(desc); 
cudnnTensorDescriptor_t ipDesc = 
MWCNNLayerImpl::getCuDNNDescriptor(ipTensorBase); 
CUDNN_CALL(cudnnActivationForward(*dJcdBfQQLhIAYHPxwQeg->getCudnnHandle(), 
rkzbRnJPJHmyWmkoOrFj,  getOnePtr(), ipDesc, ipTensor->getData(), 
getZeroPtr(), *desc, opTensor->getData())); } void 
MWSigmoidLayerImpl::cleanup() { MWSigmoidLayer* SigmoidLayer = 
static_cast<MWSigmoidLayer*>(getLayer()); 
CUDNN_CALL(cudnnDestroyActivationDescriptor(rkzbRnJPJHmyWmkoOrFj)); }