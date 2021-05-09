#include "MWYoloSoftmaxLayer.hpp"
#include "MWYoloSoftmaxLayerImpl.hpp"
#include "MWTargetNetworkImpl.hpp"
 MWYoloSoftmaxLayerImpl::MWYoloSoftmaxLayerImpl(MWCNNLayer* layer, 
MWTargetNetworkImpl* ntwk_impl, int HoqiuUUuJnXGkfDodicJ) : 
MWCNNLayerImpl(layer, ntwk_impl) , 
dkLDkRwCBjeybwDHbKiE(HoqiuUUuJnXGkfDodicJ) { 
tCfVGVGaqfGdJypAKQqq = new cudnnTensorDescriptor_t; if 
(!tCfVGVGaqfGdJypAKQqq) { 
MWCNNLayerImpl::throwAllocationError(__LINE__ , __FILE__); }  
CUDNN_CALL(cudnnCreateTensorDescriptor(tCfVGVGaqfGdJypAKQqq)); 
} MWYoloSoftmaxLayerImpl::~MWYoloSoftmaxLayerImpl() { } void 
MWYoloSoftmaxLayerImpl::propagateSize() { MWTensorBase* ipTensor = 
getLayer()->getInputTensor(0); MWTensorBase* opTensor = 
getLayer()->getOutputTensor(0); int eUSuiwvLvXVXrpUkgBVu = 
ipTensor->getChannels() / dkLDkRwCBjeybwDHbKiE; 
CUDNN_CALL(cudnnSetTensor4dDescriptor(*tCfVGVGaqfGdJypAKQqq, 
CUDNN_TENSOR_NCHW, CUDNN_DATA_FLOAT, opTensor->getBatchSize(), 
eUSuiwvLvXVXrpUkgBVu, dkLDkRwCBjeybwDHbKiE, opTensor->getWidth() * 
opTensor->getHeight())); } void MWYoloSoftmaxLayerImpl::predict() { 
MWTensorBase* ipTensorBase = getLayer()->getInputTensor(0); MWTensorBase* 
opTensorBase = getLayer()->getOutputTensor(0); MWTensor<float>* ipTensor = 
static_cast<MWTensor<float>*>(ipTensorBase); MWTensor<float>* opTensor = 
static_cast<MWTensor<float>*>(opTensorBase); CUDNN_CALL(cudnnSoftmaxForward( 
*dJcdBfQQLhIAYHPxwQeg->getCudnnHandle(), CUDNN_SOFTMAX_ACCURATE, 
CUDNN_SOFTMAX_MODE_CHANNEL, getOnePtr(), *tCfVGVGaqfGdJypAKQqq, 
ipTensor->getData(), getZeroPtr(), *tCfVGVGaqfGdJypAKQqq, 
opTensor->getData())); } void MWYoloSoftmaxLayerImpl::cleanup() { 
CUDNN_CALL(cudnnDestroyTensorDescriptor(*tCfVGVGaqfGdJypAKQqq)); 
}