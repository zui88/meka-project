#include "MWExponentialLayerImpl.hpp"
#include "MWTargetNetworkImpl.hpp"
#include "cnn_api.hpp"
#include "MWKernelHeaders.hpp"
#include <math.h>
#include <cassert>
 MWExponentialLayerImpl::MWExponentialLayerImpl(MWCNNLayer* layer, 
MWTargetNetworkImpl* ntwk_impl) : MWCNNLayerImpl(layer, ntwk_impl) { } 
MWExponentialLayerImpl::~MWExponentialLayerImpl() { } void 
MWExponentialLayerImpl::propagateSize() { } void 
MWExponentialLayerImpl::predict() { MWTensorBase* ipTensorBase = 
getLayer()->getInputTensor(0); MWTensorBase* opTensorBase = 
getLayer()->getOutputTensor(0); MWTensor<float>* ipTensor = 
static_cast<MWTensor<float>*>(ipTensorBase); MWTensor<float>* opTensor = 
static_cast<MWTensor<float>*>(opTensorBase); assert(opTensor->getData() != 
ipTensor->getData()); long int iMyHYqdPsEjdhQptHQNt = 
opTensor->getNumElements(); long int sFIUeCwGDlfadqOrGZHC = ((iMyHYqdPsEjdhQptHQNt + 
31) / 32) * 32; long int tGsvtyAVkrDznETdweDC = (sFIUeCwGDlfadqOrGZHC < 1024) 
? sFIUeCwGDlfadqOrGZHC : 1024; long int KHClOltUSuqFVVErSxVb = 
(iMyHYqdPsEjdhQptHQNt + tGsvtyAVkrDznETdweDC - 1) / 
tGsvtyAVkrDznETdweDC; exp_kernel<<<KHClOltUSuqFVVErSxVb, 
tGsvtyAVkrDznETdweDC>>>( ipTensor->getData(), opTensor->getData(), 
iMyHYqdPsEjdhQptHQNt); return; } void MWExponentialLayerImpl::cleanup() { }