#include "MWYoloExtractionLayerImpl.hpp"
#include "MWTargetNetworkImpl.hpp"
#include "cnn_api.hpp"
#include "MWKernelHeaders.hpp"
#include <math.h>
#include <cassert>
 MWYoloExtractionLayerImpl::MWYoloExtractionLayerImpl(MWCNNLayer* layer, 
MWTargetNetworkImpl* ntwk_impl, int HoqiuUUuJnXGkfDodicJ) : 
MWCNNLayerImpl(layer, ntwk_impl) , 
dkLDkRwCBjeybwDHbKiE(HoqiuUUuJnXGkfDodicJ) , fSbUUBgjKRbNXrHrlOLo(3) { } 
MWYoloExtractionLayerImpl::~MWYoloExtractionLayerImpl() { } void 
MWYoloExtractionLayerImpl::propagateSize() { } void 
MWYoloExtractionLayerImpl::predict() { MWTensorBase* ipTensorBase = 
getLayer()->getInputTensor(0); MWTensorBase* opTensorBase0 = 
getLayer()->getOutputTensor(0); MWTensorBase* opTensorBase1 = 
getLayer()->getOutputTensor(1); MWTensorBase* opTensorBase2 = 
getLayer()->getOutputTensor(2); MWTensor<float>* ipTensor = 
static_cast<MWTensor<float>*>(ipTensorBase); MWTensor<float>* opTensor0 = 
static_cast<MWTensor<float>*>(opTensorBase0); MWTensor<float>* opTensor1 = 
static_cast<MWTensor<float>*>(opTensorBase1); MWTensor<float>* opTensor2 = 
static_cast<MWTensor<float>*>(opTensorBase2); assert(opTensor0->getData() != 
ipTensor->getData()); if (ipTensor->getBatchSize() == 1) { float* 
UzaGmBLFEwmwaFXebUma = ipTensor->getData(); long int 
jaqKGCwoANNDMHgAsehk = (opTensor0->getHeight()) * 
(opTensor0->getWidth()); long int jHzoRQWaHafftmrmuvHO = 
opTensor0->getChannels() * jaqKGCwoANNDMHgAsehk; 
CUDA_CALL(cudaMemcpy(opTensor0->getData(), UzaGmBLFEwmwaFXebUma, 
sizeof(float) * jHzoRQWaHafftmrmuvHO, cudaMemcpyDeviceToDevice)); 
long int jLyhrFjMmVnNjoeDJCwH = opTensor1->getChannels() * 
jaqKGCwoANNDMHgAsehk; CUDA_CALL(cudaMemcpy(opTensor1->getData(), 
UzaGmBLFEwmwaFXebUma + jHzoRQWaHafftmrmuvHO, sizeof(float) * 
jLyhrFjMmVnNjoeDJCwH, cudaMemcpyDeviceToDevice)); long int 
iwclITrbVyVrJaArrXNr = opTensor2->getChannels() * 
jaqKGCwoANNDMHgAsehk; CUDA_CALL(cudaMemcpy(opTensor2->getData(), 
UzaGmBLFEwmwaFXebUma + jHzoRQWaHafftmrmuvHO + 
jLyhrFjMmVnNjoeDJCwH, sizeof(float) * 
iwclITrbVyVrJaArrXNr, cudaMemcpyDeviceToDevice)); } else { int 
YOWMnLKOMqAODXiVNoGy = ipTensor->getWidth(); long int 
YNmJhGSUszJKxsodxiuV = (ipTensor->getHeight()) * 
(ipTensor->getWidth()); long int YNDVziqpDddiXQKYZZhX = 
YNmJhGSUszJKxsodxiuV * (ipTensor->getChannels()); long int 
YGiQICncmsGZkNUyiQyg = YNDVziqpDddiXQKYZZhX * ipTensor->getBatchSize(); 
long int jHzoRQWaHafftmrmuvHO = YNmJhGSUszJKxsodxiuV * 
(opTensor0->getChannels()); long int jLyhrFjMmVnNjoeDJCwH = 
YNmJhGSUszJKxsodxiuV * (opTensor1->getChannels()); long int 
iwclITrbVyVrJaArrXNr = YNmJhGSUszJKxsodxiuV * 
(opTensor2->getChannels()); long int sFIUeCwGDlfadqOrGZHC = ((YGiQICncmsGZkNUyiQyg + 
31) / 32) * 32; long int tGsvtyAVkrDznETdweDC = (sFIUeCwGDlfadqOrGZHC < 1024) 
? sFIUeCwGDlfadqOrGZHC : 1024; long int KHClOltUSuqFVVErSxVb = 
(YGiQICncmsGZkNUyiQyg + tGsvtyAVkrDznETdweDC - 1) / 
tGsvtyAVkrDznETdweDC; YoloExtractionImpl<<<KHClOltUSuqFVVErSxVb, 
tGsvtyAVkrDznETdweDC>>>( ipTensor->getData(), opTensor0->getData(), 
opTensor1->getData(), opTensor2->getData(), dkLDkRwCBjeybwDHbKiE, 
YOWMnLKOMqAODXiVNoGy, YNmJhGSUszJKxsodxiuV, 
YNDVziqpDddiXQKYZZhX, jHzoRQWaHafftmrmuvHO, 
jLyhrFjMmVnNjoeDJCwH, iwclITrbVyVrJaArrXNr, 
YGiQICncmsGZkNUyiQyg); } return; } void MWYoloExtractionLayerImpl::cleanup() { }