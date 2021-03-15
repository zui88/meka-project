#include "MWFusedConvReLULayer.hpp"
#include "MWFusedConvReLULayerImpl.hpp"
#include <cassert>
#include <stdio.h>
 MWFusedConvReLULayerImpl::MWFusedConvReLULayerImpl(MWCNNLayer* layer, 
MWTargetNetworkImpl* ntwk_impl, int filt_H, int filt_W, int numGrps, int 
numChnls, int numFilts, int FrpxvsDMwwgbpqHXWxmN, int 
FwLnexHgxHRquTKmNpoa, int ClEhcJFlvGCgiavziIag, int 
CZNYmBcNFSZWvaCklqeM, int CufLFODQDXTAPyRqYodN, int 
DCdZnqpcBnvXVgEsLBnz, int AuqaQHxmPQSyYRemQvyX, int 
AzTsxYcYjIEJsGQbeYHm, int euppfEoiaoCTcVgRPVhA, const char* 
xHViLEwTujGGrPZZgmbF, const char* JwxFdqOKggeawILBfGgg) : 
MWCNNLayerImpl(layer, ntwk_impl) , vpXxoeEhdEosLSsYXkNG(NULL) , IwKnaBoXVubIRYcxEJLH(NULL) , 
UEESbUvbMihFnquvuFij(NULL) , WprSrhAStKGxyXeoxETy(NULL) , 
GFggoMvRWucDMqzlWzCl(NULL) , HhKGcPZwrclEFnIdWerH(NULL) , 
BHuHNDGoRwGRouCxeMbw(filt_H) , BLjrjqvCcCommiXWQLjs (filt_W) , 
BuyZFXzwOMxcePIbCLfl (numGrps) , BkwhtPQUCQKchmmimoXs (numChnls) , 
BlRIQPyqJZORKENzSdYf (numFilts) , FpguQZSermqZCMRiUfML(FrpxvsDMwwgbpqHXWxmN) 
, FshVHIJMRAhtQirYPlZd(FwLnexHgxHRquTKmNpoa) , 
CTCbzQMDaLxINPbODdng(ClEhcJFlvGCgiavziIag) , 
CLOUhPjbgggWoXHTtmjC(CZNYmBcNFSZWvaCklqeM) , 
CpMjJjtGOeWOzwxpAAQP(CufLFODQDXTAPyRqYodN) , 
CqtPRJvHlGJFssiPzsOm(DCdZnqpcBnvXVgEsLBnz) , 
AdmgfUbRAfzFeYHxSnQr(AuqaQHxmPQSyYRemQvyX) , 
AwZQzUhuWVLGrWgLHRuM(AzTsxYcYjIEJsGQbeYHm) , 
fvTCtkwXgyScJYogJVFU(euppfEoiaoCTcVgRPVhA) , 
IIiwAtyrOtLzLWAUlTey((CTCbzQMDaLxINPbODdng != CLOUhPjbgggWoXHTtmjC) 
|| (CpMjJjtGOeWOzwxpAAQP != CqtPRJvHlGJFssiPzsOm)) {
#if (CUDNN_MAJOR < 6)
 throw std::runtime_error("Fused ConvReLU Layer only supported for cuDNN 6 or greater");
#else
 dJcdBfQQLhIAYHPxwQeg = ntwk_impl; 
CUDNN_CALL(cudnnCreateConvolutionDescriptor(&NNhshzQGJHLSGjDiVerE)); 
CUDNN_CALL(cudnnCreateFilterDescriptor(&QVgVGfoCXYiYXzPhvVPX)); 
CUDNN_CALL(cudnnCreateTensorDescriptor(&JgLfgHrHMEMmMYTettJF));  
CUDNN_CALL(cudnnCreateActivationDescriptor(&oJUVMnJggjhEdQLWzIUC)); 
MWTensorBase* ipTensor_conv = getLayer()->getInputTensor(0); int 
NXruhrCCiguRjAgSNDuz = CTCbzQMDaLxINPbODdng; int 
NZjOkZPwLzQsdEVkwMcX = CpMjJjtGOeWOzwxpAAQP; if 
(IIiwAtyrOtLzLWAUlTey) { NXruhrCCiguRjAgSNDuz = 0; 
NZjOkZPwLzQsdEVkwMcX = 0; UEESbUvbMihFnquvuFij = new MWTensor<float>(-1, 
-1, -1, -1, -1, NULL, getLayer(), 0); if (!UEESbUvbMihFnquvuFij) { 
MWCNNLayerImpl::throwAllocationError(__LINE__ , __FILE__); } 
CUDNN_CALL(cudnnCreateTensorDescriptor(&XYbzSmRQGatVJtGmDZSo)); } else { 
UEESbUvbMihFnquvuFij = ipTensor_conv; } assert(UEESbUvbMihFnquvuFij != NULL); 
bUVPfnrJhLfHzOLUUrKk = CTCbzQMDaLxINPbODdng; cCXqPFPPcoHzYMDpnUxQ = 
CpMjJjtGOeWOzwxpAAQP; MWFusedConvReLULayer* fusedConvReluLayer = 
static_cast<MWFusedConvReLULayer*>(getLayer()); 
CUDNN_CALL(cudnnSetConvolution2dDescriptor(NNhshzQGJHLSGjDiVerE, 
NXruhrCCiguRjAgSNDuz, NZjOkZPwLzQsdEVkwMcX, FpguQZSermqZCMRiUfML, 
FshVHIJMRAhtQirYPlZd, AdmgfUbRAfzFeYHxSnQr, AwZQzUhuWVLGrWgLHRuM, 
CUDNN_CROSS_CORRELATION, CUDNN_DATA_FLOAT));
#if (FP16_ENABLED == 1 && ( CUDNN_MAJOR > 7 || (CUDNN_MAJOR == 7 && CUDNN_MINOR >= 2) ))
 CUDNN_CALL(cudnnSetConvolutionMathType(NNhshzQGJHLSGjDiVerE, CUDNN_TENSOR_OP_MATH_ALLOW_CONVERSION));
#endif
 if (BuyZFXzwOMxcePIbCLfl > 1){ 
CUDNN_CALL(cudnnSetConvolutionGroupCount(NNhshzQGJHLSGjDiVerE, 
BuyZFXzwOMxcePIbCLfl)); } 
CUDNN_CALL(cudnnSetActivationDescriptor(oJUVMnJggjhEdQLWzIUC, 
CUDNN_ACTIVATION_RELU, CUDNN_NOT_PROPAGATE_NAN, 0)); int 
etjQLJVQCaeAXRWYtqOl = BkwhtPQUCQKchmmimoXs*BuyZFXzwOMxcePIbCLfl; int 
fSKMHAqIghbYYgyIpNDw = BlRIQPyqJZORKENzSdYf*BuyZFXzwOMxcePIbCLfl; 
CUDNN_CALL(cudnnSetFilter4dDescriptor(QVgVGfoCXYiYXzPhvVPX, CUDNN_DATA_FLOAT, 
CUDNN_TENSOR_NCHW, fSKMHAqIghbYYgyIpNDw, 
etjQLJVQCaeAXRWYtqOl/BuyZFXzwOMxcePIbCLfl, BHuHNDGoRwGRouCxeMbw, 
BLjrjqvCcCommiXWQLjs)); 
CUDNN_CALL(cudnnSetTensor4dDescriptor(JgLfgHrHMEMmMYTettJF, CUDNN_TENSOR_NCHW, 
CUDNN_DATA_FLOAT, 1, fSKMHAqIghbYYgyIpNDw, 1, 1)); int weightSize = 
BkwhtPQUCQKchmmimoXs*fSKMHAqIghbYYgyIpNDw*BHuHNDGoRwGRouCxeMbw*BLjrjqvCcCommiXWQLjs; 
CUDA_CALL(cudaMalloc((void**)&vpXxoeEhdEosLSsYXkNG, sizeof(float)*weightSize)); 
CUDA_CALL(cudaMalloc((void**)&IwKnaBoXVubIRYcxEJLH, 
sizeof(float)*fSKMHAqIghbYYgyIpNDw)); 
loadWeights(xHViLEwTujGGrPZZgmbF); loadBias(JwxFdqOKggeawILBfGgg); createAndAddDescriptor(getLayer()->getOutputTensor(0)->getSourcePortIndex());
#endif
 } MWFusedConvReLULayerImpl::~MWFusedConvReLULayerImpl() { } void 
MWFusedConvReLULayerImpl::propagateSize() {
#if (CUDNN_MAJOR >= 6)
 MWTensorBase* ipTensor_conv = getLayer()->getInputTensor(0); int inputH; int 
inputW; if (IIiwAtyrOtLzLWAUlTey) { inputH = 
ipTensor_conv->getHeight() + CTCbzQMDaLxINPbODdng + CLOUhPjbgggWoXHTtmjC; 
inputW = ipTensor_conv->getWidth() + CpMjJjtGOeWOzwxpAAQP + 
CqtPRJvHlGJFssiPzsOm; } else { inputH = ipTensor_conv->getHeight(); inputW = 
ipTensor_conv->getWidth(); } UEESbUvbMihFnquvuFij->setHeight(inputH); 
UEESbUvbMihFnquvuFij->setWidth(inputW); 
UEESbUvbMihFnquvuFij->setChannels(ipTensor_conv->getChannels()); 
UEESbUvbMihFnquvuFij->setBatchSize(ipTensor_conv->getBatchSize()); 
UEESbUvbMihFnquvuFij->setSequenceLength(ipTensor_conv->getSequenceLength()); 
assert(UEESbUvbMihFnquvuFij->getSequenceLength() == 1); if 
(IIiwAtyrOtLzLWAUlTey) { 
CUDNN_CALL(cudnnSetTensor4dDescriptor(XYbzSmRQGatVJtGmDZSo, CUDNN_TENSOR_NCHW, 
CUDNN_DATA_FLOAT, UEESbUvbMihFnquvuFij->getBatchSize(), 
UEESbUvbMihFnquvuFij->getChannels(), UEESbUvbMihFnquvuFij->getHeight(), 
UEESbUvbMihFnquvuFij->getWidth())); } else { XYbzSmRQGatVJtGmDZSo = 
MWCNNLayerImpl::getCuDNNDescriptor(UEESbUvbMihFnquvuFij); } 
assert(BkwhtPQUCQKchmmimoXs == 
UEESbUvbMihFnquvuFij->getChannels()/BuyZFXzwOMxcePIbCLfl); MWTensorBase* opTensor 
= getLayer()->getOutputTensor(0); cudnnTensorDescriptor_t* desc = 
getDescriptor(opTensor->getSourcePortIndex()); assert(desc); 
setDescriptor<float>(*desc, static_cast<MWTensor<float>*>(opTensor));
#if (CUDNN_MAJOR < 7)
 { 
CUDNN_CALL(cudnnGetConvolutionForwardAlgorithm(*dJcdBfQQLhIAYHPxwQeg->getCudnnHandle(), 
XYbzSmRQGatVJtGmDZSo, QVgVGfoCXYiYXzPhvVPX, NNhshzQGJHLSGjDiVerE, *desc, 
CUDNN_CONVOLUTION_FWD_PREFER_FASTEST, 0, &NDjzAZSYJuWymuKDNZYB)); }
#else
 { const int maxAlgoCount(3); int returnedAlgoCount(-1); 
cudnnConvolutionFwdAlgoPerf_t perf_results[maxAlgoCount]; 
CUDNN_CALL(cudnnGetConvolutionForwardAlgorithm_v7(*dJcdBfQQLhIAYHPxwQeg->getCudnnHandle(), 
XYbzSmRQGatVJtGmDZSo, QVgVGfoCXYiYXzPhvVPX, NNhshzQGJHLSGjDiVerE, *desc, 
maxAlgoCount, &returnedAlgoCount, perf_results)); NDjzAZSYJuWymuKDNZYB = 
perf_results[0].algo; }
#endif
 if (CUDNN_VERSION < 7402) fixConvAlgo(); size_t tnTPxeDjBsqLAPkJcPJX = 0; 
CUDNN_CALL(cudnnGetConvolutionForwardWorkspaceSize(*dJcdBfQQLhIAYHPxwQeg->getCudnnHandle(), 
XYbzSmRQGatVJtGmDZSo, QVgVGfoCXYiYXzPhvVPX, NNhshzQGJHLSGjDiVerE, *desc, 
NDjzAZSYJuWymuKDNZYB, &tnTPxeDjBsqLAPkJcPJX)); if( tnTPxeDjBsqLAPkJcPJX > 
*dJcdBfQQLhIAYHPxwQeg->getProposedWorkSpaceSize() ) { 
dJcdBfQQLhIAYHPxwQeg->setProposedWorkSpaceSize(tnTPxeDjBsqLAPkJcPJX); }
#endif
 } void MWFusedConvReLULayerImpl::allocate() { MWTensorBase* ipTensor_conv = 
getLayer()->getInputTensor(0); if (IIiwAtyrOtLzLWAUlTey) { float* 
newInput; int inputH = ipTensor_conv->getHeight() + CTCbzQMDaLxINPbODdng + 
CLOUhPjbgggWoXHTtmjC; int inputW = ipTensor_conv->getWidth() + 
CpMjJjtGOeWOzwxpAAQP + CqtPRJvHlGJFssiPzsOm; int paddedSize = 
ipTensor_conv->getBatchSize() * ipTensor_conv->getChannels() * inputH * inputW; 
CUDA_CALL(cudaMalloc((void**)&newInput, sizeof(float)*paddedSize)); 
CUDA_CALL(cudaMemset(newInput, 0, sizeof(float)*paddedSize)); 
static_cast<MWTensor<float>*>(UEESbUvbMihFnquvuFij)->setData(newInput); } 
WprSrhAStKGxyXeoxETy = 
static_cast<MWTensor<float>*>(getLayer()->getOutputTensor(0))->getData(); 
setalpha2Ptr(getZeroPtr()); int numInputs = getLayer()->getNumInputs(); if 
(numInputs == 2) { setalpha2Ptr(getOnePtr()); WprSrhAStKGxyXeoxETy = 
static_cast<MWTensor<float>*>(getLayer()->getInputTensor(1))->getData(); } if 
(static_cast<MWTensor<float>*>(UEESbUvbMihFnquvuFij)->getData() == 
WprSrhAStKGxyXeoxETy){ int xInputTensorSize = 
getLayer()->getInputTensor(0)->getNumElements(); 
CUDA_CALL(cudaMalloc((void**)&GFggoMvRWucDMqzlWzCl, sizeof(float) * 
xInputTensorSize)); } } void MWFusedConvReLULayerImpl::deallocate() { if 
(UEESbUvbMihFnquvuFij != getLayer()->getInputTensor(0)) { 
assert(IIiwAtyrOtLzLWAUlTey); 
CUDA_FREE_CALL(static_cast<MWTensor<float>*>(UEESbUvbMihFnquvuFij)->getData()); 
static_cast<MWTensor<float>*>(UEESbUvbMihFnquvuFij)->setData((float*)NULL); } if 
(GFggoMvRWucDMqzlWzCl){ CUDA_FREE_CALL(GFggoMvRWucDMqzlWzCl); 
GFggoMvRWucDMqzlWzCl = NULL;  } } void 
MWFusedConvReLULayerImpl::predict() { MWFusedConvReLULayer* fusedConvReluLayer 
= static_cast<MWFusedConvReLULayer*>(getLayer()); MWTensorBase* ipTensorBase = 
fusedConvReluLayer->getInputTensor(); MWTensorBase* opTensorBase = 
fusedConvReluLayer->getOutputTensor(); MWTensor<float>* ipTensor = 
static_cast<MWTensor<float>*>(ipTensorBase); MWTensor<float>* opTensor = 
static_cast<MWTensor<float>*>(opTensorBase); if (UEESbUvbMihFnquvuFij != 
fusedConvReluLayer->getInputTensor()) { 
CUDA_CALL(cudaMemset(static_cast<MWTensor<float>*>(UEESbUvbMihFnquvuFij)->getData(), 
0, sizeof(float)*UEESbUvbMihFnquvuFij->getNumElements())); 
MWCNNLayerImpl::padInput(ipTensor->getData(), ipTensor->getHeight(), 
ipTensor->getWidth(), ipTensor->getChannels(), UEESbUvbMihFnquvuFij->getHeight(), 
UEESbUvbMihFnquvuFij->getWidth(), bUVPfnrJhLfHzOLUUrKk, cCXqPFPPcoHzYMDpnUxQ, 
static_cast<MWTensor<float>*>(UEESbUvbMihFnquvuFij)->getData(), 
ipTensor->getNumElements()); } cudnnTensorDescriptor_t* desc = 
getDescriptor(opTensor->getSourcePortIndex()); assert(desc);
#if (CUDNN_MAJOR >= 6)
 assert(opTensor->getData() != 
static_cast<MWTensor<float>*>(UEESbUvbMihFnquvuFij)->getData() || 
(getLayer()->getNumInputs() == 2)); float* rIcMzXptfYweLArNRnBw; if 
(static_cast<MWTensor<float>*>(UEESbUvbMihFnquvuFij)->getData() == 
WprSrhAStKGxyXeoxETy){  CUDA_CALL(cudaMemcpy(GFggoMvRWucDMqzlWzCl, 
static_cast<MWTensor<float>*>(UEESbUvbMihFnquvuFij)->getData(), sizeof(float) * 
opTensorBase->getNumElements(), cudaMemcpyDeviceToDevice)); 
rIcMzXptfYweLArNRnBw = GFggoMvRWucDMqzlWzCl; } else { 
rIcMzXptfYweLArNRnBw = 
static_cast<MWTensor<float>*>(UEESbUvbMihFnquvuFij)->getData(); } 
CUDNN_CALL(cudnnConvolutionBiasActivationForward(*dJcdBfQQLhIAYHPxwQeg->getCudnnHandle(), 
getOnePtr(), XYbzSmRQGatVJtGmDZSo, rIcMzXptfYweLArNRnBw, 
QVgVGfoCXYiYXzPhvVPX, vpXxoeEhdEosLSsYXkNG, NNhshzQGJHLSGjDiVerE, NDjzAZSYJuWymuKDNZYB, 
dJcdBfQQLhIAYHPxwQeg->getWorkSpace(), 
*dJcdBfQQLhIAYHPxwQeg->getAllocatedWorkSpaceSize(), getalpha2Ptr(),  *desc,  
WprSrhAStKGxyXeoxETy,  JgLfgHrHMEMmMYTettJF, IwKnaBoXVubIRYcxEJLH, oJUVMnJggjhEdQLWzIUC, 
*desc, opTensor->getData()));
#endif
 } void MWFusedConvReLULayerImpl::cleanup() { 
CUDNN_CALL(cudnnDestroyConvolutionDescriptor(NNhshzQGJHLSGjDiVerE)); 
CUDNN_CALL(cudnnDestroyFilterDescriptor(QVgVGfoCXYiYXzPhvVPX)); 
CUDNN_CALL(cudnnDestroyActivationDescriptor(oJUVMnJggjhEdQLWzIUC)); if 
(vpXxoeEhdEosLSsYXkNG) { CUDA_FREE_CALL(vpXxoeEhdEosLSsYXkNG); vpXxoeEhdEosLSsYXkNG = NULL; } 
CUDNN_CALL(cudnnDestroyTensorDescriptor(JgLfgHrHMEMmMYTettJF)); if 
(IwKnaBoXVubIRYcxEJLH) { CUDA_FREE_CALL(IwKnaBoXVubIRYcxEJLH); IwKnaBoXVubIRYcxEJLH = NULL; } if 
(UEESbUvbMihFnquvuFij != getLayer()->getInputTensor(0)) { 
assert(IIiwAtyrOtLzLWAUlTey); 
CUDNN_CALL(cudnnDestroyTensorDescriptor(XYbzSmRQGatVJtGmDZSo)); } } void 
MWFusedConvReLULayerImpl::loadWeights(const char* QMgBqCuvjnbWHWiVPEwn) { FILE* 
QhTesEEIHwhNmHSeYbRR = MWCNNLayer::openBinaryFile(QMgBqCuvjnbWHWiVPEwn); 
assert(QhTesEEIHwhNmHSeYbRR); int dMxIKDGTITyhdLqIHBLA = 
BkwhtPQUCQKchmmimoXs*BuyZFXzwOMxcePIbCLfl*BlRIQPyqJZORKENzSdYf*BHuHNDGoRwGRouCxeMbw*BLjrjqvCcCommiXWQLjs; 
 float* KZWeXiYFmdpQdsgidKeG = MALLOC_CALL(sizeof(float)*dMxIKDGTITyhdLqIHBLA); 
call_fread(KZWeXiYFmdpQdsgidKeG, sizeof(float), dMxIKDGTITyhdLqIHBLA, QhTesEEIHwhNmHSeYbRR, 
QMgBqCuvjnbWHWiVPEwn); CUDA_CALL(cudaMemcpy(vpXxoeEhdEosLSsYXkNG, KZWeXiYFmdpQdsgidKeG, 
sizeof(float)*dMxIKDGTITyhdLqIHBLA, cudaMemcpyHostToDevice));
#if 0
 printf("%s loaded. Size = %d. %f\n", QMgBqCuvjnbWHWiVPEwn, dMxIKDGTITyhdLqIHBLA, KZWeXiYFmdpQdsgidKeG[0]);
#endif
 free(KZWeXiYFmdpQdsgidKeG); fclose(QhTesEEIHwhNmHSeYbRR); return; } void 
MWFusedConvReLULayerImpl::loadBias(const char* QMgBqCuvjnbWHWiVPEwn) { FILE* 
QhTesEEIHwhNmHSeYbRR = MWCNNLayer::openBinaryFile(QMgBqCuvjnbWHWiVPEwn); 
assert(QhTesEEIHwhNmHSeYbRR); int dMxIKDGTITyhdLqIHBLA = 
BuyZFXzwOMxcePIbCLfl*BlRIQPyqJZORKENzSdYf;  float* KZWeXiYFmdpQdsgidKeG = 
MALLOC_CALL(sizeof(float)*dMxIKDGTITyhdLqIHBLA); call_fread(KZWeXiYFmdpQdsgidKeG, 
sizeof(float), dMxIKDGTITyhdLqIHBLA, QhTesEEIHwhNmHSeYbRR, QMgBqCuvjnbWHWiVPEwn); 
CUDA_CALL(cudaMemcpy(IwKnaBoXVubIRYcxEJLH, KZWeXiYFmdpQdsgidKeG, 
sizeof(float)*dMxIKDGTITyhdLqIHBLA, cudaMemcpyHostToDevice)); 
free(KZWeXiYFmdpQdsgidKeG); fclose(QhTesEEIHwhNmHSeYbRR); return; } void 
MWFusedConvReLULayerImpl::postSetup() { if (dJcdBfQQLhIAYHPxwQeg->getAutoTune()) 
{ getConvAlgoTuned(); } else { getConvAlgoWorkSpaceLimit(); } } void 
MWFusedConvReLULayerImpl::getConvAlgoTuned() { MWTensorBase* opTensorBase = 
getLayer()->getOutputTensor(0); MWTensor<float>* opTensor = 
static_cast<MWTensor<float>*>(opTensorBase); cudnnConvolutionFwdAlgoPerf_t 
perf_results[3]; cudnnTensorDescriptor_t* desc = 
getDescriptor(getLayer()->getOutputTensor()->getSourcePortIndex()); 
assert(desc); int returnedAlgoCount; 
CUDNN_CALL(cudnnFindConvolutionForwardAlgorithmEx(*dJcdBfQQLhIAYHPxwQeg->getCudnnHandle(), 
XYbzSmRQGatVJtGmDZSo, 
static_cast<MWTensor<float>*>(UEESbUvbMihFnquvuFij)->getData(), 
QVgVGfoCXYiYXzPhvVPX, vpXxoeEhdEosLSsYXkNG, NNhshzQGJHLSGjDiVerE, *desc, 
opTensor->getData(), 3, &returnedAlgoCount, &perf_results[0], 
dJcdBfQQLhIAYHPxwQeg->getWorkSpace(), 
*dJcdBfQQLhIAYHPxwQeg->getAllocatedWorkSpaceSize())); NDjzAZSYJuWymuKDNZYB = 
perf_results[0].algo; if (CUDNN_VERSION < 7402) fixConvAlgo(); } void 
MWFusedConvReLULayerImpl::getConvAlgoWorkSpaceLimit() { 
cudnnTensorDescriptor_t* desc = 
getDescriptor(getLayer()->getOutputTensor()->getSourcePortIndex()); assert(desc);
#if (CUDNN_MAJOR < 8)
 
CUDNN_CALL(cudnnGetConvolutionForwardAlgorithm(*dJcdBfQQLhIAYHPxwQeg->getCudnnHandle(), 
XYbzSmRQGatVJtGmDZSo, QVgVGfoCXYiYXzPhvVPX, NNhshzQGJHLSGjDiVerE, *desc, 
CUDNN_CONVOLUTION_FWD_SPECIFY_WORKSPACE_LIMIT, 
*dJcdBfQQLhIAYHPxwQeg->getAllocatedWorkSpaceSize(), &NDjzAZSYJuWymuKDNZYB));
#else
 int maxAlgoCount(-1); 
CUDNN_CALL(cudnnGetConvolutionForwardAlgorithmMaxCount(*dJcdBfQQLhIAYHPxwQeg->getCudnnHandle(), 
&maxAlgoCount)); int returnedAlgoCount(-1); 
std::vector<cudnnConvolutionFwdAlgoPerf_t> perf_results(maxAlgoCount);  
CUDNN_CALL(cudnnGetConvolutionForwardAlgorithm_v7(*dJcdBfQQLhIAYHPxwQeg->getCudnnHandle(), 
XYbzSmRQGatVJtGmDZSo, QVgVGfoCXYiYXzPhvVPX, NNhshzQGJHLSGjDiVerE, *desc, 
maxAlgoCount, &returnedAlgoCount, &perf_results[0])); 
cudnnConvolutionFwdAlgoPerf_t nextFastest; bool algoFound(false); for (int i = 
0; i < returnedAlgoCount; ++i) { nextFastest = perf_results[i]; if 
(nextFastest.memory <= *dJcdBfQQLhIAYHPxwQeg->getAllocatedWorkSpaceSize()) { 
NDjzAZSYJuWymuKDNZYB = nextFastest.algo; algoFound = true; break; } } assert(algoFound);
#endif
 if (CUDNN_VERSION < 7402) fixConvAlgo(); } void 
MWFusedConvReLULayerImpl::fixConvAlgo() { int inputH = 
UEESbUvbMihFnquvuFij->getHeight(); int inputW = UEESbUvbMihFnquvuFij->getWidth(); 
if (NDjzAZSYJuWymuKDNZYB == CUDNN_CONVOLUTION_FWD_ALGO_FFT && (inputH > 64 || 
inputW > 64)) { NDjzAZSYJuWymuKDNZYB = CUDNN_CONVOLUTION_FWD_ALGO_IMPLICIT_GEMM; 
} }