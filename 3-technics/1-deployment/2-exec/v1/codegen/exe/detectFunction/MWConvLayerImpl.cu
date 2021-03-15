#include "MWConvLayerImpl.hpp"
#include "MWConvLayer.hpp"
#include "MWTargetNetworkImpl.hpp"
#include "cnn_api.hpp"
#include <cassert>
#include <stdio.h>
 MWConvLayerImpl::MWConvLayerImpl(MWCNNLayer* layer, MWTargetNetworkImpl* 
ntwk_impl, int filt_H, int filt_W, int numGrps, int numChnls, int numFilts, int 
FrpxvsDMwwgbpqHXWxmN, int FwLnexHgxHRquTKmNpoa, int 
ClEhcJFlvGCgiavziIag, int CZNYmBcNFSZWvaCklqeM, int 
CufLFODQDXTAPyRqYodN, int DCdZnqpcBnvXVgEsLBnz, int 
AuqaQHxmPQSyYRemQvyX, int AzTsxYcYjIEJsGQbeYHm, const 
char* xHViLEwTujGGrPZZgmbF, const char* JwxFdqOKggeawILBfGgg) : 
MWCNNLayerImpl(layer, ntwk_impl) , vpXxoeEhdEosLSsYXkNG(NULL) , IwKnaBoXVubIRYcxEJLH(NULL) , 
TaAJDyqFVJXfAfCJhOuU(NULL) , BHuHNDGoRwGRouCxeMbw(filt_H) , BLjrjqvCcCommiXWQLjs 
(filt_W) , BuyZFXzwOMxcePIbCLfl (numGrps) , BkwhtPQUCQKchmmimoXs (numChnls) , 
BlRIQPyqJZORKENzSdYf (numFilts) , 
AdmgfUbRAfzFeYHxSnQr(AuqaQHxmPQSyYRemQvyX) , 
AwZQzUhuWVLGrWgLHRuM(AzTsxYcYjIEJsGQbeYHm) , 
CTCbzQMDaLxINPbODdng(ClEhcJFlvGCgiavziIag) , 
CLOUhPjbgggWoXHTtmjC(CZNYmBcNFSZWvaCklqeM) , 
CpMjJjtGOeWOzwxpAAQP(CufLFODQDXTAPyRqYodN) , 
CqtPRJvHlGJFssiPzsOm(DCdZnqpcBnvXVgEsLBnz) , 
FpguQZSermqZCMRiUfML(FrpxvsDMwwgbpqHXWxmN) , 
FshVHIJMRAhtQirYPlZd(FwLnexHgxHRquTKmNpoa) , 
IIiwAtyrOtLzLWAUlTey((CTCbzQMDaLxINPbODdng != CLOUhPjbgggWoXHTtmjC) 
|| (CpMjJjtGOeWOzwxpAAQP != CqtPRJvHlGJFssiPzsOm)) { dJcdBfQQLhIAYHPxwQeg = 
ntwk_impl; CUDNN_CALL(cudnnCreateConvolutionDescriptor(&NNhshzQGJHLSGjDiVerE)); 
CUDNN_CALL(cudnnCreateFilterDescriptor(&QVgVGfoCXYiYXzPhvVPX)); 
CUDNN_CALL(cudnnCreateTensorDescriptor(&JgLfgHrHMEMmMYTettJF));  MWConvLayer* 
convLayer = static_cast<MWConvLayer*>(getLayer()); MWTensorBase* ipTensor = 
convLayer->getInputTensor(0); if (IIiwAtyrOtLzLWAUlTey) { 
TaAJDyqFVJXfAfCJhOuU = new MWTensor<float>(-1, -1, -1, -1, -1, NULL, getLayer(), 0); 
if (!TaAJDyqFVJXfAfCJhOuU) { MWCNNLayerImpl::throwAllocationError(__LINE__ , 
__FILE__); } CUDNN_CALL(cudnnCreateTensorDescriptor(&XYbzSmRQGatVJtGmDZSo)); } 
else { TaAJDyqFVJXfAfCJhOuU = ipTensor; } assert(TaAJDyqFVJXfAfCJhOuU != NULL); int 
NXruhrCCiguRjAgSNDuz; int NZjOkZPwLzQsdEVkwMcX; if 
(IIiwAtyrOtLzLWAUlTey) { NXruhrCCiguRjAgSNDuz = 0;  
NZjOkZPwLzQsdEVkwMcX = 0; } else { NXruhrCCiguRjAgSNDuz = 
CTCbzQMDaLxINPbODdng; NZjOkZPwLzQsdEVkwMcX = CpMjJjtGOeWOzwxpAAQP; } 
bUVPfnrJhLfHzOLUUrKk = CTCbzQMDaLxINPbODdng; cCXqPFPPcoHzYMDpnUxQ = CpMjJjtGOeWOzwxpAAQP;
#if (CUDNN_MAJOR <= 5)
 { if ((AdmgfUbRAfzFeYHxSnQr != 1) && (AwZQzUhuWVLGrWgLHRuM != 1)){ 
printf("Dilated Convolution only supported for cuDNN 6 or greater "); throw 
std::runtime_error("Unsupported Dilation Factor"); } 
CUDNN_CALL(cudnnSetConvolution2dDescriptor(NNhshzQGJHLSGjDiVerE, 
NXruhrCCiguRjAgSNDuz, NZjOkZPwLzQsdEVkwMcX, FpguQZSermqZCMRiUfML, 
FshVHIJMRAhtQirYPlZd, 1, 1, CUDNN_CROSS_CORRELATION));  }
#else
 { CUDNN_CALL(cudnnSetConvolution2dDescriptor(NNhshzQGJHLSGjDiVerE, 
NXruhrCCiguRjAgSNDuz, NZjOkZPwLzQsdEVkwMcX, FpguQZSermqZCMRiUfML, 
FshVHIJMRAhtQirYPlZd, AdmgfUbRAfzFeYHxSnQr, AwZQzUhuWVLGrWgLHRuM, 
CUDNN_CROSS_CORRELATION, CUDNN_DATA_FLOAT)); }
#endif
#if (FP16_ENABLED == 1 && ( CUDNN_MAJOR > 7 || (CUDNN_MAJOR == 7 && CUDNN_MINOR >= 2) ))
 CUDNN_CALL(cudnnSetConvolutionMathType(NNhshzQGJHLSGjDiVerE, CUDNN_TENSOR_OP_MATH_ALLOW_CONVERSION));
#endif
 if (BuyZFXzwOMxcePIbCLfl > 1){ 
CUDNN_CALL(cudnnSetConvolutionGroupCount(NNhshzQGJHLSGjDiVerE, 
BuyZFXzwOMxcePIbCLfl)); } int etjQLJVQCaeAXRWYtqOl = 
BkwhtPQUCQKchmmimoXs*BuyZFXzwOMxcePIbCLfl; int fSKMHAqIghbYYgyIpNDw = 
BlRIQPyqJZORKENzSdYf*BuyZFXzwOMxcePIbCLfl; 
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
loadWeights(xHViLEwTujGGrPZZgmbF); loadBias(JwxFdqOKggeawILBfGgg); 
createAndAddDescriptor(getLayer()->getOutputTensor()->getSourcePortIndex()); } 
void MWConvLayerImpl::propagateSize() { MWTensorBase* ipTensor = 
getLayer()->getInputTensor(0); int inputH; int inputW; if 
(IIiwAtyrOtLzLWAUlTey) { inputH = ipTensor->getHeight() + 
CTCbzQMDaLxINPbODdng + CLOUhPjbgggWoXHTtmjC; inputW = ipTensor->getWidth() + 
CpMjJjtGOeWOzwxpAAQP + CqtPRJvHlGJFssiPzsOm; } else { inputH = 
ipTensor->getHeight(); inputW = ipTensor->getWidth(); } 
TaAJDyqFVJXfAfCJhOuU->setHeight(inputH); TaAJDyqFVJXfAfCJhOuU->setWidth(inputW); 
TaAJDyqFVJXfAfCJhOuU->setChannels(ipTensor->getChannels()); 
TaAJDyqFVJXfAfCJhOuU->setBatchSize(ipTensor->getBatchSize()); 
TaAJDyqFVJXfAfCJhOuU->setSequenceLength(ipTensor->getSequenceLength()); 
assert(TaAJDyqFVJXfAfCJhOuU->getSequenceLength() == 1); if 
(IIiwAtyrOtLzLWAUlTey) { 
CUDNN_CALL(cudnnSetTensor4dDescriptor(XYbzSmRQGatVJtGmDZSo, CUDNN_TENSOR_NCHW, 
CUDNN_DATA_FLOAT, TaAJDyqFVJXfAfCJhOuU->getBatchSize(), TaAJDyqFVJXfAfCJhOuU->getChannels(), 
TaAJDyqFVJXfAfCJhOuU->getHeight(), TaAJDyqFVJXfAfCJhOuU->getWidth())); } else { 
XYbzSmRQGatVJtGmDZSo = MWCNNLayerImpl::getCuDNNDescriptor(TaAJDyqFVJXfAfCJhOuU); } 
MWTensorBase* opTensor = getLayer()->getOutputTensor(0); 
cudnnTensorDescriptor_t* desc = getDescriptor(opTensor->getSourcePortIndex()); 
assert(desc); setDescriptor<float>(*desc, static_cast<MWTensor<float>*>(opTensor));
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
 size_t tnTPxeDjBsqLAPkJcPJX = 0; 
CUDNN_CALL(cudnnGetConvolutionForwardWorkspaceSize( 
*dJcdBfQQLhIAYHPxwQeg->getCudnnHandle(), XYbzSmRQGatVJtGmDZSo, 
QVgVGfoCXYiYXzPhvVPX, NNhshzQGJHLSGjDiVerE, *desc, NDjzAZSYJuWymuKDNZYB, 
&tnTPxeDjBsqLAPkJcPJX)); if (tnTPxeDjBsqLAPkJcPJX > 
*dJcdBfQQLhIAYHPxwQeg->getProposedWorkSpaceSize()) { 
dJcdBfQQLhIAYHPxwQeg->setProposedWorkSpaceSize(tnTPxeDjBsqLAPkJcPJX); } } void 
MWConvLayerImpl::allocate() { MWTensorBase* ipTensor = 
getLayer()->getInputTensor(0); if (IIiwAtyrOtLzLWAUlTey) { float* 
newInput; int inputH = ipTensor->getHeight() + CTCbzQMDaLxINPbODdng + 
CLOUhPjbgggWoXHTtmjC; int inputW = ipTensor->getWidth() + 
CpMjJjtGOeWOzwxpAAQP + CqtPRJvHlGJFssiPzsOm; int paddedSize = 
ipTensor->getBatchSize() * ipTensor->getChannels() * inputH * inputW; 
CUDA_CALL(cudaMalloc((void**)&newInput, sizeof(float)*paddedSize)); 
CUDA_CALL(cudaMemset(newInput, 0, sizeof(float)*paddedSize)); 
static_cast<MWTensor<float>*>(TaAJDyqFVJXfAfCJhOuU)->setData(newInput); } } void 
MWConvLayerImpl::deallocate() { if (TaAJDyqFVJXfAfCJhOuU != 
getLayer()->getInputTensor(0)) { assert(IIiwAtyrOtLzLWAUlTey); 
CUDA_FREE_CALL(static_cast<MWTensor<float>*>(TaAJDyqFVJXfAfCJhOuU)->getData()); 
static_cast<MWTensor<float>*>(TaAJDyqFVJXfAfCJhOuU)->setData((float*)NULL); } } void 
MWConvLayerImpl::predict() { MWConvLayer* convLayer = 
static_cast<MWConvLayer*>(getLayer()); MWTensorBase* ipTensorBase = 
convLayer->getInputTensor(); MWTensorBase* opTensorBase = 
convLayer->getOutputTensor(); MWTensor<float>* ipTensor = 
static_cast<MWTensor<float>*>(ipTensorBase); MWTensor<float>* opTensor = 
static_cast<MWTensor<float>*>(opTensorBase); if (TaAJDyqFVJXfAfCJhOuU != 
convLayer->getInputTensor()) { 
CUDA_CALL(cudaMemset(static_cast<MWTensor<float>*>(TaAJDyqFVJXfAfCJhOuU)->getData(), 
0, sizeof(float)*TaAJDyqFVJXfAfCJhOuU->getNumElements()));  
MWCNNLayerImpl::padInput(ipTensor->getData(), ipTensor->getHeight(), 
ipTensor->getWidth(), ipTensor->getChannels(), TaAJDyqFVJXfAfCJhOuU->getHeight(), 
TaAJDyqFVJXfAfCJhOuU->getWidth(), bUVPfnrJhLfHzOLUUrKk, cCXqPFPPcoHzYMDpnUxQ, 
static_cast<MWTensor<float>*>(TaAJDyqFVJXfAfCJhOuU)->getData(), 
ipTensor->getNumElements()); } assert(opTensor->getData() != 
static_cast<MWTensor<float>*>(TaAJDyqFVJXfAfCJhOuU)->getData()); 
cudnnTensorDescriptor_t* desc = getDescriptor(opTensor->getSourcePortIndex()); 
assert(desc); 
CUDNN_CALL(cudnnConvolutionForward(*dJcdBfQQLhIAYHPxwQeg->getCudnnHandle(), 
getOnePtr(), XYbzSmRQGatVJtGmDZSo, 
static_cast<MWTensor<float>*>(TaAJDyqFVJXfAfCJhOuU)->getData(), QVgVGfoCXYiYXzPhvVPX, 
vpXxoeEhdEosLSsYXkNG, NNhshzQGJHLSGjDiVerE, NDjzAZSYJuWymuKDNZYB, 
dJcdBfQQLhIAYHPxwQeg->getWorkSpace(), 
*dJcdBfQQLhIAYHPxwQeg->getAllocatedWorkSpaceSize(), getZeroPtr(), *desc, 
opTensor->getData())); 
CUDNN_CALL(cudnnAddTensor(*dJcdBfQQLhIAYHPxwQeg->getCudnnHandle(), getOnePtr(), 
JgLfgHrHMEMmMYTettJF, IwKnaBoXVubIRYcxEJLH, getOnePtr(), *desc, opTensor->getData())); } 
void MWConvLayerImpl::cleanup() { 
CUDNN_CALL(cudnnDestroyConvolutionDescriptor(NNhshzQGJHLSGjDiVerE)); 
CUDNN_CALL(cudnnDestroyFilterDescriptor(QVgVGfoCXYiYXzPhvVPX)); if 
(vpXxoeEhdEosLSsYXkNG) { CUDA_FREE_CALL(vpXxoeEhdEosLSsYXkNG); vpXxoeEhdEosLSsYXkNG = NULL; } 
CUDNN_CALL(cudnnDestroyTensorDescriptor(JgLfgHrHMEMmMYTettJF)); if 
(IwKnaBoXVubIRYcxEJLH) { CUDA_FREE_CALL(IwKnaBoXVubIRYcxEJLH); IwKnaBoXVubIRYcxEJLH = NULL; } if 
(TaAJDyqFVJXfAfCJhOuU != getLayer()->getInputTensor(0)) { 
assert(IIiwAtyrOtLzLWAUlTey); 
CUDNN_CALL(cudnnDestroyTensorDescriptor(XYbzSmRQGatVJtGmDZSo)); } } void 
MWConvLayerImpl::loadWeights(const char* QMgBqCuvjnbWHWiVPEwn) { MWConvLayer* 
convLayer = static_cast<MWConvLayer*>(getLayer()); FILE* QhTesEEIHwhNmHSeYbRR = 
MWCNNLayer::openBinaryFile(QMgBqCuvjnbWHWiVPEwn); assert(QhTesEEIHwhNmHSeYbRR); int 
dMxIKDGTITyhdLqIHBLA = 
BkwhtPQUCQKchmmimoXs*BlRIQPyqJZORKENzSdYf*BuyZFXzwOMxcePIbCLfl*BHuHNDGoRwGRouCxeMbw*BLjrjqvCcCommiXWQLjs; 
 float* KZWeXiYFmdpQdsgidKeG = MALLOC_CALL(sizeof(float)*dMxIKDGTITyhdLqIHBLA); 
call_fread(KZWeXiYFmdpQdsgidKeG, sizeof(float), dMxIKDGTITyhdLqIHBLA, QhTesEEIHwhNmHSeYbRR, 
QMgBqCuvjnbWHWiVPEwn); CUDA_CALL(cudaMemcpy(vpXxoeEhdEosLSsYXkNG, KZWeXiYFmdpQdsgidKeG, 
sizeof(float)*dMxIKDGTITyhdLqIHBLA, cudaMemcpyHostToDevice)); fclose(QhTesEEIHwhNmHSeYbRR); 
free(KZWeXiYFmdpQdsgidKeG); } void MWConvLayerImpl::loadBias(const char* 
QMgBqCuvjnbWHWiVPEwn) { MWConvLayer* convLayer = 
static_cast<MWConvLayer*>(getLayer()); FILE* QhTesEEIHwhNmHSeYbRR = 
MWCNNLayer::openBinaryFile(QMgBqCuvjnbWHWiVPEwn);  assert(QhTesEEIHwhNmHSeYbRR); int 
dMxIKDGTITyhdLqIHBLA = BlRIQPyqJZORKENzSdYf*BuyZFXzwOMxcePIbCLfl;  float* 
KZWeXiYFmdpQdsgidKeG = MALLOC_CALL(sizeof(float)*dMxIKDGTITyhdLqIHBLA); 
call_fread(KZWeXiYFmdpQdsgidKeG, sizeof(float), dMxIKDGTITyhdLqIHBLA, QhTesEEIHwhNmHSeYbRR, 
QMgBqCuvjnbWHWiVPEwn); CUDA_CALL(cudaMemcpy(IwKnaBoXVubIRYcxEJLH, KZWeXiYFmdpQdsgidKeG, 
sizeof(float)*dMxIKDGTITyhdLqIHBLA, cudaMemcpyHostToDevice)); 
free(KZWeXiYFmdpQdsgidKeG); fclose(QhTesEEIHwhNmHSeYbRR); } void 
MWConvLayerImpl::postSetup() { if (dJcdBfQQLhIAYHPxwQeg->getAutoTune()) { 
getConvAlgoTuned(); } else { getConvAlgoWorkSpaceLimit(); } } void 
MWConvLayerImpl::getConvAlgoTuned() { MWTensorBase* opTensorBase = 
getLayer()->getOutputTensor(0); MWTensor<float>* opTensor = 
static_cast<MWTensor<float>*>(opTensorBase); cudnnConvolutionFwdAlgoPerf_t 
perf_results[3]; cudnnTensorDescriptor_t* desc = 
getDescriptor(getLayer()->getOutputTensor()->getSourcePortIndex()); 
assert(desc); int returnedAlgoCount; 
CUDNN_CALL(cudnnFindConvolutionForwardAlgorithmEx(*dJcdBfQQLhIAYHPxwQeg->getCudnnHandle(), 
XYbzSmRQGatVJtGmDZSo, static_cast<MWTensor<float>*>(TaAJDyqFVJXfAfCJhOuU)->getData(), 
QVgVGfoCXYiYXzPhvVPX, vpXxoeEhdEosLSsYXkNG, NNhshzQGJHLSGjDiVerE, *desc, 
opTensor->getData(), 3, &returnedAlgoCount, &perf_results[0], 
dJcdBfQQLhIAYHPxwQeg->getWorkSpace(), 
*dJcdBfQQLhIAYHPxwQeg->getAllocatedWorkSpaceSize())); NDjzAZSYJuWymuKDNZYB = 
perf_results[0].algo; } void MWConvLayerImpl::getConvAlgoWorkSpaceLimit() { 
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
 }