#include <cstdlib>
#include <cassert>
#include <stdio.h>
#include <stdexcept>
#include "MWCNNLayerImpl.hpp"
#include "MWTargetNetworkImpl.hpp"
#include "cnn_api.hpp"
#ifdef RANDOM
#include <curand.h>
 curandGenerator_t RAtlBpdedvgxUsgDTsch; void 
curand_call_line_file(curandStatus_t rlQsibXJSWJVnUVpdNeL, const int 
atVCyzqXZAZxwlkRLBRA, const char *QMNXyOvXaZDsCpiIJPsn) { if (rlQsibXJSWJVnUVpdNeL != 
CURAND_STATUS_SUCCESS) { char buffer[100]; int numElem = sprintf(buffer, 
"%d at line: %d, file: %s\n", rlQsibXJSWJVnUVpdNeL, atVCyzqXZAZxwlkRLBRA, 
QMNXyOvXaZDsCpiIJPsn); throw std::runtime_error(buffer); } }
#endif
 float* malloc_call_line_file(size_t msize, const int atVCyzqXZAZxwlkRLBRA, const 
char *QMNXyOvXaZDsCpiIJPsn) { float * mem = (float*)malloc(msize); if (!mem) { char 
buffer[100]; int numElem = sprintf(buffer, "%s at line: %d, file: %s\n", 
"Memory allocation failed. ", atVCyzqXZAZxwlkRLBRA, QMNXyOvXaZDsCpiIJPsn); throw 
std::runtime_error(buffer); } return mem; } void 
cuda_call_line_file(cudaError_t rlQsibXJSWJVnUVpdNeL, const int atVCyzqXZAZxwlkRLBRA, 
const char *QMNXyOvXaZDsCpiIJPsn) { if (rlQsibXJSWJVnUVpdNeL != cudaSuccess) { 
throw_cuda_error(rlQsibXJSWJVnUVpdNeL, atVCyzqXZAZxwlkRLBRA, QMNXyOvXaZDsCpiIJPsn);  } } 
void throw_cuda_error(cudaError_t rlQsibXJSWJVnUVpdNeL, const int atVCyzqXZAZxwlkRLBRA, 
const char *QMNXyOvXaZDsCpiIJPsn) { char buffer[100]; int numElem = sprintf(buffer, 
"Cuda Error %d(%s) at line: %d, file: %s\n", rlQsibXJSWJVnUVpdNeL, 
cudaGetErrorString(rlQsibXJSWJVnUVpdNeL), atVCyzqXZAZxwlkRLBRA, QMNXyOvXaZDsCpiIJPsn); 
rlQsibXJSWJVnUVpdNeL = cudaGetLastError();  throw std::runtime_error(buffer);  } 
void cudnn_call_line_file(cudnnStatus_t rlQsibXJSWJVnUVpdNeL, const int 
atVCyzqXZAZxwlkRLBRA, const char *QMNXyOvXaZDsCpiIJPsn) { if (rlQsibXJSWJVnUVpdNeL != 
CUDNN_STATUS_SUCCESS) { char buffer[100]; int numElem = sprintf(buffer, 
"CuDNN Error %d(%s) at line: %d, file: %s\n", rlQsibXJSWJVnUVpdNeL, 
cudnnGetErrorString(rlQsibXJSWJVnUVpdNeL), atVCyzqXZAZxwlkRLBRA, QMNXyOvXaZDsCpiIJPsn); 
throw std::runtime_error(buffer); } } const char* 
cublasGetErrorString(cublasStatus_t rlQsibXJSWJVnUVpdNeL) { 
switch(rlQsibXJSWJVnUVpdNeL) { case CUBLAS_STATUS_SUCCESS: return 
"CUBLAS_STATUS_SUCCESS"; case CUBLAS_STATUS_NOT_INITIALIZED: return 
"CUBLAS_STATUS_NOT_INITIALIZED"; case CUBLAS_STATUS_ALLOC_FAILED: return 
"CUBLAS_STATUS_ALLOC_FAILED"; case CUBLAS_STATUS_INVALID_VALUE: return 
"CUBLAS_STATUS_INVALID_VALUE";  case CUBLAS_STATUS_ARCH_MISMATCH: return 
"CUBLAS_STATUS_ARCH_MISMATCH";  case CUBLAS_STATUS_MAPPING_ERROR: return 
"CUBLAS_STATUS_MAPPING_ERROR"; case CUBLAS_STATUS_EXECUTION_FAILED: return 
"CUBLAS_STATUS_EXECUTION_FAILED";  case CUBLAS_STATUS_INTERNAL_ERROR: return 
"CUBLAS_STATUS_INTERNAL_ERROR";  case CUBLAS_STATUS_NOT_SUPPORTED: return 
"CUBLAS_STATUS_NOT_SUPPORTED";  case CUBLAS_STATUS_LICENSE_ERROR: return 
"CUBLAS_STATUS_LICENSE_ERROR";  } return "unknown error"; } void 
cublas_call_line_file(cublasStatus_t rlQsibXJSWJVnUVpdNeL, const int 
atVCyzqXZAZxwlkRLBRA, const char *QMNXyOvXaZDsCpiIJPsn) { if (rlQsibXJSWJVnUVpdNeL != 
CUBLAS_STATUS_SUCCESS) { char buffer[100]; int numElem = sprintf(buffer, 
"CuBlas Error %d(%s) at line: %d, file: %s\n", rlQsibXJSWJVnUVpdNeL, 
cublasGetErrorString(rlQsibXJSWJVnUVpdNeL), atVCyzqXZAZxwlkRLBRA, QMNXyOvXaZDsCpiIJPsn); 
throw std::runtime_error(buffer); } } 
MWCNNLayerImpl::MWCNNLayerImpl(MWCNNLayer* layer, MWTargetNetworkImpl* 
ntwk_impl) : PVBPDNaynqYkBlDZgXgj(0.0), OwortPcLToImGdYFtbSF(1.0), 
OumvfgWXDdmsQaciHMHx(-1.0), aPzBTLIjCXEQZUlbxayX(layer), 
dJcdBfQQLhIAYHPxwQeg(ntwk_impl) { } MWCNNLayerImpl::~MWCNNLayerImpl() { 
for(std::map<int, cudnnTensorDescriptor_t*>::iterator it = 
kqftrrQBBOgGsrDSkIUk.begin(); it != kqftrrQBBOgGsrDSkIUk.end(); ++it) { 
CUDNN_CALL(cudnnDestroyTensorDescriptor(*it->second)); delete it->second; 
it->second = 0; } } template <class T> void 
MWCNNLayerImpl::allocateOutputData(int outIdx) { MWTensorBase* opTensorBase = 
getLayer()->getOutputTensor(outIdx); bool bufferReuse = 
opTensorBase->getopBufIndex() >= 0; if (bufferReuse) { 
assert(opTensorBase->isFloat()); MWTensor<float>* opTensor = 
static_cast<MWTensor<float>*>(opTensorBase); 
opTensor->setData(dJcdBfQQLhIAYHPxwQeg->memBuffer[opTensor->getopBufIndex()]); } 
else { int inIdx = getLayer()->getInPlaceIndex(outIdx); if (inIdx != -1) { 
MWTensor<T>* ipTensor = 
static_cast<MWTensor<T>*>(getLayer()->getInputTensor(inIdx)); MWTensor<T>* 
opTensor = static_cast<MWTensor<T>*>(opTensorBase); T* ipData = 
ipTensor->getData(); assert(ipData); opTensor->setData(ipData); } else { 
MWTensor<T>* opTensor = static_cast<MWTensor<T>*>(opTensorBase); T* 
OAKPrVDonUthXHZkRzEc;  CUDA_CALL(cudaMalloc((void**)&OAKPrVDonUthXHZkRzEc, 
sizeof(T)*opTensor->getNumElements())); opTensor->setData(OAKPrVDonUthXHZkRzEc); } } 
} template void MWCNNLayerImpl::allocateOutputData<float>(int); template void 
MWCNNLayerImpl::allocateOutputData<signed char>(int); template <class T> void 
MWCNNLayerImpl::deallocateOutputData(int outIdx) { 
if(getLayer()->getInPlaceIndex(outIdx) == -1) { MWTensor<T>* opTensor = 
static_cast<MWTensor<T>*>(getLayer()->getOutputTensor(outIdx)); T* data = 
opTensor->getData(); CUDA_FREE_CALL(data); } } template void 
MWCNNLayerImpl::deallocateOutputData<float>(int); template void 
MWCNNLayerImpl::deallocateOutputData<signed char>(int); float* 
MWCNNLayerImpl::getZeroPtr() { return &PVBPDNaynqYkBlDZgXgj; } float* 
MWCNNLayerImpl::getOnePtr() { return &OwortPcLToImGdYFtbSF; } float* 
MWCNNLayerImpl::getNegOnePtr() { return &OumvfgWXDdmsQaciHMHx; } 
cudnnTensorDescriptor_t* MWCNNLayerImpl::createAndAddDescriptor(int index) { 
std::map<int, cudnnTensorDescriptor_t*>::iterator it = 
kqftrrQBBOgGsrDSkIUk.find(index); assert(it == kqftrrQBBOgGsrDSkIUk.end()); 
cudnnTensorDescriptor_t* newDescriptor = new cudnnTensorDescriptor_t; if 
(!newDescriptor) { MWCNNLayerImpl::throwAllocationError(__LINE__ , __FILE__); } 
kqftrrQBBOgGsrDSkIUk[index] = newDescriptor; 
CUDNN_CALL(cudnnCreateTensorDescriptor(newDescriptor)); return newDescriptor; } 
cudnnTensorDescriptor_t* MWCNNLayerImpl::getDescriptor(int index) {  
std::map<int, cudnnTensorDescriptor_t*>::iterator it = 
kqftrrQBBOgGsrDSkIUk.find(index); if (it != kqftrrQBBOgGsrDSkIUk.end()) { 
return it->second; } else { return NULL; } } template <class T> void 
MWCNNLayerImpl::setDescriptor(cudnnTensorDescriptor_t& desc, MWTensor<T>* 
tensor) { if (tensor->getSequenceLength() == 1) { 
CUDNN_CALL(cudnnSetTensor4dDescriptor(desc, CUDNN_TENSOR_NCHW, 
MWCNNLayerImpl::getCuDNNDataType<T>(), tensor->getBatchSize(),  
tensor->getChannels(),  tensor->getHeight(),  tensor->getWidth()));  } else { 
int dims[5] = {tensor->getSequenceLength(), tensor->getBatchSize(), 
tensor->getChannels(), tensor->getHeight(), tensor->getWidth()}; int 
strides[5]; MWTensorBase::getStrides(dims, 5, strides); 
CUDNN_CALL(cudnnSetTensorNdDescriptor(desc, 
MWCNNLayerImpl::getCuDNNDataType<T>(), 5, dims, strides)); }  } template void 
MWCNNLayerImpl::setDescriptor<float>(cudnnTensorDescriptor_t&, 
MWTensor<float>*); template void MWCNNLayerImpl::setDescriptor<signed 
char>(cudnnTensorDescriptor_t&, MWTensor<signed char>*); template <> 
cudnnDataType_t MWCNNLayerImpl::getCuDNNDataType<float>() { return 
CUDNN_DATA_FLOAT; } template <> cudnnDataType_t 
MWCNNLayerImpl::getCuDNNDataType<signed char>() { return CUDNN_DATA_INT8; } 
cudnnTensorDescriptor_t MWCNNLayerImpl::getCuDNNDescriptor(MWTensorBase* 
tensor) { MWCNNLayer* layer = tensor->getOwner(); MWCNNLayerImpl* impl = 
layer->getImpl(); if(impl) { cudnnTensorDescriptor_t* desc = 
impl->getDescriptor(tensor->getSourcePortIndex()); if (desc == NULL) { 
impl->createAndAddDescriptor(tensor->getSourcePortIndex()); desc = 
impl->getDescriptor(tensor->getSourcePortIndex()); assert(desc);  } if 
(tensor->isFloat()) { MWCNNLayerImpl::setDescriptor<float>(*desc, 
static_cast<MWTensor<float>*>(tensor)); } else { assert(tensor->isInt8()); 
MWCNNLayerImpl::setDescriptor<signed char>(*desc, static_cast<MWTensor<signed 
char>*>(tensor)); } return *desc; } else { cudnnTensorDescriptor_t 
tmpDescriptor; CUDNN_CALL(cudnnCreateTensorDescriptor(&tmpDescriptor)); if 
(tensor->isFloat()) { MWCNNLayerImpl::setDescriptor<float>(tmpDescriptor, 
static_cast<MWTensor<float>*>(tensor)); } else { assert(tensor->isInt8()); 
MWCNNLayerImpl::setDescriptor<signed char>(tmpDescriptor, 
static_cast<MWTensor<signed char>*>(tensor)); } return tmpDescriptor; } } void 
__global__ __launch_bounds__(1024) padInputImpl(float* in, int inputH, int 
inputW, int inputCh, int outputH, int outputW, int offsetH, int offsetW, float* 
out, int inputElems) { for(int i = blockDim.x * blockIdx.x + threadIdx.x; i < 
inputElems; i+= blockDim.x*gridDim.x) { int idxB = i/(inputH*inputW*inputCh); 
int rem = (i - idxB*(inputH*inputW*inputCh)); int idxCh = rem/(inputH*inputW); 
int rem1 = rem - idxCh*(inputH*inputW); int idxH = rem1/inputW; int idxCol = 
rem1 - idxH*inputW; if ((idxH < inputH) && (idxCol < inputW)) { int outputR = 
idxH + offsetH; int outputCol = idxCol + offsetW; int outputCh = inputCh; 
out[idxB*(outputH*outputW*outputCh) + idxCh*(outputH*outputW) + 
outputR*(outputW) + outputCol] = in[i]; } } } void 
MWCNNLayerImpl::padInput(float* TaAJDyqFVJXfAfCJhOuU, int VFKMunbyHoAmpHUSkuUn, int 
WIxRBCJtmETvfxpuRuus, int VCbcPxtPsBLTrHYdEvqn, int lHtftnmGBvlSSoGOXVui, int 
lkGLRakytrdNuJCcpYWt, int gTcJMwtYuwiqqUmqvKhT, int gzSTokDHvkXefhiGDcWL, float* 
jmcFOAbZArjGDNhshSro, int enPbWLzEmxYCBmzGJutZ) { int tGsvtyAVkrDznETdweDC = 
(enPbWLzEmxYCBmzGJutZ + 31)/32 * 32; tGsvtyAVkrDznETdweDC = 
(tGsvtyAVkrDznETdweDC < 1024) ? tGsvtyAVkrDznETdweDC : 1024; int 
KHClOltUSuqFVVErSxVb = (enPbWLzEmxYCBmzGJutZ + tGsvtyAVkrDznETdweDC - 
1)/tGsvtyAVkrDznETdweDC; padInputImpl<<<KHClOltUSuqFVVErSxVb, 
tGsvtyAVkrDznETdweDC>>>(TaAJDyqFVJXfAfCJhOuU, VFKMunbyHoAmpHUSkuUn, 
WIxRBCJtmETvfxpuRuus, VCbcPxtPsBLTrHYdEvqn, lHtftnmGBvlSSoGOXVui, lkGLRakytrdNuJCcpYWt, 
gTcJMwtYuwiqqUmqvKhT, gzSTokDHvkXefhiGDcWL, jmcFOAbZArjGDNhshSro, enPbWLzEmxYCBmzGJutZ); } 
void __global__ __launch_bounds__(1024) fillOutputBufferImpl(signed char* in, 
int inputH, int inputW, int inputCh, int outputH, int outputW, int offsetH, int 
offsetW, signed char* out, int inputElems, int outputCh) { for(int i = 
blockDim.x * blockIdx.x + threadIdx.x; i < inputElems; i+= 
blockDim.x*gridDim.x) { int idxB = i/(inputH*inputW*inputCh); int rem = (i - 
idxB*(inputH*inputW*inputCh)); int idxCh = rem/(inputH*inputW); int rem1 = rem 
- idxCh*(inputH*inputW); int idxH = rem1/inputW; int idxCol = rem1 - 
idxH*inputW; if ((idxH < inputH) && (idxCol < inputW)) { int outputR = idxH + 
offsetH; int outputCol = idxCol + offsetW; *(out + 
idxB*(outputH*outputW*outputCh) + idxCh*(outputH*outputW) + outputR*(outputW) + 
outputCol) = *(in + i); } } } void MWCNNLayerImpl::fillOutputBuffer(signed 
char* TaAJDyqFVJXfAfCJhOuU, int VFKMunbyHoAmpHUSkuUn, int WIxRBCJtmETvfxpuRuus, int 
VCbcPxtPsBLTrHYdEvqn, int lHtftnmGBvlSSoGOXVui, int lkGLRakytrdNuJCcpYWt, int 
gTcJMwtYuwiqqUmqvKhT, int gzSTokDHvkXefhiGDcWL, signed char* jmcFOAbZArjGDNhshSro, int 
enPbWLzEmxYCBmzGJutZ, int kkqTyvjYvRFtTOyQUwrF) { int tGsvtyAVkrDznETdweDC 
= (enPbWLzEmxYCBmzGJutZ < 1024) ? enPbWLzEmxYCBmzGJutZ : 1024; int 
KHClOltUSuqFVVErSxVb = (enPbWLzEmxYCBmzGJutZ + tGsvtyAVkrDznETdweDC - 
1)/tGsvtyAVkrDznETdweDC; fillOutputBufferImpl<<<KHClOltUSuqFVVErSxVb, 
tGsvtyAVkrDznETdweDC>>>(TaAJDyqFVJXfAfCJhOuU, VFKMunbyHoAmpHUSkuUn, 
WIxRBCJtmETvfxpuRuus, VCbcPxtPsBLTrHYdEvqn, lHtftnmGBvlSSoGOXVui, lkGLRakytrdNuJCcpYWt, 
gTcJMwtYuwiqqUmqvKhT, gzSTokDHvkXefhiGDcWL, jmcFOAbZArjGDNhshSro, enPbWLzEmxYCBmzGJutZ, 
kkqTyvjYvRFtTOyQUwrF); } void MWCNNLayerImpl::throwAllocationError(const int 
line, const char * file) { char buffer[200]; int numElem = sprintf(buffer, 
"Failed to allocate memory at %d, file %s\n", line, file); throw 
std::runtime_error(buffer); } MWReLULayerImpl::MWReLULayerImpl(MWCNNLayer* 
layer, MWTargetNetworkImpl* ntwk_impl)  : MWCNNLayerImpl(layer, ntwk_impl) { 
CUDNN_CALL(cudnnCreateActivationDescriptor(&oJUVMnJggjhEdQLWzIUC)); 
createAndAddDescriptor(getLayer()->getOutputTensor(0)->getSourcePortIndex()); } 
MWReLULayerImpl::~MWReLULayerImpl() { } void MWReLULayerImpl::propagateSize() { 
MWTensorBase* opTensor = getLayer()->getOutputTensor(0); 
cudnnTensorDescriptor_t* desc = getDescriptor(opTensor->getSourcePortIndex()); 
assert(desc); setDescriptor<float>(*desc, 
static_cast<MWTensor<float>*>(opTensor)); 
CUDNN_CALL(cudnnSetActivationDescriptor(oJUVMnJggjhEdQLWzIUC, 
CUDNN_ACTIVATION_RELU, CUDNN_NOT_PROPAGATE_NAN, 0));  } void 
MWReLULayerImpl::predict() { MWReLULayer* reluLayer = 
static_cast<MWReLULayer*>(getLayer()); MWTensorBase* ipTensorBase = 
reluLayer->getInputTensor(0); MWTensorBase* opTensorBase = 
reluLayer->getOutputTensor(0); MWTensor<float>* ipTensor = 
static_cast<MWTensor<float>*>(ipTensorBase); MWTensor<float>* opTensor = 
static_cast<MWTensor<float>*>(opTensorBase); cudnnTensorDescriptor_t* desc = 
getDescriptor(opTensor->getSourcePortIndex()); assert(desc); 
cudnnTensorDescriptor_t ipDesc = 
MWCNNLayerImpl::getCuDNNDescriptor(ipTensorBase); 
CUDNN_CALL(cudnnActivationForward(*dJcdBfQQLhIAYHPxwQeg->getCudnnHandle(), 
oJUVMnJggjhEdQLWzIUC, getOnePtr(), ipDesc, ipTensor->getData(), getZeroPtr(), 
*desc, opTensor->getData())); } void MWReLULayerImpl::cleanup() { 
CUDNN_CALL(cudnnDestroyActivationDescriptor(oJUVMnJggjhEdQLWzIUC)); } 
MWNormLayerImpl::MWNormLayerImpl(MWCNNLayer* layer, MWTargetNetworkImpl* 
ntwk_impl, unsigned GDRXdUDklKFEYEfifhIH,  double AFQBkxwYGKLsACiDKwRM,  
double AHqhysOOIgbDpWZoPUFT,  double BUOdotSvmFyUWQKMUdra) : MWCNNLayerImpl(layer, 
ntwk_impl)  { CUDNN_CALL(cudnnCreateLRNDescriptor(&dAGMlbhOYuZqhuDGCqih)); 
createAndAddDescriptor(getLayer()->getOutputTensor(0)->getSourcePortIndex()); 
CUDNN_CALL(cudnnSetLRNDescriptor(dAGMlbhOYuZqhuDGCqih, 
GDRXdUDklKFEYEfifhIH, AFQBkxwYGKLsACiDKwRM, AHqhysOOIgbDpWZoPUFT, 
BUOdotSvmFyUWQKMUdra)); } MWNormLayerImpl::~MWNormLayerImpl() { } void 
MWNormLayerImpl::propagateSize() { MWTensorBase* opTensor = 
getLayer()->getOutputTensor(0); cudnnTensorDescriptor_t* desc = 
getDescriptor(opTensor->getSourcePortIndex()); assert(desc); 
setDescriptor<float>(*desc, static_cast<MWTensor<float>*>(opTensor));  } void 
MWNormLayerImpl::predict() { MWNormLayer* normLayer = 
static_cast<MWNormLayer*>(getLayer()); MWTensorBase* ipTensorBase = 
normLayer->getInputTensor();  MWTensorBase* opTensorBase = 
normLayer->getOutputTensor(); MWTensor<float>* ipTensor = 
static_cast<MWTensor<float>*>(ipTensorBase); MWTensor<float>* opTensor = 
static_cast<MWTensor<float>*>(opTensorBase); cudnnTensorDescriptor_t* desc = 
getDescriptor(opTensor->getSourcePortIndex()); assert(desc); 
cudnnTensorDescriptor_t ipDesc = 
MWCNNLayerImpl::getCuDNNDescriptor(ipTensorBase); 
CUDNN_CALL(cudnnLRNCrossChannelForward(*dJcdBfQQLhIAYHPxwQeg->getCudnnHandle(), 
dAGMlbhOYuZqhuDGCqih, CUDNN_LRN_CROSS_CHANNEL_DIM1, getOnePtr(), ipDesc, 
ipTensor->getData(), getZeroPtr(), *desc, opTensor->getData())); } void 
MWNormLayerImpl::cleanup() { 
CUDNN_CALL(cudnnDestroyLRNDescriptor(dAGMlbhOYuZqhuDGCqih)); } void __global__ 
MWSetDyForBackPropImpl(float * PQjbchiGbyJfmpiqPpOC, const int fDqxEdcpBDmVQxZEmQxm); 
void __global__ doMWMaxPoolingLayerImpl(float * UWAGLbDcvybdWBtshhsr, 
float * UVzBVEOIylFjkSgHwFMp, const int BdqURaHPmdnfzvtUvocl); 
MWMaxPoolingLayerImpl::MWMaxPoolingLayerImpl(MWCNNLayer* layer, 
MWTargetNetworkImpl* ntwk_impl, int DSsxcjIrUgZCKZovyNQf,  int 
EfvWctmlsWAPsxXgdKWf,  int FrpxvsDMwwgbpqHXWxmN,  int 
FwLnexHgxHRquTKmNpoa, int ClEhcJFlvGCgiavziIag, int 
CZNYmBcNFSZWvaCklqeM,  int CufLFODQDXTAPyRqYodN, int 
DCdZnqpcBnvXVgEsLBnz, bool GIbahSoBBDrvvZduPEqU, int fOpFYwKNwIfWjnPzNuob) 
: MWCNNLayerImpl(layer, ntwk_impl) , 
BRSPqxNffoBYKqpSVHne(GIbahSoBBDrvvZduPEqU) , UWAGLbDcvybdWBtshhsr(0) 
, PQjbchiGbyJfmpiqPpOC(0) , DRzwhbNPpftRRIXXfHzd(DSsxcjIrUgZCKZovyNQf) , 
ECTnqgWHyHCHCLBZlffd(EfvWctmlsWAPsxXgdKWf) , 
DGzdAcREJHGXjyRzNjJV(DSsxcjIrUgZCKZovyNQf) , 
DqxLTLaJwwgQqmrtCDuu(EfvWctmlsWAPsxXgdKWf) , 
CTCbzQMDaLxINPbODdng(ClEhcJFlvGCgiavziIag) , 
CLOUhPjbgggWoXHTtmjC(CZNYmBcNFSZWvaCklqeM) , 
CpMjJjtGOeWOzwxpAAQP(CufLFODQDXTAPyRqYodN) , 
CqtPRJvHlGJFssiPzsOm(DCdZnqpcBnvXVgEsLBnz) , 
FpguQZSermqZCMRiUfML(FrpxvsDMwwgbpqHXWxmN) , 
FshVHIJMRAhtQirYPlZd(FwLnexHgxHRquTKmNpoa) , 
fSbUUBgjKRbNXrHrlOLo(fOpFYwKNwIfWjnPzNuob) {  
CUDNN_CALL(cudnnCreatePoolingDescriptor(&mtolGPkUMBYDlSSqrRzc)); 
createAndAddDescriptor(getLayer()->getOutputTensor(0)->getSourcePortIndex()); } 
MWMaxPoolingLayerImpl::~MWMaxPoolingLayerImpl() { } void 
MWMaxPoolingLayerImpl::propagateSize() {  MWTensorBase* ipTensor = 
getLayer()->getInputTensor(0); MWTensorBase* opTensor = 
getLayer()->getOutputTensor(0); if ((DRzwhbNPpftRRIXXfHzd == -1) && 
(ECTnqgWHyHCHCLBZlffd == -1)) { DGzdAcREJHGXjyRzNjJV = ipTensor->getHeight(); 
DqxLTLaJwwgQqmrtCDuu = ipTensor->getWidth(); } int muwRQxtWMMXAPxSuMYBw = 
CTCbzQMDaLxINPbODdng; int nDsbARncmIrIaLubvLVZ = 
CpMjJjtGOeWOzwxpAAQP; 
CUDNN_CALL(cudnnSetPooling2dDescriptor(mtolGPkUMBYDlSSqrRzc, CUDNN_POOLING_MAX, 
CUDNN_NOT_PROPAGATE_NAN, DGzdAcREJHGXjyRzNjJV, DqxLTLaJwwgQqmrtCDuu, 
muwRQxtWMMXAPxSuMYBw, nDsbARncmIrIaLubvLVZ, FpguQZSermqZCMRiUfML, 
FshVHIJMRAhtQirYPlZd)); cudnnTensorDescriptor_t* desc = 
getDescriptor(opTensor->getSourcePortIndex()); assert(desc); 
setDescriptor<float>(*desc, static_cast<MWTensor<float>*>(opTensor)); } void 
MWMaxPoolingLayerImpl::allocate() { MWMaxPoolingLayer* maxpoolLayer = 
static_cast<MWMaxPoolingLayer*>(getLayer()); MWTensorBase* ipTensor = 
maxpoolLayer->getInputTensor(0); MWTensorBase* opTensor = 
maxpoolLayer->getOutputTensor(0); if (BRSPqxNffoBYKqpSVHne){ const int 
eVAFqeShtGZAZluKdMvQ = ipTensor->getNumElements(); 
CUDA_CALL(cudaMalloc((void**)&UWAGLbDcvybdWBtshhsr, 
sizeof(float)*eVAFqeShtGZAZluKdMvQ)); const int fDqxEdcpBDmVQxZEmQxm = 
opTensor->getNumElements(); CUDA_CALL(cudaMalloc((void**)&PQjbchiGbyJfmpiqPpOC, 
sizeof(float)*fDqxEdcpBDmVQxZEmQxm)); int tGsvtyAVkrDznETdweDC = 
(fDqxEdcpBDmVQxZEmQxm < 1024) ? fDqxEdcpBDmVQxZEmQxm : 1024; int 
KHClOltUSuqFVVErSxVb = (fDqxEdcpBDmVQxZEmQxm + tGsvtyAVkrDznETdweDC - 
1)/tGsvtyAVkrDznETdweDC; 
MWSetDyForBackPropImpl<<<KHClOltUSuqFVVErSxVb, 
tGsvtyAVkrDznETdweDC>>>( PQjbchiGbyJfmpiqPpOC, fDqxEdcpBDmVQxZEmQxm); } } void 
MWMaxPoolingLayerImpl::deallocate() { if (UWAGLbDcvybdWBtshhsr){ 
CUDA_FREE_CALL(UWAGLbDcvybdWBtshhsr); UWAGLbDcvybdWBtshhsr = 
NULL; } if (PQjbchiGbyJfmpiqPpOC){ CUDA_FREE_CALL(PQjbchiGbyJfmpiqPpOC); PQjbchiGbyJfmpiqPpOC = 
NULL; }  } void MWMaxPoolingLayerImpl::predict() { MWMaxPoolingLayer* 
maxpoolLayer = static_cast<MWMaxPoolingLayer*>(getLayer()); MWTensorBase* 
ipTensorBase = maxpoolLayer->getInputTensor(0); MWTensorBase* opTensorBase = 
maxpoolLayer->getOutputTensor(0); MWTensor<float>* ipTensor = 
static_cast<MWTensor<float>*>(ipTensorBase); MWTensor<float>* opTensor = 
static_cast<MWTensor<float>*>(opTensorBase); cudnnTensorDescriptor_t* desc = 
getDescriptor(opTensor->getSourcePortIndex()); assert(desc); 
cudnnTensorDescriptor_t XYbzSmRQGatVJtGmDZSo = 
MWCNNLayerImpl::getCuDNNDescriptor(ipTensorBase); 
CUDNN_CALL(cudnnPoolingForward(*dJcdBfQQLhIAYHPxwQeg->getCudnnHandle(), 
mtolGPkUMBYDlSSqrRzc, getOnePtr(), XYbzSmRQGatVJtGmDZSo, ipTensor->getData(), 
getZeroPtr(), *desc, opTensor->getData())); if (BRSPqxNffoBYKqpSVHne) { 
CUDNN_CALL(cudnnPoolingBackward(*dJcdBfQQLhIAYHPxwQeg->getCudnnHandle(), 
mtolGPkUMBYDlSSqrRzc, getOnePtr(), *desc, opTensor->getData(), *desc, 
PQjbchiGbyJfmpiqPpOC, XYbzSmRQGatVJtGmDZSo, ipTensor->getData(), getZeroPtr(), 
XYbzSmRQGatVJtGmDZSo, UWAGLbDcvybdWBtshhsr)); int eVAFqeShtGZAZluKdMvQ = 
ipTensor->getNumElements(); int tGsvtyAVkrDznETdweDC = 
(eVAFqeShtGZAZluKdMvQ < 1024) ? eVAFqeShtGZAZluKdMvQ : 1024; int 
KHClOltUSuqFVVErSxVb = (eVAFqeShtGZAZluKdMvQ + tGsvtyAVkrDznETdweDC - 
1)/tGsvtyAVkrDznETdweDC; 
doMWMaxPoolingLayerImpl<<<KHClOltUSuqFVVErSxVb, 
tGsvtyAVkrDznETdweDC>>>( UWAGLbDcvybdWBtshhsr, 
static_cast<MWTensor<float>*>(maxpoolLayer->getOutputTensor(1))->getData(), 
eVAFqeShtGZAZluKdMvQ); } return; } void MWMaxPoolingLayerImpl::cleanup() { 
CUDNN_CALL(cudnnDestroyPoolingDescriptor(mtolGPkUMBYDlSSqrRzc));  } float* 
MWMaxPoolingLayerImpl::getIndexData()  { return 
static_cast<MWTensor<float>*>(getLayer()->getOutputTensor(1))->getData(); } 
void __global__ __launch_bounds__(1024) MWSetDyForBackPropImpl(float * 
PQjbchiGbyJfmpiqPpOC, const int fDqxEdcpBDmVQxZEmQxm) { for(int i = blockDim.x * 
blockIdx.x + threadIdx.x; i < fDqxEdcpBDmVQxZEmQxm; i+= blockDim.x*gridDim.x) { 
PQjbchiGbyJfmpiqPpOC[i] = i+1; } } void __global__ __launch_bounds__(1024) 
doMWMaxPoolingLayerImpl(float * UWAGLbDcvybdWBtshhsr, float * 
UVzBVEOIylFjkSgHwFMp, const int BdqURaHPmdnfzvtUvocl) { for(int i = blockDim.x * 
blockIdx.x + threadIdx.x; i < BdqURaHPmdnfzvtUvocl; i+= blockDim.x*gridDim.x) { if 
(static_cast<int>(UWAGLbDcvybdWBtshhsr[i]) != 0){ 
UVzBVEOIylFjkSgHwFMp[static_cast<int>(UWAGLbDcvybdWBtshhsr[i])-1] = 
i; } } } MWFCLayerImpl::MWFCLayerImpl(MWCNNLayer* layer, MWTargetNetworkImpl* 
ntwk_impl, int XLJXOFXdnZOyJvtltbyr, int lsqeARVLtpJTWezgnTkg, const char* 
xHViLEwTujGGrPZZgmbF,  const char* JwxFdqOKggeawILBfGgg) : 
MWCNNLayerImpl(layer, ntwk_impl)  , 
CCKWXUFWgrbBMjwfpOBN(XLJXOFXdnZOyJvtltbyr) , 
CDJtexcMbXMWAmnNZsNf(lsqeARVLtpJTWezgnTkg) , vpXxoeEhdEosLSsYXkNG(NULL) , 
wJyXsrUCMgxdIKVIJSyx(NULL) , IwKnaBoXVubIRYcxEJLH(NULL) , 
xHiBGayUfxIpXKkCTDNU(false) { 
CUDNN_CALL(cudnnCreateTensorDescriptor(&JgLfgHrHMEMmMYTettJF)); 
createAndAddDescriptor(getLayer()->getOutputTensor(0)->getSourcePortIndex()); 
CUDA_CALL(cudaMalloc((void**)&vpXxoeEhdEosLSsYXkNG, 
sizeof(float)*CCKWXUFWgrbBMjwfpOBN*CDJtexcMbXMWAmnNZsNf)); 
CUDA_CALL(cudaMalloc((void**)&IwKnaBoXVubIRYcxEJLH, 
sizeof(float)*CDJtexcMbXMWAmnNZsNf)); wJyXsrUCMgxdIKVIJSyx = 
MALLOC_CALL(sizeof(float)*CCKWXUFWgrbBMjwfpOBN*CDJtexcMbXMWAmnNZsNf); 
loadWeights(xHViLEwTujGGrPZZgmbF); loadBias(JwxFdqOKggeawILBfGgg); } 
MWFCLayerImpl::~MWFCLayerImpl() { } void MWFCLayerImpl::propagateSize() { 
MWFCLayer* fcLayer = static_cast<MWFCLayer*>(getLayer()); MWTensorBase* 
opTensor = fcLayer->getOutputTensor(0); cudnnTensorDescriptor_t* desc = 
getDescriptor(opTensor->getSourcePortIndex()); assert(desc); 
setDescriptor<float>(*desc, static_cast<MWTensor<float>*>(opTensor)); if 
(opTensor->getSequenceLength() == 1) { 
CUDNN_CALL(cudnnSetTensor4dDescriptor(JgLfgHrHMEMmMYTettJF, CUDNN_TENSOR_NCHW, 
CUDNN_DATA_FLOAT, 1, CDJtexcMbXMWAmnNZsNf, 1, 1)); } else { int dims[5] 
= {1, 1, CDJtexcMbXMWAmnNZsNf, 1, 1}; int strides[5]; 
MWTensorBase::getStrides(dims, 5, strides); 
CUDNN_CALL(cudnnSetTensorNdDescriptor(JgLfgHrHMEMmMYTettJF, CUDNN_DATA_FLOAT, 5, 
dims, strides)); } } void MWFCLayerImpl::loadWeights(const char* 
QMgBqCuvjnbWHWiVPEwn) {  FILE* QhTesEEIHwhNmHSeYbRR = 
MWCNNLayer::openBinaryFile(QMgBqCuvjnbWHWiVPEwn); assert(QhTesEEIHwhNmHSeYbRR); int 
dMxIKDGTITyhdLqIHBLA = CCKWXUFWgrbBMjwfpOBN*CDJtexcMbXMWAmnNZsNf;  
call_fread(wJyXsrUCMgxdIKVIJSyx, sizeof(float), dMxIKDGTITyhdLqIHBLA, 
QhTesEEIHwhNmHSeYbRR, QMgBqCuvjnbWHWiVPEwn); fclose(QhTesEEIHwhNmHSeYbRR); } void 
MWFCLayerImpl::prepareWeights() { if (!xHiBGayUfxIpXKkCTDNU) { int 
dMxIKDGTITyhdLqIHBLA = CCKWXUFWgrbBMjwfpOBN*CDJtexcMbXMWAmnNZsNf; 
MWFCLayer* fcLayer = static_cast<MWFCLayer*>(getLayer()); MWTensorBase* 
ipTensor = fcLayer->getInputTensor(0); if( ipTensor->getHeight() != 1 && 
ipTensor->getWidth() != 1 ) { float* KZWeXiYFmdpQdsgidKeG = 
MALLOC_CALL(sizeof(float)*ipTensor->getHeight()*ipTensor->getWidth()); for(int 
k=0; k<dMxIKDGTITyhdLqIHBLA/ipTensor->getHeight()/ipTensor->getWidth(); k++) { 
for(int i=0; i<ipTensor->getHeight()*ipTensor->getWidth(); i++) 
KZWeXiYFmdpQdsgidKeG[i]=wJyXsrUCMgxdIKVIJSyx[k*ipTensor->getHeight()*ipTensor->getWidth()+i]; 
for(int j=0; j<ipTensor->getHeight(); j++) for(int i=0; i<ipTensor->getWidth(); 
i++) 
wJyXsrUCMgxdIKVIJSyx[k*ipTensor->getHeight()*ipTensor->getWidth()+j*ipTensor->getWidth()+i]=KZWeXiYFmdpQdsgidKeG[j+i*ipTensor->getHeight()]; 
} free(KZWeXiYFmdpQdsgidKeG); } CUDA_CALL(cudaMemcpy(vpXxoeEhdEosLSsYXkNG, 
wJyXsrUCMgxdIKVIJSyx, sizeof(float)*dMxIKDGTITyhdLqIHBLA, 
cudaMemcpyHostToDevice)); free(wJyXsrUCMgxdIKVIJSyx); 
wJyXsrUCMgxdIKVIJSyx = NULL; xHiBGayUfxIpXKkCTDNU = true; } } void 
MWFCLayerImpl::loadBias(const char* QMgBqCuvjnbWHWiVPEwn) { MWFCLayer* fcLayer = 
static_cast<MWFCLayer*>(getLayer()); MWTensorBase* opTensor = 
fcLayer->getOutputTensor(0); FILE* QhTesEEIHwhNmHSeYbRR = 
MWCNNLayer::openBinaryFile(QMgBqCuvjnbWHWiVPEwn); assert(QhTesEEIHwhNmHSeYbRR); int 
dMxIKDGTITyhdLqIHBLA = CDJtexcMbXMWAmnNZsNf;  float* KZWeXiYFmdpQdsgidKeG = 
MALLOC_CALL(sizeof(float)*dMxIKDGTITyhdLqIHBLA); call_fread(KZWeXiYFmdpQdsgidKeG, 
sizeof(float), dMxIKDGTITyhdLqIHBLA, QhTesEEIHwhNmHSeYbRR, QMgBqCuvjnbWHWiVPEwn); 
CUDA_CALL(cudaMemcpy(IwKnaBoXVubIRYcxEJLH, KZWeXiYFmdpQdsgidKeG, 
sizeof(float)*dMxIKDGTITyhdLqIHBLA, cudaMemcpyHostToDevice)); 
free(KZWeXiYFmdpQdsgidKeG); fclose(QhTesEEIHwhNmHSeYbRR); } void 
MWFCLayerImpl::postSetup() { prepareWeights(); } void MWFCLayerImpl::predict() 
{ MWFCLayer* fcLayer = static_cast<MWFCLayer*>(getLayer()); MWTensorBase* 
ipTensorBase = fcLayer->getInputTensor(0); MWTensorBase* opTensorBase = 
fcLayer->getOutputTensor(0); MWTensor<float>* ipTensor = 
static_cast<MWTensor<float>*>(ipTensorBase); MWTensor<float>* opTensor = 
static_cast<MWTensor<float>*>(opTensorBase); int numOutputRows = 
opTensor->getChannels(); int numOutputCols = 
ipTensor->getBatchSize()*ipTensor->getSequenceLength(); int innerDimension = 
ipTensor->getHeight()*ipTensor->getWidth()*ipTensor->getChannels(); int 
UKtMXCCqdjeyaVHabkxg=1; int URgvgDXnZskIYGdtimcU=1; if(opTensor->getBatchSize() == 1 && 
opTensor->getSequenceLength() == 1) { CUDA_CALL(cudaMemcpy(opTensor->getData(), 
IwKnaBoXVubIRYcxEJLH, sizeof(float)*numOutputRows, cudaMemcpyDeviceToDevice)); 
CUBLAS_CALL(cublasSgemv(*dJcdBfQQLhIAYHPxwQeg->getCublasHandle(), CUBLAS_OP_T, 
innerDimension, numOutputRows, getOnePtr(), vpXxoeEhdEosLSsYXkNG, innerDimension, 
ipTensor->getData(), UKtMXCCqdjeyaVHabkxg, getOnePtr(), opTensor->getData(), 
URgvgDXnZskIYGdtimcU)); } else { 
CUBLAS_CALL(cublasSgemm(*dJcdBfQQLhIAYHPxwQeg->getCublasHandle(), CUBLAS_OP_T, 
CUBLAS_OP_N, numOutputRows, numOutputCols, innerDimension, getOnePtr(), 
vpXxoeEhdEosLSsYXkNG, innerDimension, ipTensor->getData(), innerDimension, 
getZeroPtr(), opTensor->getData(), numOutputRows)); cudnnTensorDescriptor_t* 
desc = getDescriptor(opTensor->getSourcePortIndex()); assert(desc); 
CUDNN_CALL(cudnnAddTensor(*dJcdBfQQLhIAYHPxwQeg->getCudnnHandle(), getOnePtr(), 
JgLfgHrHMEMmMYTettJF, IwKnaBoXVubIRYcxEJLH, getOnePtr(), *desc, opTensor->getData())); } 
return; } void MWFCLayerImpl::cleanup() { if (vpXxoeEhdEosLSsYXkNG) { 
CUDA_FREE_CALL(vpXxoeEhdEosLSsYXkNG); vpXxoeEhdEosLSsYXkNG = NULL; } 
CUDNN_CALL(cudnnDestroyTensorDescriptor(JgLfgHrHMEMmMYTettJF)); if 
(IwKnaBoXVubIRYcxEJLH) { CUDA_FREE_CALL(IwKnaBoXVubIRYcxEJLH); IwKnaBoXVubIRYcxEJLH = NULL; } } 
MWSoftmaxLayerImpl::MWSoftmaxLayerImpl(MWCNNLayer* layer, MWTargetNetworkImpl* 
ntwk_impl) : MWCNNLayerImpl(layer, ntwk_impl)  { 
CUDNN_CALL(cudnnCreateTensorDescriptor(&shEncNmxJsMuJKwbrwok)); 
CUDNN_CALL(cudnnCreateTensorDescriptor(&sjLjZacPSDNBEjAccrGU)); } 
MWSoftmaxLayerImpl::~MWSoftmaxLayerImpl() { } void 
MWSoftmaxLayerImpl::propagateSize() { MWSoftmaxLayer* sfmxLayer = 
static_cast<MWSoftmaxLayer*>(getLayer()); MWTensorBase* ipTensor = 
sfmxLayer->getInputTensor(0); MWTensorBase* opTensor = 
sfmxLayer->getOutputTensor(0); 
CUDNN_CALL(cudnnSetTensor4dDescriptor(shEncNmxJsMuJKwbrwok, CUDNN_TENSOR_NCHW, 
CUDNN_DATA_FLOAT, ipTensor->getSequenceLength()*ipTensor->getBatchSize(), 
ipTensor->getChannels(), ipTensor->getHeight(), ipTensor->getWidth())); 
CUDNN_CALL(cudnnSetTensor4dDescriptor(sjLjZacPSDNBEjAccrGU, CUDNN_TENSOR_NCHW, 
CUDNN_DATA_FLOAT, opTensor->getSequenceLength()*opTensor->getBatchSize(), 
opTensor->getChannels(), opTensor->getHeight(), opTensor->getWidth())); } void 
MWSoftmaxLayerImpl::predict() { MWSoftmaxLayer* sfmxLayer = 
static_cast<MWSoftmaxLayer*>(getLayer()); MWTensorBase* ipTensorBase = 
sfmxLayer->getInputTensor(0); MWTensorBase* opTensorBase = 
sfmxLayer->getOutputTensor(0); MWTensor<float>* ipTensor = 
static_cast<MWTensor<float>*>(ipTensorBase); MWTensor<float>* opTensor = 
static_cast<MWTensor<float>*>(opTensorBase); 
CUDNN_CALL(cudnnSoftmaxForward(*dJcdBfQQLhIAYHPxwQeg->getCudnnHandle(), 
CUDNN_SOFTMAX_ACCURATE, CUDNN_SOFTMAX_MODE_CHANNEL, getOnePtr(), 
shEncNmxJsMuJKwbrwok, ipTensor->getData(), getZeroPtr(), 
sjLjZacPSDNBEjAccrGU, opTensor->getData())); } void 
MWSoftmaxLayerImpl::cleanup() { 
CUDNN_CALL(cudnnDestroyTensorDescriptor(shEncNmxJsMuJKwbrwok)); 
CUDNN_CALL(cudnnDestroyTensorDescriptor(sjLjZacPSDNBEjAccrGU)); } 
MWAvgPoolingLayerImpl::MWAvgPoolingLayerImpl(MWCNNLayer* layer, 
MWTargetNetworkImpl* ntwk_impl, int DSsxcjIrUgZCKZovyNQf,  int 
EfvWctmlsWAPsxXgdKWf,  int FrpxvsDMwwgbpqHXWxmN,  int 
FwLnexHgxHRquTKmNpoa,  int ClEhcJFlvGCgiavziIag,  int 
CZNYmBcNFSZWvaCklqeM, int CufLFODQDXTAPyRqYodN, int 
DCdZnqpcBnvXVgEsLBnz) : MWCNNLayerImpl(layer, ntwk_impl) , 
TaAJDyqFVJXfAfCJhOuU(NULL) , DRzwhbNPpftRRIXXfHzd(DSsxcjIrUgZCKZovyNQf) , 
ECTnqgWHyHCHCLBZlffd(EfvWctmlsWAPsxXgdKWf) , 
DGzdAcREJHGXjyRzNjJV(DSsxcjIrUgZCKZovyNQf) , 
DqxLTLaJwwgQqmrtCDuu(EfvWctmlsWAPsxXgdKWf) , 
FpguQZSermqZCMRiUfML(FrpxvsDMwwgbpqHXWxmN) , 
FshVHIJMRAhtQirYPlZd(FwLnexHgxHRquTKmNpoa) , 
CTCbzQMDaLxINPbODdng(ClEhcJFlvGCgiavziIag) , 
CLOUhPjbgggWoXHTtmjC(CZNYmBcNFSZWvaCklqeM) , 
CpMjJjtGOeWOzwxpAAQP(CufLFODQDXTAPyRqYodN) , 
CqtPRJvHlGJFssiPzsOm(DCdZnqpcBnvXVgEsLBnz) , 
IIiwAtyrOtLzLWAUlTey((CTCbzQMDaLxINPbODdng != CLOUhPjbgggWoXHTtmjC) 
|| (CpMjJjtGOeWOzwxpAAQP != CqtPRJvHlGJFssiPzsOm)) , 
muwRQxtWMMXAPxSuMYBw(ClEhcJFlvGCgiavziIag) , 
nDsbARncmIrIaLubvLVZ(CufLFODQDXTAPyRqYodN) { 
CUDNN_CALL(cudnnCreatePoolingDescriptor(&mtolGPkUMBYDlSSqrRzc)); MWTensorBase* 
ipTensor = getLayer()->getInputTensor(0); if (IIiwAtyrOtLzLWAUlTey) {  
muwRQxtWMMXAPxSuMYBw = 0;  nDsbARncmIrIaLubvLVZ = 0; 
TaAJDyqFVJXfAfCJhOuU = new MWTensor<float>(-1, -1, -1, -1, -1, NULL, getLayer(), 0); 
if (!TaAJDyqFVJXfAfCJhOuU) { MWCNNLayerImpl::throwAllocationError(__LINE__ , 
__FILE__); } CUDNN_CALL(cudnnCreateTensorDescriptor(&XYbzSmRQGatVJtGmDZSo));  } 
else { TaAJDyqFVJXfAfCJhOuU = ipTensor;  } assert(TaAJDyqFVJXfAfCJhOuU != NULL); 
MWAvgPoolingLayer* avgpoolLayer = static_cast<MWAvgPoolingLayer*>(getLayer()); 
MWTensorBase* opTensor = avgpoolLayer->getOutputTensor(0); 
createAndAddDescriptor(opTensor->getSourcePortIndex()); } 
MWAvgPoolingLayerImpl::~MWAvgPoolingLayerImpl() { } void 
MWAvgPoolingLayerImpl::propagateSize() { MWTensorBase* ipTensor = 
getLayer()->getInputTensor(0); if ((DRzwhbNPpftRRIXXfHzd == -1) && 
(ECTnqgWHyHCHCLBZlffd == -1)) { DGzdAcREJHGXjyRzNjJV = ipTensor->getHeight(); 
DqxLTLaJwwgQqmrtCDuu = ipTensor->getWidth(); } int inputH; int inputW; if 
(IIiwAtyrOtLzLWAUlTey) { inputH = ipTensor->getHeight() + 
CTCbzQMDaLxINPbODdng + CLOUhPjbgggWoXHTtmjC;  inputW = ipTensor->getWidth() + 
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
CUDNN_CALL(cudnnSetPooling2dDescriptor(mtolGPkUMBYDlSSqrRzc,  
CUDNN_POOLING_AVERAGE_COUNT_INCLUDE_PADDING,  CUDNN_NOT_PROPAGATE_NAN,  
DGzdAcREJHGXjyRzNjJV,  DqxLTLaJwwgQqmrtCDuu,  muwRQxtWMMXAPxSuMYBw,  
nDsbARncmIrIaLubvLVZ,  FpguQZSermqZCMRiUfML,  FshVHIJMRAhtQirYPlZd)); 
cudnnTensorDescriptor_t* desc = getDescriptor(opTensor->getSourcePortIndex()); 
assert(desc); setDescriptor<float>(*desc, 
static_cast<MWTensor<float>*>(opTensor)); } void 
MWAvgPoolingLayerImpl::allocate() { MWTensorBase* ipTensor = 
getLayer()->getInputTensor(0); if (IIiwAtyrOtLzLWAUlTey) { float* 
newInput; int inputH = ipTensor->getHeight() + CTCbzQMDaLxINPbODdng + 
CLOUhPjbgggWoXHTtmjC;  int inputW = ipTensor->getWidth() + 
CpMjJjtGOeWOzwxpAAQP + CqtPRJvHlGJFssiPzsOm; int paddedSize = 
ipTensor->getBatchSize() * ipTensor->getChannels() * inputH * inputW; 
CUDA_CALL(cudaMalloc((void**)&newInput, sizeof(float)*paddedSize)); 
CUDA_CALL(cudaMemset(newInput, 0, sizeof(float)*paddedSize)); 
static_cast<MWTensor<float>*>(TaAJDyqFVJXfAfCJhOuU)->setData(newInput); } } void 
MWAvgPoolingLayerImpl::deallocate() { if (TaAJDyqFVJXfAfCJhOuU != 
getLayer()->getInputTensor(0)) { assert(IIiwAtyrOtLzLWAUlTey); 
CUDA_FREE_CALL(static_cast<MWTensor<float>*>(TaAJDyqFVJXfAfCJhOuU)->getData()); 
static_cast<MWTensor<float>*>(TaAJDyqFVJXfAfCJhOuU)->setData((float*)NULL); } } void 
MWAvgPoolingLayerImpl::predict() { MWAvgPoolingLayer* avgpoolLayer = 
static_cast<MWAvgPoolingLayer*>(getLayer()); MWTensorBase* opTensorBase = 
avgpoolLayer->getOutputTensor(0); MWTensorBase* ipTensorBase = 
avgpoolLayer->getInputTensor(0); MWTensor<float>* ipTensor = 
static_cast<MWTensor<float>*>(ipTensorBase); MWTensor<float>* opTensor = 
static_cast<MWTensor<float>*>(opTensorBase); if (TaAJDyqFVJXfAfCJhOuU != 
avgpoolLayer->getInputTensor()) { 
CUDA_CALL(cudaMemset(static_cast<MWTensor<float>*>(TaAJDyqFVJXfAfCJhOuU)->getData(), 
0, sizeof(float)*TaAJDyqFVJXfAfCJhOuU->getNumElements()));  
MWCNNLayerImpl::padInput(ipTensor->getData(), ipTensor->getHeight(), 
ipTensor->getWidth(), ipTensor->getChannels(), TaAJDyqFVJXfAfCJhOuU->getHeight(), 
TaAJDyqFVJXfAfCJhOuU->getWidth(), CTCbzQMDaLxINPbODdng, CpMjJjtGOeWOzwxpAAQP, 
static_cast<MWTensor<float>*>(TaAJDyqFVJXfAfCJhOuU)->getData(), 
ipTensor->getNumElements()); } assert(opTensor->getData() != 
static_cast<MWTensor<float>*>(TaAJDyqFVJXfAfCJhOuU)->getData()); 
cudnnTensorDescriptor_t* desc = getDescriptor(opTensor->getSourcePortIndex()); 
assert(desc); 
CUDNN_CALL(cudnnPoolingForward(*dJcdBfQQLhIAYHPxwQeg->getCudnnHandle(), 
mtolGPkUMBYDlSSqrRzc, getOnePtr(), XYbzSmRQGatVJtGmDZSo, 
static_cast<MWTensor<float>*>(TaAJDyqFVJXfAfCJhOuU)->getData(), getZeroPtr(), *desc, 
opTensor->getData())); } void MWAvgPoolingLayerImpl::cleanup() { 
CUDNN_CALL(cudnnDestroyPoolingDescriptor(mtolGPkUMBYDlSSqrRzc)); if 
(TaAJDyqFVJXfAfCJhOuU != getLayer()->getInputTensor(0)) { 
assert(IIiwAtyrOtLzLWAUlTey); 
CUDNN_CALL(cudnnDestroyTensorDescriptor(XYbzSmRQGatVJtGmDZSo)); } } 
MWOutputLayerImpl::MWOutputLayerImpl(MWCNNLayer* layer, MWTargetNetworkImpl* 
ntwk_impl) : MWCNNLayerImpl(layer, ntwk_impl) { } 
MWOutputLayerImpl::~MWOutputLayerImpl() { } void 
MWOutputLayerImpl::propagateSize() { } void 
MWOutputLayerImpl::deallocateOutputData(int) { } void 
MWOutputLayerImpl::predict() { } void MWOutputLayerImpl::cleanup() { }