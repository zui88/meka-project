#include "MWElementwiseAffineLayerImpl.hpp"
#include "MWTargetNetworkImpl.hpp"
#include "MWKernelHeaders.hpp"
#include "cnn_api.hpp"
#include <math.h>
#include <cassert>
#include <stdio.h>
 MWElementwiseAffineLayerImpl::MWElementwiseAffineLayerImpl(MWCNNLayer* layer, 
MWTargetNetworkImpl* ntwk_impl, int scale_H, int scale_W, int scale_C, int 
offset_H, int offset_W, int offset_C, bool isClipped, int lowerbound, int 
upperbound, const char* qWwjVYwfnvEnFKlgpqwA, const char* 
hvqKUzPqCuUJRfoNlbwW) : MWCNNLayerImpl(layer, ntwk_impl), 
puSFZkRJmyuFPfQRswDK(NULL), gCYwEfkibolsgZAumsuW(NULL), pzUAoBDvaKAtdsmkQuct(scale_H), 
qEXwbWWsnOADJeTXfRVa(scale_W), pvpNsgGssdTxeVoFIkXI(scale_C), 
hKyfKjPACkOBDvLdESxH(offset_H), hnewnpwgzKmOdualajhn(offset_W), 
hDaNSVZAofAENeIAiWEw(offset_C), ZKjSVYDDjACizBkGbqBq(isClipped), 
bOrQjJTNlssnrexxbHdi(lowerbound), veFyKKHbdqBIvQLYBqfF(upperbound) { 
CUDA_CALL(cudaMalloc((void**)&puSFZkRJmyuFPfQRswDK, 
sizeof(float)*pzUAoBDvaKAtdsmkQuct*qEXwbWWsnOADJeTXfRVa*pvpNsgGssdTxeVoFIkXI)); 
CUDA_CALL(cudaMalloc((void**)&gCYwEfkibolsgZAumsuW, 
sizeof(float)*hKyfKjPACkOBDvLdESxH*hnewnpwgzKmOdualajhn*hDaNSVZAofAENeIAiWEw));  
loadScale(qWwjVYwfnvEnFKlgpqwA); loadOffset(hvqKUzPqCuUJRfoNlbwW); } 
MWElementwiseAffineLayerImpl::~MWElementwiseAffineLayerImpl() { } void 
MWElementwiseAffineLayerImpl::propagateSize() { } void 
MWElementwiseAffineLayerImpl::predict() { MWTensorBase* ipTensorBase = 
getLayer()->getInputTensor(0); MWTensorBase* opTensorBase = 
getLayer()->getOutputTensor(0); MWTensor<float>* ipTensor = 
static_cast<MWTensor<float>*>(ipTensorBase); MWTensor<float>* opTensor = 
static_cast<MWTensor<float>*>(opTensorBase); int WerBmCOBWhvoFbdqfitc = 
ipTensor->getHeight(); int WmXADZOqdcQvtBUvFerh = ipTensor->getWidth(); int 
WOJynDmqVUPWjAGVIuMQ = ipTensor->getChannels(); long int 
YNmJhGSUszJKxsodxiuV = WerBmCOBWhvoFbdqfitc*WmXADZOqdcQvtBUvFerh; long 
int YNDVziqpDddiXQKYZZhX = 
YNmJhGSUszJKxsodxiuV*WOJynDmqVUPWjAGVIuMQ; long int 
YGiQICncmsGZkNUyiQyg = ipTensor->getNumElements(); long int sFIUeCwGDlfadqOrGZHC = 
((YGiQICncmsGZkNUyiQyg + 31) / 32) * 32; int tGsvtyAVkrDznETdweDC = 
(sFIUeCwGDlfadqOrGZHC < 1024) ? sFIUeCwGDlfadqOrGZHC : 1024; long int 
KHClOltUSuqFVVErSxVb = (YGiQICncmsGZkNUyiQyg + tGsvtyAVkrDznETdweDC - 
1) / tGsvtyAVkrDznETdweDC; long int qBTcAwVGZERyCjGYByPe = 
pzUAoBDvaKAtdsmkQuct * qEXwbWWsnOADJeTXfRVa * pvpNsgGssdTxeVoFIkXI; long int 
hljcfGWsvZXJZNrImpJB = hKyfKjPACkOBDvLdESxH * hnewnpwgzKmOdualajhn * 
hDaNSVZAofAENeIAiWEw; assert(qBTcAwVGZERyCjGYByPe <= YGiQICncmsGZkNUyiQyg); 
assert(hljcfGWsvZXJZNrImpJB <= YGiQICncmsGZkNUyiQyg); if (qBTcAwVGZERyCjGYByPe == 
1) { scale_scalar_kernel<<<KHClOltUSuqFVVErSxVb, 
tGsvtyAVkrDznETdweDC>>>( ipTensor->getData(),  opTensor->getData(), 
puSFZkRJmyuFPfQRswDK, YGiQICncmsGZkNUyiQyg); } else if (pzUAoBDvaKAtdsmkQuct == 1 && 
qEXwbWWsnOADJeTXfRVa == 1 && qBTcAwVGZERyCjGYByPe > 1) { 
scale_vector_kernel<<<KHClOltUSuqFVVErSxVb, tGsvtyAVkrDznETdweDC>>>( 
ipTensor->getData(),  opTensor->getData(), puSFZkRJmyuFPfQRswDK, 
YNmJhGSUszJKxsodxiuV, YNDVziqpDddiXQKYZZhX, 
YGiQICncmsGZkNUyiQyg); } else if (YNDVziqpDddiXQKYZZhX == 
qBTcAwVGZERyCjGYByPe) {  scale_tensor3d_kernel<<<KHClOltUSuqFVVErSxVb, 
tGsvtyAVkrDznETdweDC>>>( ipTensor->getData(),  opTensor->getData(), 
puSFZkRJmyuFPfQRswDK,  YNDVziqpDddiXQKYZZhX, YGiQICncmsGZkNUyiQyg); } else 
{ scale_matrix2d_kernel<<<KHClOltUSuqFVVErSxVb, 
tGsvtyAVkrDznETdweDC>>>( ipTensor->getData(),  opTensor->getData(), 
puSFZkRJmyuFPfQRswDK,  YNmJhGSUszJKxsodxiuV, YGiQICncmsGZkNUyiQyg); } if 
(hljcfGWsvZXJZNrImpJB == 1) { offset_scalar_kernel<<<KHClOltUSuqFVVErSxVb, 
tGsvtyAVkrDznETdweDC>>>( opTensor->getData(),  opTensor->getData(), 
gCYwEfkibolsgZAumsuW, YGiQICncmsGZkNUyiQyg, ZKjSVYDDjACizBkGbqBq, 
bOrQjJTNlssnrexxbHdi, veFyKKHbdqBIvQLYBqfF); } else if (hKyfKjPACkOBDvLdESxH 
== 1 && hnewnpwgzKmOdualajhn == 1 && hljcfGWsvZXJZNrImpJB > 1) { 
offset_vector_kernel<<<KHClOltUSuqFVVErSxVb, tGsvtyAVkrDznETdweDC>>>( 
opTensor->getData(),  opTensor->getData(), gCYwEfkibolsgZAumsuW, 
YNmJhGSUszJKxsodxiuV, YNDVziqpDddiXQKYZZhX, 
YGiQICncmsGZkNUyiQyg, ZKjSVYDDjACizBkGbqBq, bOrQjJTNlssnrexxbHdi, 
veFyKKHbdqBIvQLYBqfF); } else if (YNDVziqpDddiXQKYZZhX == 
hljcfGWsvZXJZNrImpJB) { offset_tensor3d_kernel<<<KHClOltUSuqFVVErSxVb, 
tGsvtyAVkrDznETdweDC>>>( opTensor->getData(),  opTensor->getData(), 
gCYwEfkibolsgZAumsuW, YNDVziqpDddiXQKYZZhX, YGiQICncmsGZkNUyiQyg, 
ZKjSVYDDjACizBkGbqBq, bOrQjJTNlssnrexxbHdi, veFyKKHbdqBIvQLYBqfF); } else { 
offset_matrix2d_kernel<<<KHClOltUSuqFVVErSxVb, 
tGsvtyAVkrDznETdweDC>>>( opTensor->getData(),  opTensor->getData(), 
gCYwEfkibolsgZAumsuW, YNmJhGSUszJKxsodxiuV, YGiQICncmsGZkNUyiQyg, 
ZKjSVYDDjACizBkGbqBq, bOrQjJTNlssnrexxbHdi, veFyKKHbdqBIvQLYBqfF); } return; 
} void MWElementwiseAffineLayerImpl::cleanup() { if (puSFZkRJmyuFPfQRswDK) { 
CUDA_FREE_CALL(puSFZkRJmyuFPfQRswDK); puSFZkRJmyuFPfQRswDK = NULL; } if 
(gCYwEfkibolsgZAumsuW) { CUDA_FREE_CALL(gCYwEfkibolsgZAumsuW); gCYwEfkibolsgZAumsuW = 
NULL; }  } void MWElementwiseAffineLayerImpl::loadScale(const char* 
qWwjVYwfnvEnFKlgpqwA) { FILE* QhTesEEIHwhNmHSeYbRR = 
MWCNNLayer::openBinaryFile(qWwjVYwfnvEnFKlgpqwA); assert(QhTesEEIHwhNmHSeYbRR); long 
int dMxIKDGTITyhdLqIHBLA = pzUAoBDvaKAtdsmkQuct*qEXwbWWsnOADJeTXfRVa*pvpNsgGssdTxeVoFIkXI; 
float* KZWeXiYFmdpQdsgidKeG = MALLOC_CALL(sizeof(float)*dMxIKDGTITyhdLqIHBLA); 
call_fread(KZWeXiYFmdpQdsgidKeG, sizeof(float), dMxIKDGTITyhdLqIHBLA, QhTesEEIHwhNmHSeYbRR, 
qWwjVYwfnvEnFKlgpqwA); CUDA_CALL(cudaMemcpy(puSFZkRJmyuFPfQRswDK, 
KZWeXiYFmdpQdsgidKeG, sizeof(float)*dMxIKDGTITyhdLqIHBLA, cudaMemcpyHostToDevice)); 
free(KZWeXiYFmdpQdsgidKeG); fclose(QhTesEEIHwhNmHSeYbRR);  } void 
MWElementwiseAffineLayerImpl::loadOffset(const char* hvqKUzPqCuUJRfoNlbwW) { 
FILE* QhTesEEIHwhNmHSeYbRR = MWCNNLayer::openBinaryFile(hvqKUzPqCuUJRfoNlbwW); 
assert(QhTesEEIHwhNmHSeYbRR); long int dMxIKDGTITyhdLqIHBLA = 
hKyfKjPACkOBDvLdESxH*hnewnpwgzKmOdualajhn*hDaNSVZAofAENeIAiWEw; float* 
KZWeXiYFmdpQdsgidKeG = MALLOC_CALL(sizeof(float)*dMxIKDGTITyhdLqIHBLA); 
call_fread(KZWeXiYFmdpQdsgidKeG, sizeof(float), dMxIKDGTITyhdLqIHBLA, QhTesEEIHwhNmHSeYbRR, 
hvqKUzPqCuUJRfoNlbwW); CUDA_CALL(cudaMemcpy(gCYwEfkibolsgZAumsuW, 
KZWeXiYFmdpQdsgidKeG, sizeof(float)*dMxIKDGTITyhdLqIHBLA, cudaMemcpyHostToDevice)); 
free(KZWeXiYFmdpQdsgidKeG); fclose(QhTesEEIHwhNmHSeYbRR);  }