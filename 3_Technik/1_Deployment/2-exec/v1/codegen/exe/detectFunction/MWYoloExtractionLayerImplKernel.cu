#include "MWKernelHeaders.hpp"
 void __global__ __launch_bounds__(1024) YoloExtractionImpl(float* inputBuffer, 
float* outputBuffer_iouxy, float* outputBuffer_wh, float* outputBuffer_cscores, 
int dkLDkRwCBjeybwDHbKiE, int YOWMnLKOMqAODXiVNoGy, long int 
YNmJhGSUszJKxsodxiuV, long int YNDVziqpDddiXQKYZZhX, long int 
jHzoRQWaHafftmrmuvHO, long int jLyhrFjMmVnNjoeDJCwH, 
long int iwclITrbVyVrJaArrXNr, const long int BdqURaHPmdnfzvtUvocl) { 
for (int idx = blockDim.x * blockIdx.x + threadIdx.x; idx < BdqURaHPmdnfzvtUvocl; idx 
+= blockDim.x * gridDim.x) { int cwCXkgHfZmFQRzNVUlCO = idx / 
YNDVziqpDddiXQKYZZhX; long int FOcStuqCptsGIZXskVpC = idx - 
(YNDVziqpDddiXQKYZZhX * cwCXkgHfZmFQRzNVUlCO); int LklYEpYUjaLTgcFFAaJX = 
static_cast<int>(FOcStuqCptsGIZXskVpC / YNmJhGSUszJKxsodxiuV); long 
int FeVcBgtQmTLtmnNcJGMY = FOcStuqCptsGIZXskVpC - 
(YNmJhGSUszJKxsodxiuV * LklYEpYUjaLTgcFFAaJX); int zFaEOIBQYqPoaerpaixN = 
static_cast<int>(FeVcBgtQmTLtmnNcJGMY % YOWMnLKOMqAODXiVNoGy); int 
yCdIUfwoZFngCRRRkCTg = static_cast<int>(FeVcBgtQmTLtmnNcJGMY / YOWMnLKOMqAODXiVNoGy); 
if (LklYEpYUjaLTgcFFAaJX < 3 * dkLDkRwCBjeybwDHbKiE) { long int opIdx = cwCXkgHfZmFQRzNVUlCO * 
jHzoRQWaHafftmrmuvHO + LklYEpYUjaLTgcFFAaJX * 
YNmJhGSUszJKxsodxiuV + yCdIUfwoZFngCRRRkCTg * YOWMnLKOMqAODXiVNoGy + 
zFaEOIBQYqPoaerpaixN; outputBuffer_iouxy[opIdx] = inputBuffer[idx]; } else if 
(LklYEpYUjaLTgcFFAaJX >= 3 * dkLDkRwCBjeybwDHbKiE && LklYEpYUjaLTgcFFAaJX < 5 * 
dkLDkRwCBjeybwDHbKiE) { int LtEgcYoEYjkrWuohutgw = LklYEpYUjaLTgcFFAaJX - (3 * 
dkLDkRwCBjeybwDHbKiE); long int opIdx = cwCXkgHfZmFQRzNVUlCO * 
jLyhrFjMmVnNjoeDJCwH + LtEgcYoEYjkrWuohutgw * 
YNmJhGSUszJKxsodxiuV + yCdIUfwoZFngCRRRkCTg * YOWMnLKOMqAODXiVNoGy + 
zFaEOIBQYqPoaerpaixN; outputBuffer_wh[opIdx] = inputBuffer[idx]; } else { int 
LtEgcYoEYjkrWuohutgw = LklYEpYUjaLTgcFFAaJX - (5 * dkLDkRwCBjeybwDHbKiE); long int opIdx = 
cwCXkgHfZmFQRzNVUlCO * iwclITrbVyVrJaArrXNr + LtEgcYoEYjkrWuohutgw * 
YNmJhGSUszJKxsodxiuV + yCdIUfwoZFngCRRRkCTg * YOWMnLKOMqAODXiVNoGy + 
zFaEOIBQYqPoaerpaixN; outputBuffer_cscores[opIdx] = inputBuffer[idx]; } } }