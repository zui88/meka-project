#include "MWKernelHeaders.hpp"
#include <math.h>
#include <stdio.h>
 void __global__ __launch_bounds__(1024) scale_scalar_kernel(float* 
inputBuffer, float* outputBuffer, float* puSFZkRJmyuFPfQRswDK, long int 
YGiQICncmsGZkNUyiQyg) {  for (long int idx = blockDim.x * blockIdx.x + 
threadIdx.x; idx < YGiQICncmsGZkNUyiQyg; idx += blockDim.x * gridDim.x) {  
outputBuffer[idx] = puSFZkRJmyuFPfQRswDK[0]*inputBuffer[idx]; } } void __global__ 
__launch_bounds__(1024) scale_vector_kernel(float* inputBuffer, float* 
outputBuffer, float* puSFZkRJmyuFPfQRswDK, double YNmJhGSUszJKxsodxiuV, 
double YNDVziqpDddiXQKYZZhX, long int YGiQICncmsGZkNUyiQyg) {  for 
(long int idx = blockDim.x * blockIdx.x + threadIdx.x; idx < 
YGiQICncmsGZkNUyiQyg; idx += blockDim.x * gridDim.x) { double batchIdx = 
floor(idx / YNDVziqpDddiXQKYZZhX); double i_batch = idx - (batchIdx * 
YNDVziqpDddiXQKYZZhX); double channelIdx = floor(i_batch / 
YNmJhGSUszJKxsodxiuV); outputBuffer[idx] = 
puSFZkRJmyuFPfQRswDK[static_cast<long int>(channelIdx)]*inputBuffer[idx]; } } void 
__global__ __launch_bounds__(1024) scale_matrix2d_kernel(float* inputBuffer, 
float* outputBuffer, float* puSFZkRJmyuFPfQRswDK, double 
YNmJhGSUszJKxsodxiuV, long int YGiQICncmsGZkNUyiQyg) {  for (long int 
idx = blockDim.x * blockIdx.x + threadIdx.x; idx < YGiQICncmsGZkNUyiQyg; idx += 
blockDim.x * gridDim.x) { double totalChannelIdx = floor(idx / 
YNmJhGSUszJKxsodxiuV); double i_channel = idx - (totalChannelIdx * 
YNmJhGSUszJKxsodxiuV); outputBuffer[idx] = 
puSFZkRJmyuFPfQRswDK[static_cast<long int>(i_channel)]*inputBuffer[idx]; } } void 
__global__ __launch_bounds__(1024) scale_tensor3d_kernel(float* inputBuffer, 
float* outputBuffer, float* puSFZkRJmyuFPfQRswDK, double 
YNDVziqpDddiXQKYZZhX, long int YGiQICncmsGZkNUyiQyg) {  for (long int 
idx = blockDim.x * blockIdx.x + threadIdx.x; idx < YGiQICncmsGZkNUyiQyg; idx += 
blockDim.x * gridDim.x) { double batchIdx = floor(idx / 
YNDVziqpDddiXQKYZZhX); double i_batch = idx - (batchIdx * 
YNDVziqpDddiXQKYZZhX); outputBuffer[idx] = 
puSFZkRJmyuFPfQRswDK[static_cast<long int>(i_batch)]*inputBuffer[idx]; } }  void 
__global__ __launch_bounds__(1024) offset_scalar_kernel(float* inputBuffer, 
float* outputBuffer, float* gCYwEfkibolsgZAumsuW, long int YGiQICncmsGZkNUyiQyg, 
bool ZKjSVYDDjACizBkGbqBq, int bOrQjJTNlssnrexxbHdi, int 
veFyKKHbdqBIvQLYBqfF) {  for (long int idx = blockDim.x * blockIdx.x + 
threadIdx.x; idx < YGiQICncmsGZkNUyiQyg; idx += blockDim.x * gridDim.x) { float 
out = inputBuffer[idx] + gCYwEfkibolsgZAumsuW[0]; if (ZKjSVYDDjACizBkGbqBq){ out = 
out > veFyKKHbdqBIvQLYBqfF ? veFyKKHbdqBIvQLYBqfF : out; out = out < 
bOrQjJTNlssnrexxbHdi ? bOrQjJTNlssnrexxbHdi : out; } outputBuffer[idx] = out; 
} } void __global__ __launch_bounds__(1024) offset_vector_kernel(float* 
inputBuffer, float* outputBuffer, float* gCYwEfkibolsgZAumsuW,  double 
YNmJhGSUszJKxsodxiuV, double YNDVziqpDddiXQKYZZhX, long int 
YGiQICncmsGZkNUyiQyg, bool ZKjSVYDDjACizBkGbqBq, int bOrQjJTNlssnrexxbHdi, int 
veFyKKHbdqBIvQLYBqfF) {  for (long int idx = blockDim.x * blockIdx.x + 
threadIdx.x; idx < YGiQICncmsGZkNUyiQyg; idx += blockDim.x * gridDim.x) { 
double batchIdx = floor(idx / YNDVziqpDddiXQKYZZhX); double i_batch = 
idx - (batchIdx * YNDVziqpDddiXQKYZZhX); double channelIdx = 
floor(i_batch / YNmJhGSUszJKxsodxiuV); float out = inputBuffer[idx] + 
gCYwEfkibolsgZAumsuW[static_cast<long int>(channelIdx)]; if 
(ZKjSVYDDjACizBkGbqBq){ out = out > veFyKKHbdqBIvQLYBqfF ? 
veFyKKHbdqBIvQLYBqfF : out; out = out < bOrQjJTNlssnrexxbHdi ? 
bOrQjJTNlssnrexxbHdi : out; } outputBuffer[idx] = out; } } void __global__ 
__launch_bounds__(1024) offset_matrix2d_kernel(float* inputBuffer, float* 
outputBuffer, float* gCYwEfkibolsgZAumsuW, double YNmJhGSUszJKxsodxiuV, 
long int YGiQICncmsGZkNUyiQyg, bool ZKjSVYDDjACizBkGbqBq, int 
bOrQjJTNlssnrexxbHdi, int veFyKKHbdqBIvQLYBqfF) {  for (long int idx = 
blockDim.x * blockIdx.x + threadIdx.x; idx < YGiQICncmsGZkNUyiQyg; idx += 
blockDim.x * gridDim.x) { double totalChannelIdx = floor(idx / 
YNmJhGSUszJKxsodxiuV); double i_channel = idx - (totalChannelIdx * 
YNmJhGSUszJKxsodxiuV); float out = inputBuffer[idx] + 
gCYwEfkibolsgZAumsuW[static_cast<long int>(i_channel)]; if (ZKjSVYDDjACizBkGbqBq){ 
out = out > veFyKKHbdqBIvQLYBqfF ? veFyKKHbdqBIvQLYBqfF : out; out = out < 
bOrQjJTNlssnrexxbHdi ? bOrQjJTNlssnrexxbHdi : out; } outputBuffer[idx] = out; 
} } void __global__ __launch_bounds__(1024) offset_tensor3d_kernel(float* 
inputBuffer, float* outputBuffer, float* gCYwEfkibolsgZAumsuW, double 
YNDVziqpDddiXQKYZZhX, long int YGiQICncmsGZkNUyiQyg, bool 
ZKjSVYDDjACizBkGbqBq, int bOrQjJTNlssnrexxbHdi, int veFyKKHbdqBIvQLYBqfF) {  
for (long int idx = blockDim.x * blockIdx.x + threadIdx.x; idx < 
YGiQICncmsGZkNUyiQyg; idx += blockDim.x * gridDim.x) { double batchIdx = 
floor(idx / YNDVziqpDddiXQKYZZhX); double i_batch = idx - (batchIdx * 
YNDVziqpDddiXQKYZZhX); float out = inputBuffer[idx] + 
gCYwEfkibolsgZAumsuW[static_cast<long int>(i_batch)]; if (ZKjSVYDDjACizBkGbqBq){ 
out = out > veFyKKHbdqBIvQLYBqfF ? veFyKKHbdqBIvQLYBqfF : out; out = out < 
bOrQjJTNlssnrexxbHdi ? bOrQjJTNlssnrexxbHdi : out; } outputBuffer[idx] = out; 
} } 