/* Copyright 2018-2019 The MathWorks, Inc. */

#ifndef __MW_SORT_WITH_INDEX_UTIL_H__
#define __MW_SORT_WITH_INDEX_UTIL_H__

#ifdef __CUDACC__

#include <thrust/sort.h>
#include <thrust/execution_policy.h>
#include <thrust/fill.h>
#include "MWSortFunctors.h"
#include "MWShuffleUtility.h"

#define BLOCK_SIZE 512

template <class T> MW_HOST_DEVICE
void sortAlongEdgeDimWithIndex(T* d_key, double* d_ind, int s, const char dir, int nelements, int __compiling_host__);

template <class T> MW_HOST_DEVICE
void sortAlongOtherDimWithIndex(T* d_key, double* d_ind, int numDims, const int* dims, int sortAlongDim, const char dir, int nelements, bool isRowMajor);

template <class T> MW_HOST_DEVICE
void sortAlongOtherDimWithIndexDevice(T* d_key, double* d_ind, int numDims, const int* dims, int sortAlongDim, const char dir, int nelements, bool isRowMajor);

template <class T> MW_HOST_DEVICE
void callThrustSortByKeyWithIndex(T* d_key, double* d_ind, int* d_seq, const char dir, int nelements);

template <class T> MW_HOST_DEVICE
void callThrustSortByKeyWithIndexDevice(T* d_key, double* d_ind, int* d_seq, const char dir, int nelements);

template <class T> MW_HOST_DEVICE
void callThrustSortByKeyWithIndexSimple(T* d_key, double* d_ind, int* d_seq, const char dir, int nelements);

template <class T> MW_HOST_DEVICE
void callThrustSortByKeyWithIndexSimpleDevice(T* d_key, double* d_ind, int* d_seq, const char dir, int nelements);

template <class T> MW_HOST_DEVICE
void thrustSortImplWithIndex(T* d_key, double* d_ind, int numDims, const int* dims, int sortAlongDim, const char dir, bool isRowMajor);


template <class T> MW_HOST_DEVICE
void sortAlongEdgeDimWithIndex(T* d_key, double* d_ind, int s, const char dir, int nelements, int __compiling_host__) {

    int *d_seq;
    cudaMalloc(&d_seq, sizeof(int) * nelements);
    
    dim3 dimBlock(BLOCK_SIZE, 1, 1);
    int numBlocksX = (nelements - 1) / BLOCK_SIZE + 1;
    dim3 dimGrid(numBlocksX, 1, 1);

    if(__compiling_host__ == 1) {
        createSeqEdgeDimWithIndex<<<dimGrid,dimBlock>>>(d_seq, d_ind, s, nelements);
    } else {
        createSeqEdgeDimWithIndexDevice(d_seq, d_ind, s, nelements);
    }
    cudaDeviceSynchronize();
    
    if(__compiling_host__ == 1) {
        callThrustSortByKeyWithIndex<T>(d_key, d_ind, d_seq, dir, nelements);
    } else {
        callThrustSortByKeyWithIndexDevice<T>(d_key, d_ind, d_seq, dir, nelements);
    }
    cudaFree(d_seq);
}

template <class T> MW_HOST_DEVICE
void sortAlongOtherDimWithIndex(T* d_key, double* d_ind, int numDims, const int* dims, int sortAlongDim, const char dir, int nelements, bool isRowMajor) {

    dim3 dimBlock(BLOCK_SIZE, 1, 1);
    int numBlocksX = (nelements - 1) / BLOCK_SIZE + 1;
    dim3 dimGrid(numBlocksX, 1, 1);

    int* oldDim = new int[numDims];
    int* stride = new int[numDims];
    int* shflStride = new int[numDims];
    int *d_seq;    
    T* d_key_out;
    double* d_ind_out;
    int* d_shflStride;
    int* d_oldDim;
    int nelem2;
    int s;
    int d;
    
    cudaMalloc(&d_seq    ,sizeof(int) * nelements);
    cudaMalloc(&d_key_out, sizeof(T)  * nelements);
    cudaMalloc(&d_ind_out, sizeof(double)* nelements);
    cudaMalloc(&d_shflStride, sizeof(int) * numDims);
    cudaMalloc(&d_oldDim, sizeof(int) * numDims);
        
    if(isRowMajor) {
        nelem2 = calcRowMajorStrideAndDim(dims, numDims, sortAlongDim, stride, shflStride, oldDim);
    } else {
        nelem2 = calcColMajorStrideAndDim(dims, numDims, sortAlongDim, stride, shflStride, oldDim);
    }

    s = stride[sortAlongDim];
    d = dims[sortAlongDim];
    createSeqOtherDimWithIndex<<<dimGrid,dimBlock>>>(d_seq, d_ind, s, d, nelements, nelem2);
    cudaDeviceSynchronize();
        
    callThrustSortByKeyWithIndex<T>(d_key, d_ind, d_seq, dir, nelements);
    
    // Wrapping cudaMemcpy with this flag to supress warning: calling a host fcn "cudaMemcpy" from host device fcn
    #ifndef __CUDA_ARCH__
        cudaMemcpy(d_shflStride, shflStride, numDims * sizeof(int), cudaMemcpyHostToDevice);
        cudaMemcpy(d_oldDim, oldDim, numDims * sizeof(int), cudaMemcpyHostToDevice);
    #endif

    if(isRowMajor) {
        shuffleRowMajorWithIndex<T><<<dimGrid,dimBlock>>>(d_key, d_key_out, d_ind, d_ind_out, d_shflStride, d_oldDim, numDims, nelements);
    } else {
        shuffleColMajorWithIndex<T><<<dimGrid,dimBlock>>>(d_key, d_key_out, d_ind, d_ind_out, d_shflStride, d_oldDim, numDims, nelements);
    }
    cudaDeviceSynchronize();
    
    #ifndef __CUDA_ARCH__
        cudaMemcpy(d_key, d_key_out, nelements * sizeof(T), cudaMemcpyDeviceToDevice);
        cudaMemcpy(d_ind, d_ind_out, nelements * sizeof(double), cudaMemcpyDeviceToDevice);
    #endif
    
    cudaFree(d_key_out);
    cudaFree(d_ind_out);
    cudaFree(d_seq);
    cudaFree(d_shflStride);
    cudaFree(d_oldDim);
    delete[] oldDim;
    delete[] stride;
    delete[] shflStride;
}

template <class T> MW_HOST_DEVICE
void sortAlongOtherDimWithIndexDevice(T* d_key, double* d_ind, int numDims, const int* dims, int sortAlongDim, const char dir, int nelements, bool isRowMajor) {

    int *d_seq;    
    T* d_key_out;
    double* d_ind_out;
    int* d_stride;
    int* d_shflStride;
    int* d_oldDim;
    int nelem2;
    int s;
    int d;
    
    cudaMalloc(&d_seq    ,sizeof(int) * nelements);
    cudaMalloc(&d_key_out, sizeof(T)  * nelements);
    cudaMalloc(&d_ind_out, sizeof(double)* nelements);
    cudaMalloc(&d_stride, sizeof(int) * numDims);
    cudaMalloc(&d_shflStride, sizeof(int) * numDims);
    cudaMalloc(&d_oldDim, sizeof(int) * numDims);
        
    if(isRowMajor) {
        nelem2 = calcRowMajorStrideAndDim(dims, numDims, sortAlongDim, d_stride, d_shflStride, d_oldDim);
    } else {
        nelem2 = calcColMajorStrideAndDim(dims, numDims, sortAlongDim, d_stride, d_shflStride, d_oldDim);
    }

    s = d_stride[sortAlongDim];
    d = dims[sortAlongDim];
    createSeqOtherDimWithIndexDevice(d_seq, d_ind, s, d, nelements, nelem2);
    cudaDeviceSynchronize();
        
    callThrustSortByKeyWithIndexDevice<T>(d_key, d_ind, d_seq, dir, nelements);

    if(isRowMajor) {
        shuffleRowMajorWithIndexDevice<T>(d_key, d_key_out, d_ind, d_ind_out, d_shflStride, d_oldDim, numDims, nelements);
    } else {
        shuffleColMajorWithIndexDevice<T>(d_key, d_key_out, d_ind, d_ind_out, d_shflStride, d_oldDim, numDims, nelements);
    }
    cudaDeviceSynchronize();
    
    memcpy(d_key, d_key_out, nelements * sizeof(T));
    memcpy(d_ind, d_ind_out, nelements * sizeof(double));
    
    cudaFree(d_key_out);
    cudaFree(d_ind_out);
    cudaFree(d_seq);
    cudaFree(d_stride);
    cudaFree(d_shflStride);
    cudaFree(d_oldDim);
}

template<class T> MW_HOST_DEVICE
void callThrustSortByKeyWithIndex(T* d_key, double* d_ind, int* d_seq, const char dir, int nelements) {
    
    if(dir == 'd') {
        thrust::stable_sort_by_key(thrust::device, d_key, d_key + nelements,
                                   thrust::make_zip_iterator(thrust::make_tuple(d_seq, d_ind)),
                                   customGreater<T>());
    } else if(dir == 'a') {
        thrust::stable_sort_by_key(thrust::device, d_key, d_key + nelements,
                                   thrust::make_zip_iterator(thrust::make_tuple(d_seq, d_ind)),
                                   customLesser<T>());
    }
                               
    thrust::stable_sort_by_key(thrust::device, d_seq, d_seq + nelements,
                               thrust::make_zip_iterator(thrust::make_tuple(d_key, d_ind)));
    cudaDeviceSynchronize();
}

template<class T> MW_HOST_DEVICE
void callThrustSortByKeyWithIndexDevice(T* d_key, double* d_ind, int* d_seq, const char dir, int nelements) {
    
    if(dir == 'd') {
        thrust::stable_sort_by_key(thrust::seq, d_key, d_key + nelements,
                                   thrust::make_zip_iterator(thrust::make_tuple(d_seq, d_ind)),
                                   customGreater<T>());
    } else if(dir == 'a') {
        thrust::stable_sort_by_key(thrust::seq, d_key, d_key + nelements,
                                   thrust::make_zip_iterator(thrust::make_tuple(d_seq, d_ind)),
                                   customLesser<T>());
    }
                               
    thrust::stable_sort_by_key(thrust::seq, d_seq, d_seq + nelements,
                               thrust::make_zip_iterator(thrust::make_tuple(d_key, d_ind)));
    cudaDeviceSynchronize();
}

template<class T> MW_HOST_DEVICE
void callThrustSortByKeyWithIndexSimple(T* d_key, double* d_ind, const char dir, int nelements) {

    thrust::sequence(thrust::device, d_ind, d_ind + nelements, 1); //starts at 1
    if(dir == 'd') {
        thrust::stable_sort_by_key(thrust::device, d_key, d_key + nelements, d_ind, customGreater<T>());
    } else if(dir == 'a') {
        thrust::stable_sort_by_key(thrust::device, d_key, d_key + nelements, d_ind, customLesser<T>());
    }
    cudaDeviceSynchronize();
}

template<class T> MW_HOST_DEVICE
void callThrustSortByKeyWithIndexSimpleDevice(T* d_key, double* d_ind, const char dir, int nelements) {

    thrust::sequence(thrust::seq, d_ind, d_ind + nelements, 1); //starts at 1
    if(dir == 'd') {
        thrust::stable_sort_by_key(thrust::seq, d_key, d_key + nelements, d_ind, customGreater<T>());
    } else if(dir == 'a') {
        thrust::stable_sort_by_key(thrust::seq, d_key, d_key + nelements, d_ind, customLesser<T>());
    }
    cudaDeviceSynchronize();
}

template <class T> MW_HOST_DEVICE
void thrustSortImplWithIndex(T* d_key, double* d_ind, int numDims, const int* dims, int sortAlongDim, const char dir, bool isRowMajor) {

    int nelements = 1;
    for(int i=0; i < numDims; i++)
        nelements *= dims[i];

    // 0-based indexing
    sortAlongDim--;

    int __compiling_host__ = 0;  // 0 = device

    #ifndef __CUDA_ARCH__
        __compiling_host__ = 1;   // 1 = host
    #endif

    if(nelements == 1           ||  // Sorting scalar
       sortAlongDim > numDims-1 ||  // Return back the input like matlab does
       dims[sortAlongDim] == 1 ) {  // Sorting along singleton dimension, nothing to be done
        if(__compiling_host__) {
            thrust::fill(thrust::device, d_ind, d_ind + nelements, 1); // all elements = 1
        } else {
            thrust::fill(thrust::seq, d_ind, d_ind + nelements, 1); // all elements = 1
        }
        cudaDeviceSynchronize();
        return;
    }
    
    // Sorting row/column vector
    if(numDims == 2 && (dims[0] == 1 || dims[1] == 1)) {
        if(__compiling_host__ == 1) { 
            callThrustSortByKeyWithIndexSimple<T>(d_key, d_ind, dir, nelements);
        } else {
            callThrustSortByKeyWithIndexSimpleDevice<T>(d_key, d_ind, dir, nelements);
        }
        return;
    }

    // Sorting 2d, 3d ... nd matrices along "sortAlongDim"
    if((sortAlongDim == 0 && !isRowMajor) || (sortAlongDim == numDims-1 && isRowMajor)) {
        sortAlongEdgeDimWithIndex<T>(d_key, d_ind, dims[sortAlongDim], dir, nelements, __compiling_host__);
    } else {
        if(__compiling_host__ == 1) {
            sortAlongOtherDimWithIndex<T>(d_key, d_ind, numDims, dims, sortAlongDim, dir, nelements, isRowMajor);
        } else {
            sortAlongOtherDimWithIndexDevice<T>(d_key, d_ind, numDims, dims, sortAlongDim, dir, nelements, isRowMajor);
        }
    }
    
}

#endif

#endif
