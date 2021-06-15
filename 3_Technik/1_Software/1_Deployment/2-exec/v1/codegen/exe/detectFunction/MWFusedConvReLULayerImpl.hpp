/* Copyright 2018 The MathWorks, Inc. */

#ifndef __FUSED_CONV_RELU_LAYER_IMPL_HPP
#define __FUSED_CONV_RELU_LAYER_IMPL_HPP

#include "MWFusedConvReLULayer.hpp"
#include "MWCNNLayerImpl.hpp"
#include "MWTargetNetworkImpl.hpp"


class MWFusedConvReLULayerImpl: public MWCNNLayerImpl
{

private:
    int BHuHNDGoRwGRouCxeMbw;           //Filter height for CONV and FC
    int BLjrjqvCcCommiXWQLjs;            //Filter width for CONV and FC

    int BuyZFXzwOMxcePIbCLfl;
    int BkwhtPQUCQKchmmimoXs;
    int BlRIQPyqJZORKENzSdYf;

    int FpguQZSermqZCMRiUfML;
    int FshVHIJMRAhtQirYPlZd;
    int CTCbzQMDaLxINPbODdng;
    int CLOUhPjbgggWoXHTtmjC;
    int CpMjJjtGOeWOzwxpAAQP;
    int CqtPRJvHlGJFssiPzsOm;
    int AdmgfUbRAfzFeYHxSnQr;
    int AwZQzUhuWVLGrWgLHRuM;

    int fvTCtkwXgyScJYogJVFU;

    float* vpXxoeEhdEosLSsYXkNG;
    float* IwKnaBoXVubIRYcxEJLH;
    MWTensorBase* UEESbUvbMihFnquvuFij; // for pre-padded input
    int bUVPfnrJhLfHzOLUUrKk;
    int cCXqPFPPcoHzYMDpnUxQ;

    float* WprSrhAStKGxyXeoxETy;

    float* HhKGcPZwrclEFnIdWerH; // Scaling factor for addition

    bool IIiwAtyrOtLzLWAUlTey;

    // Temporary buffer for Xinput, CuDNN 8 upgrade
    float* GFggoMvRWucDMqzlWzCl;    
    
public:
    MWFusedConvReLULayerImpl(MWCNNLayer*, MWTargetNetworkImpl*,
                             int, int,
                             int, int, int,
                             int, int,
                             int, int, int, int,
                             int, int,
                             int,
                             const char*, const char*);
    ~MWFusedConvReLULayerImpl();

    void predict();
    void cleanup();
    void propagateSize();
    void allocate();
    void deallocate();
    void postSetup();

private:
    void loadWeights(const char*);
    void loadBias(const char*);
    void getConvAlgoTuned();
    void getConvAlgoWorkSpaceLimit();
    void fixConvAlgo(); // g1916490

    void setalpha2Ptr(float* alpha2){ HhKGcPZwrclEFnIdWerH = alpha2;}
    float* getalpha2Ptr() {return HhKGcPZwrclEFnIdWerH;}


private:
    cudnnConvolutionDescriptor_t  NNhshzQGJHLSGjDiVerE;
    cudnnConvolutionFwdAlgo_t     NDjzAZSYJuWymuKDNZYB;

    cudnnFilterDescriptor_t       QVgVGfoCXYiYXzPhvVPX;
    cudnnTensorDescriptor_t       JgLfgHrHMEMmMYTettJF;

    cudnnTensorDescriptor_t      XYbzSmRQGatVJtGmDZSo;

    cudnnActivationDescriptor_t   oJUVMnJggjhEdQLWzIUC;

};

#endif
