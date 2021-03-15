/* Copyright 2018 The MathWorks, Inc. */

#ifndef _MW_CUDNN_CONV_LAYER_IMPL_
#define _MW_CUDNN_CONV_LAYER_IMPL_

#include "MWCNNLayerImpl.hpp"

/**
 * Codegen class for Convolution layer implementation
 */
class MWConvLayerImpl : public MWCNNLayerImpl
{   
  private:
    
    int BHuHNDGoRwGRouCxeMbw;          
    int BLjrjqvCcCommiXWQLjs;          

    int BuyZFXzwOMxcePIbCLfl;
    int BkwhtPQUCQKchmmimoXs;
    int BlRIQPyqJZORKENzSdYf;

    int AdmgfUbRAfzFeYHxSnQr;
    int AwZQzUhuWVLGrWgLHRuM;

    int CTCbzQMDaLxINPbODdng;
    int CLOUhPjbgggWoXHTtmjC;
    int CpMjJjtGOeWOzwxpAAQP;
    int CqtPRJvHlGJFssiPzsOm;

    int FpguQZSermqZCMRiUfML;
    int FshVHIJMRAhtQirYPlZd;  

    float* vpXxoeEhdEosLSsYXkNG;
    float* IwKnaBoXVubIRYcxEJLH;

    // for temporary pre-padded input for asymmetric padding
    MWTensorBase* TaAJDyqFVJXfAfCJhOuU;
    int bUVPfnrJhLfHzOLUUrKk;
    int cCXqPFPPcoHzYMDpnUxQ;

    bool IIiwAtyrOtLzLWAUlTey;

  public:
    
    MWConvLayerImpl(MWCNNLayer* layer,
                    MWTargetNetworkImpl* ntwk_impl,
                    int filt_H,
                    int filt_W,
                    int numGrps,
                    int numChannels,
                    int numFilts,
                    int FpguQZSermqZCMRiUfML,
                    int FshVHIJMRAhtQirYPlZd,
                    int CTCbzQMDaLxINPbODdng,
                    int CLOUhPjbgggWoXHTtmjC,
                    int CpMjJjtGOeWOzwxpAAQP,
                    int CqtPRJvHlGJFssiPzsOm,
                    int AdmgfUbRAfzFeYHxSnQr,
                    int AwZQzUhuWVLGrWgLHRuM,
                    const char* xHViLEwTujGGrPZZgmbF,
                    const char* JwxFdqOKggeawILBfGgg);
    
    ~MWConvLayerImpl(){}

    void predict();
    void cleanup();
    void postSetup();
    void propagateSize();
    void allocate();
    void deallocate();
    
  private:
    
    void loadWeights(const char*);
    void loadBias(const char*);
    void getConvAlgoTuned();
    void getConvAlgoWorkSpaceLimit();
		
  private:
    
    cudnnConvolutionDescriptor_t  NNhshzQGJHLSGjDiVerE;
    cudnnConvolutionFwdAlgo_t     NDjzAZSYJuWymuKDNZYB;

    cudnnFilterDescriptor_t       QVgVGfoCXYiYXzPhvVPX;
    cudnnTensorDescriptor_t       JgLfgHrHMEMmMYTettJF;

    cudnnTensorDescriptor_t       XYbzSmRQGatVJtGmDZSo;

};

#endif
