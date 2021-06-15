###########################################################################
## Makefile generated for component 'detectFunction'. 
## 
## Makefile     : detectFunction_rtw.mk
## Generated on : Fri Mar 12 17:28:09 2021
## Final product: ./detectFunction.a
## Product type : static-library
## 
###########################################################################

###########################################################################
## MACROS
###########################################################################

# Macro Descriptions:
# PRODUCT_NAME            Name of the system to build
# MAKEFILE                Name of this makefile
# MODELLIB                Static library target

PRODUCT_NAME              = detectFunction
MAKEFILE                  = detectFunction_rtw.mk
MATLAB_ROOT               = $(MATLAB_WORKSPACE)/C/Program_Files/MATLAB/R2020b
MATLAB_BIN                = $(MATLAB_WORKSPACE)/C/Program_Files/MATLAB/R2020b/bin
MATLAB_ARCH_BIN           = $(MATLAB_BIN)/win64
START_DIR                 = $(MATLAB_WORKSPACE)/C/Users/Marco/projects/meka-project/3-technics/1-deployment/1-static-lib/v1_1/codegen/lib/detectFunction
TGT_FCN_LIB               = ISO_C++
SOLVER_OBJ                = 
CLASSIC_INTERFACE         = 0
MODEL_HAS_DYNAMICALLY_LOADED_SFCNS = 
RELATIVE_PATH_TO_ANCHOR   = .
C_STANDARD_OPTS           = 
CPP_STANDARD_OPTS         = 
MODELLIB                  = detectFunction.a

###########################################################################
## TOOLCHAIN SPECIFICATIONS
###########################################################################

# Toolchain Name:          NVCC for NVIDIA Embedded Processors
# Supported Version(s):    
# ToolchainInfo Version:   2020b
# Specification Revision:  1.0
# 

#-----------
# MACROS
#-----------

CCOUTPUTFLAG = --output_file=
LDOUTPUTFLAG = --output_file=

TOOLCHAIN_SRCS = 
TOOLCHAIN_INCS = 
TOOLCHAIN_LIBS = -lm -lm

#------------------------
# BUILD TOOL COMMANDS
#------------------------

# C Compiler: NVCC for NVIDIA Embedded Processors1.0 NVIDIA CUDA C Compiler Driver
CC = nvcc

# Linker: NVCC for NVIDIA Embedded Processors1.0 NVIDIA CUDA C Linker
LD = nvcc

# C++ Compiler: NVCC for NVIDIA Embedded Processors1.0 NVIDIA CUDA C++ Compiler Driver
CPP = nvcc

# C++ Linker: NVCC for NVIDIA Embedded Processors1.0 NVIDIA CUDA C++ Linker
CPP_LD = nvcc

# Archiver: NVCC for NVIDIA Embedded Processors1.0 Archiver
AR = ar

# MEX Tool: MEX Tool
MEX_PATH = $(MATLAB_ARCH_BIN)
MEX = $(MEX_PATH)/mex

# Download: Download
DOWNLOAD =

# Execute: Execute
EXECUTE = $(PRODUCT)

# Builder: Make Tool
MAKE = make


#-------------------------
# Directives/Utilities
#-------------------------

CDEBUG              = -g -G
C_OUTPUT_FLAG       = -o
LDDEBUG             = -g -G
OUTPUT_FLAG         = -o
CPPDEBUG            = -g -G
CPP_OUTPUT_FLAG     = -o
CPPLDDEBUG          = -g -G
OUTPUT_FLAG         = -o
ARDEBUG             =
STATICLIB_OUTPUT_FLAG =
MEX_DEBUG           = -g
RM                  =
ECHO                = echo
MV                  =
RUN                 =

#--------------------------------------
# "Faster Runs" Build Configuration
#--------------------------------------

ARFLAGS              = -ruvs
CFLAGS               = -rdc=true -Xcudafe "--diag_suppress=unsigned_compare_with_zero" \
                       -c \
                       -Xcompiler -MMD,-MP \
                       -O2
CPPFLAGS             = -rdc=true -Xcudafe "--diag_suppress=unsigned_compare_with_zero" \
                       -c \
                       -Xcompiler -MMD,-MP \
                       -O2
CPP_LDFLAGS          = -lm -lrt -ldl \
                       -Xlinker -rpath,/usr/lib32 -Xnvlink -w -lcudart -lcuda -Wno-deprecated-gpu-targets
CPP_SHAREDLIB_LDFLAGS  = -shared  \
                         -lm -lrt -ldl \
                         -Xlinker -rpath,/usr/lib32 -Xnvlink -w -lcudart -lcuda -Wno-deprecated-gpu-targets
DOWNLOAD_FLAGS       =
EXECUTE_FLAGS        =
LDFLAGS              = -lm -lrt -ldl \
                       -Xlinker -rpath,/usr/lib32 -Xnvlink -w -lcudart -lcuda -Wno-deprecated-gpu-targets
MEX_CPPFLAGS         =
MEX_CPPLDFLAGS       =
MEX_CFLAGS           =
MEX_LDFLAGS          =
MAKE_FLAGS           = -f $(MAKEFILE)
SHAREDLIB_LDFLAGS    = -shared  \
                       -lm -lrt -ldl \
                       -Xlinker -rpath,/usr/lib32 -Xnvlink -w -lcudart -lcuda -Wno-deprecated-gpu-targets



###########################################################################
## OUTPUT INFO
###########################################################################

PRODUCT = ./detectFunction.a
PRODUCT_TYPE = "static-library"
BUILD_TYPE = "Static Library"

###########################################################################
## INCLUDE PATHS
###########################################################################

INCLUDES_BUILDINFO = -I$(START_DIR) -I$(MATLAB_WORKSPACE)/C/Users/Marco/projects/meka-project/3-technics/1-deployment/1-static-lib/v1_1 -I$(MATLAB_WORKSPACE)/C/ProgramData/MATLAB/SupportPackages/R2020b/toolbox/target/supportpackages/nvidia/sources/server -I$(MATLAB_WORKSPACE)/C/ProgramData/MATLAB/SupportPackages/R2020b/toolbox/target/supportpackages/nvidia/include -I$(MATLAB_WORKSPACE)/C/ProgramData/MATLAB/SupportPackages/R2020b/toolbox/target/supportpackages/nvidia/sources/utils -I$(MATLAB_ROOT)/toolbox/coder/rtiostream/src/utils -I$(MATLAB_ROOT)/extern/include

INCLUDES = $(INCLUDES_BUILDINFO)

###########################################################################
## DEFINES
###########################################################################

DEFINES_ = -D_MW_MATLABTGT_ -DMW_CUDA_ARCH=350 -D__MW_TARGET_USE_HARDWARE_RESOURCES_H__ -DMW_DL_DATA_PATH="$(START_DIR)" -DMW_SCHED_OTHER=1
DEFINES_CUSTOM = 
DEFINES_SKIPFORSIL = -D__linux__ -DARM_PROJECT -D_USE_TARGET_UDP_ -D_RUNONTARGETHARDWARE_BUILD_ -DSTACK_SIZE=200000
DEFINES_STANDARD = -DMODEL=detectFunction

DEFINES = $(DEFINES_) $(DEFINES_CUSTOM) $(DEFINES_SKIPFORSIL) $(DEFINES_STANDARD)

###########################################################################
## SOURCE FILES
###########################################################################

SRCS = $(MATLAB_WORKSPACE)/MWConcatenationLayer.cpp $(MATLAB_WORKSPACE)/MWConvLayer.cpp $(MATLAB_WORKSPACE)/MWExponentialLayer.cpp $(MATLAB_WORKSPACE)/MWFusedConvReLULayer.cpp $(MATLAB_WORKSPACE)/MWSigmoidLayer.cpp $(MATLAB_WORKSPACE)/MWYoloExtractionLayer.cpp $(MATLAB_WORKSPACE)/MWYoloSoftmaxLayer.cpp $(MATLAB_WORKSPACE)/cnn_api.cpp $(MATLAB_WORKSPACE)/MWCNNLayerImpl.cu $(MATLAB_WORKSPACE)/MWConcatenationLayerImpl.cu $(MATLAB_WORKSPACE)/MWConvLayerImpl.cu $(MATLAB_WORKSPACE)/MWExponentialLayerImpl.cu $(MATLAB_WORKSPACE)/MWExponentialLayerImplKernel.cu $(MATLAB_WORKSPACE)/MWFusedConvReLULayerImpl.cu $(MATLAB_WORKSPACE)/MWSigmoidLayerImpl.cu $(MATLAB_WORKSPACE)/MWYoloExtractionLayerImpl.cu $(MATLAB_WORKSPACE)/MWYoloExtractionLayerImplKernel.cu $(MATLAB_WORKSPACE)/MWYoloSoftmaxLayerImpl.cu $(MATLAB_WORKSPACE)/MWTargetNetworkImpl.cu $(MATLAB_WORKSPACE)/MWCustomLayerForCuDNN.cpp $(MATLAB_WORKSPACE)/C/ProgramData/MATLAB/SupportPackages/R2020b/toolbox/target/supportpackages/nvidia/sources/server/frameReader.c $(MATLAB_WORKSPACE)/C/ProgramData/MATLAB/SupportPackages/R2020b/toolbox/target/supportpackages/nvidia/sources/server/getCameraProps.c $(START_DIR)/detectFunction_data.cu $(START_DIR)/rt_nonfinite.cu $(START_DIR)/rtGetNaN.cu $(START_DIR)/rtGetInf.cu $(START_DIR)/detectFunction_initialize.cu $(START_DIR)/detectFunction_terminate.cu $(START_DIR)/detectFunction.cu $(START_DIR)/DeepLearningNetwork.cu $(START_DIR)/activations.cu $(START_DIR)/reduce_codegen.cu

ALL_SRCS = $(SRCS)

###########################################################################
## OBJECTS
###########################################################################

OBJS = MWConcatenationLayer.o MWConvLayer.o MWExponentialLayer.o MWFusedConvReLULayer.o MWSigmoidLayer.o MWYoloExtractionLayer.o MWYoloSoftmaxLayer.o cnn_api.o MWCNNLayerImpl.o MWConcatenationLayerImpl.o MWConvLayerImpl.o MWExponentialLayerImpl.o MWExponentialLayerImplKernel.o MWFusedConvReLULayerImpl.o MWSigmoidLayerImpl.o MWYoloExtractionLayerImpl.o MWYoloExtractionLayerImplKernel.o MWYoloSoftmaxLayerImpl.o MWTargetNetworkImpl.o MWCustomLayerForCuDNN.o frameReader.o getCameraProps.o detectFunction_data.o rt_nonfinite.o rtGetNaN.o rtGetInf.o detectFunction_initialize.o detectFunction_terminate.o detectFunction.o DeepLearningNetwork.o activations.o reduce_codegen.o

ALL_OBJS = $(OBJS)

###########################################################################
## PREBUILT OBJECT FILES
###########################################################################

PREBUILT_OBJS = 

###########################################################################
## LIBRARIES
###########################################################################

LIBS = 

###########################################################################
## SYSTEM LIBRARIES
###########################################################################

SYSTEM_LIBS =  -lm -lstdc++ -lcufft -lcublas -lcusolver

###########################################################################
## ADDITIONAL TOOLCHAIN FLAGS
###########################################################################

#---------------
# C Compiler
#---------------

CFLAGS_ = -Xcompiler `pkg-config --cflags --libs gstreamer-app-1.0`
CFLAGS_CU_OPTS = -arch sm_35 
CFLAGS_BASIC = $(DEFINES) $(INCLUDES)

CFLAGS += $(CFLAGS_) $(CFLAGS_CU_OPTS) $(CFLAGS_BASIC)

#-----------------
# C++ Compiler
#-----------------

CPPFLAGS_ = -Xcompiler `pkg-config --cflags --libs gstreamer-app-1.0`
CPPFLAGS_CU_OPTS = -arch sm_35 
CPPFLAGS_BASIC = $(DEFINES) $(INCLUDES)

CPPFLAGS += $(CPPFLAGS_) $(CPPFLAGS_CU_OPTS) $(CPPFLAGS_BASIC)

#---------------
# C++ Linker
#---------------

CPP_LDFLAGS_ = -Xcompiler `pkg-config --cflags --libs gstreamer-app-1.0` -lcudnn -lcublas -arch sm_35 

CPP_LDFLAGS += $(CPP_LDFLAGS_)

#------------------------------
# C++ Shared Library Linker
#------------------------------

CPP_SHAREDLIB_LDFLAGS_ = -Xcompiler `pkg-config --cflags --libs gstreamer-app-1.0` -lcudnn -lcublas -arch sm_35 

CPP_SHAREDLIB_LDFLAGS += $(CPP_SHAREDLIB_LDFLAGS_)

#-----------
# Linker
#-----------

LDFLAGS_ = -Xcompiler `pkg-config --cflags --libs gstreamer-app-1.0` -lcudnn -lcublas -arch sm_35 

LDFLAGS += $(LDFLAGS_)

#--------------------------
# Shared Library Linker
#--------------------------

SHAREDLIB_LDFLAGS_ = -Xcompiler `pkg-config --cflags --libs gstreamer-app-1.0` -lcudnn -lcublas -arch sm_35 

SHAREDLIB_LDFLAGS += $(SHAREDLIB_LDFLAGS_)

###########################################################################
## INLINED COMMANDS
###########################################################################


DERIVED_SRCS = $(subst .o,.dep,$(OBJS))

build:

%.dep:



-include codertarget_assembly_flags.mk
-include *.dep


###########################################################################
## PHONY TARGETS
###########################################################################

.PHONY : all build clean info prebuild download execute


all : build
	echo "### Successfully generated all binary outputs."


build : prebuild $(PRODUCT)


prebuild : 


download : $(PRODUCT)


execute : download


###########################################################################
## FINAL TARGET
###########################################################################

#---------------------------------
# Create a static library         
#---------------------------------

$(PRODUCT) : $(OBJS) $(PREBUILT_OBJS)
	echo "### Creating static library "$(PRODUCT)" ..."
	$(AR) $(ARFLAGS)  $(PRODUCT) $(OBJS)
	echo "### Created: $(PRODUCT)"


###########################################################################
## INTERMEDIATE TARGETS
###########################################################################

#---------------------
# SOURCE-TO-OBJECT
#---------------------

%.o : %.c
	$(CC) $(CFLAGS) -o $@ $<


%.o : %.c
	$(CC) $(CFLAGS) -o $@ $<


%.o : %.cpp
	$(CPP) $(CPPFLAGS) -o $@ $<


%.o : %.cu
	$(CPP) $(CPPFLAGS) -o $@ $<


%.o : $(RELATIVE_PATH_TO_ANCHOR)/%.c
	$(CC) $(CFLAGS) -o $@ $<


%.o : $(RELATIVE_PATH_TO_ANCHOR)/%.c
	$(CC) $(CFLAGS) -o $@ $<


%.o : $(RELATIVE_PATH_TO_ANCHOR)/%.cpp
	$(CPP) $(CPPFLAGS) -o $@ $<


%.o : $(RELATIVE_PATH_TO_ANCHOR)/%.cu
	$(CPP) $(CPPFLAGS) -o $@ $<


%.o : $(START_DIR)/%.c
	$(CC) $(CFLAGS) -o $@ $<


%.o : $(START_DIR)/%.c
	$(CC) $(CFLAGS) -o $@ $<


%.o : $(START_DIR)/%.cpp
	$(CPP) $(CPPFLAGS) -o $@ $<


%.o : $(START_DIR)/%.cu
	$(CPP) $(CPPFLAGS) -o $@ $<


%.o : $(MATLAB_WORKSPACE)/C/Users/Marco/projects/meka-project/3-technics/1-deployment/1-static-lib/v1_1/%.c
	$(CC) $(CFLAGS) -o $@ $<


%.o : $(MATLAB_WORKSPACE)/C/Users/Marco/projects/meka-project/3-technics/1-deployment/1-static-lib/v1_1/%.c
	$(CC) $(CFLAGS) -o $@ $<


%.o : $(MATLAB_WORKSPACE)/C/Users/Marco/projects/meka-project/3-technics/1-deployment/1-static-lib/v1_1/%.cpp
	$(CPP) $(CPPFLAGS) -o $@ $<


%.o : $(MATLAB_WORKSPACE)/C/Users/Marco/projects/meka-project/3-technics/1-deployment/1-static-lib/v1_1/%.cu
	$(CPP) $(CPPFLAGS) -o $@ $<


frameReader.o : $(MATLAB_WORKSPACE)/C/ProgramData/MATLAB/SupportPackages/R2020b/toolbox/target/supportpackages/nvidia/sources/server/frameReader.c
	$(CC) $(CFLAGS) -o $@ $<


getCameraProps.o : $(MATLAB_WORKSPACE)/C/ProgramData/MATLAB/SupportPackages/R2020b/toolbox/target/supportpackages/nvidia/sources/server/getCameraProps.c
	$(CC) $(CFLAGS) -o $@ $<


detectFunction_data.o : $(START_DIR)/detectFunction_data.cu
	$(CPP) $(CPPFLAGS) -o $@ $<


rt_nonfinite.o : $(START_DIR)/rt_nonfinite.cu
	$(CPP) $(CPPFLAGS) -o $@ $<


rtGetNaN.o : $(START_DIR)/rtGetNaN.cu
	$(CPP) $(CPPFLAGS) -o $@ $<


rtGetInf.o : $(START_DIR)/rtGetInf.cu
	$(CPP) $(CPPFLAGS) -o $@ $<


detectFunction_initialize.o : $(START_DIR)/detectFunction_initialize.cu
	$(CPP) $(CPPFLAGS) -o $@ $<


detectFunction_terminate.o : $(START_DIR)/detectFunction_terminate.cu
	$(CPP) $(CPPFLAGS) -o $@ $<


detectFunction.o : $(START_DIR)/detectFunction.cu
	$(CPP) $(CPPFLAGS) -o $@ $<


DeepLearningNetwork.o : $(START_DIR)/DeepLearningNetwork.cu
	$(CPP) $(CPPFLAGS) -o $@ $<


activations.o : $(START_DIR)/activations.cu
	$(CPP) $(CPPFLAGS) -o $@ $<


reduce_codegen.o : $(START_DIR)/reduce_codegen.cu
	$(CPP) $(CPPFLAGS) -o $@ $<


###########################################################################
## DEPENDENCIES
###########################################################################

$(ALL_OBJS) : rtw_proj.tmw $(MAKEFILE)


###########################################################################
## MISCELLANEOUS TARGETS
###########################################################################

info : 
	echo "### PRODUCT = $(PRODUCT)"
	echo "### PRODUCT_TYPE = $(PRODUCT_TYPE)"
	echo "### BUILD_TYPE = $(BUILD_TYPE)"
	echo "### INCLUDES = $(INCLUDES)"
	echo "### DEFINES = $(DEFINES)"
	echo "### ALL_SRCS = $(ALL_SRCS)"
	echo "### ALL_OBJS = $(ALL_OBJS)"
	echo "### LIBS = $(LIBS)"
	echo "### MODELREF_LIBS = $(MODELREF_LIBS)"
	echo "### SYSTEM_LIBS = $(SYSTEM_LIBS)"
	echo "### TOOLCHAIN_LIBS = $(TOOLCHAIN_LIBS)"
	echo "### CFLAGS = $(CFLAGS)"
	echo "### LDFLAGS = $(LDFLAGS)"
	echo "### SHAREDLIB_LDFLAGS = $(SHAREDLIB_LDFLAGS)"
	echo "### CPPFLAGS = $(CPPFLAGS)"
	echo "### CPP_LDFLAGS = $(CPP_LDFLAGS)"
	echo "### CPP_SHAREDLIB_LDFLAGS = $(CPP_SHAREDLIB_LDFLAGS)"
	echo "### ARFLAGS = $(ARFLAGS)"
	echo "### MEX_CFLAGS = $(MEX_CFLAGS)"
	echo "### MEX_CPPFLAGS = $(MEX_CPPFLAGS)"
	echo "### MEX_LDFLAGS = $(MEX_LDFLAGS)"
	echo "### MEX_CPPLDFLAGS = $(MEX_CPPLDFLAGS)"
	echo "### DOWNLOAD_FLAGS = $(DOWNLOAD_FLAGS)"
	echo "### EXECUTE_FLAGS = $(EXECUTE_FLAGS)"
	echo "### MAKE_FLAGS = $(MAKE_FLAGS)"


clean : 
	$(ECHO) "### Deleting all derived files..."
	$(RM) $(PRODUCT)
	$(RM) $(ALL_OBJS)
	$(RM) *.c.dep
	$(RM) *.cpp.dep .cu.dep
	$(ECHO) "### Deleted all derived files."


