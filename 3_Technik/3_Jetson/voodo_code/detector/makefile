PRODUCT        = detector
START_DIR      = /home/jetson/voodoocode

# C++ Compiler: NVCC for NVIDIA Embedded Processors1.0 NVIDIA CUDA C++ Compiler Driver
CPP            = nvcc
# C++ Linker: NVCC for NVIDIA Embedded Processors1.0 NVIDIA CUDA C++ Linker
CPP_LD         = nvcc

# All needed flags and libs for compiling cuda/c++ code
CPPFLAGS       = -rdc=true -Xcudafe "--diag_suppress=unsigned_compare_with_zero" -c -Xcompiler -MMD,-MP -O2
CPPFLAGS_      = -Xcompiler `pkg-config --cflags --libs gstreamer-app-1.0`
CPPFLAGS_CU_OPTS = -arch sm_35 
DEFINES_       = -D_MW_MATLABTGT_ -DMW_CUDA_ARCH=350 -D__MW_TARGET_USE_HARDWARE_RESOURCES_H__ -DMW_DL_DATA_PATH="$(START_DIR)" -DMW_SCHED_OTHER=1
CPPCUSTOM      = -I../dependencies/include
CPPFLAGS_BASIC = $(DEFINES) $(CPPCUSTOM)
CPPFLAGS       += $(CPPFLAGS_) $(CPPFLAGS_CU_OPTS) $(CPPFLAGS_BASIC)

# All needed libraries for linking
CPP_LDFLAGS    = -lm -lrt -ldl -Xlinker -rpath,/usr/lib32 -Xnvlink -w -lcudart -lcuda -Wno-deprecated-gpu-targets
CPP_LDFLAGS_   = -Xcompiler `pkg-config --cflags --libs gstreamer-app-1.0` -lcudnn -lcublas -arch sm_35
CPP_LDCUSTOM      = -L../dependencies/lib -ldocopt -lfmt -ldetectFunction -ltransmission
CPP_LDFLAGS    += $(CPP_LDFLAGS_) $(CPP_LDCUSTOM)

SYSTEM_LIBS    =  -lm -lstdc++ -lcufft -lcublas -lcusolver

# detectFunction.a is static library -> it ist dependend on ~/yoloCNNForDetection/... path
CUOBJS         =  $(wildcard *.cu)
CPPOBJS        =  $(wildcard *.cpp)
OBJS           =  $(patsubst %.cu,%.o,$(CUOBJS))
OBJS           += $(patsubst %.cpp,%.o,$(CPPOBJS))
DEPS           =  $(patsubst %.o,%.d,$(OBJS))

###########################################################################
## FINAL TARGET
###########################################################################

#-------------------------------------------
# Create a standalone executable            
#-------------------------------------------

$(PRODUCT) : $(OBJS)
	echo "### Creating standalone executable "$(PRODUCT)" ..."
	$(CPP_LD) $(CPP_LDFLAGS) -o $(PRODUCT) $(OBJS) $(SYSTEM_LIBS)
	echo "### Created: $(PRODUCT)"

###########################################################################
## INTERMEDIATE TARGETS
###########################################################################

#---------------------
# SOURCE-TO-OBJECT
#---------------------

%.o : %.cu
	$(CPP) $(CPPFLAGS) -o $@ $<

%.o : %.cpp
	$(CPP) $(CPPFLAGS) -o $@ $<


clean:
	rm $(OBJS) $(DEPS) 
