ifneq ($(PREBUILT_MTCNN_NCNN), true)

LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)

LOCAL_MODULE := mtcnn_ncnn-$(TARGET_ARCH_ABI)
LOCAL_MODULE_TAGS := optional

LOCAL_SRC_FILES := mtcnn.cpp \
                   xmsdl_mtcnn.cpp \
                   xm_squeeze_ncnn.cpp \
                   ncnn/allocator.cpp \
                   ncnn/benchmark.cpp \
                   ncnn/blob.cpp \
                   ncnn/cpu.cpp \
                   ncnn/layer.cpp \
                   ncnn/layer/absval.cpp \
                   ncnn/layer/arm/absval_arm.cpp \
                   ncnn/layer/arm/batchnorm_arm.cpp \
                   ncnn/layer/arm/bias_arm.cpp \
                   ncnn/layer/arm/convolution_arm.cpp \
                   ncnn/layer/arm/convolutiondepthwise_arm.cpp \
                   ncnn/layer/arm/deconvolution_arm.cpp \
                   ncnn/layer/arm/deconvolutiondepthwise_arm.cpp \
                   ncnn/layer/arm/dequantize_arm.cpp \
                   ncnn/layer/arm/eltwise_arm.cpp \
                   ncnn/layer/arm/innerproduct_arm.cpp \
                   ncnn/layer/arm/lrn_arm.cpp \
                   ncnn/layer/arm/pooling_arm.cpp \
                   ncnn/layer/arm/prelu_arm.cpp \
                   ncnn/layer/arm/quantize_arm.cpp \
                   ncnn/layer/arm/relu_arm.cpp \
                   ncnn/layer/arm/scale_arm.cpp \
                   ncnn/layer/arm/sigmoid_arm.cpp \
                   ncnn/layer/arm/softmax_arm.cpp \
                   ncnn/layer/batchnorm.cpp \
                   ncnn/layer/bias.cpp \
                   ncnn/layer/binaryop.cpp \
                   ncnn/layer/bnll.cpp \
                   ncnn/layer/clip.cpp \
                   ncnn/layer/concat.cpp \
                   ncnn/layer/convolution.cpp \
                   ncnn/layer/convolutiondepthwise.cpp \
                   ncnn/layer/crop.cpp \
                   ncnn/layer/deconvolution.cpp \
                   ncnn/layer/deconvolutiondepthwise.cpp \
                   ncnn/layer/dequantize.cpp \
                   ncnn/layer/detectionoutput.cpp \
                   ncnn/layer/dropout.cpp \
                   ncnn/layer/eltwise.cpp \
                   ncnn/layer/elu.cpp \
                   ncnn/layer/embed.cpp \
                   ncnn/layer/exp.cpp \
                   ncnn/layer/expanddims.cpp \
                   ncnn/layer/flatten.cpp \
                   ncnn/layer/innerproduct.cpp \
                   ncnn/layer/input.cpp \
                   ncnn/layer/instancenorm.cpp \
                   ncnn/layer/interp.cpp \
                   ncnn/layer/log.cpp \
                   ncnn/layer/lrn.cpp \
                   ncnn/layer/memorydata.cpp \
                   ncnn/layer/mvn.cpp \
                   ncnn/layer/normalize.cpp \
                   ncnn/layer/padding.cpp \
                   ncnn/layer/permute.cpp \
                   ncnn/layer/pooling.cpp \
                   ncnn/layer/power.cpp \
                   ncnn/layer/prelu.cpp \
                   ncnn/layer/priorbox.cpp \
                   ncnn/layer/proposal.cpp \
                   ncnn/layer/quantize.cpp \
                   ncnn/layer/reduction.cpp \
                   ncnn/layer/relu.cpp \
                   ncnn/layer/reorg.cpp \
                   ncnn/layer/reshape.cpp \
                   ncnn/layer/roipooling.cpp \
                   ncnn/layer/scale.cpp \
                   ncnn/layer/shufflechannel.cpp \
                   ncnn/layer/sigmoid.cpp \
                   ncnn/layer/slice.cpp \
                   ncnn/layer/softmax.cpp \
                   ncnn/layer/split.cpp \
                   ncnn/layer/squeeze.cpp \
                   ncnn/layer/tanh.cpp \
                   ncnn/layer/threshold.cpp \
                   ncnn/layer/unaryop.cpp \
                   ncnn/layer/yolodetectionoutput.cpp \
                   ncnn/mat.cpp \
                   ncnn/mat_pixel.cpp \
                   ncnn/modelbin.cpp \
                   ncnn/net.cpp \
                   ncnn/opencv.cpp \
                   ncnn/paramdict.cpp

LOCAL_C_INCLUDES := $(LOCAL_PATH)/include
LOCAL_C_INCLUDES += $(LOCAL_PATH)/ncnn
LOCAL_C_INCLUDES += $(LOCAL_PATH)/ncnn/layer
LOCAL_C_INCLUDES += $(LOCAL_PATH)/ncnn/layer/arm
LOCAL_C_INCLUDES += $(LOCAL_PATH)/../../../ijkyuv/include

LOCAL_STATIC_LIBRARIES := yuv_static

LOCAL_CFLAGS := -O2 -fvisibility=hidden -fomit-frame-pointer -fstrict-aliasing -ffunction-sections -fdata-sections -ffast-math
LOCAL_CPPFLAGS := -O2 -fvisibility=hidden -fvisibility-inlines-hidden -fomit-frame-pointer -fstrict-aliasing -ffunction-sections -fdata-sections -ffast-math
LOCAL_LDFLAGS += -Wl,--gc-sections
LOCAL_CPPFLAGS += -std=c++11
LOCAL_CPPFLAGS += -mfpu=neon

LOCAL_CFLAGS += -fopenmp
LOCAL_CPPFLAGS += -fopenmp
LOCAL_LDFLAGS += -fopenmp

LOCAL_LDLIBS := -lz -llog

include $(BUILD_SHARED_LIBRARY)
endif
