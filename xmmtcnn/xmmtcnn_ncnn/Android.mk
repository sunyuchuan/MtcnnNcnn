ifeq ($(SUPPORT_MTCNN_NCNN),true)

XMMTCNN_PATH := $(call my-dir)
include $(call all-subdir-makefiles)
LOCAL_PATH := $(XMMTCNN_PATH)

ifeq ($(PREBUILT_MTCNN_NCNN), true)
include $(CLEAR_VARS)
LOCAL_MODULE := mtcnn_ncnn-$(TARGET_ARCH_ABI)
LOCAL_SRC_FILES := $(LOCAL_PATH)/lib/libmtcnn_ncnn-$(TARGET_ARCH_ABI).so
include $(PREBUILT_SHARED_LIBRARY)
endif

include $(CLEAR_VARS)

ifeq ($(TARGET_ARCH_ABI),armeabi-v7a)
LOCAL_CFLAGS += -mfloat-abi=softfp
LOCAL_CFLAGS += -DARMV7A
endif

ifneq ($(TARGET_ARCH_ABI),armeabi)
LOCAL_CFLAGS += -DSUPPORT_OPENGLES30
LOCAL_LDLIBS += -lGLESv3
LOCAL_ARM_NEON := true
LOCAL_CFLAGS += -DSUPPORT_ARM_NEON
endif

LOCAL_CFLAGS += -std=c99
LOCAL_CFLAGS += -Wno-deprecated-declarations
LOCAL_LDLIBS += -lz -llog -ljnigraphics

LOCAL_MODULE := xmmtcnn_ncnn-$(TARGET_ARCH_ABI)
LOCAL_MODULE_TAGS := optional

LOCAL_SRC_FILES := xmsdl_mtcnn_jni.c \
                   xm_mtcnn_ncnn.c \
                   xm_mtcnn_ncnn_jni.c

LOCAL_C_INCLUDES += $(LOCAL_PATH)/mtcnn_ncnn
LOCAL_C_INCLUDES += $(LOCAL_PATH)/mtcnn_ncnn/include
LOCAL_C_INCLUDES += $(realpath $(LOCAL_PATH)/../..)
LOCAL_C_INCLUDES += $(realpath $(LOCAL_PATH)/../../ijksdl)
LOCAL_C_INCLUDES += $(realpath $(LOCAL_PATH)/../../ijkj4a)
LOCAL_C_INCLUDES += $(realpath $(LOCAL_PATH)/../../xmrecorder/xm_media_recorder)
LOCAL_C_INCLUDES += $(MY_APP_FFMPEG_INCLUDE_PATH)

LOCAL_SHARED_LIBRARIES := ijksdl-$(TARGET_ARCH_ABI) ijkffmpeg-$(TARGET_ARCH_ABI) mtcnn_ncnn-$(TARGET_ARCH_ABI)

LOCAL_STATIC_LIBRARIES := yuv_static

include $(BUILD_SHARED_LIBRARY)
endif
