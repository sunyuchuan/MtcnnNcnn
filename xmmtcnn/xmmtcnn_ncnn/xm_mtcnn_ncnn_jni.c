//
// Created by sunyc on 19-5-27.
//
#include "xm_mtcnn_ncnn_jni.h"
#include "xm_mtcnn_ncnn.h"
#include "ijksdl_log.h"
#include "android/ijksdl_android_jni.h"
#include "ijksdl_misc.h"
#include "libyuv.h"
#include <android/log.h>
#ifdef SUPPORT_OPENGLES30
#include <GLES3/gl3.h>
#endif

#define TAG "xm_mtcnn_ncnn_jni"
#define LOGD(...) __android_log_print(ANDROID_LOG_DEBUG, TAG, __VA_ARGS__)
#define LOGI(...) __android_log_print(ANDROID_LOG_INFO, TAG, __VA_ARGS__)
#define LOGE(...) __android_log_print(ANDROID_LOG_ERROR, TAG, __VA_ARGS__)

#define JNI_CLASS_XM_MTCNN_NCNN "com/xmly/media/camera/view/detection/XMMtcnnNcnn"

typedef struct xm_mtcnn_ncnn_fields_t {
    pthread_mutex_t mutex;
    jclass clazz;
    jfieldID field_mNativeXMMtcnnNcnn;
} xm_mtcnn_ncnn_fields_t;

static xm_mtcnn_ncnn_fields_t g_clazz;

jlong jni_nativeXMMtcnnNcnn_get(JNIEnv *env, jobject thiz)
{
    return (*env)->GetLongField(env, thiz, g_clazz.field_mNativeXMMtcnnNcnn);
}

static void jni_nativeXMMtcnnNcnn_set(JNIEnv *env, jobject thiz, jlong value)
{
    (*env)->SetLongField(env, thiz, g_clazz.field_mNativeXMMtcnnNcnn, value);
}

static XMMtcnnNcnn *jni_get_mtcnn_ncnn(JNIEnv* env, jobject thiz)
{
    pthread_mutex_lock(&g_clazz.mutex);

    XMMtcnnNcnn *mn = (XMMtcnnNcnn *) (intptr_t) jni_nativeXMMtcnnNcnn_get(env, thiz);
    if (mn) {
        xmmn_inc_ref(mn);
    }

    pthread_mutex_unlock(&g_clazz.mutex);
    return mn;
}

static XMMtcnnNcnn *jni_set_mtcnn_ncnn(JNIEnv* env, jobject thiz, XMMtcnnNcnn *mn)
{
    pthread_mutex_lock(&g_clazz.mutex);

    XMMtcnnNcnn *oldmn = (XMMtcnnNcnn *) (intptr_t) jni_nativeXMMtcnnNcnn_get(env, thiz);
    if (mn) {
        xmmn_inc_ref(mn);
    }
    jni_nativeXMMtcnnNcnn_set(env, thiz, (intptr_t) mn);

    pthread_mutex_unlock(&g_clazz.mutex);

    if (oldmn != NULL) {
        xmmn_dec_ref_p(&oldmn);
    }

    return oldmn;
}

#ifdef SUPPORT_OPENGLES30
static void
XMMtcnnNcnn_glReadPixels(JNIEnv *env, jobject obj, jint x, jint y, jint width, jint height,
        jint format, jint type) {
    glReadPixels(x, y, width, height, format, type, 0);
}

static void
XMMtcnnNcnn_glMapBufferRange(JNIEnv *env, jobject thiz, jint target, jint offset,
        jint length, jint access, jint w, jint h, jint pixelStride, jint rowPadding, jint format) {
    GLubyte *src = (GLubyte*)glMapBufferRange(target, offset, length, access);
    if(!src) {
        LOGE("glMapBufferRange failed");
        return;
    }

    XMMtcnnNcnn *mn = jni_get_mtcnn_ncnn(env, thiz);
    JNI_CHECK_GOTO(mn, env, "java/lang/IllegalStateException", "xmmnjni: glMapBufferRange: null mn", LABEL_RETURN);
    xm_mtcnn_ncnn_detect_glMapBufferRange(mn, src, length, w, h ,pixelStride, rowPadding, format);
LABEL_RETURN:
    xmmn_dec_ref_p(&mn);
}
#else
static void
XMMtcnnNcnn_glReadPixels(JNIEnv *env, jobject obj, jint x, jint y, jint width, jint height,
        jint format, jint type) {
    LOGE("XMMediaRecorder_glReadPixels is not supported");
}

static void
XMMtcnnNcnn_glMapBufferRange(JNIEnv *env, jobject thiz, jint target, jint offset,
        jint length, jint access, jint w, jint h, jint pixelStride, jint rowPadding, jint format) {
    LOGE("XMMediaRecorder_glMapBufferRange_put is not supported");
}
#endif

static void
XMMtcnnNcnn_NV21toABGR(JNIEnv *env, jobject thiz, jbyteArray yuv420sp, jint width, jint height,
    jbyteArray rgbaOut, jint rotation, jboolean flipHorizontal, jboolean flipVertical)
{
    XMMtcnnNcnn *mn = jni_get_mtcnn_ncnn(env, thiz);
    JNI_CHECK_GOTO(mn, env, "java/lang/IllegalStateException", "xmmnjni: detect: null mn", LABEL_RETURN);

    unsigned char *rgbaData = (unsigned char *) ((*env)->GetPrimitiveArrayCritical(env, rgbaOut, 0));
    unsigned char *yuv = (unsigned char *) (*env)->GetPrimitiveArrayCritical(env, yuv420sp, 0);
    unsigned char *temp1 = (unsigned char *)av_mallocz(sizeof(char)*width*height*4);
    unsigned char *temp2 = (unsigned char *)av_mallocz(sizeof(char)*width*height*4);
    int dst_w = width;
    int dst_h = height;
    bool flip_horizontal = flipHorizontal;
    bool flip_vertical = flipVertical;
    enum RotationMode mode = kRotate0;

    switch(rotation) {
        case 90:
            mode = kRotate90;
            dst_w = height;
            dst_h = width;
            flip_horizontal = flipVertical;
            flip_vertical = flipHorizontal;
            break;
        case 180:
            mode = kRotate180;
            break;
        case 270:
            mode = kRotate270;
            dst_w = height;
            dst_h = width;
            flip_horizontal = flipVertical;
            flip_vertical = flipHorizontal;
            break;
        default:
            break;
    }

    NV21ToARGB(yuv, width,
        yuv + width*height, ((width + 1) / 2)*2,
        temp1, width*4,
        width, height);

    if (flip_vertical && (mode == kRotate90 || mode == kRotate270)) {
        ARGBMirror(temp1, width * 4,
            temp2, width * 4,
            width, height);

        ARGBRotate(temp2, width*4,
            temp1, dst_w * 4,
            width, height, mode);

        if (flip_horizontal) {
            ARGBMirror(temp1, dst_w * 4,
                temp2, dst_w * 4,
                dst_w, dst_h);
            ARGBToABGR(temp2, dst_w*4,
                rgbaData, dst_w*4,
                dst_w, dst_h);
        } else {
            ARGBToABGR(temp1, dst_w*4,
                rgbaData, dst_w*4,
                dst_w, dst_h);
        }
    } else {
        ARGBRotate(temp1, width*4,
            temp2, dst_w * 4,
            width, height, mode);

        if (flip_horizontal) {
            ARGBMirror(temp2, dst_w * 4,
                temp1, dst_w * 4,
                dst_w, dst_h);

            ARGBToABGR(temp1, dst_w*4,
                rgbaData, dst_w*4,
                dst_w, dst_h);
        } else {
            ARGBToABGR(temp2, dst_w*4,
                rgbaData, dst_w*4,
                dst_w, dst_h);
        }
    }

    RgbaData qdata;
    qdata.w = dst_w;
    qdata.h = dst_h;
    qdata.rotate_degree = 0;
    qdata.flipHorizontal = false;
    qdata.flipVertical = false;
    qdata.rgba_size = 4*dst_w*dst_h;
    qdata.processed = false;
    qdata.rgba = rgbaData;
    xm_mtcnn_ncnn_detect(mn, &qdata);

    (*env)->ReleasePrimitiveArrayCritical(env, rgbaOut, rgbaData, 0);
    (*env)->ReleasePrimitiveArrayCritical(env, yuv420sp, yuv, 0);
    av_free(temp1);
    av_free(temp2);
LABEL_RETURN:
    xmmn_dec_ref_p(&mn);
}

static jintArray
XMMtcnnNcnn_get_rect(JNIEnv* env, jobject thiz)
{
    jintArray jarr = NULL;
    XMMtcnnNcnn *mn = jni_get_mtcnn_ncnn(env, thiz);
    JNI_CHECK_GOTO(mn, env, "java/lang/IllegalStateException", "xmmnjni: XMMtcnnNcnn_get_rect: null mn", LABEL_RETURN);

    jarr = (*env)->NewIntArray(env, FACE_DETECT_OUTPUT_LEN);
    jint *arr = (*env)->GetIntArrayElements(env, jarr, NULL);
    if(!xm_mtcnn_ncnn_get_rect(mn, arr)) {
        for(int i = 0; i < FACE_DETECT_OUTPUT_LEN; i++)
        {
            arr[i] = 0;
        }
    }
    (*env)->ReleaseIntArrayElements(env, jarr, arr, 0);
LABEL_RETURN:
    xmmn_dec_ref_p(&mn);
    return jarr;
}

static void XMMtcnnNcnn_stop(JNIEnv *env, jobject thiz)
{
    LOGD("%s\n", __func__);
    XMMtcnnNcnn *mn = jni_get_mtcnn_ncnn(env, thiz);
    JNI_CHECK_GOTO(mn, env, "java/lang/IllegalStateException", "xmmnjni: stop: null mn", LABEL_RETURN);

    xm_mtcnn_ncnn_stop(mn);
LABEL_RETURN:
    xmmn_dec_ref_p(&mn);
}

static void XMMtcnnNcnn_enable(JNIEnv* env, jobject thiz, jboolean enable)
{
    LOGD("%s\n", __func__);
    XMMtcnnNcnn *mn = jni_get_mtcnn_ncnn(env, thiz);
    JNI_CHECK_GOTO(mn, env, "java/lang/IllegalStateException", "xmmnjni: mtcnn_enable: null mn", LABEL_RETURN);

    xm_mtcnn_ncnn_enable(mn, enable);
LABEL_RETURN:
    xmmn_dec_ref_p(&mn);
}

static void XMMtcnnNcnn_release(JNIEnv *env, jobject thiz)
{
    LOGD("%s\n", __func__);
    XMMtcnnNcnn *mn = jni_get_mtcnn_ncnn(env, thiz);
    if(mn == NULL) {
        LOGI("XMMtcnnNcnn_release mn is NULL\n");
        goto LABEL_RETURN;
    }

    (*env)->DeleteGlobalRef(env, (jobject)xmmn_set_weak_thiz(mn, NULL));
    jni_set_mtcnn_ncnn(env, thiz, NULL);
LABEL_RETURN:
    xmmn_dec_ref_p(&mn);
}

static void XMMtcnnNcnn_native_finalize(JNIEnv *env, jobject thiz)
{
    LOGD("%s\n", __func__);
    XMMtcnnNcnn_release(env, thiz);
}

static void XMMtcnnNcnn_model_init(JNIEnv* env, jobject thiz, jstring path)
{
    LOGD("%s\n", __func__);
    XMMtcnnNcnn *mn = jni_get_mtcnn_ncnn(env, thiz);
    JNI_CHECK_GOTO(mn, env, "java/lang/IllegalStateException", "xmmnjni: model_init: null mn", LABEL_RETURN);

    const char *c_path = (*env)->GetStringUTFChars(env, path, NULL);
    xm_mtcnn_ncnn_model_init(mn, c_path);
    (*env)->ReleaseStringUTFChars(env, path, c_path);
LABEL_RETURN:
    xmmn_dec_ref_p(&mn);
}

static void
XMMtcnnNcnn_native_setup(JNIEnv *env, jobject thiz, jobject weak_this)
{
    LOGD("%s\n", __func__);
    XMMtcnnNcnn *mn = xm_mtcnn_ncnn_create();
    JNI_CHECK_GOTO(mn, env, "java/lang/OutOfMemoryError", "xmmnjni: native_setup: xm_mtcnn_ncnn_create() failed", LABEL_RETURN);

    jni_set_mtcnn_ncnn(env, thiz, mn);
    xmmn_set_weak_thiz(mn, (*env)->NewGlobalRef(env, weak_this));
LABEL_RETURN:
    xmmn_dec_ref_p(&mn);
}

static JNINativeMethod g_methods[] = {
    { "native_setup",           "(Ljava/lang/Object;)V",      (void *) XMMtcnnNcnn_native_setup },
    { "_model_init",            "(Ljava/lang/String;)V",      (void *) XMMtcnnNcnn_model_init },
    { "native_finalize",        "()V",                        (void *) XMMtcnnNcnn_native_finalize },
    { "_release",               "()V",                        (void *) XMMtcnnNcnn_release },
    { "_enable",                "(Z)V",                       (void *) XMMtcnnNcnn_enable },
    { "_stop",                  "()V",                        (void *) XMMtcnnNcnn_stop },
    { "_NV21toABGR",            "([BII[BIZZ)V",               (void *) XMMtcnnNcnn_NV21toABGR },
    { "_get_rect",              "()[I",                       (void *) XMMtcnnNcnn_get_rect },
    { "glReadPixels",			"(IIIIII)V",				  (void *) XMMtcnnNcnn_glReadPixels },
    { "_glMapBufferRange",	"(IIIIIIIII)V",				 (void *) XMMtcnnNcnn_glMapBufferRange },
};

int XMMtcnnNcnn_global_init(JNIEnv *env) {
    int ret = 0;
    LOGD("%s\n", __func__);

    pthread_mutex_init(&g_clazz.mutex, NULL );

    IJK_FIND_JAVA_CLASS(env, g_clazz.clazz, JNI_CLASS_XM_MTCNN_NCNN);
    (*env)->RegisterNatives(env, g_clazz.clazz, g_methods, NELEM(g_methods));

    g_clazz.field_mNativeXMMtcnnNcnn = (*env)->GetFieldID(env, g_clazz.clazz, "mNativeXMMtcnnNcnn", "J");

    return ret;
}

int XMMtcnnNcnn_global_uninit() {
    int ret = 0;
    LOGD("%s\n", __func__);

    pthread_mutex_destroy(&g_clazz.mutex);

    return ret;
}

