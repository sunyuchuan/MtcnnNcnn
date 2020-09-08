//
// Created by sunyc on 18-11-08.
//
#include <assert.h>
#include <pthread.h>
#include "xmsdl_mtcnn.h"
#include "xm_mtcnn_ncnn_jni.h"
#include "xm_squeeze_ncnn.h"
#include "ijksdl_log.h"
#include "android/ijksdl_android_jni.h"
#include "ijksdl_misc.h"
#include "libyuv.h"
#include "xm_mtcnn_ncnn.h"

#include <android/log.h>
#include <android/bitmap.h>

#define TAG "xm_mtcnn_ncnn_jni"
#define LOGD(...) __android_log_print(ANDROID_LOG_DEBUG, TAG, __VA_ARGS__)

#define JNI_CLASS_XM_MTCNN_SDL "com/xmly/media/camera/view/detection/XMMtcnnSdl"

typedef struct xm_mtcnn_ncnn_fields_t {
    pthread_mutex_t mutex;
    jclass clazz;
} xm_mtcnn_ncnn_fields_t;

static xm_mtcnn_ncnn_fields_t g_clazz;
static JavaVM* g_jvm;

static jstring
XMNcnnSqueeze_detect(JNIEnv* env, jobject thiz, jobject bitmap)
{
    AndroidBitmapInfo info;
    AndroidBitmap_getInfo(env, bitmap, &info);
    int width = info.width;
    int height = info.height;

    if (width != 227 || height != 227)
        return NULL;
    if (info.format != ANDROID_BITMAP_FORMAT_RGBA_8888)
        return NULL;

    unsigned char * bitmapData = (unsigned char *)malloc(4*width*height);

    void* indata;
    AndroidBitmap_lockPixels(env, bitmap, &indata);
    memcpy(bitmapData, indata, 4*width*height);
    AndroidBitmap_unlockPixels(env, bitmap);

    char result_str[1024];
    bool ret = SqueezeNcnn_Detect(bitmapData, width, height, result_str);
    if(!ret) {
        free(bitmapData);
        return NULL;
    }
    jstring result = (*env)->NewStringUTF(env, result_str);

    free(bitmapData);
    return result;
}

static jboolean
XMNcnnSqueeze_init(JNIEnv *env, jobject thiz,
                                           jbyteArray param, jbyteArray bin, jbyteArray words)
{
    // init param
    int paramLen = (*env)->GetArrayLength(env, param);
    unsigned char *paramData = (unsigned char *)(*env)->GetByteArrayElements(env, param, NULL);
    if (NULL == paramData) {
        (*env)->ReleaseByteArrayElements(env, param, (jbyte *)paramData, 0);
        return JNI_FALSE;
    }
    // init bin
    int binLen = (*env)->GetArrayLength(env, bin);
    unsigned char *binData = (unsigned char *)(*env)->GetByteArrayElements(env, bin, NULL);
    if (NULL == binData) {
        (*env)->ReleaseByteArrayElements(env, bin, (jbyte *)binData, 0);
        return JNI_FALSE;
    }
    // init words
    int wordsLen = (*env)->GetArrayLength(env, words);
    unsigned char *wordsData = (unsigned char *)(*env)->GetByteArrayElements(env, words, NULL);
    if (NULL == wordsData) {
        (*env)->ReleaseByteArrayElements(env, words, (jbyte *)wordsData, 0);
        return JNI_FALSE;
    }

    if(!SqueezeNcnn_Init(paramData, paramLen, binData, binLen, wordsData, wordsLen))
    {
        (*env)->ReleaseByteArrayElements(env, param, (jbyte *)paramData, 0);
        (*env)->ReleaseByteArrayElements(env, bin, (jbyte *)binData, 0);
        (*env)->ReleaseByteArrayElements(env, words, (jbyte *)wordsData, 0);
        return JNI_FALSE;
    }

    (*env)->ReleaseByteArrayElements(env, param, (jbyte *)paramData, 0);
    (*env)->ReleaseByteArrayElements(env, bin, (jbyte *)binData, 0);
    (*env)->ReleaseByteArrayElements(env, words, (jbyte *)wordsData, 0);
    return JNI_TRUE;
}

static void
XMMtcnnNcnn_NV21toABGR(JNIEnv *env, jobject obj, jbyteArray yuv420sp, jint width, jint height, jbyteArray rgbaOut)
{
    unsigned char *rgbaData = (unsigned char *) ((*env)->GetPrimitiveArrayCritical(env, rgbaOut, 0));
    unsigned char *yuv = (unsigned char *) (*env)->GetPrimitiveArrayCritical(env, yuv420sp, 0);
    unsigned char *temp = (unsigned char *)malloc(sizeof(char)*width*height*4);

    NV21ToARGB(yuv, width,
               yuv + width*height, ((width + 1) / 2)*2,
               temp, width*4,
               width, height);

    ARGBToABGR(temp, width*4,
               rgbaData, width*4,
               width, height);

    (*env)->ReleasePrimitiveArrayCritical(env, rgbaOut, rgbaData, 0);
    (*env)->ReleasePrimitiveArrayCritical(env, yuv420sp, yuv, 0);
    free(temp);
}

static void
XMMtcnnNcnn_ABGRScale(JNIEnv * env, jobject obj, jbyteArray src, jint src_w, jint src_h, jbyteArray dst, jint dst_w, jint dst_h)
{
    unsigned char *srcData = (unsigned char *) ((*env)->GetPrimitiveArrayCritical(env, src, 0));
    unsigned char *dstData = (unsigned char *) ((*env)->GetPrimitiveArrayCritical(env, dst, 0));

    ARGBScale(srcData, src_w*4,
               src_w, src_h,
               dstData, dst_w*4,
               dst_w, dst_h,
               kFilterNone);

    (*env)->ReleasePrimitiveArrayCritical(env, src, srcData, 0);
    (*env)->ReleasePrimitiveArrayCritical(env, dst, dstData, 0);
}

static void
XMMtcnnNcnn_ABGRRotate(JNIEnv * env, jobject obj, jbyteArray input, jint w, jint h, jint degrees)
{
    int dst_w = w;
    int dst_h = h;
    enum RotationMode mode = kRotate0;
    switch(degrees) {
        case 90:
            mode = kRotate90;
            dst_w = h;
            dst_h = w;
            break;
        case 180:
            mode = kRotate180;
            break;
        case 270:
            mode = kRotate270;
            dst_w = h;
            dst_h = w;
            break;
        default:
            return;
    }

    unsigned char *inputData = (unsigned char *) ((*env)->GetPrimitiveArrayCritical(env, input, 0));
    unsigned char *temp = (unsigned char *)malloc(sizeof(char)*dst_w*dst_h*4);
    ARGBRotate(inputData, w * 4,
               temp, dst_w * 4,
               w, h, mode);

    ARGBCopy(temp, dst_w * 4,
             inputData, dst_w * 4,
             dst_w, dst_h);

    (*env)->ReleasePrimitiveArrayCritical(env, input, inputData, 0);
    free(temp);
}

static void
XMMtcnnNcnn_ABGRFlipHoriz(JNIEnv * env, jobject obj, jbyteArray input, jbyteArray output, jint w, jint h)
{
    unsigned char *inputData = (unsigned char *) ((*env)->GetPrimitiveArrayCritical(env, input, 0));
    unsigned char *outputData = (unsigned char *) ((*env)->GetPrimitiveArrayCritical(env, output, 0));
    for(int i = 0; i < h; i++)
    {
        for(int j = 0; j < w; j++)
        {
            outputData[(i*w + j)*4 + 0] = inputData[(i*w + ((w - 1) - j))*4 + 0];
            outputData[(i*w + j)*4 + 1] = inputData[(i*w + ((w - 1) - j))*4 + 1];
            outputData[(i*w + j)*4 + 2] = inputData[(i*w + ((w - 1) - j))*4 + 2];
            outputData[(i*w + j)*4 + 3] = inputData[(i*w + ((w - 1) - j))*4 + 3];
        }
    }
    (*env)->ReleasePrimitiveArrayCritical(env, input, inputData, 0);
    (*env)->ReleasePrimitiveArrayCritical(env, output, outputData, 0);
}

static jboolean
XMMtcnnNcnn_Init(JNIEnv *env, jobject instance, jstring faceDetectionModelPath_)
{
    jboolean tRet = false;
    if (NULL == faceDetectionModelPath_) {
        return tRet;
    }

    const char *faceDetectionModelPath = (*env)->GetStringUTFChars(env, faceDetectionModelPath_, 0);
    if (NULL == faceDetectionModelPath) {
        return tRet;
    }

    tRet = Mtcnn_FaceDetectionModelInit(faceDetectionModelPath);
    (*env)->ReleaseStringUTFChars(env, faceDetectionModelPath_, faceDetectionModelPath);
    return tRet;
}

static jintArray
XMMtcnnNcnn_FaceDetect(JNIEnv *env, jobject instance, jbyteArray imageDate_,
                                    jint imageWidth, jint imageHeight, jint imageChannel, jint format)
{
    jbyte *imageDate = (*env)->GetByteArrayElements(env, imageDate_, NULL);
    if (NULL == imageDate) {
        (*env)->ReleaseByteArrayElements(env, imageDate_, imageDate, 0);
        return NULL;
    }

    int faceInfo[1+10*14];
    int type = PIXEL_RGBA2RGB;
    switch(format) {
        case FORMAT_YUY2:
            type = PIXEL_RGBA2BGR;
            break;
    }
    if(!Mtcnn_FaceDetect((unsigned char*)imageDate, imageWidth, imageHeight, imageChannel, faceInfo, 1+10*14, type))
    {
        return NULL;
    }

    int out_size = 1+faceInfo[0]*14;
    jintArray tFaceInfo = (*env)->NewIntArray(env, out_size);
    (*env)->SetIntArrayRegion(env, tFaceInfo,0,out_size,faceInfo);
    (*env)->ReleaseByteArrayElements(env, imageDate_, imageDate, 0);
    return tFaceInfo;
}

static jintArray
XMMtcnnNcnn_MaxFaceDetect(JNIEnv *env, jobject instance, jbyteArray imageDate_,
                                       jint imageWidth, jint imageHeight, jint imageChannel, jint format)
{
    jbyte *imageDate = (*env)->GetByteArrayElements(env, imageDate_, NULL);
    if (NULL == imageDate){
        (*env)->ReleaseByteArrayElements(env, imageDate_, imageDate, 0);
        return NULL;
    }

    int faceInfo[1+2*14];
    int type = PIXEL_RGBA2RGB;
    switch(format) {
        case FORMAT_YUY2:
            type = PIXEL_RGBA2BGR;
            break;
    }
    if(!Mtcnn_MaxFaceDetect((unsigned char*)imageDate, imageWidth, imageHeight, imageChannel, faceInfo, 1+2*14, type))
    {
        return NULL;
    }

    int out_size = 1+faceInfo[0]*14;
    jintArray tFaceInfo = (*env)->NewIntArray(env, out_size);
    (*env)->SetIntArrayRegion(env, tFaceInfo,0,out_size,faceInfo);
    (*env)->ReleaseByteArrayElements(env, imageDate_, imageDate, 0);
    return tFaceInfo;
}

static jboolean
XMMtcnnNcnn_UnInit(JNIEnv *env, jobject instance)
{
    return Mtcnn_FaceDetectionModelUnInit();
}

static jboolean
XMMtcnnNcnn_SetMinFaceSize(JNIEnv *env, jobject instance, jint minSize)
{
    return Mtcnn_SetMinFaceSize(minSize);
}

static jboolean
XMMtcnnNcnn_SetThreadsNumber(JNIEnv *env, jobject instance, jint threadsNumber)
{
    return Mtcnn_SetThreadsNumber(threadsNumber);
}

static jboolean
XMMtcnnNcnn_SetTimeCount(JNIEnv *env, jobject instance, jint timeCount)
{
    return Mtcnn_SetTimeCount(timeCount);
}

static JNINativeMethod g_methods[] = {
    { "FaceDetectionModelInit", "(Ljava/lang/String;)Z", (void *) XMMtcnnNcnn_Init },
    { "FaceDetect", "([BIIII)[I", (void *) XMMtcnnNcnn_FaceDetect },
    { "MaxFaceDetect", "([BIIII)[I", (void *) XMMtcnnNcnn_MaxFaceDetect },
    { "FaceDetectionModelUnInit", "()Z", (void *) XMMtcnnNcnn_UnInit },
    { "SetMinFaceSize", "(I)Z", (void *) XMMtcnnNcnn_SetMinFaceSize },
    { "SetThreadsNumber", "(I)Z", (void *) XMMtcnnNcnn_SetThreadsNumber },
    { "SetTimeCount", "(I)Z", (void *) XMMtcnnNcnn_SetTimeCount },
    { "NV21toABGR", "([BII[B)V", (void *) XMMtcnnNcnn_NV21toABGR },
    { "ABGRScale", "([BII[BII)V", (void *) XMMtcnnNcnn_ABGRScale },
    { "ABGRRotate", "([BIII)V", (void *) XMMtcnnNcnn_ABGRRotate },
    { "ABGRFlipHoriz", "([B[BII)V", (void *) XMMtcnnNcnn_ABGRFlipHoriz },
    { "SqueezeNcnn_Init", "([B[B[B)Z", (void *) XMNcnnSqueeze_init },
    { "SqueezeNcnn_Detect", "(Landroid/graphics/Bitmap;)Ljava/lang/String;", (void *) XMNcnnSqueeze_detect },
};

JNIEXPORT jint JNI_OnLoad(JavaVM *vm, void *reserved)
{
    JNIEnv* env = NULL;
    g_jvm = vm;

    if ((*vm)->GetEnv(vm, (void**) &env, JNI_VERSION_1_4) != JNI_OK) {
        return -1;
    }
    assert(env != NULL);

    pthread_mutex_init(&g_clazz.mutex, NULL );

    IJK_FIND_JAVA_CLASS(env, g_clazz.clazz, JNI_CLASS_XM_MTCNN_SDL);
    (*env)->RegisterNatives(env, g_clazz.clazz, g_methods, NELEM(g_methods));

    XMMtcnnNcnn_global_init(env);

    return JNI_VERSION_1_4;
}

JNIEXPORT void JNI_OnUnload(JavaVM *jvm, void *reserved)
{
    pthread_mutex_destroy(&g_clazz.mutex);

    XMMtcnnNcnn_global_uninit();
}

