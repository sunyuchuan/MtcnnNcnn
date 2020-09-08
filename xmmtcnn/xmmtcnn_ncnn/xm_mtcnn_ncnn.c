#include "xm_mtcnn_ncnn.h"
#include "xmsdl_mtcnn.h"
#include "ijksdl_misc.h"
#include <android/log.h>
#include "libyuv.h"
#include <jni.h>
#include <assert.h>
#include "libavutil/mem.h"
#include "android/ijksdl_android_jni.h"

#define RGBA_CHANNEL 4
#define TAG "xm_mtcnn_ncnn"
#define LOGD(...) __android_log_print(ANDROID_LOG_DEBUG, TAG, __VA_ARGS__)
#define LOGE(...) __android_log_print(ANDROID_LOG_ERROR, TAG, __VA_ARGS__)

#define RGBA_MIN_W_H 227
#define FACE_INFO_LEN (1 + 14 * 100)

void *xmmn_get_weak_thiz(XMMtcnnNcnn *mn)
{
    if(!mn)
        return NULL;

    return mn->weak_thiz;
}

void *xmmn_set_weak_thiz(XMMtcnnNcnn *mn, void *weak_thiz)
{
    if(!mn)
        return NULL;

    void *prev_weak_thiz = mn->weak_thiz;

    mn->weak_thiz = weak_thiz;

    return prev_weak_thiz;
}

void xmmn_inc_ref(XMMtcnnNcnn *mn)
{
    assert(mn);
    __sync_fetch_and_add(&mn->ref_count, 1);
}

void xmmn_dec_ref(XMMtcnnNcnn *mn)
{
    if (!mn)
        return;

    int ref_count = __sync_sub_and_fetch(&mn->ref_count, 1);
    if (ref_count == 0) {
        LOGD("xmmn_dec_ref(): ref=0\n");
        xm_mtcnn_ncnn_stop(mn);
        xm_mtcnn_ncnn_freep(&mn);
    }
}

void xmmn_dec_ref_p(XMMtcnnNcnn **mn)
{
    if (!mn || !*mn)
        return;

    xmmn_dec_ref(*mn);
    *mn = NULL;
}

static void xmmn_notify(Thread *thread)
{
    if(!thread)
        return;

    pthread_mutex_lock(&thread->mLock);
    pthread_cond_signal(&thread->mCondition);
    pthread_mutex_unlock(&thread->mLock);
}

static void xmmn_waitOnNotify(Thread *thread)
{
    if(!thread)
        return;

    pthread_mutex_lock(&thread->mLock);
    pthread_cond_wait(&thread->mCondition, &thread->mLock);
    pthread_mutex_unlock(&thread->mLock);
}

static bool xmmn_getRunStatus(Thread *thread)
{
    bool status = false;
    if(!thread)
        return status;

    pthread_mutex_lock(&thread->mLock);
    status = thread->mRunning;
    pthread_mutex_unlock(&thread->mLock);
    return status;
}

static void xmmn_setRunStatus(Thread *thread, bool status)
{
    if(!thread)
        return;

    pthread_mutex_lock(&thread->mLock);
    thread->mRunning = status;
    pthread_mutex_unlock(&thread->mLock);
}

static bool xmmn_getDetectStatus(Thread *thread)
{
    bool status = false;
    if(!thread)
        return status;

    pthread_mutex_lock(&thread->mLock);
    status = thread->mDetecting;
    pthread_mutex_unlock(&thread->mLock);
    return status;
}

static void xmmn_setDetectStatus(Thread *thread, bool status)
{
    if(!thread)
        return;

    pthread_mutex_lock(&thread->mLock);
    thread->mDetecting = status;
    pthread_mutex_unlock(&thread->mLock);
}

static void xmmn_coordinate_transform(int rect[4], int angle, int origin[2], int h)
{
    float pi = 3.1415926;
    int point[4] = {0};
    float cos_r = cos(pi / 180.0 * angle);
    float sin_r = sin(pi / 180.0 * angle);
    int x2 = origin[0];
    int y2 = h -origin[1];

    for (int i = 0; i < 4; i += 2) {
        int x1 = rect[i];
        int y1 = h -rect[i+1];
        point[i] = (x1 - x2) * cos_r - (y1 - y2) * sin_r + x2;
        point[i+1] = (x1 - x2) * sin_r + (y1 - y2) * cos_r + y2;
        point[i+1] = h -point[i+1];
    }

    for(int i = 0; i < 2; i++) {
        if(point[i] < point[i+2]) {
            rect[i] = point[i];
            rect[i+2] = point[i+2];
        } else {
            rect[i] = point[i+2];
            rect[i+2] = point[i];
        }
    }
}

static void xmmn_fill_rect(int *dst, int *src, Thread *thread, int len)
{
    if(!dst || !dst || !thread)
        return;

    pthread_mutex_lock(&thread->mLock);
    for(int i = 0; i < len; i++)
    {
        dst[i] = src[i];
    }
    pthread_mutex_unlock(&thread->mLock);
}

static int xmmn_detect_start(void *arg)
{
    if(!arg) {
        Mtcnn_FaceDetectionModelUnInit();
        return -1;
    }

    XMMtcnnNcnn *mn = arg;
    Thread *thread = mn->thread;
    RgbaData *image = NULL;

    JNIEnv *env = NULL;
    if (JNI_OK != SDL_JNI_SetupThreadEnv(&env)) {
        LOGE("SetupThreadEnv failed\n");
        goto end;
    }

    LOGD("xm_mtcnn_ncnn thread start\n");
    xmmn_setRunStatus(thread, true);
    while(!thread->mAbortRequest)
    {
        xmmn_waitOnNotify(thread);

        image = mn->image;
        xmmn_setDetectStatus(thread, true);

        Mtcnn_SetMinFaceSize(150);
        Mtcnn_SetTimeCount(1);
        Mtcnn_SetThreadsNumber(4);
        int faceInfo[FACE_INFO_LEN] = {0};
        int type = PIXEL_RGBA2RGB;
        switch(image->format) {
            case FORMAT_YUY2:
                type = PIXEL_RGBA2BGR;
                break;
            case FORMAT_RGBA8888:
            default:
                type = PIXEL_RGBA2RGB;
                break;
        }
        bool ret = Mtcnn_MaxFaceDetect(image->rgba, image->w, image->h, RGBA_CHANNEL, faceInfo, FACE_INFO_LEN, type);
        if(ret && faceInfo[0] > 0)
        {
            float scale = 0.35;
            int left = (int) ((float)(faceInfo[1]) - (faceInfo[3] - faceInfo[1])*scale);
            int top = (int) ((float)faceInfo[2] - (faceInfo[4] - faceInfo[2])*scale);
            int right = (int) ((float)(faceInfo[3]) + (faceInfo[3] - faceInfo[1])*scale);
            int bottom = (int) ((float)faceInfo[4] + (faceInfo[4] - faceInfo[2])*scale);
            //pixelCoordinates2PlaneCoordinates_rotate(rectangleCoordinates, angle, origin, rgba_image_h);
            int rect[6] = {left, top, right, bottom, image->w, image->h};
            xmmn_fill_rect(mn->rect, rect, thread, FACE_DETECT_OUTPUT_LEN);
        } else {
            int rect[6] = {0, 0, 0, 0, image->w, image->h};
            xmmn_fill_rect(mn->rect, rect, thread, FACE_DETECT_OUTPUT_LEN);
        }
        xmmn_setDetectStatus(thread, false);
    }

end:
    LOGD("xm_mtcnn_ncnn thread quit\n");
    xmmn_setRunStatus(thread, false);
    Mtcnn_FaceDetectionModelUnInit();
    return 0;
}

static void xmmn_thread_freep(Thread **thread)
{
    if(!thread || !*thread)
        return;

    LOGD("xmmn_thread_freep\n");
    pthread_mutex_destroy(&(*thread)->mLock);
    pthread_cond_destroy(&(*thread)->mCondition);
    av_free(*thread);
    *thread = NULL;
}

static Thread *xmmn_thread_init()
{
    Thread *thread = (Thread *)av_mallocz(sizeof(Thread));
    if (!thread) {
        LOGE("av_mallocz Thread failed!\n");
        return NULL;
    }

    thread->mRunning = false;
    thread->mDetecting = false;
    thread->mAbortRequest = false;
    pthread_mutex_init(&thread->mLock, NULL);
    pthread_cond_init(&thread->mCondition, NULL);
    return thread;
}

static int xmmn_thread_start(XMMtcnnNcnn *mn)
{
    if(NULL == mn) {
        LOGE("xmmn_thread_start() failed, XMMtcnnNcnn object invalid");
        return -1;
    }

    Thread *thread = xmmn_thread_init();
    if(!thread) {
        LOGE("xmmn_thread_init() failed");
        return -1;
    }

    pthread_mutex_lock(&thread->mLock);
    mn->thread = thread;
    pthread_mutex_unlock(&thread->mLock);
    xmmn_setRunStatus(mn->thread, true);

    LOGD("model_path %s",mn->model_path);
    Mtcnn_FaceDetectionModelInit(mn->model_path);
    thread->tid = SDL_CreateThreadEx(&thread->_tid, xmmn_detect_start, mn, "xm_mtcnn_ncnn thread");
    if (!thread->tid) {
        LOGE("SDL_CreateThreadEx() failed");
        xm_mtcnn_ncnn_freep(&mn);
        return -1;
    }

    return 0;
}

static void xmmn_image_freep(RgbaData **data)
{
    if(!data || !*data)
        return;

    LOGD("xmmn_image_freep\n");
    if((*data)->rgba)
    {
        av_free((*data)->rgba);
    }

    av_free(*data);
    *data = NULL;
}

#ifdef SUPPORT_ARM_NEON
#ifdef ARMV7A
static void xmmn_memcpy_neon(volatile unsigned char *dst, volatile unsigned char *src, int32_t sz)
{
    if (sz & 63) {
        sz = (sz & -64) + 64;
    }

    __asm__ __volatile__ (
        "MTCNNNEONCopy:                          \n"
        "    VLDM %[src]!,{d0-d7}                 \n"
        "    VSTM %[dst]!,{d0-d7}                 \n"
        "    SUBS %[sz],%[sz],#0x40                 \n"
        "    BGT MTCNNNEONCopy                  \n"
        : [dst]"+r"(dst), [src]"+r"(src), [sz]"+r"(sz)
        :
        : "d0", "d1", "d2", "d3", "d4", "d5", "d6", "d7", "cc", "memory"
    );
}
#else
static void xmmn_memcpy_neon(volatile unsigned char *dst, volatile unsigned char *src, int64_t sz)
{
    if (sz & 31) {
        sz = (sz & -32) + 32;
    }

    __asm__ __volatile__ (
        "NEONCopy:            \n"
        "    ldp q0, q1, [%[src]], #32	 \n"  // load 32
        "    subs %[sz],%[sz], #32    \n"  // 32 processed per loop
        "    stp q0, q1, [%[dst]], #32	 \n"  // store 32
        "    b.gt NEONCopy \n"
        : [dst]"+r"(dst),   // %0
          [src]"+r"(src),   // %1
          [sz]"+r"(sz)   // %2  // Output registers
        :                             // Input registers
        : "v0", "v1", "cc", "memory"  // Clobber List
    );
}
#endif
#else
static void xmmn_memcpy_neon(unsigned char *dst, const unsigned char *src, int sz)
{
    memcpy(dst, src, sz);
}
#endif

static void xmmn_image_scale(unsigned char *dst, int dst_w, int dst_h,
    const unsigned char *src, int src_w, int src_h)
{
    if(!src || !dst)
        return;

    ARGBScale(src, src_w*4,
               src_w, src_h,
               dst, dst_w*4,
               dst_w, dst_h,
               kFilterNone);
}

static bool xmmn_fill_image(XMMtcnnNcnn *mn, RgbaData *inputImage)
{
    if(!mn || !mn->thread || !mn->image || !inputImage || !inputImage->rgba)
        return false;

    RgbaData *image = mn->image;
    Thread *thread = mn->thread;

    pthread_mutex_lock(&thread->mLock);
    xmmn_image_scale(image->rgba, image->w, image->h, inputImage->rgba, inputImage->w, inputImage->h);
    pthread_mutex_unlock(&thread->mLock);

    return true;
}

static RgbaData *xmmn_image_create(int w, int h)
{
    if(w == 0 || h == 0)
        return NULL;

    RgbaData *image = (RgbaData *)av_mallocz(sizeof(RgbaData));
    if (!image) {
        LOGE("av_mallocz RgbaData failed!\n");
        return NULL;
    }

    image->w = w;
    image->h = h;
    image->rgba_size = w*h*RGBA_CHANNEL;
    image->rgba = (unsigned char *)av_mallocz(sizeof(char) * image->rgba_size);
    if(!image->rgba)
    {
        LOGE("av_mallocz rgba failed!\n");
        xmmn_image_freep(&image);
        return NULL;
    }

    return image;
}

void xm_mtcnn_ncnn_free(XMMtcnnNcnn *mn)
{
    if(!mn)
        return;

    if(mn->model_path)
        av_free(mn->model_path);
    mn->model_path = NULL;

    xmmn_thread_freep(&mn->thread);
    xmmn_image_freep(&mn->image);
}

void xm_mtcnn_ncnn_freep(XMMtcnnNcnn **mn)
{
    if(!mn || !*mn)
        return;

    xm_mtcnn_ncnn_free(*mn);
    av_free(*mn);
    *mn = NULL;
}

int xm_mtcnn_ncnn_wait(Thread *thread)
{
    if(!thread)
        return -1;

    if(!thread->mRunning || !thread->tid)
    {
        return -1;
    }

    SDL_WaitThread(thread->tid, NULL);
    thread->mRunning = false;
    return 0;
}

void xm_mtcnn_ncnn_abort(Thread *thread)
{
    if(!thread)
        return;

    pthread_mutex_lock(&thread->mLock);
    thread->mAbortRequest = true;
    pthread_cond_signal(&thread->mCondition);
    pthread_mutex_unlock(&thread->mLock);
}

void xm_mtcnn_ncnn_stop(XMMtcnnNcnn *mn)
{
    if(!mn)
        return;

    LOGD("xm_mtcnn_ncnn_stop\n");
    if(mn->thread) {
        xm_mtcnn_ncnn_abort(mn->thread);
        xm_mtcnn_ncnn_wait(mn->thread);
    }
}

bool xm_mtcnn_ncnn_get_rect(XMMtcnnNcnn *mn, int *rect)
{
    bool ret = false;
    if(!mn || !rect || !mn->thread || !mn->enable)
        return ret;

    pthread_mutex_lock(&mn->thread->mLock);
    for(int i = 0; i < FACE_DETECT_OUTPUT_LEN; i++)
    {
        rect[i] = mn->rect[i];
    }
    ret = true;
    pthread_mutex_unlock(&mn->thread->mLock);

    return ret;
}

bool xm_mtcnn_ncnn_detect_glMapBufferRange(XMMtcnnNcnn *mn,
    unsigned char *src, int size, int w, int h, int pixelStride, int rowPadding, int format)
{
    bool ret = false;
    if(!mn || !src || !mn->enable)
        return ret;

    bool running = xmmn_getRunStatus(mn->thread);
    if(!running)
        xmmn_thread_start(mn);

    bool detecting = xmmn_getDetectStatus(mn->thread);
    if(running && !detecting)
    {
        RgbaData *image = mn->image;
        unsigned char *glMapBuffer = NULL;
        int image_w = w;
        int image_h = h;
        switch(format) {
            case FORMAT_YUY2:
                image_w = 2 * w;
                image_h = h;
                break;
            default:
                break;
        }

        if(NULL == image || (image_w != image->w || image_h != image->h))
        {
            xmmn_image_freep(&mn->image);
            mn->image = (RgbaData *)av_mallocz(sizeof(RgbaData));
            if (!mn->image) {
                LOGE("av_mallocz RgbaData failed!\n");
                return ret;
            }
            mn->image->w = image_w;
            mn->image->h =image_h;
            mn->image->rgba_size = image_w*image_h*RGBA_CHANNEL;
            mn->image->rgba = (unsigned char *)av_mallocz(sizeof(char)*mn->image->rgba_size);
            if(mn->image->rgba == NULL)
            {
                LOGE("mallocz image->rgba failed");
                return ret;
            }
        }

        if (format == FORMAT_RGBA8888 && pixelStride == 4
            && rowPadding == 0 && mn->image->rgba_size == size) {
            glMapBuffer = mn->image->rgba;
        } else {
            glMapBuffer = (unsigned char *)av_mallocz(sizeof(char) * size);
            if(glMapBuffer == NULL)
            {
                LOGE("mallocz glMapBuffer failed");
                return ret;
            }
        }
        pthread_mutex_lock(&mn->thread->mLock);
        xmmn_memcpy_neon(glMapBuffer, src, size);
        pthread_mutex_unlock(&mn->thread->mLock);

         switch(format) {
            case FORMAT_RGBA8888:
                mn->image->format = FORMAT_RGBA8888;
                if (pixelStride == 4 && rowPadding == 0
                    && mn->image->rgba_size == size) {
                    //have copied the data to mn->image->rgba in front.
                } else {
                    int offset = 0;
                    pthread_mutex_lock(&mn->thread->mLock);
                    for (int i = 0; i < h; ++i) {
                        for (int j = 0; j < w; ++j) {
                            mn->image->rgba[(i*w + j) * RGBA_CHANNEL + 3] = (glMapBuffer[offset + 3] & 0xff); // A
                            mn->image->rgba[(i*w + j) * RGBA_CHANNEL + 2] = (glMapBuffer[offset + 2] & 0xff); // B
                            mn->image->rgba[(i*w + j) * RGBA_CHANNEL + 1] = (glMapBuffer[offset + 1] & 0xff); // G
                            mn->image->rgba[(i*w + j) * RGBA_CHANNEL + 0] = (glMapBuffer[offset] & 0xff); // R
                            offset += pixelStride;
                        }
                        offset += rowPadding;
                    }
                    pthread_mutex_unlock(&mn->thread->mLock);
                }
                break;
            case FORMAT_YUY2:
                mn->image->format = FORMAT_YUY2;
                if (pixelStride == 4 && rowPadding == 0
                    && mn->image->rgba_size == 2 * size) {
                    pthread_mutex_lock(&mn->thread->mLock);
                    YUY2ToARGB(glMapBuffer, 2 * w * 2,
                        mn->image->rgba, RGBA_CHANNEL * image_w,
                        image_w, image_h);
                    pthread_mutex_unlock(&mn->thread->mLock);
                } else {
                    unsigned char *tmp = (unsigned char *)av_mallocz(sizeof(char)*w*h*RGBA_CHANNEL);
                    if(tmp == NULL)
                    {
                        LOGE("mallocz tmp failed");
                        return ret;
                    }
                    int offset = 0;
                    for (int i = 0; i < h; ++i) {
                        for (int j = 0; j < w; ++j) {
                            tmp[(i*w + j) * RGBA_CHANNEL + 3] = (glMapBuffer[offset + 3] & 0xff);
                            tmp[(i*w + j) * RGBA_CHANNEL + 2] = (glMapBuffer[offset + 2] & 0xff);
                            tmp[(i*w + j) * RGBA_CHANNEL + 1] = (glMapBuffer[offset + 1] & 0xff);
                            tmp[(i*w + j) * RGBA_CHANNEL + 0] = (glMapBuffer[offset] & 0xff);
                            offset += pixelStride;
                        }
                        offset += rowPadding;
                    }
                    pthread_mutex_lock(&mn->thread->mLock);
                    YUY2ToARGB(tmp, 2 * w * 2,
                        mn->image->rgba, RGBA_CHANNEL * image_w,
                        image_w, image_h);
                    pthread_mutex_unlock(&mn->thread->mLock);
                    av_freep(&tmp);
                }
            default:
                LOGE("unsupport format %d", format);
                break;
        }

        if (glMapBuffer != mn->image->rgba) {
            av_freep(&glMapBuffer);
        }
        xmmn_notify(mn->thread);
        ret = true;
    }

    return ret;
}

bool xm_mtcnn_ncnn_detect(XMMtcnnNcnn *mn, RgbaData *inputImage)
{
    bool ret = false;
    if(!mn || !inputImage || !mn->enable)
        return ret;

    bool running = xmmn_getRunStatus(mn->thread);
    if(!running)
        xmmn_thread_start(mn);

    bool detecting = xmmn_getDetectStatus(mn->thread);
    if(running && !detecting)
    {
        RgbaData *image = mn->image;
        int dst_image_w = 0;
        int dst_image_h = 0;
        if(inputImage->w > inputImage->h)
        {
            dst_image_h = IJKALIGN(RGBA_MIN_W_H, 4);
            dst_image_w = IJKALIGN((int)(dst_image_h * ((double)inputImage->w / (double)inputImage->h)), 4);
        } else {
            dst_image_w = IJKALIGN(RGBA_MIN_W_H, 4);
            dst_image_h = IJKALIGN((int)(dst_image_w * ((double)inputImage->h / (double)inputImage->w)), 4);
        }

        if(NULL == image || (dst_image_w != image->w || dst_image_h != image->h))
        {
            xmmn_image_freep(&mn->image);
            mn->image = xmmn_image_create(dst_image_w, dst_image_h);
        }

        if(xmmn_fill_image(mn, inputImage))
        {
            xmmn_notify(mn->thread);
        }
        ret = true;
    }

    return ret;
}

void xm_mtcnn_ncnn_enable(XMMtcnnNcnn *mn, bool enable)
{
    if(!mn)
        return;

    mn->enable = enable;
    if(!enable) {
        xm_mtcnn_ncnn_stop(mn);
    }
}

void xm_mtcnn_ncnn_model_init(XMMtcnnNcnn *mn, const char *model_path)
{
    if(!mn)
        return;

    if(!model_path) {
        mn->model_path = "/sdcard/mtcnn/";
        return;
    }

    if(mn->model_path)
        av_free(mn->model_path);
    mn->model_path = av_strdup(model_path);
}

XMMtcnnNcnn *xm_mtcnn_ncnn_create()
{
    XMMtcnnNcnn *mn = (XMMtcnnNcnn *)av_mallocz(sizeof(XMMtcnnNcnn));
    if (!mn) {
        LOGE("av_mallocz XMMtcnnNcnn failed!\n");
        return NULL;
    }

    xmmn_inc_ref(mn);
    return mn;
}
