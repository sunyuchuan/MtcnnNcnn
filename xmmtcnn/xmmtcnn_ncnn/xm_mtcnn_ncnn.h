#ifndef _XM_MTCNN_NCNN_H_
#define _XM_MTCNN_NCNN_H_
#include "ijksdl/ijksdl_thread.h"
#include <pthread.h>
#include "xm_rgba_data.h"

#define FACE_DETECT_OUTPUT_LEN 6

enum NCNN_PIXEL
{
    PIXEL_CONVERT_SHIFT = 16,
    PIXEL_FORMAT_MASK = 0x0000ffff,
    PIXEL_CONVERT_MASK = 0xffff0000,

    PIXEL_RGB		= 1,
    PIXEL_BGR		= (1 << 1),
    PIXEL_GRAY		= (1 << 2),
    PIXEL_RGBA		= (1 << 3),

    PIXEL_RGB2BGR	= PIXEL_RGB | (PIXEL_BGR << PIXEL_CONVERT_SHIFT),
    PIXEL_RGB2GRAY	= PIXEL_RGB | (PIXEL_GRAY << PIXEL_CONVERT_SHIFT),

    PIXEL_BGR2RGB	= PIXEL_BGR | (PIXEL_RGB << PIXEL_CONVERT_SHIFT),
    PIXEL_BGR2GRAY	= PIXEL_BGR | (PIXEL_GRAY << PIXEL_CONVERT_SHIFT),

    PIXEL_GRAY2RGB	= PIXEL_GRAY | (PIXEL_RGB << PIXEL_CONVERT_SHIFT),
    PIXEL_GRAY2BGR	= PIXEL_GRAY | (PIXEL_BGR << PIXEL_CONVERT_SHIFT),

    PIXEL_RGBA2RGB	= PIXEL_RGBA | (PIXEL_RGB << PIXEL_CONVERT_SHIFT),
    PIXEL_RGBA2BGR	= PIXEL_RGBA | (PIXEL_BGR << PIXEL_CONVERT_SHIFT),
    PIXEL_RGBA2GRAY = PIXEL_RGBA | (PIXEL_GRAY << PIXEL_CONVERT_SHIFT),
} NCNN_PIXEL;

typedef struct Thread {
    SDL_Thread *tid;
    SDL_Thread _tid;
    pthread_mutex_t mLock;
    pthread_cond_t mCondition;
    volatile bool mRunning;
    volatile bool mDetecting;
    volatile bool mAbortRequest;
} Thread;

typedef struct XMMtcnnNcnn {
    volatile int ref_count;
    Thread *thread;
    RgbaData *image;
    void *weak_thiz;
    volatile bool enable;
    char *model_path;
    int rect[FACE_DETECT_OUTPUT_LEN];
} XMMtcnnNcnn;

void *xmmn_get_weak_thiz(XMMtcnnNcnn *mn);
void *xmmn_set_weak_thiz(XMMtcnnNcnn *mn, void *weak_thiz);
void xmmn_inc_ref(XMMtcnnNcnn *mn);
void xmmn_dec_ref(XMMtcnnNcnn *mn);
void xmmn_dec_ref_p(XMMtcnnNcnn **mn);
void xm_mtcnn_ncnn_free(XMMtcnnNcnn *mn);
void xm_mtcnn_ncnn_freep(XMMtcnnNcnn **mn);
int xm_mtcnn_ncnn_wait(Thread *thread);
void xm_mtcnn_ncnn_abort(Thread *thread);
void xm_mtcnn_ncnn_stop(XMMtcnnNcnn *mn);
bool xm_mtcnn_ncnn_get_rect(XMMtcnnNcnn *mn, int *rect);
bool xm_mtcnn_ncnn_detect_glMapBufferRange(XMMtcnnNcnn *mn,
    unsigned char *src, int size, int w, int h, int pixelStride, int rowPadding, int format);
bool xm_mtcnn_ncnn_detect(XMMtcnnNcnn *mn, RgbaData *image);
void xm_mtcnn_ncnn_enable(XMMtcnnNcnn *mn, bool enable);
void xm_mtcnn_ncnn_model_init(XMMtcnnNcnn *mn, const char *model_path);
XMMtcnnNcnn *xm_mtcnn_ncnn_create();
#endif
