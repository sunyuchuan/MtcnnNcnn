#include <android/log.h>
#include <string>
#include <vector>

// ncnn
#include "net.h"

#include "mtcnn.h"
#include "xmsdl_mtcnn.h"

using namespace std;
#define TAG "xmsdl_mtcnn"
#define LOGD(...) __android_log_print(ANDROID_LOG_DEBUG,TAG,__VA_ARGS__)
static MTCNN *mtcnn;

bool detection_sdk_init_ok = false;

bool Mtcnn_FaceDetectionModelInit(char *faceDetectionModelPath) {
    LOGD("Mtcnn_FaceDetectionModelInit");
    if (detection_sdk_init_ok) {
        LOGD("detection_sdk_init_ok");
        return true;
    }

    bool tRet = false;
    if (NULL == faceDetectionModelPath) {
        LOGD("NULL == faceDetectionModelPath");
        return tRet;
    }

    string tFaceModelDir = faceDetectionModelPath;
    string tLastChar = tFaceModelDir.substr(tFaceModelDir.length() - 1, 1);
    if ("\\" == tLastChar) {
        tFaceModelDir = tFaceModelDir.substr(0, tFaceModelDir.length() - 1) + "/";
    } else if (tLastChar != "/") {
        tFaceModelDir += "/";
    }
    LOGD("init, tFaceModelDir=%s", tFaceModelDir.c_str());

    mtcnn = new MTCNN(tFaceModelDir);
    mtcnn->SetMinFace(40);

    detection_sdk_init_ok = true;
    tRet = true;
    return tRet;
}

bool Mtcnn_FaceDetect(unsigned char *imageDate, int imageWidth,
        int imageHeight, int imageChannel,
        int *dst, int dst_len, int type) {
    LOGD("Mtcnn_FaceDetect");
    if(!detection_sdk_init_ok) {
        return false;
    }

    if (NULL == imageDate) {
        LOGD("imageDate is null");
        return false;
    }

    if(imageWidth<20||imageHeight<20){
        LOGD("width or height too small");
        return false;
    }

    if(3 == imageChannel || 4 == imageChannel){
    }else{
        LOGD("imageChannel not support");
        return false;
    }

    //int32_t minFaceSize=40;
    //mtcnn->SetMinFace(minFaceSize);

    ncnn::Mat ncnn_img;
    if(imageChannel==3) {
       ncnn_img = ncnn::Mat::from_pixels(imageDate, type,
                                                    imageWidth, imageHeight);
    } else {
        ncnn_img = ncnn::Mat::from_pixels(imageDate, type, imageWidth, imageHeight);
    }

    std::vector<Bbox> finalBbox;
    mtcnn->detect(ncnn_img, finalBbox);

    int32_t num_face = static_cast<int32_t>(finalBbox.size());
    LOGD("num_face %d\n", num_face);
    if(NULL == dst || dst_len > 1 + 10*14)
    {
        LOGD("dst faceInfo is invalid, dst_len %d\n", dst_len);
        return false;
    }

    int out_size = 1+num_face*14;
    if(dst_len < out_size)
        dst[0] = (dst_len-1)/14;
    else
        dst[0] = num_face;

    for(int i=0;i<dst[0];i++) {
        dst[14*i+1] = finalBbox[i].x1;//left
        dst[14*i+2] = finalBbox[i].y1;//top
        dst[14*i+3] = finalBbox[i].x2;//right
        dst[14*i+4] = finalBbox[i].y2;//bottom
        for (int j =0;j<10;j++){
            dst[14*i+5+j]=static_cast<int>(finalBbox[i].ppoint[j]);
        }
    }

    return true;
}

bool Mtcnn_MaxFaceDetect(unsigned char *imageDate, int imageWidth,
        int imageHeight, int imageChannel,
        int *dst, int dst_len, int type) {
    LOGD("Mtcnn_MaxFaceDetect");
    if(!detection_sdk_init_ok){
        return false;
    }

    if(imageWidth<20||imageHeight<20) {
        LOGD("width or height too small");
        return false;
    }

    if(3 == imageChannel || 4 == imageChannel) {
    } else {
        LOGD("imageChannel not be support");
        return false;
    }

    //int32_t minFaceSize=40;
    //mtcnn->SetMinFace(minFaceSize);

    ncnn::Mat ncnn_img;
    if(imageChannel==3) {
        ncnn_img = ncnn::Mat::from_pixels(imageDate, type,
                                          imageWidth, imageHeight);
    } else {
        ncnn_img = ncnn::Mat::from_pixels(imageDate, type, imageWidth, imageHeight);
    }

    std::vector<Bbox> finalBbox;
    mtcnn->detectMaxFace(ncnn_img, finalBbox);

    int32_t num_face = static_cast<int32_t>(finalBbox.size());
    LOGD("num_face %d", num_face);

    int out_size = 1+num_face*14;
    if(dst_len < out_size)
        dst[0] = (dst_len-1)/14;
    else
        dst[0] = num_face;

    for(int i=0;i<dst[0];i++) {
        dst[14*i+1] = finalBbox[i].x1;//left
        dst[14*i+2] = finalBbox[i].y1;//top
        dst[14*i+3] = finalBbox[i].x2;//right
        dst[14*i+4] = finalBbox[i].y2;//bottom
        for (int j =0;j<10;j++){
            dst[14*i+5+j]=static_cast<int>(finalBbox[i].ppoint[j]);
        }
    }

    return true;
}

bool Mtcnn_FaceDetectionModelUnInit() {
    if(!detection_sdk_init_ok){
        return true;
    }
    bool tDetectionUnInit = false;
    delete mtcnn;
    mtcnn = NULL;

    detection_sdk_init_ok=false;
    tDetectionUnInit = true;
    return tDetectionUnInit;
}

bool Mtcnn_SetMinFaceSize(int minSize) {
    if(!detection_sdk_init_ok) {
        return false;
    }

    if(minSize<=20){
        minSize=20;
    }

    mtcnn->SetMinFace(minSize);
    return true;
}

bool Mtcnn_SetThreadsNumber(int threadsNumber) {
    if(!detection_sdk_init_ok) {
        return false;
    }

    if(threadsNumber!=1&&threadsNumber!=2&&threadsNumber!=4&&threadsNumber!=8){
        return false;
    }

    mtcnn->SetNumThreads(threadsNumber);
    return  true;
}

bool Mtcnn_SetTimeCount(int timeCount) {

    if(!detection_sdk_init_ok){
        return false;
    }

    mtcnn->SetTimeCount(timeCount);
    return true;
}

