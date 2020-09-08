#ifndef XM_MTCNN_SDL_H
#define XM_MTCNN_SDL_H
#include <stdbool.h>

#define FBC_API_PUBLIC __attribute__((visibility ("default")))
#define FBC_API_LOCAL __attribute__((visibility("hidden")))

#ifdef __cplusplus
extern "C" {
#endif
FBC_API_PUBLIC bool Mtcnn_FaceDetectionModelInit(char *faceDetectionModelPath);
FBC_API_PUBLIC bool Mtcnn_FaceDetect(unsigned char *imageDate, int imageWidth,
        int imageHeight, int imageChannel,
        int *dst, int dst_len, int type);
FBC_API_PUBLIC bool Mtcnn_MaxFaceDetect(unsigned char *imageDate, int imageWidth,
        int imageHeight, int imageChannel,
        int *dst, int dst_len, int type);
FBC_API_PUBLIC bool Mtcnn_FaceDetectionModelUnInit();
FBC_API_PUBLIC bool Mtcnn_SetMinFaceSize(int minSize);
FBC_API_PUBLIC bool Mtcnn_SetThreadsNumber(int threadsNumber);
FBC_API_PUBLIC bool Mtcnn_SetTimeCount(int timeCount);
#ifdef __cplusplus
}
#endif

#endif
