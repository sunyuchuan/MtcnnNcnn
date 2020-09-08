#ifndef XM_SQUEEZE_NCNN_H
#define XM_SQUEEZE_NCNN_H
#include <stdbool.h>

#define FBC_API_PUBLIC __attribute__((visibility ("default")))
#define FBC_API_LOCAL __attribute__((visibility("hidden")))

#ifdef __cplusplus
extern "C" {
#endif
FBC_API_PUBLIC bool
SqueezeNcnn_Init(const unsigned char *param, int param_len, const unsigned char *bin, int bin_len, const unsigned char *words, int words_len);
FBC_API_PUBLIC bool
SqueezeNcnn_Detect(const unsigned char *bitmap, int bitmap_w, int bitmap_h, char *result);
#ifdef __cplusplus
}
#endif

#endif
