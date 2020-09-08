#include "xm_squeeze_ncnn.h"
#include <android/bitmap.h>
#include <android/log.h>
#include <string>
#include <cstring>

#include <vector>
// ncnn
#include "net.h"

#include "squeezenet_v1.1.id.h"

#include <sys/time.h>
#include <unistd.h>

static struct timeval tv_begin;
static struct timeval tv_end;
static double elasped;

#define TAG "xm_squeeze_ncnn"
#define LOGD(...) __android_log_print(ANDROID_LOG_DEBUG,TAG,__VA_ARGS__)

using namespace std;

static void bench_start()
{
    gettimeofday(&tv_begin, NULL);
}

static void bench_end(const char* comment)
{
    gettimeofday(&tv_end, NULL);
    elasped = ((tv_end.tv_sec - tv_begin.tv_sec) * 1000000.0f + tv_end.tv_usec - tv_begin.tv_usec) / 1000.0f;
    LOGD("elasped %.2fms   comment %s", elasped, comment);
}

static ncnn::UnlockedPoolAllocator g_blob_pool_allocator;
static ncnn::PoolAllocator g_workspace_pool_allocator;

static ncnn::Mat squeezenet_param;
static ncnn::Mat squeezenet_bin;
static std::vector<std::string> squeezenet_words;
static ncnn::Net squeezenet;

static std::vector<std::string> split_string(const std::string& str, const std::string& delimiter)
{
    std::vector<std::string> strings;

    std::string::size_type pos = 0;
    std::string::size_type prev = 0;
    while ((pos = str.find(delimiter, prev)) != std::string::npos)
    {
        strings.push_back(str.substr(prev, pos - prev));
        prev = pos + 1;
    }

    // To get the last substring (or only, if delimiter is not found)
    strings.push_back(str.substr(prev));

    return strings;
}

// public native boolean Init(byte[] param, byte[] bin, byte[] words);
bool SqueezeNcnn_Init(const unsigned char *param, int param_len, const unsigned char *bin, int bin_len, const unsigned char *words, int words_len)
{
    // init param
    {
        squeezenet_param.create(param_len, (size_t)1u);
        memcpy((unsigned char *)squeezenet_param, param, param_len);
        int ret = squeezenet.load_param((const unsigned char*)squeezenet_param);
    }
    // init bin
    {
        squeezenet_bin.create(bin_len, (size_t)1u);
        memcpy((unsigned char *)squeezenet_bin, bin, bin_len);
        int ret = squeezenet.load_model((const unsigned char*)squeezenet_bin);
    }
    // init words
    {
        std::string words_buffer;
        words_buffer.resize(words_len);
        memcpy((unsigned char *)words_buffer.data(), words, words_len);
        squeezenet_words = split_string(words_buffer, "\n");
    }

    ncnn::Option opt;
    opt.lightmode = true;
    opt.num_threads = 4;
    opt.blob_allocator = &g_blob_pool_allocator;
    opt.workspace_allocator = &g_workspace_pool_allocator;
    ncnn::set_default_option(opt);

    return true;
}

// public native String Detect(Bitmap bitmap);
bool SqueezeNcnn_Detect(const unsigned char *bitmap, int bitmap_w, int bitmap_h, char *result)
{
    bench_start();

    // ncnn from bitmap
    ncnn::Mat in;
    {
        if (bitmap_w != 227 || bitmap_h != 227)
            return false;

        in = ncnn::Mat::from_pixels((const unsigned char*)bitmap, ncnn::Mat::PIXEL_RGBA2BGR, bitmap_w, bitmap_h);
    }

    // squeezenet
    std::vector<float> cls_scores;
    {
        const float mean_vals[3] = {104.f, 117.f, 123.f};
        in.substract_mean_normalize(mean_vals, 0);

        ncnn::Extractor ex = squeezenet.create_extractor();

        ex.input(squeezenet_v1_1_param_id::BLOB_data, in);

        ncnn::Mat out;
        ex.extract(squeezenet_v1_1_param_id::BLOB_prob, out);

        cls_scores.resize(out.w);
        for (int j=0; j<out.w; j++)
        {
            cls_scores[j] = out[j];
        }
    }

    // return top class
    int top_class = 0;
    float max_score = 0.f;
    for (size_t i=0; i<cls_scores.size(); i++)
    {
        float s = cls_scores[i];
        if (s > max_score)
        {
            top_class = i;
            max_score = s;
        }
    }

    const std::string& word = squeezenet_words[top_class];
    char tmp[32];
    sprintf(tmp, "%.3f", max_score);
    std::string result_str = std::string(word.c_str() + 10) + " = " + tmp;

    bench_end("detect");

    std::strcpy(result, result_str.c_str());
    return true;
}

