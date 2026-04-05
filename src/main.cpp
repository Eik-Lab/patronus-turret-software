#include "deepstream/nvdinfer/yolo_inference_rgb.h"
#include "deepstream/nvdinfer/yolo_inference_mono.h"
#include <thread>
#include <cstdio>
#include <queue>
#include "gstnvdsmeta.h"

std::mutex detection_rgb_mutex;
std::mutex detection_mono_mutex;

extern std::queue<NvDsObjectMeta> detection_rgb;
extern std::queue<NvDsObjectMeta> detection_mono;


void increment_rgb(){
    detection_rgb_mutex.lock();
    if (!detection_rgb.empty()) {
        NvDsObjectMeta obj = detection_rgb.front();
        detection_rgb.pop();
        detection_rgb_mutex.unlock();

        float left = obj.rect_params.left;
        float top = obj.rect_params.top;
        float width = obj.rect_params.width;
        float height = obj.rect_params.height;
        float cx = left + width / 2.0f;
        float cy = top - height / 2.0f;
    } else {
        detection_rgb_mutex.unlock();
    }
}

void increment_mono(){
    detection_mono_mutex.lock();
    if (!detection_mono.empty()) {
        NvDsObjectMeta obj = detection_mono.front();
        detection_mono.pop();
        detection_mono_mutex.unlock();

        float left = obj.rect_params.left;
        float top = obj.rect_params.top;
        float width = obj.rect_params.width;
        float height = obj.rect_params.height;
        float cx = left + width / 2.0f;
        float cy = top - height / 2.0f;
    } else {
        detection_mono_mutex.unlock();
    }
}

int main(int argc, char *argv[])
{
    if (argc != 3) {
        fprintf(stderr, "Usage: %s <rgb_config> <mono_config>\n", argv[0]);
        return -1;
    }

    // Build argv for each pipeline: { program_name, config_path }
    char *rgb_argv[]  = { argv[0], argv[1] };
    char *mono_argv[] = { argv[0], argv[2] };

    std::thread rgb_thread([&]() {
        run_pipeline_rgb(2, rgb_argv);
    });
    std::thread mono_thread([&](){
        run_pipeline_mono(2,mono_argv);
    });
    //TODO: add loop for tracking, choosing detection logic, send to motors. Later point Kalman filter

    rgb_thread.join();
    return 0;
}
