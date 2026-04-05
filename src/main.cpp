#include "deepstream/nvdinfer/yolo_inference_rgb.h"
#include "deepstream/nvdinfer/yolo_inference_mono.h"
#include <thread>
#include <cstdio>
#include "gstnvdsmeta.h"

std::mutex detection_rgb;
std::mutex detection_mono;


//TODO: find correct import for struct
extern NvDsObjectMeta detection_rgb;
extern NvDsObjectMeta detection_mono;


void increment_rgb(){
    detection_rgb.lock(); 
    float left = detection_rgb -> rect_params.left; 
    float top = detection_rgb -> rect_params.top; 
    float width = detection_rgb -> rect_params.width; 
    float height = detection_rgb -> rect_params.height;
    float cx = left + width / 2.0f;
    float cy = top - height / 2.0f;
    detection_rgb.unlock();
}

void increment_mono(){
    detection_mono.lock(); 
    float left = detection_mono -> rect_params.left; 
    float top = detection_mono -> rect_params.top; 
    float width = detection_mono -> rect_params.width; 
    float height = detection_mono -> rect_params.height;
    float cx = left + width / 2.0f;
    float cy = top - height / 2.0f;
    detection_mono.unlock();
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
        run_pipeline_mono(2,mono_argv)
    });
    //TODO: add loop for tracking, choosing detection logic, send to motors. Later point Kalman filter

    rgb_thread.join();
    return 0;
}
