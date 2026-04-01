#include "deepstream/nvdinfer/yolo_inference_rgb.h"
#include "deepstream/nvdinfer/yolo_inference_mono.h"
#include <thread>
#include <cstdio>

extern "C" {
    extern float drone_bbox_left_rgb, drone_bbox_top_rgb;
    extern float drone_bbox_width_rgb, drone_bbox_height_rgb;
    extern float drone_bbox_left_mono, drone_bbox_top_mono;
    extern float drone_bbox_width_mono, drone_bbox_height_mono;
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
    float cx = drone_bbox_left_rgb + drone_bbox_width_rgb / 2.0f;                                                                                                                                                            
    float cy = drone_bbox_top_rgb  + drone_bbox_height_rgb / 2.0f;

    float cx = drone_bbox_left_mono + drone_bbox_width_mono / 2.0f;                                                                                                                                                            
    float cy = drone_bbox_top_mono  + drone_bbox_height_mono / 2.0f;
    rgb_thread.join();
    return 0;
}
