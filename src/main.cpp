#include "deepstream/nvdinfer/yolo_inference_rgb.h"
#include "deepstream/nvdinfer/yolo_inference_mono.h"
#include <thread>
#include <cstdio>

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

    run_pipeline_mono(2, mono_argv);

    rgb_thread.join();
    return 0;
}
