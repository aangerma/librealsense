// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2021 Intel Corporation. All Rights Reserved.

//#cmake: static!
//#cmake:add-file ../../../src/proc/color-formats-converter.cpp

#include "../algo-common.h"
#include <librealsense2/rsutil.h>
#include "../src/types.h"
#include "../src/proc/color-formats-converter.h"

const int W = 16*3;
const int H = 8;
const int BPP_RGB = 24;
const int BPP_Y11 = 12;
const int SOURCE_BYTES = W * H*BPP_Y11 / 8;
const int DEST_BYTES = W * H* BPP_RGB / 8;


void conv_y411_to_rgb(byte * const dest, const byte * s, int w, int h, int actual_size)
{

}
#if 1 //TODO: check why sse tests fails on LibCi
TEST_CASE("unpack_y411_sse")
{
    byte dest[DEST_BYTES];
    byte source[SOURCE_BYTES];

    for (auto i = 0; i < SOURCE_BYTES; i++)
    {
        source[i] = i;
    }


    std::chrono::high_resolution_clock::time_point start, end;
    start = std::chrono::high_resolution_clock::now();
    librealsense::unpack_y411_sse(&dest[0], source, W, H, SOURCE_BYTES);
    end = std::chrono::high_resolution_clock::now();

    std::cout << std::chrono::duration_cast<std::chrono::microseconds>(end - start).count() << std::endl;
}
#endif
TEST_CASE("unpack_y411_native")
{
    byte dest[DEST_BYTES];
    byte source[SOURCE_BYTES];

    for (auto i = 0; i < SOURCE_BYTES; i++)
    {
        source[i] = i;
    }

    std::chrono::high_resolution_clock::time_point start, end;
    start = std::chrono::high_resolution_clock::now();

    librealsense::unpack_y411_native(&dest[0], source, W, H, SOURCE_BYTES);
    end = std::chrono::high_resolution_clock::now();

    std::cout << std::chrono::duration_cast<std::chrono::microseconds>(end - start).count() << std::endl;
}


#ifdef RS2_USE_CUDA
TEST_CASE("inverse_brown_conrady_cuda_deproject")
{
    std::vector<float3> point(1280 * 720, { 0,0,0 });

    librealsense::float2 pixel = { 0, 0 };

    std::vector<uint16_t> depth(1280 * 720, 1000);
    rscuda::deproject_depth_cuda((float*)point.data(), intrin, depth.data(), 1);
    for (auto i = 0; i < 720; i++)
    {
        for (auto j = 0; j < 1280; j++)
        {
            CAPTURE(i, j);
            rs2_project_point_to_pixel((float*)&pixel, &intrin, (float*)&point[i* 1280 +j]);
            compare({ (float)j,(float)i }, pixel);
        }
    }
}

TEST_CASE("brown_conrady_cuda_deproject")
{
    std::vector<float3> point(1280 * 720, { 0,0,0 });

    librealsense::float2 pixel = { 0, 0 };

    std::vector<uint16_t> depth(1280 * 720, 1000);
    rscuda::deproject_depth_cuda((float*)point.data(), intrin, depth.data(), 1);
    for (auto i = 0; i < 720; i++)
    {
        for (auto j = 0; j < 1280; j++)
        {
            CAPTURE(i, j);
            rs2_project_point_to_pixel((float*)&pixel, &intrin, (float*)&point[i * 1280 + j]);
            compare({ (float)j,(float)i }, pixel);
        }
    }
}
#endif