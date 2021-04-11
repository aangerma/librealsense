// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2021 Intel Corporation. All Rights Reserved.

//#cmake:add-file ../../../src/proc/sse/sse-pointcloud.cpp
//#cmake: static!

#include <librealsense2/hpp/rs_internal.hpp>
#include "../algo-common.h"
#include <librealsense2/rsutil.h>
#include "../src/proc/sse/sse-pointcloud.h"
#include "../src/cuda/cuda-pointcloud.cuh"
#include "../src/types.h"
#include <librealsense2-gl/rs_processing_gl.hpp> // Include GPU-Processing API

rs2_intrinsics intrin
= { 1280,
    720,
    643.720581f,
    357.821259f,
    904.170471f,
    905.155090f,
    RS2_DISTORTION_NONE,
    { 0.180086836f, -0.534179211f, -0.00139013783f, 0.000118769123f, 0.470662683f } };

void compare(librealsense::float2 pixel1, librealsense::float2 pixel2)
{
    CAPTURE(pixel1[0], pixel1[1]);
    CAPTURE(pixel2[0], pixel2[1]);
    for (auto i = 0; i < 2; i++)
    {
        //CAPTURE(i);
        REQUIRE(std::abs(pixel1[i] - pixel2[i]) <= 0.5);
    }
}

#include <GLFW/glfw3.h>
TEST_CASE("inverse_brown_conrady_glsl_deproject")
{
    glfwInit();
    glfwWindowHint(GLFW_VISIBLE, 0);
    auto win = glfwCreateWindow(100, 100, "offscreen", 0, 0);
    glfwMakeContextCurrent(win);
#ifndef __APPLE__
    rs2::gl::init_processing(true);
#endif

    rs2::gl::pointcloud pc;
    rs2::software_device dev;
    auto depth_sensor = dev.add_sensor("Depth");

    auto stream = depth_sensor.add_video_stream({ RS2_STREAM_DEPTH, 0, 0,
                               1280,720, 60, 2,
                                RS2_FORMAT_Z16, intrin });

    rs2::frame_queue queue;
    depth_sensor.open(stream);
    depth_sensor.start(queue);
    std::vector<uint16_t> data(1280*720, 2000);
    depth_sensor.on_video_frame({ data.data(), // Frame pixels from capture API
            [](void*) {}, // Custom deleter (if required)
            1280*2,2, // Stride and Bytes-per-pixel
            1, RS2_TIMESTAMP_DOMAIN_HARDWARE_CLOCK, 1, // Timestamp, Frame# for potential sync services
            stream });

    depth_sensor.add_read_only_option(RS2_OPTION_DEPTH_UNITS, 0.001f);

    auto f = queue.wait_for_frame();

    pc.map_to(f);
    auto points = pc.calculate(f);
    auto xy = points.get_texture_coordinates();

    float point[3] = { 0 };

    std::vector<librealsense::float2> vec(1280 * 720);
    for (auto i = 718; i < 720 - 1; i++)
    {
        for (auto j = 1278; j < 1280 -1; j++)
        {
            CAPTURE(i, j);
           
            auto pix = ((librealsense::float2*)xy)[i * 1280 + j];
            pix.x *= 1280;
            pix.y *= 720;
            compare(pix, { (float)j+0.5f,(float)i + 0.5f });
        }
    }
}

TEST_CASE( "inverse_brown_conrady_deproject" )
{
    float point[3] = { 0 };
    
    librealsense::float2 pixel1 = { 1, 1 };
    librealsense::float2 pixel2 = { 0, 0 };
    float depth = 10.5;
    rs2_deproject_pixel_to_point( point, &intrin, (float*)&pixel1, depth );
    rs2_project_point_to_pixel((float*)&pixel2, &intrin, point );

    compare(pixel1, pixel2);
}

TEST_CASE( "brown_conrady_deproject" )
{
    float point[3] = { 0 };

    librealsense::float2 pixel1 = { 1, 1 };
    librealsense::float2 pixel2 = { 0, 0 };
    float depth = 10.5;
    rs2_deproject_pixel_to_point( point, &intrin, (float*)&pixel1, depth );
    rs2_project_point_to_pixel((float*)&pixel2, &intrin, point );

    compare(pixel1, pixel2);
}

#ifdef 0
TEST_CASE("inverse_brown_conrady_sse_deproject")
{
    librealsense::pointcloud_sse pc_sse;

    librealsense::float2 pixel[4] = { {1, 1}, {0,2},{1,3},{1,4} };
    float depth = 10.5;
    librealsense::float3 points[4] = {};

    // deproject with native code because sse deprojection doesn't implement distortion
    for (auto i = 0; i < 4; i++)
    {
        rs2_deproject_pixel_to_point((float*)&points[i], &intrin, (float*)&pixel[i], depth);
    }
   
    std::vector<librealsense::float2> res(4, { 0,0 });
    std::vector<librealsense::float2> unnormalized_res(4, { 0,0 });
    rs2_extrinsics extrin = { {1,0,0,
        0,1,0,
        0,0,1},{0,0,0} };

    /*pc_sse.get_texture_map_sse((librealsense::float2*)res.data(), points, 4, 1, intrin, extrin, (librealsense::float2*)unnormalized_res.data());

    for (auto i = 0; i < 4; i++)
    {
        compare(unnormalized_res[i], pixel[i]);
    }*/
}

TEST_CASE("brown_conrady_sse_deproject")
{
    librealsense::pointcloud_sse pc_sse;
    librealsense::float2 pixel[4] = { {1, 1}, {0,2},{1,3},{1,4} };
    float depth = 10.5;
    librealsense::float3 points[4] = {};

    // deproject with native code because sse deprojection doesn't implement distortion
    for (auto i = 0; i < 4; i++)
    {
        rs2_deproject_pixel_to_point((float*)&points[i], &intrin, (float*)&pixel[i], depth);
    }

    std::vector<librealsense::float2> res(4, { 0,0 });
    std::vector<librealsense::float2> unnormalized_res(4, { 0,0 });
    rs2_extrinsics extrin = { {1,0,0,
        0,1,0,
        0,0,1},{0,0,0} };

    pc_sse.get_texture_map_sse((librealsense::float2*)res.data(), points, 4, 1, intrin, extrin, (librealsense::float2*)unnormalized_res.data());

    for (auto i = 0; i < 4; i++)
    {
        compare(unnormalized_res[i], pixel[i]);
    }
}
#endif

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