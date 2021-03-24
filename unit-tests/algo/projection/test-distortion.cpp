// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2021 Intel Corporation. All Rights Reserved.

//#cmake:add-file ../../../src/proc/sse/sse-pointcloud.cpp

#include "../algo-common.h"
#include <librealsense2/rsutil.h>
#include "../src/proc/sse/sse-pointcloud.h"
#include "../src/types.h"

void compare(librealsense::float2 pixel1, librealsense::float2 pixel2)
{
    for (auto i = 0; i < 2; i++)
    {
        REQUIRE(std::abs(pixel1[i] - pixel2[i]) < 0.0001);
    }
}

TEST_CASE( "inverse_brown_conrady_deproject" )
{
    float point[3] = { 0 };

    rs2_intrinsics intrin
        = { 1280,
            720,
            643.720581,
            357.821259,
            904.170471,
            905.155090,
            RS2_DISTORTION_INVERSE_BROWN_CONRADY,
            { 0.180086836, -0.534179211, -0.00139013783, 0.000118769123, 0.470662683 } };
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

    rs2_intrinsics intrin
        = { 1280,
            720,
            643.720581,
            357.821259,
            904.170471,
            905.155090,
            RS2_DISTORTION_BROWN_CONRADY,
            { 0.180086836, -0.534179211, -0.00139013783, 0.000118769123, 0.470662683 } };
    librealsense::float2 pixel1 = { 1, 1 };
    librealsense::float2 pixel2 = { 0, 0 };
    float depth = 10.5;
    rs2_deproject_pixel_to_point( point, &intrin, (float*)&pixel1, depth );
    rs2_project_point_to_pixel((float*)&pixel2, &intrin, point );

    compare(pixel1, pixel2);
}

TEST_CASE("inverse_brown_conrady_sse_deproject")
{
    std::shared_ptr<librealsense::pointcloud_sse> pc_sse = std::make_shared<librealsense::pointcloud_sse >();

    rs2_intrinsics intrin
        = { 1280,
            720,
            643.720581,
            357.821259,
            904.170471,
            905.155090,
            RS2_DISTORTION_INVERSE_BROWN_CONRADY,
            { 0.180086836, -0.534179211, -0.00139013783, 0.000118769123, 0.470662683 } };

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

    pc_sse->get_texture_map_sse((librealsense::float2*)res.data(), points, 4, 1, intrin, extrin, (librealsense::float2*)unnormalized_res.data());

    for (auto i = 0; i < 4; i++)
    {
        compare(unnormalized_res[i], pixel[i]);
    }
}