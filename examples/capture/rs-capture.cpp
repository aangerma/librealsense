// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2017 Intel Corporation. All Rights Reserved.

#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include "example.hpp"          // Include short list of convenience functions for rendering
#include <thread>

// Capture Example demonstrates how to
// capture depth and color video streams and render them to the screen
int main(int argc, char * argv[]) try
{
    using namespace rs2;
    rs2::log_to_file(RS2_LOG_SEVERITY_DEBUG);
    // Create a simple OpenGL window for rendering:
    window app(1280, 720, "RealSense Capture Example");

    // Declare depth colorizer for pretty visualization of depth data
    rs2::colorizer color_map;
    // Declare rates printer for showing streaming rates of the enabled streams.
    rs2::rates_printer printer;

    // Declare RealSense pipeline, encapsulating the actual device and sensors
    //rs2::pipeline pipe;
    rs2::context c;
    auto sensors = c.query_all_sensors();
   // std::shared_ptr<rs2::motion_sensor> motion;
    std::shared_ptr<rs2::depth_sensor> depth;
    std::shared_ptr<rs2::color_sensor> color;

    for (auto s : sensors)
      /*  if (s.is< rs2::motion_sensor>())
            motion = std::make_shared<rs2::motion_sensor>(s.as< rs2::motion_sensor>());*/
       if (s.is< rs2::depth_sensor>())
            depth = std::make_shared<rs2::depth_sensor>(s.as< rs2::depth_sensor>());
        else if (s.is< rs2::color_sensor>())
            color = std::make_shared<rs2::color_sensor>(s.as< rs2::color_sensor>());
    

    depth->set_option(RS2_OPTION_GLOBAL_TIME_ENABLED, 0);
    color->set_option(RS2_OPTION_GLOBAL_TIME_ENABLED, 0);
    depth->set_option(RS2_OPTION_ERROR_POLLING_ENABLED, 0);
    //color->set_option(RS2_OPTION_ERROR_POLLING_ENABLED, 0);
    //auto profs = motion->get_stream_profiles();
   
    std::vector<stream_profile> profs_m;
    std::vector<stream_profile> profs_d;
    std::vector<stream_profile> profs_c;

  /*  for (auto f : profs)
    {
        if (f.fps() == 400)
            profs_m.push_back(f);
    }

    motion->open(profs_m);*/

    auto profs = depth->get_stream_profiles();
    for (auto f : profs)
    {
        if (f.fps() == 30 && f.as<video_stream_profile>().width() == 1280 && f.as<video_stream_profile>().height() == 720 && (f.format() == RS2_FORMAT_Z16 || f.format() == RS2_FORMAT_Y8) )
            profs_d.push_back(f);
    }
    depth->open(profs_d);

    profs = color->get_stream_profiles();
    for (auto f : profs)
    {
        if (f.fps() == 30 && f.as<video_stream_profile>().width() == 640 && f.as<video_stream_profile>().height() == 480 && f.format() == RS2_FORMAT_RGB8)
            profs_c.push_back(f);
    }
    color->open(profs_c);

    double gyro_ts = 0;
    int  gyro_fn = 0;
    double accel_ts = 0;
    int  accel_fn = 0;
    double depth_ts = 0;
    int  depth_fn = 0;
   double ir1_ts = 0;
    int  ir1_fn = 0;
    double ir2_ts = 0;
    int  ir2_fn = 0;
    double rgb_ts = 0;
    int  rgb_fn = 0;

  /*  motion->start([&](frame f)
    {
        if (f.get_profile().stream_type() == RS2_STREAM_GYRO)
        {
            if (gyro_ts > 0)
            {
                auto curr = f.get_timestamp();
                if ((curr - gyro_ts) > 60)
                    std::cout << "RS2_STREAM_GYRO fn: " <<std::fixed<< f.get_frame_number() << " ts: " << std::ios::fixed << curr << " " << " fn: " << std::fixed << gyro_fn << " ts:  " << std::ios::fixed << gyro_ts << " diff: " << std::fixed << curr - gyro_ts << std::endl;

            }
            gyro_ts = f.get_timestamp();
            gyro_fn = f.get_frame_number();
        }
        if (f.get_profile().stream_type() == RS2_STREAM_ACCEL)
        {
            if (accel_ts > 0)
            {
                auto curr = f.get_timestamp();
                if ((curr - gyro_ts) > 60)
                    std::cout << "RS2_STREAM_ACCEL fn: " << std::fixed << f.get_frame_number() << " ts: " << std::ios::fixed << curr << " " << " fn: " << std::fixed << accel_fn << " ts:  " << std::ios::fixed << accel_ts << " diff: " << std::fixed << curr - accel_ts << std::endl;

            }
            accel_ts = f.get_timestamp();
            accel_fn = f.get_frame_number();
        }
    });*/

    color->start([&](frame f)
    {
        if (f.get_profile().stream_type() == RS2_STREAM_COLOR)
        {
            if (rgb_ts > 0)
            {
                auto curr = f.get_frame_number();
                if ((curr - rgb_fn) > 2)
                    std::cout << "RS2_STREAM_COLOR fn: "  << curr << " " << " fn: "  << rgb_fn << std::endl;
            }
            rgb_ts = f.get_timestamp();
            rgb_fn = f.get_frame_number();
        }
    });
    depth->start([&](frame f)
    {
        if (f.get_profile().stream_type() == RS2_STREAM_DEPTH)
        {
            if (depth_ts > 0)
            {
                auto curr = f.get_frame_number();
                auto curr_ts = f.get_timestamp();
                if ((curr - depth_fn) > 2)
                    std::cout << "RS2_STREAM_DEPTH fn: "  << curr <<" ts: "<<std::fixed<< curr_ts << " fn: "  << depth_fn<<" ts: "<<std::fixed<< depth_ts<<std::endl;

            }
            depth_ts = f.get_timestamp();
            depth_fn = f.get_frame_number();
        }
        else if (f.get_profile().stream_type() == RS2_STREAM_INFRARED && f.get_profile().stream_index() == 1)
        {
            if (ir1_ts > 0)
            {
                auto curr = f.get_frame_number();
                if ((curr - ir1_fn) > 2)
                    std::cout << "RS2_STREAM_INFRARED 1 fn: " << curr << " " << " fn: "  << ir1_fn << std::endl;
            }
            ir1_ts = f.get_timestamp();
            ir1_fn = f.get_frame_number();
        }
        else if (f.get_profile().stream_type() == RS2_STREAM_INFRARED && f.get_profile().stream_index() == 2)
        {
            if (ir2_ts > 0)
            {
                auto curr = f.get_frame_number();
                if ((curr - ir2_fn) > 2)
                    std::cout << "RS2_STREAM_INFRARED 2 fn: " << curr << " " << " fn: " << ir2_fn << std::endl;
            }
            ir2_ts = f.get_timestamp();
            ir2_fn = f.get_frame_number();
        }
    });



//    std::thread t([]()
//    {
//        auto i = 0;
//        while (true)
//            /*std::cout<<*/i++;
//    });

//    std::thread t1([]()
//    {
//        auto i = 0;
//        while (true)
//            std::cout<<i++;
//    });
    while (app) // Application still alive?
    {



        //rs2::frameset data = pipe.wait_for_frames().    // Wait for next set of frames from the camera
        //                     apply_filter(printer).     // Print each enabled stream frame rate
        //                     apply_filter(color_map);   // Find and colorize the depth data

        //// The show method, when applied on frameset, break it to frames and upload each frame into a gl textures
        //// Each texture is displayed on different viewport according to it's stream unique id
        //app.show(data);
    }

    return EXIT_SUCCESS;
}
catch (const rs2::error & e)
{
    std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
    return EXIT_FAILURE;
}
catch (const std::exception& e)
{
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}
