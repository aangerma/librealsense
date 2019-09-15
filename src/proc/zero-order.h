// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2019 Intel Corporation. All Rights Reserved.

#pragma once

#include "../include/librealsense2/hpp/rs_frame.hpp"
#include "synthetic-stream.h"
#include "option.h"
#include "l500/l500-private.h"

#define IR_THRESHOLD 120
#define RTD_THRESHOLD 50
#define BASELINE -10
#define PATCH_SIZE 5
#define Z_MAX_VALUE 1200
#define IR_MIN_VALUE 75
#define THRESHOLD_OFFSET 10
#define THRESHOLD_SCALE 20

namespace librealsense
{
    struct  zero_order_options
    {
        zero_order_options(): 
            ir_threshold(IR_THRESHOLD),
            rtd_high_threshold(RTD_THRESHOLD),
            rtd_low_threshold(RTD_THRESHOLD),
            baseline(BASELINE),
            patch_size(PATCH_SIZE),
            z_max(Z_MAX_VALUE),
            ir_min(IR_MIN_VALUE),
            threshold_offset(THRESHOLD_OFFSET),
            threshold_scale(THRESHOLD_SCALE)
        {}

        uint8_t                 ir_threshold;
        uint16_t                rtd_high_threshold;
        uint16_t                rtd_low_threshold;
        float                   baseline;
        bool                    read_baseline;
        int                     patch_size;
        int                     z_max;
        int                     ir_min;
        int                     threshold_offset;
        int                     threshold_scale;
    };

    class rtd_calc
    {
    public:
        virtual void z2rtd(const rs2::vertex* vertices, float* rtd, const rs2_intrinsics& intrinsics, int baseline) = 0;
        virtual std::pair<float, float> get_bounds(float zo_value, uint16_t rtd_low_threshold, uint16_t rtd_high_threshold) = 0;
    };

    class rtd_calc_base: public rtd_calc
    {
    public:
        void z2rtd(const rs2::vertex* vertices, float* rtd, const rs2_intrinsics& intrinsics, int baseline) override;
        float get_pixel_rtd(const rs2::vertex & v, int baseline);
        virtual std::pair<float, float> get_bounds(float zo_value, uint16_t rtd_high_threshold, uint16_t rtd_low_threshold) override;
    };

    class rtd_calc_optimaized : public rtd_calc
    {
    public:
        float get_pixel_rtd(const rs2::vertex & v, int baseline);
        void z2rtd(const rs2::vertex* vertices, float* rtd, const rs2_intrinsics& intrinsics, int baseline) override;
        virtual std::pair<float, float> get_bounds(float zo_value, uint16_t rtd_high_threshold, uint16_t rtd_low_threshold) override;
    };

    class zero_order : public generic_processing_block
    {
    public:
        zero_order(bool optimaized = true);

        rs2::frame process_frame(const rs2::frame_source& source, const rs2::frame& f) override;

    protected:
        bool zero_order_invalidation(const rs2::frame depth_in, const rs2::frame ir_in, const rs2::frame confidence_in,
            const rs2::frame depth_out, const rs2::frame confidence_out,
            const rs2::vertex * vertices, rs2_intrinsics intrinsics,
            const zero_order_options & options,
            int zo_point_x, int zo_point_y);

        void detect_zero_order(const float * rtd, const rs2::frame depth_in, const rs2::frame ir_in, const rs2::frame confidence_in,
            const rs2::frame depth_out, const rs2::frame confidence_out,
            const rs2_intrinsics& intrinsics, const zero_order_options& options,
            float zo_value, uint8_t iro_value);

        template<typename T>
        std::vector <T> get_zo_point_values(const T* frame_data_in, const rs2_intrinsics& intrinsics, int zo_point_x, int zo_point_y, int patch_r);

        template<typename T>
        T get_zo_point_value(std::vector<T>& values);

        bool try_get_zo_rtd_ir_point_values(const float* rtd, const uint16_t* depth_data_in, const uint8_t* ir_data,
            const rs2_intrinsics& intrinsics, const zero_order_options& options, int zo_point_x, int zo_point_y,
            float *rtd_zo_value, uint8_t* ir_zo_data);

    private:
        bool should_process(const rs2::frame& frame) override;
        rs2::frame prepare_output(const rs2::frame_source& source, rs2::frame input, std::vector<rs2::frame> results) override;
        const char * get_option_name(rs2_option option) const override;
        bool try_read_baseline(const rs2::frame& frame);
        ivcam2::intrinsic_params try_read_intrinsics(const rs2::frame& frame);

        std::pair<int, int> get_zo_point(const rs2::frame& frame);

        rs2::stream_profile     _source_profile_depth;
        rs2::stream_profile     _target_profile_depth;

        rs2::stream_profile     _source_profile_confidence;
        rs2::stream_profile     _target_profile_confidence;

        rs2::pointcloud         _pc;

        bool                    _first_frame;

        zero_order_options      _options;
        ivcam2::intrinsic_params _resolutions_depth;

        std::shared_ptr<rtd_calc> _rtd_calc;
    };
    MAP_EXTENSION(RS2_EXTENSION_ZERO_ORDER_FILTER, librealsense::zero_order);


    template<typename T>
    inline std::vector<T> zero_order::get_zo_point_values(const T * frame_data_in, const rs2_intrinsics & intrinsics, int zo_point_x, int zo_point_y, int patch_r)
    {
        //scoped_timer t("get_zo_point_values");
        std::vector<T> values;
        values.reserve((patch_r + 2) *(patch_r + 2));

        for (auto i = zo_point_y - 1 - patch_r; i <= (zo_point_y + patch_r) && i < intrinsics.height; i++)
        {
            for (auto j = (zo_point_x - 1 - patch_r); j <= (zo_point_x + patch_r) && i < intrinsics.width; j++)
            {
                values.push_back(frame_data_in[i*intrinsics.width + j]);
            }
        }

        return values;
    }
    template<typename T>
    inline T zero_order::get_zo_point_value(std::vector<T>& values)
    {
        std::sort(values.begin(), values.end());

        if ((values.size()) % 2 == 0)
        {
            return (values[values.size() / 2 - 1] + values[values.size() / 2]) / 2;
        }
        else if (values.size() > 0)
            return values[values.size() / 2];

        return 0;
    }
}
