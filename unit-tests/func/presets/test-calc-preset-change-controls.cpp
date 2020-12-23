// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2020 Intel Corporation. All Rights Reserved.

#include "../func-common.h"
#include "presets-common.h"
#include <l500/l500-options.h>

using namespace rs2;

TEST_CASE( "calc preset after change controls", "[l500][live]" )
{
    auto devices = find_devices_by_product_line_or_exit( RS2_PRODUCT_LINE_L500 );
    auto dev = devices[0];

    auto depth_sens = dev.first< rs2::depth_sensor >();

    auto preset_to_expected_map = build_preset_to_expected_values_map( depth_sens );

    REQUIRE( depth_sens.supports( RS2_OPTION_SENSOR_MODE ) );

    for_each_preset_mode_combination( [&]( rs2_l500_visual_preset preset, rs2_sensor_mode mode ) {

        REQUIRE_NOTHROW( depth_sens.set_option( RS2_OPTION_SENSOR_MODE, (float)mode ) );

        set_option_values(
            depth_sens,
            preset_to_expected_map[{  preset ,  mode }] );

        CAPTURE( preset );
        CAPTURE(  mode );

        validate_presets_value( depth_sens, preset );
    } );

}
