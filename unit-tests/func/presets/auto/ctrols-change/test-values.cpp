// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2020 Intel Corporation. All Rights Reserved.

//#cmake: static!

#include "../../../func-common.h"
#include "../../presets-common.h"
#include <l500/l500-options.h>

using namespace rs2;

// This test checks that after changing one of the hw control, current and default values 
// of all the others controls are correct
TEST_CASE( "check values after hw control changed", "[l500][live]" )
{
    auto devices = find_devices_by_product_line_or_exit( RS2_PRODUCT_LINE_L500 );
    auto dev = devices[0];

    exit_if_fw_version_is_under( dev, MIN_GET_DEFAULT_FW_VERSION );

    auto depth_sens = dev.first< rs2::depth_sensor >();

    REQUIRE_NOTHROW(
        depth_sens.set_option( RS2_OPTION_VISUAL_PRESET, RS2_L500_VISUAL_PRESET_AUTOMATIC ) );
    REQUIRE_NOTHROW(depth_sens.set_option(RS2_OPTION_SENSOR_MODE,
        RS2_SENSOR_MODE_VGA));

    std::map< rs2_option, float > expected_values, expected_defs;
    build_preset_to_expected_values_map(
        dev,
        depth_sens,
        { RS2_L500_VISUAL_PRESET_AUTOMATIC, RS2_SENSOR_MODE_VGA },
        expected_values,
        expected_defs );

    for( auto & o : preset_dependent_options )
    {
        if( o == RS2_OPTION_DIGITAL_GAIN )
            continue;

        REQUIRE_NOTHROW(
            depth_sens.set_option(RS2_OPTION_VISUAL_PRESET, RS2_L500_VISUAL_PRESET_AUTOMATIC));

        if (depth_sens.is_option_read_only(o))
            continue;

        auto range = depth_sens.get_option_range( o );
        depth_sens.set_option( o, range.min );

        auto expected_curr_values = expected_values;
        expected_curr_values[o] = range.min;

        compare_to_actual( depth_sens, expected_curr_values, expected_defs);
    }
}