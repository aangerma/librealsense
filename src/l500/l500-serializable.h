// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2018 Intel Corporation. All Rights Reserved.

#pragma once

#include "serializable.h"
#include "hw-monitor.h"
#include "sensor.h"

namespace librealsense
{
    class l500_serializable_base : public serializable_interface
    {
    public:
        l500_serializable_base(std::shared_ptr<hw_monitor> hw_monitor, synthetic_sensor& depth_sensor);
        std::vector<uint8_t> serialize_json() const override;
        void load_json(const std::string& json_content) override;

    private:
        std::shared_ptr<hw_monitor> _hw_monitor;
        synthetic_sensor& _depth_sensor;
    };
} // namespace librealsense
