#pragma once

#include <pose/config/ant_enu_transform_config.hpp>
#include <pose/config/radar_attitude_factory_config.hpp>

#include <utility>

namespace pose
{
struct PoseConfig
{
    PoseConfig(RadarAttitudeFactoryConfig attitude, AntEnuTransformConfig transform)
        : attitude(attitude),
          transform(std::move(transform))
    {
    }

    RadarAttitudeFactoryConfig attitude;
    AntEnuTransformConfig transform;
};
}
