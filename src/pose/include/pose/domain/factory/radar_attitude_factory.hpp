#pragma once

#include <pose/domain/radar_attitude.hpp>
#include <pose/config/radar_attitude_factory_config.hpp>

#include <math/quantity/angle.hpp>

namespace pose
{
class RadarAttitudeFactory
{
public:
    explicit RadarAttitudeFactory(RadarAttitudeFactoryConfig config);

    // roll [-180°, 180°], yaw [-360°, 360°], 절대 pitch는 설정 한계 미만이어야 한다.
    RadarAttitude create(math::Angle roll, math::TiltAngle pitch, math::Angle yaw) const;

private:
    RadarAttitudeFactoryConfig _config;
};
}
