#include <pose/domain/factory/radar_attitude_factory.hpp>

#include <sstream>
#include <stdexcept>

namespace pose
{
using namespace math::literals;

RadarAttitudeFactory::RadarAttitudeFactory(RadarAttitudeFactoryConfig config) : _config(config)
{
}

RadarAttitude RadarAttitudeFactory::create(math::Angle roll, math::TiltAngle pitch, math::Angle yaw) const
{
    if (roll < -180_deg || roll > 180_deg)
    {
        std::ostringstream message;
        message << "RadarAttitude: roll must be in [-180, 180] degrees"
                << ": roll=" << roll.deg();
        throw std::invalid_argument(message.str());
    }

    if (yaw < -360_deg || yaw > 360_deg)
    {
        std::ostringstream message;
        message << "RadarAttitude: yaw must be in [-360, 360] degrees"
                << ": yaw=" << yaw.deg();
        throw std::invalid_argument(message.str());
    }

    const auto pitchAngle = math::Angle::fromRadians(pitch.rad());
    const auto& maxAbsPitch = _config.maxAbsPitch();
    if (pitchAngle <= -maxAbsPitch || pitchAngle >= maxAbsPitch)
    {
        std::ostringstream message;
        message << "RadarAttitude: pitch is too close to gimbal lock"
                << ": pitch=" << pitch.deg()
                << ", maxAbsPitch=" << maxAbsPitch.deg();
        throw std::invalid_argument(message.str());
    }

    return RadarAttitude(math::CircularAngle(roll), pitch, math::CircularAngle(yaw));
}
}
