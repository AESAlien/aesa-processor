#include "observation.hpp"

#include <cmath>
#include <stdexcept>
#include <utility>

namespace target
{

Observation::Observation(
    math::SphericalPosition position,
    math::LinearVelocity dopplerVelocity,
    float power_db,
    std::chrono::milliseconds time
) : _position(std::move(position)),
    _dopplerVelocity(dopplerVelocity),
    _power_db(power_db),
    _time(time)
{
    using namespace math::literals;
    if (time.count() < 0 ||
        _position.radius.isNonpositive() ||
        !std::isfinite(power_db)
    ) {
        throw std::invalid_argument("Observation requires a valid measurement and time");
    }
}

const math::SphericalPosition& Observation::sphericalPosition() const
{
    return _position;
}

math::Distance Observation::slantRange() const
{
    return _position.radius;
}

math::CircularAngle Observation::azimuth() const
{
    return _position.direction.azimuth;
}

math::TiltAngle Observation::elevation() const
{
    return _position.direction.elevation;
}

math::LinearVelocity Observation::dopplerVelocity() const
{
    return _dopplerVelocity;
}

float Observation::power_db() const
{
    return _power_db;
}

std::chrono::milliseconds Observation::lastObservationTime() const
{
    return _time;
}

} // namespace target
