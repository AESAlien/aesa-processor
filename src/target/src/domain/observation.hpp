#pragma once

#include <math/geometry/spherical_position.hpp>
#include <math/quantity/linear_velocity.hpp>

#include <chrono>

namespace target
{

// A measurement has no repository identity or tracking strategy.
class Observation
{
public:
    Observation(
        math::SphericalPosition position,
        math::LinearVelocity dopplerVelocity,
        float power_db,
        std::chrono::milliseconds time
    );

    const math::SphericalPosition& sphericalPosition() const;
    math::Distance slantRange() const;
    math::CircularAngle azimuth() const;
    math::TiltAngle elevation() const;
    math::LinearVelocity dopplerVelocity() const;
    float power_db() const;
    std::chrono::milliseconds lastObservationTime() const;

private:
    const math::SphericalPosition _position;
    const math::LinearVelocity _dopplerVelocity;
    const float _power_db;
    const std::chrono::milliseconds _time;
};

} // namespace target
