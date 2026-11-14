#pragma once

#include <math/bounded/circular_angle.hpp>
#include <math/bounded/tilt_angle.hpp>
#include <math/quantity/length.hpp>

namespace pose
{
struct GeoPosition
{
    GeoPosition(
        math::TiltAngle latitude,
        math::CircularAngle longitude,
        math::Length altitude
    ) : latitude(latitude),
        longitude(longitude),
        altitude(altitude)
    {
    }

    math::TiltAngle latitude;
    math::CircularAngle longitude;
    math::Length altitude;
};
}
