#pragma once

#include <cstdint>
#include <math/angle.hpp>
#include <math/distance.hpp>
#include <math/velocity.hpp>
#include <chrono>

#include <cstdint>

#include <target/domain/track_state.hpp>

namespace target
{

using math::literals::operator""_deg;
using math::literals::operator""_km;
using std::chrono_literals::operator""ms;

struct TrackEvent
{
    enum class TrackState
    {
        INIT = 1,
        TRACKING = 2
    };

    std::uint16_t id;
    TrackState state;
    math::Distance slantRange;
    math::Distance groundRange;
    math::Angle azimuth;
    math::Angle elevation;
    math::Velocity dopplerVelocity;
    math::Angle latitude;
    math::Angle longitude;
    math::Distance altitude;
    math::Velocity velocity;
    math::Angle heading;
    math::Angle flightPathAngle;
    // Elapsed time since the shared start at 0 ms.
    std::chrono::milliseconds timestamp = 0ms;
};

} // namespace target
