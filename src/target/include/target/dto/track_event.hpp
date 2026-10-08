#pragma once

#include <chrono>
#include <cstdint>
#include <math/angle.hpp>
#include <math/distance.hpp>
#include <math/velocity.hpp>

#include <target/domain/track_state.hpp>

namespace target
{

struct TrackEvent
{
    enum class TrackState
    {
        INIT = 1,
        TRACKING = 2
    };

    TrackEvent(
        uint16_t id,
        TrackState state,
        math::Distance slantRange,
        math::Distance groundRange,
        math::Angle azimuth,
        math::Angle elevation,
        math::Velocity dopplerVelocity,
        math::Angle latitude,
        math::Angle longitude,
        math::Distance altitude,
        math::Velocity velocity,
        math::Angle heading,
        math::Angle flightPathAngle,
        std::chrono::milliseconds timestamp
    ) : id(id),
        state(state),
        slantRange(slantRange),
        groundRange(groundRange),
        azimuth(azimuth),
        elevation(elevation),
        dopplerVelocity(dopplerVelocity),
        latitude(latitude),
        longitude(longitude),
        altitude(altitude),
        velocity(velocity),
        heading(heading),
        flightPathAngle(flightPathAngle),
        timestamp(timestamp)
    {
    }

    const uint16_t id;
    const TrackState state;
    const math::Distance slantRange;
    const math::Distance groundRange;
    const math::Angle azimuth;
    const math::Angle elevation;
    const math::Velocity dopplerVelocity;
    const math::Angle latitude;
    const math::Angle longitude;
    const math::Distance altitude;
    const math::Velocity velocity;
    const math::Angle heading;
    const math::Angle flightPathAngle;
    // Elapsed time since the shared start at 0 ms.
    const std::chrono::milliseconds timestamp;
};

} // namespace target
