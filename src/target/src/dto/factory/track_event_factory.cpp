#include "track_event_factory.hpp"

#include "domain/confirmed_track.hpp"
#include "math/trigonometry.hpp"

#include <limits>
#include <stdexcept>

namespace target
{

TrackEvent TrackEventFactory::from(const ConfirmedTrack& track)
{
    if (track.id() > std::numeric_limits<std::uint16_t>::max())
    {
        throw std::overflow_error("Target ID cannot be represented by TrackEvent");
    }

    const auto& state = *track.state();
    using namespace math::literals;

    const auto& position = state.position();
    const auto& velocity = state.velocity();
    const auto horizontalRange = position.horizontalDistance();
    const auto horizontalSpeed = velocity.horizontalSpeed();
    const auto speed = velocity.speed();
    const math::CircularAngle heading(
        horizontalSpeed.greaterThan(0.0_mps, math::LinearVelocity::NONE_TOLERANCE)
            ? math::atan2(velocity.x, velocity.y)
            : 0_deg
    );
    const math::TiltAngle flightPathAngle(
        speed.greaterThan(0.0_mps, math::LinearVelocity::NONE_TOLERANCE)
            ? math::atan2(velocity.z, horizontalSpeed)
            : 0_deg
    );
    return TrackEvent(
        static_cast<std::uint16_t>(track.id()),
        TrackEvent::TrackStatus::TRACKING,
        track.slantRange(),
        horizontalRange,
        track.azimuth(),
        track.elevation(),
        track.dopplerVelocity(),
        math::TiltAngle(0_deg),
        math::CircularAngle(0_deg),
        position.z,
        speed,
        heading,
        flightPathAngle,
        track.lastObservationTime()
    );
}

} // namespace target
