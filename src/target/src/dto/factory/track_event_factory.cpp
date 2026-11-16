#include "track_event_factory.hpp"

#include "domain/confirmed_track.hpp"
#include "math/trigonometry.hpp"

#include <limits>
#include <stdexcept>

namespace target
{

TrackEvent TrackEventFactory::from(const ConfirmedTrack& confirmedTrack, const pose::GeoPosition& geodeticPosition)
{
    if (confirmedTrack.id() > std::numeric_limits<std::uint16_t>::max())
    {
        throw std::overflow_error("Target ID cannot be represented by TrackEvent");
    }

    const auto& state = *confirmedTrack.state();
    using namespace math::literals;

    const auto& position = state.position();
    const auto& velocity = state.velocity();
    const auto horizontalRange = position.horizontalDistance();
    const auto horizontalSpeed = velocity.horizontalSpeed();
    const auto speed = velocity.speed();
    const auto heading = math::CircularAngle(
        horizontalSpeed.isPositive()
            ? math::atan2(velocity.x, velocity.y)
            : 0_deg
    );
    const auto flightPathAngle = math::TiltAngle(
        speed.isPositive()
            ? math::atan2(velocity.z, horizontalSpeed.asLinearVelocity())
            : 0_deg
    );
    return TrackEvent::Builder()
        .id(static_cast<std::uint16_t>(confirmedTrack.id()))
        .state(TrackEvent::TrackStatus::TRACKING)
        .slantRange(confirmedTrack.slantRange())
        .groundRange(horizontalRange)
        .azimuth(confirmedTrack.azimuth())
        .elevation(confirmedTrack.elevation())
        .dopplerVelocity(confirmedTrack.dopplerVelocity())
        .latitude(geodeticPosition.latitude)
        .longitude(geodeticPosition.longitude)
        .altitude(geodeticPosition.altitude)
        .speed(speed)
        .heading(heading)
        .flightPathAngle(flightPathAngle)
        .timestamp(confirmedTrack.lastObservationTime())
        .build();
}

} // namespace target
