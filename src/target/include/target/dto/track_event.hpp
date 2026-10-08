#pragma once

#include <chrono>
#include <cstdint>
#include <optional>
#include <math/angle.hpp>
#include <math/distance.hpp>
#include <math/velocity.hpp>

namespace target
{

struct TrackEvent
{
    enum class TrackState;

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

    class Builder
    {
    public:
        Builder& id(uint16_t value) { _id = value; return *this; }
        Builder& state(TrackState value) { _state = value; return *this; }
        Builder& slantRange(math::Distance value) { _slantRange = value; return *this; }
        Builder& groundRange(math::Distance value) { _groundRange = value; return *this; }
        Builder& azimuth(math::Angle value) { _azimuth = value; return *this; }
        Builder& elevation(math::Angle value) { _elevation = value; return *this; }
        Builder& dopplerVelocity(math::Velocity value) { _dopplerVelocity = value; return *this; }
        Builder& latitude(math::Angle value) { _latitude = value; return *this; }
        Builder& longitude(math::Angle value) { _longitude = value; return *this; }
        Builder& altitude(math::Distance value) { _altitude = value; return *this; }
        Builder& velocity(math::Velocity value) { _velocity = value; return *this; }
        Builder& heading(math::Angle value) { _heading = value; return *this; }
        Builder& flightPathAngle(math::Angle value) { _flightPathAngle = value; return *this; }
        Builder& timestamp(std::chrono::milliseconds value) { _timestamp = value; return *this; }

        TrackEvent build() const
        {
            return TrackEvent(
                _id.value(), _state.value(), _slantRange.value(), _groundRange.value(),
                _azimuth.value(), _elevation.value(), _dopplerVelocity.value(),
                _latitude.value(), _longitude.value(), _altitude.value(), _velocity.value(),
                _heading.value(), _flightPathAngle.value(), _timestamp.value());
        }

    private:
        std::optional<uint16_t> _id;
        std::optional<TrackState> _state;
        std::optional<math::Distance> _slantRange;
        std::optional<math::Distance> _groundRange;
        std::optional<math::Angle> _azimuth;
        std::optional<math::Angle> _elevation;
        std::optional<math::Velocity> _dopplerVelocity;
        std::optional<math::Angle> _latitude;
        std::optional<math::Angle> _longitude;
        std::optional<math::Distance> _altitude;
        std::optional<math::Velocity> _velocity;
        std::optional<math::Angle> _heading;
        std::optional<math::Angle> _flightPathAngle;
        std::optional<std::chrono::milliseconds> _timestamp;
    };
};

} // namespace target
