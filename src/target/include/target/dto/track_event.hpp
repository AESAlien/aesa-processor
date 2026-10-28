#pragma once

#include <chrono>
#include <cstdint>
#include <optional>
#include <math/quantity/angle.hpp>
#include <math/geometry/azimuth.hpp>
#include <math/geometry/elevation.hpp>
#include <math/quantity/length.hpp>
#include <math/quantity/linear_velocity.hpp>

namespace target
{

struct TrackEvent
{
    enum class TrackState;

    uint16_t id;
    TrackState state;
    math::Length slantRange;
    math::Length groundRange;
    math::Azimuth azimuth;
    math::Elevation elevation;
    math::LinearVelocity dopplerVelocity;
    math::Angle latitude;
    math::Angle longitude;
    math::Length altitude;
    math::LinearVelocity velocity;
    math::Angle heading;
    math::Angle flightPathAngle;
    // Elapsed time since the shared start at 0 ms.
    std::chrono::milliseconds timestamp;

    enum class TrackState
    {
        INIT = 1,
        TRACKING = 2
    };

    TrackEvent(
        uint16_t id,
        TrackState state,
        math::Length slantRange,
        math::Length groundRange,
        math::Azimuth azimuth,
        math::Elevation elevation,
        math::LinearVelocity dopplerVelocity,
        math::Angle latitude,
        math::Angle longitude,
        math::Length altitude,
        math::LinearVelocity velocity,
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
        Builder& slantRange(math::Length value) { _slantRange = value; return *this; }
        Builder& groundRange(math::Length value) { _groundRange = value; return *this; }
        Builder& azimuth(math::Azimuth value) { _azimuth = value; return *this; }
        Builder& elevation(math::Elevation value) { _elevation = value; return *this; }
        Builder& dopplerVelocity(math::LinearVelocity value) { _dopplerVelocity = value; return *this; }
        Builder& latitude(math::Angle value) { _latitude = value; return *this; }
        Builder& longitude(math::Angle value) { _longitude = value; return *this; }
        Builder& altitude(math::Length value) { _altitude = value; return *this; }
        Builder& velocity(math::LinearVelocity value) { _velocity = value; return *this; }
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
        std::optional<math::Length> _slantRange;
        std::optional<math::Length> _groundRange;
        std::optional<math::Azimuth> _azimuth;
        std::optional<math::Elevation> _elevation;
        std::optional<math::LinearVelocity> _dopplerVelocity;
        std::optional<math::Angle> _latitude;
        std::optional<math::Angle> _longitude;
        std::optional<math::Length> _altitude;
        std::optional<math::LinearVelocity> _velocity;
        std::optional<math::Angle> _heading;
        std::optional<math::Angle> _flightPathAngle;
        std::optional<std::chrono::milliseconds> _timestamp;
    };
};

} // namespace target
