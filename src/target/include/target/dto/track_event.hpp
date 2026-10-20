#pragma once

#include <chrono>
#include <cstdint>
#include <optional>
#include <math/quantity/angle.hpp>
#include <math/quantity/length.hpp>
#include <math/quantity/linear_velocity.hpp>

namespace aesa_processor::target
{

struct TrackEvent
{
    enum class TrackState;

    const uint16_t id;
    const TrackState state;
    const aesa_processor::math::Length slantRange;
    const aesa_processor::math::Length groundRange;
    const aesa_processor::math::Angle azimuth;
    const aesa_processor::math::Angle elevation;
    const aesa_processor::math::LinearVelocity dopplerVelocity;
    const aesa_processor::math::Angle latitude;
    const aesa_processor::math::Angle longitude;
    const aesa_processor::math::Length altitude;
    const aesa_processor::math::LinearVelocity velocity;
    const aesa_processor::math::Angle heading;
    const aesa_processor::math::Angle flightPathAngle;
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
        aesa_processor::math::Length slantRange,
        aesa_processor::math::Length groundRange,
        aesa_processor::math::Angle azimuth,
        aesa_processor::math::Angle elevation,
        aesa_processor::math::LinearVelocity dopplerVelocity,
        aesa_processor::math::Angle latitude,
        aesa_processor::math::Angle longitude,
        aesa_processor::math::Length altitude,
        aesa_processor::math::LinearVelocity velocity,
        aesa_processor::math::Angle heading,
        aesa_processor::math::Angle flightPathAngle,
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
        Builder& slantRange(aesa_processor::math::Length value) { _slantRange = value; return *this; }
        Builder& groundRange(aesa_processor::math::Length value) { _groundRange = value; return *this; }
        Builder& azimuth(aesa_processor::math::Angle value) { _azimuth = value; return *this; }
        Builder& elevation(aesa_processor::math::Angle value) { _elevation = value; return *this; }
        Builder& dopplerVelocity(aesa_processor::math::LinearVelocity value) { _dopplerVelocity = value; return *this; }
        Builder& latitude(aesa_processor::math::Angle value) { _latitude = value; return *this; }
        Builder& longitude(aesa_processor::math::Angle value) { _longitude = value; return *this; }
        Builder& altitude(aesa_processor::math::Length value) { _altitude = value; return *this; }
        Builder& velocity(aesa_processor::math::LinearVelocity value) { _velocity = value; return *this; }
        Builder& heading(aesa_processor::math::Angle value) { _heading = value; return *this; }
        Builder& flightPathAngle(aesa_processor::math::Angle value) { _flightPathAngle = value; return *this; }
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
        std::optional<aesa_processor::math::Length> _slantRange;
        std::optional<aesa_processor::math::Length> _groundRange;
        std::optional<aesa_processor::math::Angle> _azimuth;
        std::optional<aesa_processor::math::Angle> _elevation;
        std::optional<aesa_processor::math::LinearVelocity> _dopplerVelocity;
        std::optional<aesa_processor::math::Angle> _latitude;
        std::optional<aesa_processor::math::Angle> _longitude;
        std::optional<aesa_processor::math::Length> _altitude;
        std::optional<aesa_processor::math::LinearVelocity> _velocity;
        std::optional<aesa_processor::math::Angle> _heading;
        std::optional<aesa_processor::math::Angle> _flightPathAngle;
        std::optional<std::chrono::milliseconds> _timestamp;
    };
};

} // namespace aesa_processor::target
