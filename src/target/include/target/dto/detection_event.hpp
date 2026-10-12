#pragma once

#include <beam/dto/beam_command.hpp>
#include <math/angle.hpp>
#include <math/distance.hpp>
#include <math/velocity.hpp>

#include <chrono>
#include <cstdint>
#include <optional>
#include <utility>
#include <vector>

namespace target
{

struct DetectionEvent
{
    struct Plot;

    const beam::BeamCommand::BeamType beamType;
    const uint32_t requestId;
    const std::chrono::milliseconds transmitTime;
    const std::vector<Plot> plots;

    struct Plot
    {
        const math::Distance slantRange;
        const math::Angle azimuth;
        const math::Angle elevation;
        const math::Velocity dopplerVelocity;
        const float power_db;

        Plot(
            math::Distance slantRange,
            math::Angle azimuth,
            math::Angle elevation,
            math::Velocity dopplerVelocity,
            float power_db
        ) : slantRange(slantRange),
            azimuth(azimuth),
            elevation(elevation),
            dopplerVelocity(dopplerVelocity),
            power_db(power_db)
        {
        }

        class Builder
        {
        public:
            Builder& slantRange(math::Distance value) { _slantRange = value; return *this; }
            Builder& azimuth(math::Angle value) { _azimuth = value; return *this; }
            Builder& elevation(math::Angle value) { _elevation = value; return *this; }
            Builder& dopplerVelocity(math::Velocity value) { _dopplerVelocity = value; return *this; }
            Builder& power_db(float value) { _power_db = value; return *this; }

            Plot build() const
            {
                return Plot(
                    _slantRange.value(), _azimuth.value(), _elevation.value(),
                    _dopplerVelocity.value(), _power_db.value());
            }

        private:
            std::optional<math::Distance> _slantRange;
            std::optional<math::Angle> _azimuth;
            std::optional<math::Angle> _elevation;
            std::optional<math::Velocity> _dopplerVelocity;
            std::optional<float> _power_db;
        };
    };

    DetectionEvent(
        beam::BeamCommand::BeamType beamType,
        uint32_t requestId,
        std::chrono::milliseconds transmitTime,
        std::vector<Plot> plots
    ) : beamType(beamType),
        requestId(requestId),
        transmitTime(transmitTime),
        plots(std::move(plots))
    {
    }

    class Builder
    {
    public:
        Builder& beamType(beam::BeamCommand::BeamType value) { _beamType = value; return *this; }
        Builder& requestId(uint32_t value) { _requestId = value; return *this; }
        Builder& transmitTime(std::chrono::milliseconds value) { _transmitTime = value; return *this; }
        Builder& plots(std::vector<Plot> value) { _plots = std::move(value); return *this; }

        DetectionEvent build() const
        {
            return DetectionEvent(
                _beamType.value(), _requestId.value(), _transmitTime.value(), _plots.value());
        }

    private:
        std::optional<beam::BeamCommand::BeamType> _beamType;
        std::optional<uint32_t> _requestId;
        std::optional<std::chrono::milliseconds> _transmitTime;
        std::optional<std::vector<Plot>> _plots;
    };
};

} // namespace target
