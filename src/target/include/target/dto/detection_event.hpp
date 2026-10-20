#pragma once

#include <beam/dto/beam_type.hpp>
#include <math/quantity/angle.hpp>
#include <math/quantity/length.hpp>
#include <math/quantity/linear_velocity.hpp>

#include <chrono>
#include <cstdint>
#include <optional>
#include <utility>
#include <vector>

namespace aesa_processor::target
{

struct DetectionEvent
{
    struct Plot;

    const aesa_processor::beam::BeamType beamType;
    const uint32_t requestId;
    const std::chrono::milliseconds transmitTime;
    const std::vector<Plot> plots;

    struct Plot
    {
        const aesa_processor::math::Length slantRange;
        const aesa_processor::math::Angle azimuth;
        const aesa_processor::math::Angle elevation;
        const aesa_processor::math::LinearVelocity dopplerVelocity;
        const float power_db;

        Plot(
            aesa_processor::math::Length slantRange,
            aesa_processor::math::Angle azimuth,
            aesa_processor::math::Angle elevation,
            aesa_processor::math::LinearVelocity dopplerVelocity,
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
            Builder& slantRange(aesa_processor::math::Length value) { _slantRange = value; return *this; }
            Builder& azimuth(aesa_processor::math::Angle value) { _azimuth = value; return *this; }
            Builder& elevation(aesa_processor::math::Angle value) { _elevation = value; return *this; }
            Builder& dopplerVelocity(aesa_processor::math::LinearVelocity value) { _dopplerVelocity = value; return *this; }
            Builder& power_db(float value) { _power_db = value; return *this; }

            Plot build() const
            {
                return Plot(
                    _slantRange.value(), _azimuth.value(), _elevation.value(),
                    _dopplerVelocity.value(), _power_db.value());
            }

        private:
            std::optional<aesa_processor::math::Length> _slantRange;
            std::optional<aesa_processor::math::Angle> _azimuth;
            std::optional<aesa_processor::math::Angle> _elevation;
            std::optional<aesa_processor::math::LinearVelocity> _dopplerVelocity;
            std::optional<float> _power_db;
        };
    };

    DetectionEvent(
        aesa_processor::beam::BeamType beamType,
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
        Builder& beamType(aesa_processor::beam::BeamType value) { _beamType = value; return *this; }
        Builder& requestId(uint32_t value) { _requestId = value; return *this; }
        Builder& transmitTime(std::chrono::milliseconds value) { _transmitTime = value; return *this; }
        Builder& plots(std::vector<Plot> value) { _plots = std::move(value); return *this; }

        DetectionEvent build() const
        {
            return DetectionEvent(
                _beamType.value(), _requestId.value(), _transmitTime.value(), _plots.value());
        }

    private:
        std::optional<aesa_processor::beam::BeamType> _beamType;
        std::optional<uint32_t> _requestId;
        std::optional<std::chrono::milliseconds> _transmitTime;
        std::optional<std::vector<Plot>> _plots;
    };
};

} // namespace aesa_processor::target
