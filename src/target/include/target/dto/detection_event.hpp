#pragma once

#include <beam/dto/beam_command.hpp>
#include <math/angle.hpp>
#include <math/distance.hpp>
#include <math/velocity.hpp>

#include <chrono>
#include <cstdint>
#include <utility>
#include <vector>

namespace target
{

struct DetectionEvent
{
    struct Plot
    {
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

        const math::Distance slantRange;
        const math::Angle azimuth;
        const math::Angle elevation;
        const math::Velocity dopplerVelocity;
        const float power_db;
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

    const beam::BeamCommand::BeamType beamType;
    const uint32_t requestId;
    const std::chrono::milliseconds transmitTime;
    const std::vector<Plot> plots;
};

} // namespace target
