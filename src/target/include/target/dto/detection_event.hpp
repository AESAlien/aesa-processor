#pragma once

#include <beam/dto/beam_command.hpp>
#include <math/angle.hpp>
#include <math/distance.hpp>
#include <math/velocity.hpp>

#include <cstdint>

namespace target
{

struct DetectionEvent
{
    DetectionEvent(
        beam::BeamCommand::BeamType beamType,
        uint32_t requestId,
        math::Distance slantRange,
        math::Angle azimuth,
        math::Angle elevation,
        math::Velocity dopplerVelocity,
        float power_db
    ) : beamType(beamType),
        requestId(requestId),
        slantRange(slantRange),
        azimuth(azimuth),
        elevation(elevation),
        dopplerVelocity(dopplerVelocity),
        power_db(power_db)
    {
    }

    const beam::BeamCommand::BeamType beamType;
    const uint32_t requestId;
    const math::Distance slantRange;
    const math::Angle azimuth;
    const math::Angle elevation;
    const math::Velocity dopplerVelocity;
    const float power_db;
};

} // namespace target
