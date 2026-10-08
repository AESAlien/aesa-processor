#pragma once

#include <beam/dto/beam_command.hpp>
#include <math/angle.hpp>
#include <math/distance.hpp>
#include <math/velocity.hpp>

namespace target
{

using math::literals::operator""_deg;
using math::literals::operator""_km;
struct DetectionEvent
{
    beam::BeamCommand::BeamType beamType;
    math::Distance slantRange;
    math::Angle azimuth;
    math::Angle elevation;
    math::Velocity dopplerVelocity;
    float power_db;
};

} // namespace target
