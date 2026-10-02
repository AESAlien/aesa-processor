#pragma once

#include <beam/dto/beam_command.hpp>

namespace target
{

struct DetectionEvent
{
    beam::BeamCommand::BeamType beamType{};
    float slantRange_km{};
    float azimuth_deg{};
    float elevation_deg{};
    float dopplerVelocity_mps{};
    float power_db{};
};

} // namespace target
