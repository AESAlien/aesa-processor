#pragma once

#include <beam/domain/beam_status.hpp>

namespace target {

struct DetectionDto
{
    beam::BeamType beamType{};
    float slantRange_km{};
    float azimuth_deg{};
    float elevation_deg{};
    float dopplerVelocity_mps{};
    float power_db{};
};

} // namespace target
