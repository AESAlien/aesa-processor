#pragma once

#include <beam/domain/beam_type.hpp>

namespace target {

struct Detection
{
    beam::BeamType beamType{};
    float slantRange_km{};
    float azimuth_deg{};
    float elevation_deg{};
    float dopplerVelocity_mps{};
    float power_db{};
};

} // namespace target
