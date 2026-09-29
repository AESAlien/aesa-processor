#pragma once

#include <cstdint>

#include <target/domain/track_state.hpp>

namespace target {

struct TrackSnapshotDto
{
    std::uint16_t id{};
    TrackState state{TrackState::Init};
    float slantRange_km{};
    float groundRange_km{};
    float azimuth_deg{};
    float elevation_deg{};
    float dopplerVelocity_mps{};
    float latitude_deg{};
    float longitude_deg{};
    float altitude_km{};
    float velocity_mps{};
    float heading_deg{};
    float flightPathAngle_deg{};
    std::uint32_t timestamp_ms{};
};

} // namespace target
