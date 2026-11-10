#pragma once

#include <math/geometry/spherical_direction.hpp>
#include <beam/domain/beam_width.hpp>
#include <cstdint>

namespace beam
{

struct SearchBeam
{
    uint32_t beamId;
    // ENU 기준 빔 지향 방향. 생성 시점에 범위가 검증된다.
    math::SphericalDirection direction_enu;
    BeamWidth beamWidth;

    SearchBeam(
        uint32_t beamId,
        math::SphericalDirection direction_enu,
        BeamWidth beamWidth
    ) : beamId(beamId),
        direction_enu(direction_enu),
        beamWidth(beamWidth)
    {
    }
};

} // namespace beam
