#pragma once

#include <math/geometry/spherical_direction.hpp>
#include <math/quantity/angle.hpp>
#include <cstdint>

namespace beam
{

struct SearchBeam
{
    const uint32_t beamId;
    // 안테나 좌표계 빔 지향 방향. 생성 시점에 범위가 검증된다.
    const math::SphericalDirection direction_ant;
    const math::Angle azimuthBeamWidth;
    const math::Angle elevationBeamWidth;

    SearchBeam(
        uint32_t beamId,
        math::SphericalDirection direction_ant,
        math::Angle azimuthBeamWidth,
        math::Angle elevationBeamWidth
    ) : beamId(beamId),
        direction_ant(direction_ant),
        azimuthBeamWidth(azimuthBeamWidth),
        elevationBeamWidth(elevationBeamWidth)
    {
    }
};

} // namespace beam
