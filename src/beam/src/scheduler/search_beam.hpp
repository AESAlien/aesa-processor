#pragma once

#include <math/geometry/spherical_direction.hpp>
#include <math/quantity/angle.hpp>
#include <cstdint>

namespace beam
{

struct SearchBeam
{
    const uint32_t beamId;
    // ENU 기준 빔 지향 방향. 생성 시점에 범위가 검증된다.
    const math::SphericalDirection direction_enu;
    const math::Angle azimuthBeamWidth;
    const math::Angle elevationBeamWidth;

    SearchBeam(
        uint32_t beamId,
        math::SphericalDirection direction_enu,
        math::Angle azimuthBeamWidth,
        math::Angle elevationBeamWidth
    ) : beamId(beamId),
        direction_enu(direction_enu),
        azimuthBeamWidth(azimuthBeamWidth),
        elevationBeamWidth(elevationBeamWidth)
    {
    }
};

} // namespace beam
