#pragma once

#include <math/geometry/spherical_direction.hpp>
#include <math/quantity/angle.hpp>
#include <cstdint>

namespace aesa_processor::beam
{

struct SearchBeam
{
    const uint32_t beamId;
    // ENU 기준 빔 지향 방향. 생성 시점에 범위가 검증된다.
    const aesa_processor::math::SphericalDirection direction_enu;
    const aesa_processor::math::Angle azimuthBeamWidth;
    const aesa_processor::math::Angle elevationBeamWidth;

    SearchBeam(
        uint32_t beamId,
        aesa_processor::math::SphericalDirection direction_enu,
        aesa_processor::math::Angle azimuthBeamWidth,
        aesa_processor::math::Angle elevationBeamWidth
    ) : beamId(beamId),
        direction_enu(direction_enu),
        azimuthBeamWidth(azimuthBeamWidth),
        elevationBeamWidth(elevationBeamWidth)
    {
    }
};

} // namespace aesa_processor::beam
