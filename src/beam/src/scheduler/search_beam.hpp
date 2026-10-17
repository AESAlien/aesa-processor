#pragma once

#include <math/geometry/spherical_direction.hpp>
#include <math/quantity/angle.hpp>
#include <cstdint>

namespace beam
{

using math::literals::operator""_deg;

struct SearchBeam
{
    uint32_t beamId;
    // 안테나 좌표계 빔 지향 방향. 생성 시점에 범위가 검증된다.
    math::SphericalDirection direction_ant;
    math::Angle azimuthBeamWidth = 0_deg;
    math::Angle elevationBeamWidth = 0_deg;
};

} // namespace beam
