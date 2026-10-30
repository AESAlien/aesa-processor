#pragma once

#include <math/bounded/angular_width.hpp>

namespace beam
{

struct BeamWidth
{
    BeamWidth(
        math::Angle azimuth,
        math::Angle elevation);

    BeamWidth(
        math::AngularWidth azimuth,
        math::AngularWidth elevation);

    bool operator==(const BeamWidth& other) const noexcept;
    bool operator!=(const BeamWidth& other) const noexcept;

    math::AngularWidth azimuth;
    math::AngularWidth elevation;
};

}
