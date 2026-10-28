#pragma once

#include <math/geometry/angular_width.hpp>

namespace beam
{

struct BeamWidth
{
    BeamWidth(
        math::AngularWidth azimuth,
        math::AngularWidth elevation);

    bool operator==(const BeamWidth& other) const noexcept;
    bool operator!=(const BeamWidth& other) const noexcept;

    math::AngularWidth azimuth;
    math::AngularWidth elevation;
};

}
