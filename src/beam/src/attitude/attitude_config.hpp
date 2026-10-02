#pragma once

#include <math/matrix.hpp>

namespace beam
{

struct AttitudeConfig
{
    double maxAbsPitch_deg = 89.0;
    math::Matrix mountAntToBodyRotation = math::Matrix::identity(3);
    double mountOrthogonalTolerance = 1e-6;
};

} // namespace beam
