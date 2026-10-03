#pragma once

#include <math/angle.hpp>

#include <math/matrix.hpp>

namespace beam
{

using math::literals::operator""_deg;

struct AttitudeConfig
{
    math::Angle maxAbsPitch = 89_deg;
    math::Matrix mountAntToBodyRotation = math::Matrix::identity(3);
    double mountOrthogonalTolerance = 1e-6;
};

} // namespace beam
