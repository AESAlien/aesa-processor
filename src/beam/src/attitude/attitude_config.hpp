#pragma once

#include <math/angle.hpp>

#include <math/square_matrix.hpp>

namespace beam
{

using math::literals::operator""_deg;

struct AttitudeConfig
{
    math::Angle maxAbsPitch = 89_deg;
    math::SquareMatrix mountAntToBodyRotation = math::SquareMatrix::identity(3);
    double mountOrthogonalTolerance = 1e-6;
};

} // namespace beam
