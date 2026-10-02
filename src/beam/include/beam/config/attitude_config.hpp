#pragma once

#include <math/matrix.hpp>

namespace beam
{

struct AttitudeConfig
{
    double maxAbsPitch_deg = 89.0;
    math::Matrix mountAntToBody = math::Matrix::Identity(3);
    double mountOrthoTol = 1e-6;
};

} // namespace beam
