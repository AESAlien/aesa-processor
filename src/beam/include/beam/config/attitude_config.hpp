#pragma once

#include <math/matrix.hpp>

namespace beam
{

struct AttitudeConfig
{
    double max_abs_pitch_deg = 89.0;
    math::Matrix mount_ant_to_body = math::Matrix::Identity(3);
    double mount_ortho_tol = 1e-6;
};

}   // namespace beam
