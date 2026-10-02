#pragma once
#include <math/matrix.hpp>

namespace beam
{

math::Matrix bodyToEnu(double roll_deg, double pitch_deg, double yaw_deg);

}   // namespace beam