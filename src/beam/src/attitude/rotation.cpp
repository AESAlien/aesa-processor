#include "rotation.hpp"
#include <math/angle.hpp>
#include <cmath>

namespace beam
{

math::Matrix bodyToEnu(double roll_deg, double pitch_deg, double yaw_deg)
{
    const double cr = std::cos(math::DegToRad(roll_deg));
    const double sr = std::sin(math::DegToRad(roll_deg));
    const double cp = std::cos(math::DegToRad(pitch_deg));
    const double sp = std::sin(math::DegToRad(pitch_deg));
    const double cy = std::cos(math::DegToRad(yaw_deg));
    const double sy = std::sin(math::DegToRad(yaw_deg));

    const math::Matrix Rz{ {cy, sy, 0}, {-sy, cy, 0}, {0, 0, 1} };
    const math::Matrix Rx{ {1, 0, 0}, {0, cp, -sp}, {0, sp, cp} };
    const math::Matrix Ry{ {cr, 0, sr}, {0, 1, 0}, {-sr, 0, cr} };

    return Rz * (Rx * Ry);
}

}   // namespace beam
