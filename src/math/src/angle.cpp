#include "math/angle.hpp"

namespace math
{

const double PI = 3.14159265358979323846;

double degToRad(double degrees) noexcept
{
    return degrees * (PI / 180.0);
}

double radToDeg(double radians) noexcept
{
    return radians * (180.0 / PI);
}

} // namespace math
