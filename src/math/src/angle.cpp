#include "math/angle.hpp"

namespace math
{

const double Pi = 3.14159265358979323846;

double DegToRad(double degrees) noexcept
{
    return degrees * (Pi / 180.0);
}

double RadToDeg(double radians) noexcept
{
    return radians * (180.0 / Pi);
}

} // namespace math
