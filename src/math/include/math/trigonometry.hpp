#pragma once

#include "math/angle.hpp"

namespace math
{

double sin(Angle angle);
double cos(Angle angle);
double tan(Angle angle);

Angle asin(double value);
Angle acos(double value);
Angle atan(double value);

}
