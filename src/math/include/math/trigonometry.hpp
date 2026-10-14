#pragma once

#include "math/quantity/angle.hpp"
#include "math/quantity/distance.hpp"

namespace math
{

double sin(Angle angle);
double cos(Angle angle);
double tan(Angle angle);

Angle asin(double value);
Angle acos(double value);
Angle atan(double value);
Angle atan2(Distance y, Distance x);

}
