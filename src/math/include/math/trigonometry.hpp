#pragma once

#include "math/geometry/azimuth.hpp"
#include "math/geometry/elevation.hpp"
#include "math/quantity/angle.hpp"
#include "math/quantity/length.hpp"
#include "math/quantity/linear_velocity.hpp"

namespace math
{

double sin(Angle angle);
double cos(Angle angle);
double tan(Angle angle);
double sin(Azimuth azimuth);
double cos(Azimuth azimuth);
double tan(Azimuth azimuth);
double sin(Elevation elevation);
double cos(Elevation elevation);
double tan(Elevation elevation);

Angle asin(double value);
Angle acos(double value);
Angle atan(double value);
Angle atan2(Length y, Length x);
Angle atan2(LinearVelocity y, LinearVelocity x);

}
