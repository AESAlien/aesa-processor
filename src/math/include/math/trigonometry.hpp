#pragma once

#include "math/bounded/circular_angle.hpp"
#include "math/bounded/tilt_angle.hpp"
#include "math/quantity/angle.hpp"
#include "math/quantity/length.hpp"
#include "math/quantity/linear_velocity.hpp"

namespace math
{

double sin(Angle angle);
double cos(Angle angle);
double tan(Angle angle);
double sin(CircularAngle angle);
double cos(CircularAngle angle);
double tan(CircularAngle angle);
double sin(TiltAngle angle);
double cos(TiltAngle angle);
double tan(TiltAngle angle);

Angle asin(double value);
Angle acos(double value);
Angle atan(double value);
Angle atan2(Length y, Length x);
Angle atan2(LinearVelocity y, LinearVelocity x);

}
