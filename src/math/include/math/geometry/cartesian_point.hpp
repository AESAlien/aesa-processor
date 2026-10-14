#pragma once

#include "math/quantity/distance.hpp"

namespace math
{

class CartesianPoint
{
public:
    CartesianPoint(Distance x, Distance y, Distance z) noexcept;

    Distance x() const noexcept;
    Distance y() const noexcept;
    Distance z() const noexcept;

private:
    Distance _x;
    Distance _y;
    Distance _z;
};

}
