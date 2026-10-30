#pragma once

#include "math/quantity/angle.hpp"

namespace math
{

class CircularAngle
{
public:
    explicit CircularAngle(Angle angle) noexcept;

    static CircularAngle fromDegrees(double degrees);
    static CircularAngle fromRadians(double radians);

    double deg() const noexcept;
    double rad() const noexcept;

    bool isEquivalentTo(
        const CircularAngle& other,
        const QuantityTolerance<Angle>& tolerance = Angle::DEFAULT_TOLERANCE
    ) const noexcept;
    bool isEquivalentTo(
        Angle other,
        const QuantityTolerance<Angle>& tolerance = Angle::DEFAULT_TOLERANCE
    ) const noexcept;

    CircularAngle operator+(Angle offset) const;
    CircularAngle operator-(Angle offset) const;
    CircularAngle& operator+=(Angle offset);
    CircularAngle& operator-=(Angle offset);

    bool operator==(const CircularAngle& other) const noexcept;
    bool operator!=(const CircularAngle& other) const noexcept;
    bool operator==(Angle other) const noexcept;
    bool operator!=(Angle other) const noexcept;

    friend CircularAngle operator+(Angle offset, const CircularAngle& azimuth);
    friend bool operator==(Angle angle, const CircularAngle& azimuth) noexcept;
    friend bool operator!=(Angle angle, const CircularAngle& azimuth) noexcept;

private:
    Angle _angle;
};

}
