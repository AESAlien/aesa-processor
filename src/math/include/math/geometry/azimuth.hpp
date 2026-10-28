#pragma once

#include "math/quantity/angle.hpp"

namespace math
{

class Azimuth
{
public:
    explicit Azimuth(Angle angle) noexcept;

    static Azimuth fromDegrees(double degrees);
    static Azimuth fromRadians(double radians);

    double deg() const noexcept;
    double rad() const noexcept;

    bool isEquivalentTo(
        const Azimuth& other,
        const QuantityTolerance<Angle>& tolerance = Angle::DEFAULT_TOLERANCE
    ) const noexcept;
    bool isEquivalentTo(
        Angle other,
        const QuantityTolerance<Angle>& tolerance = Angle::DEFAULT_TOLERANCE
    ) const noexcept;

    Azimuth operator+(Angle offset) const;
    Azimuth operator-(Angle offset) const;
    Azimuth& operator+=(Angle offset);
    Azimuth& operator-=(Angle offset);

    bool operator==(const Azimuth& other) const noexcept;
    bool operator!=(const Azimuth& other) const noexcept;
    bool operator==(Angle other) const noexcept;
    bool operator!=(Angle other) const noexcept;

    friend Azimuth operator+(Angle offset, const Azimuth& azimuth);
    friend bool operator==(Angle angle, const Azimuth& azimuth) noexcept;
    friend bool operator!=(Angle angle, const Azimuth& azimuth) noexcept;

private:
    Angle _angle;
};

}
