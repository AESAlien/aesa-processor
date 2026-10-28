#pragma once

#include "math/quantity/angle.hpp"

namespace math
{

class Elevation
{
public:
    explicit Elevation(Angle angle);

    static Elevation fromDegrees(double degrees);
    static Elevation fromRadians(double radians);

    double deg() const noexcept;
    double rad() const noexcept;

    bool equals(
        const Elevation& other,
        const QuantityTolerance<Angle>& tolerance = Angle::DEFAULT_TOLERANCE
    ) const noexcept;
    bool equals(
        Angle other,
        const QuantityTolerance<Angle>& tolerance = Angle::DEFAULT_TOLERANCE
    ) const noexcept;

    Elevation operator+(Angle offset) const;
    Elevation operator-(Angle offset) const;
    Elevation& operator+=(Angle offset);
    Elevation& operator-=(Angle offset);

    bool operator==(const Elevation& other) const noexcept;
    bool operator!=(const Elevation& other) const noexcept;
    bool operator==(Angle other) const noexcept;
    bool operator!=(Angle other) const noexcept;

    friend Elevation operator+(Angle offset, const Elevation& elevation);
    friend bool operator==(Angle angle, const Elevation& elevation) noexcept;
    friend bool operator!=(Angle angle, const Elevation& elevation) noexcept;

private:
    void validate() const;

    Angle _angle;
};

}
