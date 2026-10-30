#pragma once

#include "math/quantity/angle.hpp"

namespace math
{

class TiltAngle
{
public:
    explicit TiltAngle(Angle angle);

    static TiltAngle fromDegrees(double degrees);
    static TiltAngle fromRadians(double radians);

    double deg() const noexcept;
    double rad() const noexcept;

    bool equals(
        const TiltAngle& other,
        const QuantityTolerance<Angle>& tolerance = Angle::DEFAULT_TOLERANCE
    ) const noexcept;
    bool equals(
        Angle other,
        const QuantityTolerance<Angle>& tolerance = Angle::DEFAULT_TOLERANCE
    ) const noexcept;

    TiltAngle operator+(Angle offset) const;
    TiltAngle operator-(Angle offset) const;
    TiltAngle& operator+=(Angle offset);
    TiltAngle& operator-=(Angle offset);

    bool operator==(const TiltAngle& other) const noexcept;
    bool operator!=(const TiltAngle& other) const noexcept;
    bool operator==(Angle other) const noexcept;
    bool operator!=(Angle other) const noexcept;

    friend TiltAngle operator+(Angle offset, const TiltAngle& elevation);
    friend bool operator==(Angle angle, const TiltAngle& elevation) noexcept;
    friend bool operator!=(Angle angle, const TiltAngle& elevation) noexcept;

private:
    void validate() const;

    Angle _angle;
};

}
