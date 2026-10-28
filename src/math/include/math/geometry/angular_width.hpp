#pragma once

#include "math/quantity/angle.hpp"

namespace math
{

class AngularWidth
{
public:
    explicit AngularWidth(Angle angle);

    double deg() const noexcept;
    double rad() const noexcept;

    bool equals(
        const AngularWidth& other,
        const QuantityTolerance<Angle>& tolerance = Angle::DEFAULT_TOLERANCE
    ) const noexcept;
    bool equals(
        Angle other,
        const QuantityTolerance<Angle>& tolerance = Angle::DEFAULT_TOLERANCE
    ) const noexcept;

    AngularWidth operator+(Angle offset) const;
    AngularWidth operator-(Angle offset) const;
    AngularWidth& operator+=(Angle offset);
    AngularWidth& operator-=(Angle offset);

    bool operator==(const AngularWidth& other) const noexcept;
    bool operator!=(const AngularWidth& other) const noexcept;
    bool operator==(Angle other) const noexcept;
    bool operator!=(Angle other) const noexcept;

    friend AngularWidth operator+(Angle offset, const AngularWidth& width);
    friend bool operator==(Angle angle, const AngularWidth& width) noexcept;
    friend bool operator!=(Angle angle, const AngularWidth& width) noexcept;

private:
    void validate() const;

    Angle _angle;
};

}
