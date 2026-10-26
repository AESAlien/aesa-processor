#pragma once

#include <math/quantity/angle.hpp>
#include <math/linear_algebra/square_matrix.hpp>

#include <utility>
#include <cstddef>

namespace beam
{
using math::literals::operator""_deg;

struct AttitudeConfig
{
public:
    class Builder
    {
    public:
        Builder& maxAbsPitch(math::Angle value)                     { _maxAbsPitch = value; return *this; }
        Builder& mountAntToBodyRotation(math::SquareMatrix value)   { _mountAntToBodyRotation = std::move(value); return *this; }
        Builder& mountOrthogonalTolerance(double value)             { _mountOrthogonalTolerance = value; return *this; }

        AttitudeConfig build() const;

    private:
        math::Angle _maxAbsPitch = 89_deg;
        math::SquareMatrix _mountAntToBodyRotation = math::SquareMatrix::identity(3);
        double _mountOrthogonalTolerance = 1e-6;
    }; // class Builder

    AttitudeConfig();
    AttitudeConfig(math::Angle maxAbsPitch,
                   math::SquareMatrix mountAntToBodyRotation,
                   double mountOrthogonalTolerance);

    const math::Angle maxAbsPitch;
    const math::SquareMatrix mountAntToBodyRotation;
    const double mountOrthogonalTolerance;

private:
    void validate() const;
};

} // namespace beam