#pragma once

#include <math/quantity/angle.hpp>
#include <math/linear_algebra/square_matrix.hpp>
#include <math/tolerance.hpp>

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
        Builder& mountOrthogonalTolerance(const math::Tolerance& value) {
            _mountOrthogonalAbsoluteTolerance = value.absolute;
            _mountOrthogonalRelativeTolerance = value.relative;
            return *this;
        }

        AttitudeConfig build() const;

    private:
        math::Angle _maxAbsPitch = 89_deg;
        math::SquareMatrix _mountAntToBodyRotation = math::SquareMatrix::identity(3);
        double _mountOrthogonalAbsoluteTolerance = 1e-6;
        double _mountOrthogonalRelativeTolerance = 0.0;
    }; // class Builder

    AttitudeConfig();
    AttitudeConfig(math::Angle maxAbsPitch,
                   math::SquareMatrix mountAntToBodyRotation,
                   math::Tolerance mountOrthogonalTolerance);

    const math::Angle maxAbsPitch;
    const math::SquareMatrix mountAntToBodyRotation;
    const math::Tolerance mountOrthogonalTolerance;

private:
    void validate() const;
};

} // namespace beam