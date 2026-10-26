#pragma once

#include <math/quantity/angle.hpp>
#include <math/linear_algebra/square_matrix.hpp>
#include <math/tolerance.hpp>

#include <cstddef>
#include <optional>
#include <utility>

namespace beam
{
struct AttitudeConfig
{
public:
    class Builder
    {
    public:
        Builder& maxAbsPitch(math::Angle value)                     { _maxAbsPitch = value; return *this; }
        Builder& mountAntToBodyRotation(math::SquareMatrix value)   { _mountAntToBodyRotation = std::move(value); return *this; }
        Builder& mountOrthogonalTolerance(const math::Tolerance& value) { _mountOrthogonalTolerance.emplace(value); return *this; }

        AttitudeConfig build() const;

    private:
        std::optional<math::Angle> _maxAbsPitch;
        std::optional<math::SquareMatrix> _mountAntToBodyRotation;
        std::optional<math::Tolerance> _mountOrthogonalTolerance;
    }; // class Builder

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
