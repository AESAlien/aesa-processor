#pragma once

#include <math/quantity/angle.hpp>
#include <math/linear_algebra/square_matrix.hpp>

#include <utility>
#include <cstddef>

namespace aesa_processor::beam
{
using aesa_processor::math::literals::operator""_deg;

class AttitudeConfig
{
public:
    class Builder
    {
    public:
        Builder& maxAbsPitch(aesa_processor::math::Angle value)                     { _maxAbsPitch = value; return *this; }
        Builder& mountAntToBodyRotation(aesa_processor::math::SquareMatrix value)   { _mountAntToBodyRotation = std::move(value); return *this; }
        Builder& mountOrthogonalTolerance(double value)             { _mountOrthogonalTolerance = value; return *this; }

        AttitudeConfig build() const;

    private:
        aesa_processor::math::Angle _maxAbsPitch = 89_deg;
        aesa_processor::math::SquareMatrix _mountAntToBodyRotation = aesa_processor::math::SquareMatrix::identity(3);
        double _mountOrthogonalTolerance = 1e-6;
    }; // class Builder

    AttitudeConfig();
    AttitudeConfig(aesa_processor::math::Angle maxAbsPitch,
                   aesa_processor::math::SquareMatrix mountAntToBodyRotation,
                   double mountOrthogonalTolerance);

    aesa_processor::math::Angle maxAbsPitch() const noexcept;
    const aesa_processor::math::SquareMatrix& mountAntToBodyRotation() const noexcept;
    double mountOrthogonalTolerance() const noexcept;
private:
    void validate() const;

    aesa_processor::math::Angle _maxAbsPitch;
    aesa_processor::math::SquareMatrix _mountAntToBodyRotation;
    double _mountOrthogonalTolerance;
};

} // namespace aesa_processor::beam