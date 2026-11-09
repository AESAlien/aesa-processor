#pragma once

#include <math/geometry/spherical_position.hpp>
#include <math/linear_algebra/vector.hpp>

#include <optional>

#include <math/linear_algebra/square_matrix.hpp>

#include <cstddef>

namespace target
{

// Owns a valid factorization of one covariance; reusable for multiple residuals.
// Computes r^T S^-1 r without applying an association threshold.
class NdsCalculation final
{
public:
    static constexpr std::size_t DIMENSION = 3;

    static std::optional<NdsCalculation> from(const math::SquareMatrix& covariance);
    static std::optional<double> calculate(const math::Vector& residual, const math::SquareMatrix& covariance);
    std::optional<double> calculate(const math::Vector& residual) const;
    std::optional<double> calculate(
        const math::SphericalPosition& observation,
        const math::SphericalPosition& reference,
        std::optional<math::Distance> maximumTravel = std::nullopt
    ) const;

private:
    explicit NdsCalculation(math::SquareMatrix lowerCovariance);

    math::SquareMatrix _lowerCovariance;
};

} // namespace target
