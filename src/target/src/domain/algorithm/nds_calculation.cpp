#include "nds_calculation.hpp"

#include <math/geometry/cartesian_displacement.hpp>
#include <math/geometry/cartesian_position.hpp>
#include <math/tolerance.hpp>

#include <cmath>
#include <stdexcept>
#include <utility>

namespace target
{

std::optional<double> NdsCalculation::calculate(
    const math::SphericalPosition& observation,
    const math::SphericalPosition& reference,
    std::optional<math::Length> maximumTravel
) const {
    if (maximumTravel && (!std::isfinite(maximumTravel->m()) || maximumTravel->m() < 0.0))
    {
        return std::nullopt;
    }
    try
    {
        const auto displacement = observation.toCartesian() - reference.toCartesian();
        auto residual = displacement.toVector();
        if (maximumTravel)
        {
            const auto distance = displacement.distance();
            residual = distance <= *maximumTravel
                ? math::Vector(DIMENSION)
                : residual * ((distance - *maximumTravel) / distance);
        }
        return calculate(residual);
    }
    catch (const std::invalid_argument&)
    {
        return std::nullopt;
    }
}


NdsCalculation::NdsCalculation(math::SquareMatrix lowerCovariance) : _lowerCovariance(std::move(lowerCovariance))
{
}

std::optional<NdsCalculation> NdsCalculation::from(
    const math::SquareMatrix& covariance
) {
    if (covariance.size() != DIMENSION ||
        !covariance.isSymmetric(math::Tolerance(0.0, 1e-9)))
    {
        return std::nullopt;
    }

    // Preserve the accepted roundoff tolerance before math's exact symmetry check.
    auto symmetricCovariance = covariance;
    for (std::size_t row = 0; row < DIMENSION; ++row)
    {
        for (std::size_t column = 0; column < row; ++column)
        {
            const double first = covariance(row, column);
            const double second = covariance(column, row);
            const double entry = first + 0.5 * (second - first);
            if (!std::isfinite(entry))
            {
                return std::nullopt;
            }
            symmetricCovariance(row, column) = entry;
            symmetricCovariance(column, row) = entry;
        }
    }

    try
    {
        return NdsCalculation(math::cholesky(symmetricCovariance));
    }
    catch (const std::domain_error&)
    {
        return std::nullopt;
    }
}

// Cartesian residual components are metres; covariance entries are square metres.
// Returns the dimensionless squared distance r^T S^-1 r, not a gated decision.
std::optional<double> NdsCalculation::calculate(const math::Vector& residual) const
{
    if (residual.size() != DIMENSION || !residual.isFinite())
    {
        return std::nullopt;
    }

    try
    {
        const auto transformed = _lowerCovariance.solveLowerTriangular(residual);

        // Solve L y = r: ||y||^2 = r^T S^-1 r. norm() preserves tiny components.
        const double norm = transformed.norm();
        const double score = norm * norm;
        return std::isfinite(score) ? std::optional<double>(score) : std::nullopt;
    }
    catch (const std::domain_error&)
    {
        return std::nullopt;
    }
}

std::optional<double> NdsCalculation::calculate(
    const math::Vector& residual,
    const math::SquareMatrix& covariance
) {
    if (residual.size() != DIMENSION || !residual.isFinite())
    {
        return std::nullopt;
    }
    const auto calculation = from(covariance);
    return calculation ? calculation->calculate(residual) : std::nullopt;
}

} // namespace target
