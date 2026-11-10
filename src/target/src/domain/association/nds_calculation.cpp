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
    std::optional<math::Distance> maximumTravel
) const {
    try
    {
        const auto displacement = observation.toCartesian() - reference.toCartesian();
        auto residual = displacement.toVector();
        if (maximumTravel)
        {
            const auto distance = displacement.distance();
            residual = distance <= *maximumTravel
                ? math::Vector(DIMENSION)
                : residual * (1.0 - *maximumTravel / distance);
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
    if (covariance.size() != DIMENSION || !covariance.isSymmetric())
    {
        return std::nullopt;
    }

    // 허용 오차 내의 비대칭을 보정해 콜레스키 분해의 엄격한 대칭 조건을 맞춘다.
    auto symmetricCovariance = covariance;
    for (std::size_t row = 0; row < DIMENSION; ++row)
    {
        for (std::size_t column = 0; column < row; ++column)
        {
            const auto first = covariance(row, column);
            const auto second = covariance(column, row);
            const auto entry = first + 0.5 * (second - first);
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

std::optional<double> NdsCalculation::calculate(const math::Vector& residual) const
{
    if (residual.size() != DIMENSION || !residual.isFinite())
    {
        return std::nullopt;
    }

    try
    {
        // S = L L^T에서 L y = r을 풀면 ||y||^2 = r^T S^-1 r이다.
        const auto transformed = _lowerCovariance.solveLowerTriangular(residual);

        const auto norm = transformed.norm();
        const auto score = norm * norm;

        if (!std::isfinite(score))
        {
            return std::nullopt;
        }

        return std::optional<double>(score);
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

    if (!calculation)
    {
        return std::nullopt;
    }

    return calculation->calculate(residual);
}

} // namespace target
