#pragma once

#include "domain/association/association_calculation.hpp"

#include <math/geometry/spherical_position.hpp>
#include <math/linear_algebra/vector.hpp>

#include <optional>

#include <math/linear_algebra/square_matrix.hpp>

#include <cstddef>

namespace target
{

// 공분산 S의 분해를 재사용해 잔차마다 무차원 점수 r^T S^-1 r을 계산한다.
class NdsCalculation final : public IAssociationCalculation
{
public:
    static constexpr std::size_t DIMENSION = 3;

    static std::optional<NdsCalculation> from(const math::SquareMatrix& covariance);
    // 직교 좌표 잔차의 단위는 m, 공분산 원소의 단위는 m^2다.
    static std::optional<double> calculate(const math::Vector& residual, const math::SquareMatrix& covariance);
    std::optional<double> calculate(const math::Vector& residual) const override;
    std::optional<double> calculate(
        const math::SphericalPosition& observation,
        const math::SphericalPosition& reference,
        std::optional<math::Distance> maximumTravel = std::nullopt
    ) const override;

private:
    explicit NdsCalculation(math::SquareMatrix lowerCovariance);

    math::SquareMatrix _lowerCovariance;
};

} // namespace target
