#pragma once

#include <math/bounded/distance.hpp>
#include <math/geometry/spherical_position.hpp>
#include <math/linear_algebra/vector.hpp>

#include <optional>

namespace target
{

// 준비한 기준을 독립적으로 보관하며 반복 계산에서 상태를 바꾸지 않는다.
class IAssociationCalculation
{
public:
    virtual ~IAssociationCalculation() = default;

    // 점수가 낮을수록 연관성이 높으며, 계산할 수 없으면 nullopt를 반환한다.
    virtual std::optional<double> calculate(const math::Vector& residual) const = 0;
    virtual std::optional<double> calculate(
        const math::SphericalPosition& observation,
        const math::SphericalPosition& reference,
        std::optional<math::Distance> maximumTravel = std::nullopt
    ) const = 0;
};

} // namespace target
