#pragma once

#include "math/quantity/angle.hpp"
#include "math/bounded/tilt_angle.hpp"

namespace math
{

// 정규화하지 않는 닫힌 각도 구간 [start, end]. 원형 구간은 표현하지 않는다.
class AngleRange
{
public:
    // start >= end이면 std::invalid_argument를 던진다. 검증에는 허용오차를 적용하지 않는다.
    AngleRange(Angle start, Angle end);
    AngleRange(TiltAngle start, TiltAngle end);

    const Angle& start() const noexcept;
    const Angle& end() const noexcept;

    // 양 끝점을 포함하며, 지정한 허용오차 안의 값도 포함한다.
    bool contains(
        Angle value,
        const QuantityTolerance<Angle>& tolerance = Angle::DEFAULT_TOLERANCE
    ) const noexcept;
    bool contains(
        TiltAngle value,
        const QuantityTolerance<Angle>& tolerance = Angle::DEFAULT_TOLERANCE
    ) const noexcept;

private:
    Angle _start;
    Angle _end;
};

}
