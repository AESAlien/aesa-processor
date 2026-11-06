#pragma once

#include "math/bounded/circular_angle.hpp"

namespace math
{

// start에서 end까지 각도가 증가하는 방향의 닫힌 원형 구간.
class CircularAngleRange
{
public:
    // 같은 방향의 경계는 거부한다. 전체 원은 fullCircle()로 생성한다.
    CircularAngleRange(CircularAngle start, CircularAngle end);
    // 편의 오버로드. 원래 경계 차이는 [-360, 360]도여야 하며 ±360도는 전체 원이다.
    CircularAngleRange(Angle start, Angle end);

    static CircularAngleRange fullCircle();

    const CircularAngle& start() const noexcept;
    const CircularAngle& end() const noexcept;
    bool isFullCircle() const noexcept;

    bool contains(
        CircularAngle value,
        const QuantityTolerance<Angle>& tolerance = Angle::DEFAULT_TOLERANCE
    ) const noexcept;
    bool contains(
        Angle value,
        const QuantityTolerance<Angle>& tolerance = Angle::DEFAULT_TOLERANCE
    ) const noexcept;

private:
    CircularAngleRange(CircularAngle start, CircularAngle end, bool fullCircle);

    CircularAngle _start;
    CircularAngle _end;
    Angle _sweep;
};

}
