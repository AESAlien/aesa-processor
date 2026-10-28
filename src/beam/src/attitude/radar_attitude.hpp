#pragma once

#include <math/quantity/angle.hpp>

namespace beam
{

// 레이더 자세정보. 생성된 이후에는 항상 유효한 값을 가진다.
// 탐색 빔 계산에 쓰는 roll/pitch/yaw만 갖는다. 위도·경도·고도는 빔 모듈에서 쓰지 않으므로 받지 않는다.
class RadarAttitude
{
public:
    // 범위를 벗어나면 std::invalid_argument를 던진다. Angle은 항상 유한한 값만 가지므로 유한성은 검사하지 않는다.
    // 범위(경계 포함): roll [-180, 180]도, yaw [-360, 360]도.
    // pitch는 범위를 검사하지 않는다. 짐벌락 판정은 AttitudeConfig에 의존하므로 AntEnuTransform에서 한다.
    RadarAttitude(math::Angle roll, math::Angle pitch, math::Angle yaw);

    math::Angle roll() const noexcept;
    math::Angle pitch() const noexcept;
    math::Angle yaw() const noexcept;

private:
    void validate() const;

    math::Angle _roll;
    math::Angle _pitch;
    math::Angle _yaw;
};

} // namespace beam
