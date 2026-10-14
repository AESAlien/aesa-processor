#pragma once

#include <math/quantity/angle.hpp>
#include <math/quantity/distance.hpp>

namespace beam
{

// 레이더 자세정보. 생성된 이후에는 항상 유효한 값을 가진다.
class RadarAttitude
{
public:
    // 값이 NaN 또는 Inf이면 AttitudeError(NOT_FINITE)를, 범위를 벗어나면 AttitudeError(OUT_OF_RANGE)를 던진다.
    // 범위(경계 포함): 위도 [-90, 90]도, 경도 [-180, 180]도, 고도 [-0.5, 100]km, roll [-180, 180]도, yaw [-360, 360]도.
    // pitch는 유한성만 검사한다. 짐벌락 판정은 AttitudeConfig에 의존하므로 AntEnuTransform에서 한다.
    RadarAttitude(
        math::Angle latitude,
        math::Angle longitude,
        math::Distance altitude,
        math::Angle roll,
        math::Angle pitch,
        math::Angle yaw);

    math::Angle latitude() const noexcept;
    math::Angle longitude() const noexcept;
    math::Distance altitude() const noexcept;
    math::Angle roll() const noexcept;
    math::Angle pitch() const noexcept;
    math::Angle yaw() const noexcept;

private:
    math::Angle _latitude;
    math::Angle _longitude;
    math::Distance _altitude;
    math::Angle _roll;
    math::Angle _pitch;
    math::Angle _yaw;
};

} // namespace beam
