#pragma once

#include <math/quantity/angle.hpp>
#include <math/quantity/length.hpp>

namespace aesa_processor::beam
{

// 레이더 자세정보. 생성된 이후에는 항상 유효한 값을 가진다.
class RadarAttitude
{
public:
    // 범위를 벗어나면 AttitudeError(OUT_OF_RANGE)를 던진다. Angle과 Length는 항상 유한한 값만 가지므로 유한성은 검사하지 않는다.
    // 범위(경계 포함): 위도 [-90, 90]도, 경도 [-180, 180]도, 고도 [-0.5, 100]km, roll [-180, 180]도, yaw [-360, 360]도.
    // pitch는 범위를 검사하지 않는다. 짐벌락 판정은 AttitudeConfig에 의존하므로 AntEnuTransform에서 한다.
    RadarAttitude(
        aesa_processor::math::Angle latitude,
        aesa_processor::math::Angle longitude,
        aesa_processor::math::Length altitude,
        aesa_processor::math::Angle roll,
        aesa_processor::math::Angle pitch,
        aesa_processor::math::Angle yaw);

    aesa_processor::math::Angle latitude() const noexcept;
    aesa_processor::math::Angle longitude() const noexcept;
    aesa_processor::math::Length altitude() const noexcept;
    aesa_processor::math::Angle roll() const noexcept;
    aesa_processor::math::Angle pitch() const noexcept;
    aesa_processor::math::Angle yaw() const noexcept;

private:
    void validate() const;

    aesa_processor::math::Angle _latitude;
    aesa_processor::math::Angle _longitude;
    aesa_processor::math::Length _altitude;
    aesa_processor::math::Angle _roll;
    aesa_processor::math::Angle _pitch;
    aesa_processor::math::Angle _yaw;
};

} // namespace aesa_processor::beam
