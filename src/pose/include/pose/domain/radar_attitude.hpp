#pragma once

#include <math/bounded/circular_angle.hpp>
#include <math/bounded/tilt_angle.hpp>

namespace pose
{

// 레이더 자세정보. 생성된 이후에는 항상 유효한 값을 가진다.
// 안테나 좌표계와 ENU 좌표계 간의 변환에 쓰는 roll/pitch/yaw를 갖는다.
class RadarAttitude
{
public:
    math::CircularAngle roll() const noexcept;
    math::TiltAngle pitch() const noexcept;
    math::CircularAngle yaw() const noexcept;

private:
    friend class RadarAttitudeFactory;

    RadarAttitude(math::CircularAngle roll, math::TiltAngle pitch, math::CircularAngle yaw);

    math::CircularAngle _roll;
    math::TiltAngle _pitch;
    math::CircularAngle _yaw;
};

} // namespace pose
