#pragma once

#include <math/quantity/angle.hpp>
#include <math/bounded/tilt_angle.hpp>

#include <optional>

namespace beam
{

// 확인·추적 빔을 조향할 수 있는 영역(로직맵의 추적 영역). 안테나 정면 기준 각도이다.
// - 방위각은 안테나 정면을 0도로 하는 [-180, 180] 범위의 값으로 나타낸다. math::CircularAngle은 [0, 360)으로
//   정규화되어 음수 경계를 표현할 수 없으므로 math::Angle로 둔다.
// - 경계는 영역에 포함된다.
class RequestCoverageConfig
{
public:
    class Builder
    {
    public:
        Builder& azimuthMin(math::Angle value)          { _azimuthMin = value; return *this; }
        Builder& azimuthMax(math::Angle value)          { _azimuthMax = value; return *this; }
        Builder& elevationMin(math::TiltAngle value)    { _elevationMin.emplace(value); return *this; }
        Builder& elevationMax(math::TiltAngle value)    { _elevationMax.emplace(value); return *this; }

        RequestCoverageConfig build() const;

    private:
        std::optional<math::Angle> _azimuthMin;
        std::optional<math::Angle> _azimuthMax;
        std::optional<math::TiltAngle> _elevationMin;
        std::optional<math::TiltAngle> _elevationMax;
    }; // class Builder

    // 다음 중 하나라도 해당하면 std::invalid_argument를 던진다.
    // - azimuthMin < -180도 또는 azimuthMax > 180도
    // - azimuthMin >= azimuthMax
    // - elevationMin >= elevationMax
    // 고각이 [-90, 90]도를 벗어나는 경우는 math::TiltAngle 생성 단계에서 거부된다.
    RequestCoverageConfig(math::Angle azimuthMin,
                          math::Angle azimuthMax,
                          math::TiltAngle elevationMin,
                          math::TiltAngle elevationMax);

    const math::Angle& azimuthMin() const noexcept;
    const math::Angle& azimuthMax() const noexcept;
    const math::TiltAngle& elevationMin() const noexcept;
    const math::TiltAngle& elevationMax() const noexcept;

private:
    math::Angle _azimuthMin;
    math::Angle _azimuthMax;
    math::TiltAngle _elevationMin;
    math::TiltAngle _elevationMax;
    void validate() const;
};

} // namespace beam
