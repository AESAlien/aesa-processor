#pragma once

#include <math/quantity/angle.hpp>
#include <math/bounded/tilt_angle.hpp>
#include <math/range/angle_range.hpp>
#include <math/range/circular_angle_range.hpp>

#include <optional>

namespace beam
{

// 확인·추적 빔을 조향할 수 있는 영역(로직맵의 추적 영역). 안테나 정면 기준 각도이다.
// - 방위각은 각도가 증가하는 방향의 원형 구간으로 나타낸다.
// - 경계는 영역에 포함된다.
class RequestCoverageConfig
{
public:
    class Builder
    {
    public:
        Builder& azimuthRange(math::CircularAngleRange value) { _azimuthRange = value; return *this; }
        Builder& elevationRange(math::AngleRange value) { _elevationRange = value; return *this; }

        RequestCoverageConfig build() const;

    private:
        std::optional<math::CircularAngleRange> _azimuthRange;
        std::optional<math::AngleRange> _elevationRange;
    }; // class Builder

    // 다음 중 하나라도 해당하면 std::invalid_argument를 던진다.
    // - elevationRange.start() < -90도 또는 elevationRange.end() > 90도
    RequestCoverageConfig(
        math::CircularAngleRange azimuthRange,
        math::AngleRange elevationRange
    );

    const math::CircularAngleRange& azimuthRange() const noexcept;
    const math::AngleRange& elevationRange() const noexcept;

private:
    math::CircularAngleRange _azimuthRange;
    math::AngleRange _elevationRange;
    void validate() const;
};

} // namespace beam
