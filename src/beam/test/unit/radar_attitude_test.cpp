#include "attitude/radar_attitude.hpp"

#include <gtest/gtest.h>

#include <stdexcept>

using namespace math::literals;

namespace
{

beam::RadarAttitude makeAttitude(math::Angle roll, math::Angle pitch, math::Angle yaw)
{
    return beam::RadarAttitude(roll, math::TiltAngle(pitch), yaw);
}

} // namespace

// ---------------------------------------------------------------- RadarAttitude 검증

// 자세 객체는 전달받은 롤과 피치 및 요를 보관한다.
TEST(RadarAttitudeTest, StoresGivenValues)
{
    // Given: 각 축을 구분할 수 있는 자세 각도를 준비한다.
    const math::Angle roll = 4_deg;
    const math::TiltAngle pitch(5_deg);
    const math::Angle yaw = 6_deg;

    // When: 세 각도로 자세 객체를 생성한다.
    const beam::RadarAttitude attitude(roll, pitch, yaw);

    // Then: 각 축의 값이 전달한 자세와 일치한다.
    EXPECT_TRUE(attitude.roll().isEquivalentTo(roll));
    EXPECT_EQ(attitude.pitch(), pitch);
    EXPECT_TRUE(attitude.yaw().isEquivalentTo(yaw));
}

// 자세 객체는 롤과 요의 명시된 범위를 벗어난 값을 거부한다.
TEST(RadarAttitudeTest, ThrowsOnOutOfRangeValues)
{
    // Given: 생성자에 전달할 입력을 준비한다.
    const math::TiltAngle pitch(0_deg);
    const math::Angle invalidRolls[] = {181_deg, -181_deg};
    const math::Angle invalidYaws[] = {361_deg, -361_deg};

    // When: 준비한 값으로 대상 객체 생성을 시도한다.
    const auto construct = [&](math::Angle roll, math::Angle yaw) {
        return beam::RadarAttitude(roll, pitch, yaw);
    };

    // Then: 자세 객체는 롤과 요의 명시된 범위를 벗어난 값을 거부한다.
    for (const auto roll : invalidRolls)
    {
        EXPECT_THROW(construct(roll, 0_deg), std::invalid_argument);
    }
    for (const auto yaw : invalidYaws)
    {
        EXPECT_THROW(construct(0_deg, yaw), std::invalid_argument);
    }
}

// 자세 객체는 검증된 롤과 요를 정규화해 보관한다.
TEST(RadarAttitudeTest, NormalizesValidatedRollAndYaw)
{
    // Given: 유효 범위의 음수 경계 각도를 준비한다.
    const math::Angle roll = -180_deg;
    const math::TiltAngle pitch(-90_deg);
    const math::Angle yaw = -360_deg;

    // When: 경계 각도로 자세 객체를 생성한다.
    const beam::RadarAttitude attitude(roll, pitch, yaw);

    // Then: 롤과 요는 정규화되고 피치는 그대로 유지된다.
    EXPECT_DOUBLE_EQ(attitude.roll().deg(), 180.0);
    EXPECT_DOUBLE_EQ(attitude.pitch().deg(), -90.0);
    EXPECT_DOUBLE_EQ(attitude.yaw().deg(), 0.0);
}

// 자세 객체는 주기상 유효 범위와 같아도 실제 값이 범위 밖이면 거부한다.
TEST(RadarAttitudeTest, ThrowsOnValuesEquivalentByPeriodToValidRange)
{
    // Given: 생성자에 전달할 입력을 준비한다.
    const math::TiltAngle pitch(0_deg);
    const math::Angle invalidRoll = 540_deg;
    const math::Angle invalidYaws[] = {720_deg, -720_deg};

    // When: 준비한 값으로 대상 객체 생성을 시도한다.
    const auto construct = [&](math::Angle roll, math::Angle yaw) {
        return beam::RadarAttitude(roll, pitch, yaw);
    };

    // Then: 자세 객체는 주기상 유효 범위와 같아도 실제 값이 범위 밖이면 거부한다.
    EXPECT_THROW(construct(invalidRoll, 0_deg), std::invalid_argument);
    for (const auto yaw : invalidYaws)
    {
        EXPECT_THROW(construct(0_deg, yaw), std::invalid_argument);
    }
}

// 자세 객체는 롤과 요의 양 끝 경계를 허용한다.
TEST(RadarAttitudeTest, AcceptsBoundaryAnglesIncludingEndpoints)
{
    // Given: 생성자에 전달할 입력을 준비한다.
    const math::TiltAngle pitch(0_deg);
    const math::Angle upperRoll = 180_deg;
    const math::Angle lowerRoll = -180_deg;
    const math::Angle upperYaw = 360_deg;
    const math::Angle lowerYaw = -360_deg;

    // When: 준비한 값으로 대상 객체 생성을 시도한다.
    const auto construct = [&](math::Angle roll, math::Angle yaw) {
        return beam::RadarAttitude(roll, pitch, yaw);
    };

    // Then: 자세 객체는 롤과 요의 양 끝 경계를 허용한다.
    EXPECT_NO_THROW(construct(upperRoll, upperYaw));
    EXPECT_NO_THROW(construct(lowerRoll, lowerYaw));
}

// 자세 객체는 짐벌락 경계의 유효한 고각을 보관할 수 있다.
TEST(RadarAttitudeTest, KeepsBoundaryPitchWithoutGimbalLockValidation)
{
    // Given: 물리 범위의 양 끝에 해당하는 고각을 준비한다.
    const math::TiltAngle upperPitch(90_deg);
    const math::TiltAngle lowerPitch(-90_deg);

    // When: 경계 고각으로 자세 객체를 생성한다.
    const beam::RadarAttitude upper(0_deg, upperPitch, 0_deg);
    const beam::RadarAttitude lower(0_deg, lowerPitch, 0_deg);

    // Then: 짐벌락 판정 없이 전달받은 고각을 보관한다.
    EXPECT_EQ(upper.pitch(), upperPitch);
    EXPECT_EQ(lower.pitch(), lowerPitch);
}
