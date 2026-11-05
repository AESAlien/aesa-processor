#include "attitude/radar_attitude.hpp"

#include <gtest/gtest.h>

#include <stdexcept>

using namespace math::literals;

// 자세 객체는 전달받은 롤과 피치 및 요를 보관한다.
TEST(RadarAttitudeTest, StoresGivenValues)
{
    // Given: 각 축을 구분할 수 있는 자세 각도를 준비한다.
    const auto roll = 4_deg;
    const auto pitch = math::TiltAngle(5_deg);
    const auto yaw = 6_deg;

    // When: 세 각도로 자세 객체를 생성한다.
    const auto attitude = beam::RadarAttitude(roll, pitch, yaw);

    // Then: 각 축의 값이 전달한 자세와 일치한다.
    EXPECT_TRUE(attitude.roll().isEquivalentTo(roll));
    EXPECT_EQ(attitude.pitch(), pitch);
    EXPECT_TRUE(attitude.yaw().isEquivalentTo(yaw));
}

// 자세 객체는 롤과 요의 명시된 범위를 벗어난 값을 거부한다.
TEST(RadarAttitudeTest, ThrowsOnOutOfRangeValues)
{
    // Given: 생성자에 전달할 입력을 준비한다.
    const auto pitch = math::TiltAngle(0_deg);
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

// 자세 객체는 주기상 유효 범위와 같아도 실제 값이 범위 밖이면 거부한다.
TEST(RadarAttitudeTest, ThrowsOnValuesEquivalentByPeriodToValidRange)
{
    // Given: 생성자에 전달할 입력을 준비한다.
    const auto pitch = math::TiltAngle(0_deg);
    const auto invalidRoll = 540_deg;
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
    const auto pitch = math::TiltAngle(0_deg);
    const auto upperRoll = 180_deg;
    const auto lowerRoll = -180_deg;
    const auto upperYaw = 360_deg;
    const auto lowerYaw = -360_deg;

    // When: 준비한 값으로 대상 객체 생성을 시도한다.
    const auto construct = [&](math::Angle roll, math::Angle yaw) {
        return beam::RadarAttitude(roll, pitch, yaw);
    };

    // Then: 자세 객체는 롤과 요의 양 끝 경계를 허용한다.
    EXPECT_NO_THROW(construct(upperRoll, upperYaw));
    EXPECT_NO_THROW(construct(lowerRoll, lowerYaw));
}
