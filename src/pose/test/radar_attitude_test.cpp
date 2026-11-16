#include <pose/domain/factory/radar_attitude_factory.hpp>

#include <gtest/gtest.h>

#include <stdexcept>
#include <string>

using namespace math::literals;

namespace
{
const pose::RadarAttitudeFactory FACTORY(pose::RadarAttitudeFactoryConfig(89_deg));

// 함수가 던진 std::invalid_argument의 메시지를 반환한다. 예외가 없으면 실패로 기록하고 빈 문자열을 반환한다.
template <typename Function>
std::string invalidArgumentMessage(Function&& function)
{
    try
    {
        function();
    }
    catch (const std::invalid_argument& error)
    {
        return error.what();
    }

    ADD_FAILURE() << "std::invalid_argument was not thrown";
    return {};
}

} // namespace

// 자세 객체는 전달받은 롤과 피치 및 요를 보관한다.
TEST(RadarAttitudeFactoryTest, StoresGivenValues)
{
    // Given: 각 축을 구분할 수 있는 자세 각도를 준비한다.
    const auto roll = 4_deg;
    const auto pitch = math::TiltAngle(5_deg);
    const auto yaw = 6_deg;

    // When: 세 각도로 자세 객체를 생성한다.
    const auto attitude = FACTORY.create(roll, pitch, yaw);

    // Then: 각 축의 값이 전달한 자세와 일치한다.
    EXPECT_TRUE(attitude.roll().isEquivalentTo(roll));
    EXPECT_EQ(attitude.pitch(), pitch);
    EXPECT_TRUE(attitude.yaw().isEquivalentTo(yaw));
}

// 자세 객체는 롤과 요의 명시된 범위를 벗어난 값을 거부한다.
TEST(RadarAttitudeFactoryTest, ThrowsOnOutOfRangeValues)
{
    // Given: 생성자에 전달할 입력을 준비한다.
    const auto pitch = math::TiltAngle(0_deg);
    const math::Angle invalidRolls[] = {181_deg, -181_deg};
    const math::Angle invalidYaws[] = {361_deg, -361_deg};

    // When: 준비한 값으로 대상 객체 생성을 시도한다.
    const auto construct = [&](math::Angle roll, math::Angle yaw) {
        return FACTORY.create(roll, pitch, yaw);
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
TEST(RadarAttitudeFactoryTest, ThrowsOnValuesEquivalentByPeriodToValidRange)
{
    // Given: 생성자에 전달할 입력을 준비한다.
    const auto pitch = math::TiltAngle(0_deg);
    const auto invalidRoll = 540_deg;
    const math::Angle invalidYaws[] = {720_deg, -720_deg};

    // When: 준비한 값으로 대상 객체 생성을 시도한다.
    const auto construct = [&](math::Angle roll, math::Angle yaw) {
        return FACTORY.create(roll, pitch, yaw);
    };

    // Then: 자세 객체는 주기상 유효 범위와 같아도 실제 값이 범위 밖이면 거부한다.
    EXPECT_THROW(construct(invalidRoll, 0_deg), std::invalid_argument);
    for (const auto yaw : invalidYaws)
    {
        EXPECT_THROW(construct(0_deg, yaw), std::invalid_argument);
    }
}

// 자세 객체는 롤과 요의 양 끝 경계를 허용한다.
TEST(RadarAttitudeFactoryTest, AcceptsBoundaryAnglesIncludingEndpoints)
{
    // Given: 생성자에 전달할 입력을 준비한다.
    const auto pitch = math::TiltAngle(0_deg);
    const auto upperRoll = 180_deg;
    const auto lowerRoll = -180_deg;
    const auto upperYaw = 360_deg;
    const auto lowerYaw = -360_deg;

    // When: 준비한 값으로 대상 객체 생성을 시도한다.
    const auto construct = [&](math::Angle roll, math::Angle yaw) {
        return FACTORY.create(roll, pitch, yaw);
    };

    // Then: 자세 객체는 롤과 요의 양 끝 경계를 허용한다.
    EXPECT_NO_THROW(construct(upperRoll, upperYaw));
    EXPECT_NO_THROW(construct(lowerRoll, lowerYaw));
}

// 자세 객체의 범위 오류 메시지는 입력된 롤과 요 값을 포함한다.
TEST(RadarAttitudeFactoryTest, OutOfRangeMessageIncludesInputValue)
{
    // Given: 롤과 요가 각각 범위를 벗어난 입력을 준비한다.
    const auto pitch = math::TiltAngle(0_deg);

    // When: 각 입력으로 자세 객체 생성을 시도한다.
    const auto rollMessage = invalidArgumentMessage([&] { FACTORY.create(200_deg, pitch, 0_deg); });
    const auto yawMessage = invalidArgumentMessage([&] { FACTORY.create(0_deg, pitch, -400_deg); });

    // Then: 메시지에 원인과 입력값이 함께 담긴다.
    EXPECT_NE(rollMessage.find("roll must be in"), std::string::npos) << rollMessage;
    EXPECT_NE(rollMessage.find("roll=200"), std::string::npos) << rollMessage;
    EXPECT_NE(yawMessage.find("yaw must be in"), std::string::npos) << yawMessage;
    EXPECT_NE(yawMessage.find("yaw=-400"), std::string::npos) << yawMessage;
}

TEST(RadarAttitudeFactoryTest, RejectsPitchAtOrNearEitherConfiguredLimit)
{
    const math::Angle rejected[] = {
        89_deg, -89_deg, 88.9999999999_deg, -88.9999999999_deg, -90_deg
    };
    for (const auto pitch : rejected)
    {
        EXPECT_THROW(FACTORY.create(0_deg, math::TiltAngle(pitch), 0_deg), std::invalid_argument);
    }
    EXPECT_NO_THROW(FACTORY.create(0_deg, math::TiltAngle(88.9_deg), 0_deg));
}

TEST(RadarAttitudeFactoryTest, PitchErrorIncludesInputAndLimit)
{
    const auto factory = pose::RadarAttitudeFactory(pose::RadarAttitudeFactoryConfig(60_deg));
    const auto message = invalidArgumentMessage([&] { factory.create(0_deg, math::TiltAngle(70_deg), 0_deg); });

    EXPECT_NE(message.find("gimbal lock"), std::string::npos) << message;
    EXPECT_NE(message.find("pitch=70"), std::string::npos) << message;
    EXPECT_NE(message.find("maxAbsPitch=60"), std::string::npos) << message;
}
