#include <beam/config/search_grid_config.hpp>

#include <gtest/gtest.h>

#include <limits>
#include <stdexcept>
#include <type_traits>

using namespace math::literals;

namespace
{

static_assert(std::is_copy_assignable_v<beam::SearchGridConfig>);

const double MAX_VALUE = std::numeric_limits<double>::max();

// NaN·Inf Angle은 math::Angle 생성 단계에서 std::invalid_argument로 거부되므로 설정 검증 대상이 아니다.
const math::Angle MAX_ANGLE = math::Angle::fromDegrees(MAX_VALUE);

beam::SearchGridConfig::Builder validConfigBuilder()
{
    return beam::SearchGridConfig::Builder()
        .azimuthCount(21)
        .elevationCount(9)
        .azimuthStart(-42_deg)
        .azimuthStep(4.2_deg)
        .elevationStart(3_deg)
        .elevationStep(4.4_deg)
        .beamWidth(beam::BeamWidth(6_deg, 6_deg));
}

} // namespace

// 격자 설정은 어느 축이든 개수가 0이면 나누기 전에 거부한다.
TEST(SearchGridConfigTest, ZeroCountThrowsWithoutDividingByZero)
{
    // Given: 한 축 또는 두 축의 개수가 0인 설정을 준비한다.
    const auto zeroAzimuth = validConfigBuilder().azimuthCount(0);
    const auto zeroElevation = validConfigBuilder().elevationCount(0);
    const auto bothZero = validConfigBuilder().azimuthCount(0).elevationCount(0);
    const auto build = [](const beam::SearchGridConfig::Builder& builder) { return builder.build(); };

    // When & Then: 0인 축의 조합과 무관하게 나눗셈 전에 거부한다.
    EXPECT_THROW(build(zeroAzimuth), std::invalid_argument);
    EXPECT_THROW(build(zeroElevation), std::invalid_argument);
    EXPECT_THROW(build(bothZero), std::invalid_argument);
}

// 빔 ID 범위를 초과하는 격자 크기를 거부한다.
TEST(SearchGridConfigTest, BeamCountBeyondUint32Throws)
{
    // Given: 검증할 입력과 설정을 준비한다.
    const auto builder = validConfigBuilder().azimuthCount(70000).elevationCount(70000);

    // When: 준비한 입력으로 대상 객체의 동작을 시도한다.
    const auto build = [&builder] { return builder.build(); };

    // Then: 빔 ID 범위를 초과하는 격자 크기를 거부한다.
    EXPECT_THROW(build(), std::invalid_argument);
}

// 격자 설정은 전달받은 빔 폭을 보관한다.
TEST(SearchGridConfigTest, KeepsGivenBeamWidth)
{
    // Given: 방위각과 고각의 폭이 서로 다른 빔 폭을 준비한다.
    const beam::BeamWidth width(4_deg, 5_deg);
    const auto builder = validConfigBuilder().beamWidth(width);

    // When: 빔 폭이 지정된 격자 설정을 생성한다.
    const beam::SearchGridConfig config = builder.build();

    // Then: 설정이 주어진 두 축의 빔 폭을 보관한다.
    EXPECT_EQ(config.beamWidth(), width);
}

// 격자의 마지막 각도를 계산할 때 발생하는 오버플로를 거부한다.
TEST(SearchGridConfigTest, GridAngleOverflowThrows)
{
    // Given: 검증할 입력과 설정을 준비한다.
    const auto azimuthOverflow = validConfigBuilder().azimuthCount(3).azimuthStep(MAX_ANGLE);
    const auto elevationOverflow = validConfigBuilder().elevationCount(3).elevationStep(MAX_ANGLE);
    const auto endOverflow = validConfigBuilder()
        .elevationCount(2).elevationStart(90_deg).elevationStep(MAX_ANGLE);

    // When: 준비한 입력으로 대상 객체의 동작을 시도한다.
    const auto build = [](const beam::SearchGridConfig::Builder& builder) { return builder.build(); };

    // Then: 격자의 마지막 각도를 계산할 때 발생하는 오버플로를 거부한다.
    EXPECT_THROW(build(azimuthOverflow), std::invalid_argument);
    EXPECT_THROW(build(elevationOverflow), std::invalid_argument);
    EXPECT_THROW(build(endOverflow), std::invalid_argument);
}

// 단일 빔 격자는 사용하지 않는 큰 각도 간격을 허용한다.
TEST(SearchGridConfigTest, LargeStepWithSingleBeamPerAxisIsAccepted)
{
    // Given: 검증할 입력과 설정을 준비한다.
    const auto builder = validConfigBuilder().azimuthCount(1).elevationCount(1).azimuthStep(MAX_ANGLE).elevationStep(MAX_ANGLE);

    // When: 준비한 입력으로 대상 객체의 동작을 시도한다.
    const auto build = [&builder] { return builder.build(); };

    // Then: 단일 빔 격자는 사용하지 않는 큰 각도 간격을 허용한다.
    EXPECT_NO_THROW(build());
}

// 격자에서 계산된 끝 고각이 물리 범위를 벗어나면 거부한다.
TEST(SearchGridConfigTest, ElevationOutsideSphericalRangeThrows)
{
    // Given: 검증할 입력과 설정을 준비한다.
    const auto rising = validConfigBuilder()
        .elevationCount(3).elevationStart(80_deg).elevationStep(6_deg);
    const auto falling = validConfigBuilder()
        .elevationCount(3).elevationStart(0_deg).elevationStep(-50_deg);

    // When: 준비한 입력으로 대상 객체의 동작을 시도한다.
    const auto build = [](const beam::SearchGridConfig::Builder& builder) { return builder.build(); };

    // Then: 격자에서 계산된 끝 고각이 물리 범위를 벗어나면 거부한다.
    EXPECT_THROW(build(rising), std::invalid_argument);
    EXPECT_THROW(build(falling), std::invalid_argument);
}

// 끝 고각이 한 바퀴를 돌아 물리 범위를 벗어나면 격자를 거부한다.
TEST(SearchGridConfigTest, ElevationEquivalentByPeriodToValidRangeThrows)
{
    // Given: 시작 고각은 유효하지만 끝 고각은 450도가 되는 격자를 준비한다.
    const auto builder = validConfigBuilder()
        .elevationCount(2)
        .elevationStart(90_deg)
        .elevationStep(360_deg);

    // When: 격자 설정 생성을 시도한다.
    const auto build = [&builder] { return builder.build(); };

    // Then: 주기상 동치여도 끝 고각이 물리 범위 밖이므로 거부한다.
    EXPECT_THROW(build(), std::invalid_argument);
}

// 격자 설정은 시작각과 간격으로 인덱스별 각도를 계산한다.
TEST(SearchGridConfigTest, AngleAtIndexIsStartPlusStepTimesIndex)
{
    // Given: 시작각과 간격이 지정된 격자 설정을 준비한다.
    const beam::SearchGridConfig config = validConfigBuilder().build();

    // When: 각 축의 처음과 마지막 인덱스에 해당하는 각도를 구한다.
    const auto firstAzimuth = config.azimuthAt(0);
    const auto lastAzimuth = config.azimuthAt(20);
    const auto firstElevation = config.elevationAt(0);
    const auto lastElevation = config.elevationAt(8);

    // Then: 각도는 시작각에 간격과 인덱스의 곱을 더한 값이다.
    EXPECT_EQ(firstAzimuth, math::CircularAngle(-42_deg));
    EXPECT_EQ(lastAzimuth, math::CircularAngle(-42_deg) + 4.2_deg * 20.0);
    EXPECT_EQ(firstElevation, math::TiltAngle(3_deg));
    EXPECT_EQ(lastElevation, math::TiltAngle(3_deg) + 4.4_deg * 8.0);
}
