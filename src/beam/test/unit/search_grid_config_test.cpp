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

beam::SearchGridConfig::Builder gridBuilder()
{
    return beam::SearchGridConfig::Builder()
        .azimuthCount(21)
        .elevationCount(9)
        .azimuthStart(math::CircularAngle(-42_deg))
        .azimuthStep(4.2_deg)
        .elevationStart(math::TiltAngle(3_deg))
        .elevationStep(4.4_deg)
        .beamWidth(beam::BeamWidth(math::AngularWidth(6_deg), math::AngularWidth(6_deg)));
}

} // namespace

// 필수 값이 누락된 빌더는 설정을 생성할 수 없다.
TEST(SearchGridConfigTest, BuilderRequiresEveryValue)
{
    // Given: 필수 값을 지정하지 않은 빌더를 준비한다.
    const beam::SearchGridConfig::Builder builder;

    // When: 빌더로 설정 생성을 시도한다.
    const auto build = [&builder] { return builder.build(); };

    // Then: 누락된 필수 값 때문에 예외가 발생한다.
    EXPECT_THROW(build(), std::logic_error);
}

// 격자 설정 빌더는 지정한 값을 변경하고 다른 값은 유지한다.
TEST(SearchGridConfigTest, BuilderOverridesOnlyGivenValues)
{
    // Given: 필수 값이 설정된 빌더를 준비한다.
    auto builder = gridBuilder();

    // When: 각 격자 속성을 변경해 설정을 생성한다.
    const beam::SearchGridConfig config = builder
        .azimuthCount(3)
        .elevationCount(2)
        .azimuthStart(math::CircularAngle(-10_deg))
        .azimuthStep(5_deg)
        .elevationStart(math::TiltAngle(1_deg))
        .elevationStep(2_deg)
        .beamWidth(beam::BeamWidth(4_deg, 4_deg))
        .build();

    const beam::SearchGridConfig partial = gridBuilder().azimuthCount(5).build();

    // Then: 격자 설정 빌더는 지정한 값을 변경하고 다른 값은 유지한다.
    EXPECT_EQ(config.azimuthCount(), 3u);
    EXPECT_EQ(config.elevationCount(), 2u);
    EXPECT_TRUE(config.azimuthStart().isEquivalentTo(-10_deg));
    EXPECT_EQ(config.azimuthStep(), 5_deg);
    EXPECT_EQ(config.elevationStart(), 1_deg);
    EXPECT_EQ(config.elevationStep(), 2_deg);
    EXPECT_EQ(config.beamWidth().azimuth, 4_deg);
    EXPECT_EQ(config.beamWidth().elevation, 4_deg);

    EXPECT_EQ(partial.azimuthCount(), 5u);
    EXPECT_EQ(partial.elevationCount(), 9u);
}

// 격자 생성자는 유효한 개수를 허용하고 0인 개수는 거부한다.
TEST(SearchGridConfigTest, ConstructorValidatesLikeBuilder)
{
    // Given: 검증할 입력과 설정을 준비한다.
    const math::CircularAngle azimuth(-10_deg);
    const math::TiltAngle elevation(1_deg);
    const beam::BeamWidth width(4_deg, 4_deg);

    // When: 준비한 입력으로 대상 객체의 동작을 시도한다.
    const auto construct = [&](std::size_t azimuthCount) {
        return beam::SearchGridConfig(azimuthCount, 2, azimuth, 5_deg, elevation, 2_deg, width);
    };

    // Then: 격자 생성자는 유효한 개수를 허용하고 0인 개수는 거부한다.
    EXPECT_NO_THROW(construct(3));
    EXPECT_THROW(construct(0), std::invalid_argument);
}

// 방위각 개수가 0인 격자 설정을 거부한다.
TEST(SearchGridConfigTest, ZeroAzimuthCountThrows)
{
    // Given: 검증할 입력과 설정을 준비한다.
    const auto builder = gridBuilder().azimuthCount(0);

    // When: 준비한 입력으로 대상 객체의 동작을 시도한다.
    const auto build = [&builder] { return builder.build(); };

    // Then: 방위각 개수가 0인 격자 설정을 거부한다.
    EXPECT_THROW(build(), std::invalid_argument);
}

// 고각 개수가 0인 격자 설정을 거부한다.
TEST(SearchGridConfigTest, ZeroElevationCountThrows)
{
    // Given: 검증할 입력과 설정을 준비한다.
    const auto builder = gridBuilder().elevationCount(0);

    // When: 준비한 입력으로 대상 객체의 동작을 시도한다.
    const auto build = [&builder] { return builder.build(); };

    // Then: 고각 개수가 0인 격자 설정을 거부한다.
    EXPECT_THROW(build(), std::invalid_argument);
}

// 두 축의 개수가 모두 0이면 나누기 전에 격자 설정을 거부한다.
TEST(SearchGridConfigTest, BothCountsZeroThrowsWithoutDividingByZero)
{
    // Given: 검증할 입력과 설정을 준비한다.
    const auto builder = gridBuilder().azimuthCount(0).elevationCount(0);

    // When: 준비한 입력으로 대상 객체의 동작을 시도한다.
    const auto build = [&builder] { return builder.build(); };

    // Then: 두 축의 개수가 모두 0이면 나누기 전에 격자 설정을 거부한다.
    EXPECT_THROW(build(), std::invalid_argument);
}

// 빔 ID 범위를 초과하는 격자 크기를 거부한다.
TEST(SearchGridConfigTest, BeamCountBeyondUint32Throws)
{
    // Given: 검증할 입력과 설정을 준비한다.
    const auto builder = gridBuilder().azimuthCount(70000).elevationCount(70000);

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
    const auto builder = gridBuilder().beamWidth(width);

    // When: 빔 폭이 지정된 격자 설정을 생성한다.
    const beam::SearchGridConfig config = builder.build();

    // Then: 설정이 주어진 두 축의 빔 폭을 보관한다.
    EXPECT_EQ(config.beamWidth(), width);
}

// 격자의 마지막 각도를 계산할 때 발생하는 오버플로를 거부한다.
TEST(SearchGridConfigTest, GridAngleOverflowThrows)
{
    // Given: 검증할 입력과 설정을 준비한다.
    const auto azimuthOverflow = gridBuilder().azimuthCount(3).azimuthStep(MAX_ANGLE);
    const auto elevationOverflow = gridBuilder().elevationCount(3).elevationStep(MAX_ANGLE);
    const auto endOverflow = gridBuilder()
        .elevationCount(2).elevationStart(math::TiltAngle(90_deg)).elevationStep(MAX_ANGLE);

    // When: 준비한 입력으로 대상 객체의 동작을 시도한다.
    const auto build = [](const beam::SearchGridConfig::Builder& builder) { return builder.build(); };

    // Then: 격자의 마지막 각도를 계산할 때 발생하는 오버플로를 거부한다.
    EXPECT_THROW(build(azimuthOverflow), std::invalid_argument);
    EXPECT_THROW(build(elevationOverflow), std::invalid_argument);
    EXPECT_THROW(build(endOverflow), std::invalid_argument);
}

// 큰 유한 방위각 간격을 가진 격자 설정을 허용한다.
TEST(SearchGridConfigTest, WrapsLargeFiniteAzimuthStep)
{
    // Given: 검증할 입력과 설정을 준비한다.
    const auto builder = gridBuilder().azimuthCount(2).azimuthStart(math::CircularAngle(MAX_ANGLE)).azimuthStep(MAX_ANGLE);

    // When: 준비한 입력으로 대상 객체의 동작을 시도한다.
    const auto build = [&builder] { return builder.build(); };

    // Then: 큰 유한 방위각 간격을 가진 격자 설정을 허용한다.
    EXPECT_NO_THROW(build());
}

// 단일 빔 격자는 사용하지 않는 큰 각도 간격을 허용한다.
TEST(SearchGridConfigTest, LargeStepWithSingleBeamPerAxisIsAccepted)
{
    // Given: 검증할 입력과 설정을 준비한다.
    const auto builder = gridBuilder().azimuthCount(1).elevationCount(1).azimuthStep(MAX_ANGLE).elevationStep(MAX_ANGLE);

    // When: 준비한 입력으로 대상 객체의 동작을 시도한다.
    const auto build = [&builder] { return builder.build(); };

    // Then: 단일 빔 격자는 사용하지 않는 큰 각도 간격을 허용한다.
    EXPECT_NO_THROW(build());
}

// 격자에서 계산된 끝 고각이 물리 범위를 벗어나면 거부한다.
TEST(SearchGridConfigTest, ElevationOutsideSphericalRangeThrows)
{
    // Given: 검증할 입력과 설정을 준비한다.
    const auto rising = gridBuilder()
        .elevationCount(3).elevationStart(math::TiltAngle(80_deg)).elevationStep(6_deg);
    const auto falling = gridBuilder()
        .elevationCount(3).elevationStart(math::TiltAngle(0_deg)).elevationStep(-50_deg);

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
    const auto builder = gridBuilder()
        .elevationCount(2)
        .elevationStart(math::TiltAngle(90_deg))
        .elevationStep(360_deg);

    // When: 격자 설정 생성을 시도한다.
    const auto build = [&builder] { return builder.build(); };

    // Then: 주기상 동치여도 끝 고각이 물리 범위 밖이므로 거부한다.
    EXPECT_THROW(build(), std::invalid_argument);
}

// 격자는 고각 경계에 도달하거나 음의 간격으로 내려갈 수 있다.
TEST(SearchGridConfigTest, ElevationBoundaryIsAccepted)
{
    // Given: 검증할 입력과 설정을 준비한다.
    const auto boundary = gridBuilder()
        .elevationCount(3).elevationStart(math::TiltAngle(80_deg)).elevationStep(5_deg);
    const auto descending = gridBuilder()
        .elevationCount(3).elevationStart(math::TiltAngle(0_deg)).elevationStep(-5_deg);

    // When: 준비한 입력으로 대상 객체의 동작을 시도한다.
    const auto build = [](const beam::SearchGridConfig::Builder& builder) { return builder.build(); };

    // Then: 격자는 고각 경계에 도달하거나 음의 간격으로 내려갈 수 있다.
    EXPECT_NO_THROW(build(boundary));
    EXPECT_NO_THROW(build(descending));
}

// 격자는 반 바퀴를 넘는 방위각과 정규화된 큰 시작각을 허용한다.
TEST(SearchGridConfigTest, AzimuthBeyondHalfTurnIsAccepted)
{
    // Given: 검증할 입력과 설정을 준비한다.
    const auto wide = gridBuilder().azimuthCount(100).azimuthStep(10_deg);
    const auto largeStart = gridBuilder()
        .azimuthCount(1).elevationCount(1).azimuthStart(math::CircularAngle(1e18_deg));

    // When: 준비한 입력으로 대상 객체의 동작을 시도한다.
    const auto build = [](const beam::SearchGridConfig::Builder& builder) { return builder.build(); };

    // Then: 격자는 반 바퀴를 넘는 방위각과 정규화된 큰 시작각을 허용한다.
    EXPECT_NO_THROW(build(wide));
    EXPECT_NO_THROW(build(largeStart));
}

// 격자 설정은 시작각과 간격으로 인덱스별 각도를 계산한다.
TEST(SearchGridConfigTest, AngleAtIndexIsStartPlusStepTimesIndex)
{
    // Given: 시작각과 간격이 지정된 격자 설정을 준비한다.
    const beam::SearchGridConfig config = gridBuilder().build();

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
