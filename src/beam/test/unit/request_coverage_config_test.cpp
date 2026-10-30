#include <beam/config/request_coverage_config.hpp>

#include <gtest/gtest.h>

#include <stdexcept>
#include <type_traits>

using namespace math::literals;

namespace
{

static_assert(std::is_copy_assignable_v<beam::RequestCoverageConfig>);

beam::RequestCoverageConfig::Builder coverageBuilder()
{
    return beam::RequestCoverageConfig::Builder()
        .azimuthRange(math::CircularAngleRange(-60_deg, 60_deg))
        .elevationRange(math::AngleRange(-10_deg, 60_deg));
}

} // namespace

// 영역 설정 빌더는 방위각과 고각 범위가 모두 필요하다.
TEST(RequestCoverageConfigTest, BuilderRequiresEveryValue)
{
    // Given: 검증할 입력과 설정을 준비한다.
    const beam::RequestCoverageConfig::Builder empty;
    const auto azimuthOnly = beam::RequestCoverageConfig::Builder()
        .azimuthRange(math::CircularAngleRange(-60_deg, 60_deg));

    // When: 준비한 입력으로 대상 객체의 동작을 시도한다.
    const auto buildEmpty = [&] { return empty.build(); };
    const auto buildAzimuthOnly = [&] { return azimuthOnly.build(); };

    // Then: 영역 설정 빌더는 방위각과 고각 범위가 모두 필요하다.
    EXPECT_THROW(buildEmpty(), std::logic_error);
    EXPECT_THROW(buildAzimuthOnly(), std::logic_error);
}

// 영역 설정 빌더는 지정한 방위각과 고각 범위를 반영한다.
TEST(RequestCoverageConfigTest, BuilderOverridesGivenValues)
{
    // Given: 필수 값이 설정된 빌더를 준비한다.
    auto builder = coverageBuilder();

    // When: 두 축의 범위를 변경해 영역 설정을 생성한다.
    const beam::RequestCoverageConfig config = builder
        .azimuthRange(math::CircularAngleRange(-50_deg, 40_deg))
        .elevationRange(math::AngleRange(-5_deg, 70_deg))
        .build();

    // Then: 영역 설정 빌더는 지정한 방위각과 고각 범위를 반영한다.
    EXPECT_EQ(config.azimuthRange().start(), -50_deg);
    EXPECT_EQ(config.azimuthRange().end(), 40_deg);
    EXPECT_EQ(config.elevationRange().start(), -5_deg);
    EXPECT_EQ(config.elevationRange().end(), 70_deg);
}

// 영역 설정은 경계를 가로지르는 범위와 전체 방위각 범위를 허용한다.
TEST(RequestCoverageConfigTest, AcceptsCircularAzimuthRanges)
{
    // Given: 검증할 입력과 설정을 준비한다.
    const math::CircularAngleRange ranges[] = {
        {-180_deg, 180_deg}, {350_deg, 10_deg}, math::CircularAngleRange::fullCircle()};

    // When: 준비한 입력으로 대상 객체의 동작을 시도한다.
    const auto build = [](const math::CircularAngleRange& range) {
        return coverageBuilder().azimuthRange(range).build();
    };

    // Then: 영역 설정은 경계를 가로지르는 범위와 전체 방위각 범위를 허용한다.
    for (const auto& range : ranges)
    {
        EXPECT_NO_THROW(build(range));
    }
}

// 영역 설정은 유효한 원형 방위각 범위를 그대로 보관한다.
TEST(RequestCoverageConfigTest, KeepsGivenCircularAzimuthRanges)
{
    // Given: 0도를 통과하는 범위와 반대편으로 도는 넓은 범위를 준비한다.
    const math::CircularAngleRange wrapped(350_deg, 10_deg);
    const math::CircularAngleRange wide(20_deg, 10_deg);

    // When: 각 방위각 범위로 영역 설정을 생성한다.
    const auto wrappedConfig = coverageBuilder().azimuthRange(wrapped).build();
    const auto wideConfig = coverageBuilder().azimuthRange(wide).build();

    // Then: 각 설정이 전달받은 범위의 시작과 끝을 보관한다.
    EXPECT_EQ(wrappedConfig.azimuthRange().start(), wrapped.start());
    EXPECT_EQ(wrappedConfig.azimuthRange().end(), wrapped.end());
    EXPECT_EQ(wideConfig.azimuthRange().start(), wide.start());
    EXPECT_EQ(wideConfig.azimuthRange().end(), wide.end());
}

// 영역 설정은 고각의 양 끝이 -90도부터 90도 사이여야 한다.
TEST(RequestCoverageConfigTest, ElevationOutsideQuarterTurnThrows)
{
    // Given: 검증할 입력과 설정을 준비한다.
    const math::AngleRange boundary(-90_deg, 90_deg);
    const math::AngleRange invalidRanges[] = {
        {-90.001_deg, 60_deg}, {-10_deg, 90.001_deg}, {-91_deg, 60_deg}, {-10_deg, 91_deg}};
    const math::CircularAngleRange azimuth(-60_deg, 60_deg);

    // When: 준비한 입력으로 대상 객체의 동작을 시도한다.
    const auto build = [](const math::AngleRange& range) {
        return coverageBuilder().elevationRange(range).build();
    };
    const auto construct = [&](const math::AngleRange& range) {
        return beam::RequestCoverageConfig(azimuth, range);
    };

    // Then: 영역 설정은 고각의 양 끝이 -90도부터 90도 사이여야 한다.
    EXPECT_NO_THROW(build(boundary));
    EXPECT_NO_THROW(construct(boundary));
    for (const auto& range : invalidRanges)
    {
        EXPECT_THROW(build(range), std::invalid_argument);
        EXPECT_THROW(construct(range), std::invalid_argument);
    }
}
