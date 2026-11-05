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
