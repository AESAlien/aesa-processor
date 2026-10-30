#include <beam/config/request_beam_config.hpp>

#include <gtest/gtest.h>

#include <stdexcept>
#include <type_traits>

using namespace math::literals;

namespace
{

static_assert(std::is_copy_assignable_v<beam::RequestBeamConfig>);

beam::RequestBeamConfig::Builder requestBuilder()
{
    return beam::RequestBeamConfig::Builder()
        .confirmationBeamWidth(beam::BeamWidth(math::AngularWidth(3_deg), math::AngularWidth(3_deg)))
        .trackingBeamWidth(beam::BeamWidth(math::AngularWidth(3_deg), math::AngularWidth(3_deg)));
}

} // namespace

// 필수 값이 누락된 빌더는 설정을 생성할 수 없다.
TEST(RequestBeamConfigTest, BuilderRequiresEveryValue)
{
    // Given: 필수 값을 지정하지 않은 빌더를 준비한다.
    const beam::RequestBeamConfig::Builder builder;

    // When: 빌더로 설정 생성을 시도한다.
    const auto build = [&builder] { return builder.build(); };

    // Then: 누락된 필수 값 때문에 예외가 발생한다.
    EXPECT_THROW(build(), std::logic_error);
}

// 요청 빔 설정 빌더는 지정한 확인 및 추적 빔 폭을 반영한다.
TEST(RequestBeamConfigTest, BuilderOverridesGivenValues)
{
    // Given: 필수 값이 설정된 빌더를 준비한다.
    auto builder = requestBuilder();

    // When: 서로 다른 확인 및 추적 빔 폭으로 설정을 생성한다.
    const beam::RequestBeamConfig config = builder
        .confirmationBeamWidth(beam::BeamWidth(2_deg, 2_deg))
        .trackingBeamWidth(beam::BeamWidth(4_deg, 4_deg))
        .build();

    // Then: 요청 빔 설정 빌더는 지정한 확인 및 추적 빔 폭을 반영한다.
    EXPECT_EQ(config.confirmationBeamWidth.azimuth, 2_deg);
    EXPECT_EQ(config.confirmationBeamWidth.elevation, 2_deg);
    EXPECT_EQ(config.trackingBeamWidth.azimuth, 4_deg);
    EXPECT_EQ(config.trackingBeamWidth.elevation, 4_deg);
}

// 요청 빔 설정은 전달받은 두 빔 폭을 보관한다.
TEST(RequestBeamConfigTest, ConstructorKeepsGivenBeamWidths)
{
    // Given: 서로 다른 확인 빔 폭과 추적 빔 폭을 준비한다.
    const beam::BeamWidth confirmationWidth(2_deg, 3_deg);
    const beam::BeamWidth trackingWidth(4_deg, 5_deg);

    // When: 두 빔 폭으로 설정을 생성한다.
    const beam::RequestBeamConfig config(confirmationWidth, trackingWidth);

    // Then: 각 빔 폭이 전달한 값으로 보관된다.
    EXPECT_EQ(config.confirmationBeamWidth, confirmationWidth);
    EXPECT_EQ(config.trackingBeamWidth, trackingWidth);
}
