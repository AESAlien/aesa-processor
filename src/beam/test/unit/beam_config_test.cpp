#include <beam/config/beam_config.hpp>

#include <gtest/gtest.h>

#include <stdexcept>
#include <type_traits>

using namespace math::literals;

namespace
{

static_assert(std::is_copy_assignable_v<beam::BeamConfig>);

beam::AttitudeConfig::Builder attitudeBuilder()
{
    return beam::AttitudeConfig::Builder()
        .maxAbsPitch(89_deg)
        .mountAntToBodyRotation(math::SquareMatrix::identity(3))
        .mountOrthogonalTolerance(math::Tolerance{1e-6, 0.0});
}

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

beam::RequestBeamConfig::Builder requestBuilder()
{
    return beam::RequestBeamConfig::Builder()
        .confirmationBeamWidth(beam::BeamWidth(math::AngularWidth(3_deg), math::AngularWidth(3_deg)))
        .trackingBeamWidth(beam::BeamWidth(math::AngularWidth(3_deg), math::AngularWidth(3_deg)));
}

beam::RequestCoverageConfig::Builder coverageBuilder()
{
    return beam::RequestCoverageConfig::Builder()
        .azimuthRange(math::CircularAngleRange(-60_deg, 60_deg))
        .elevationRange(math::AngleRange(-10_deg, 60_deg));
}

} // namespace

// 필수 값이 누락된 빌더는 설정을 생성할 수 없다.
TEST(BeamConfigTest, BuilderRequiresEveryValue)
{
    // Given: 필수 값을 지정하지 않은 빌더를 준비한다.
    const beam::BeamConfig::Builder builder;

    // When: 빌더로 설정 생성을 시도한다.
    const auto build = [&builder] { return builder.build(); };

    // Then: 누락된 필수 값 때문에 예외가 발생한다.
    EXPECT_THROW(build(), std::logic_error);
}

// 빔 설정 빌더는 교체한 하위 설정과 나머지 설정을 함께 보관한다.
TEST(BeamConfigTest, BuilderReplacesOnlyGivenSubConfigs)
{
    // Given: 하위 설정을 조합할 빌더를 준비한다.
    beam::BeamConfig::Builder builder;

    // When: 하위 설정을 지정하고 일부 설정을 교체한 뒤 생성한다.
    const beam::BeamConfig config = builder
        .attitudeConfig(attitudeBuilder().build())
        .searchGridConfig(gridBuilder().build())
        .requestBeamConfig(requestBuilder().build())
        .requestCoverageConfig(coverageBuilder().build())
        .searchGridConfig(gridBuilder().azimuthCount(5).build())
        .requestBeamConfig(requestBuilder().trackingBeamWidth(beam::BeamWidth(4_deg, 4_deg)).build())
        .build();

    // Then: 빔 설정 빌더는 교체한 하위 설정과 나머지 설정을 함께 보관한다.
    EXPECT_EQ(config.searchGrid.azimuthCount(), 5u);
    EXPECT_EQ(config.requestBeam.trackingBeamWidth.azimuth, 4_deg);
    EXPECT_EQ(config.requestBeam.trackingBeamWidth.elevation, 4_deg);
    EXPECT_EQ(config.requestBeam.confirmationBeamWidth.azimuth, 3_deg);
    EXPECT_EQ(config.requestBeam.confirmationBeamWidth.elevation, 3_deg);
    EXPECT_EQ(config.attitude.maxAbsPitch(), 89_deg);
    EXPECT_EQ(config.requestCoverage.azimuthRange().end(), 60_deg);
}

// 요청 영역 설정이 빠진 빌더는 빔 설정을 생성할 수 없다.
TEST(BeamConfigTest, BuilderRequiresRequestCoverageConfig)
{
    // Given: 검증할 입력과 설정을 준비한다.
    const auto builder = beam::BeamConfig::Builder()
        .attitudeConfig(attitudeBuilder().build())
        .searchGridConfig(gridBuilder().build())
        .requestBeamConfig(requestBuilder().build());

    // When: 준비한 입력으로 대상 객체의 동작을 시도한다.
    const auto build = [&builder] { return builder.build(); };

    // Then: 요청 영역 설정이 빠진 빌더는 빔 설정을 생성할 수 없다.
    EXPECT_THROW(build(), std::logic_error);
}

// 빔 설정 생성자는 전달받은 하위 설정을 보관한다.
TEST(BeamConfigTest, ConstructorKeepsGivenSubConfigs)
{
    // Given: 구분 가능한 값으로 하위 설정을 준비한다.
    const auto attitude = attitudeBuilder().maxAbsPitch(70_deg).build();
    const auto grid = gridBuilder().elevationCount(4).build();
    const auto request = requestBuilder().confirmationBeamWidth(beam::BeamWidth(2_deg, 2_deg)).build();
    const auto coverage = coverageBuilder().elevationRange(math::AngleRange(-5_deg, 60_deg)).build();

    // When: 하위 설정을 묶어 빔 설정을 생성한다.
    const beam::BeamConfig config(attitude, grid, request, coverage);

    // Then: 빔 설정 생성자는 전달받은 하위 설정을 보관한다.
    EXPECT_EQ(config.attitude.maxAbsPitch(), 70_deg);
    EXPECT_EQ(config.searchGrid.elevationCount(), 4u);
    EXPECT_EQ(config.requestBeam.confirmationBeamWidth.azimuth, 2_deg);
    EXPECT_EQ(config.requestBeam.confirmationBeamWidth.elevation, 2_deg);
    EXPECT_EQ(config.requestCoverage.elevationRange().start(), -5_deg);
}
