#include <beam/config/attitude_config.hpp>
#include <beam/config/beam_config.hpp>
#include <beam/config/request_beam_config.hpp>
#include <beam/config/request_coverage_config.hpp>
#include <beam/config/search_grid_config.hpp>


#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <stdexcept>
#include <type_traits>

using namespace math::literals;

namespace
{

static_assert(std::is_copy_assignable_v<beam::AttitudeConfig>);
static_assert(std::is_copy_assignable_v<beam::SearchGridConfig>);
static_assert(std::is_copy_assignable_v<beam::RequestBeamConfig>);
static_assert(std::is_copy_assignable_v<beam::RequestCoverageConfig>);
static_assert(std::is_copy_assignable_v<beam::BeamConfig>);

const double NAN_VALUE = std::numeric_limits<double>::quiet_NaN();
const double INF_VALUE = std::numeric_limits<double>::infinity();
const double MAX_VALUE = std::numeric_limits<double>::max();

// NaN·Inf Angle은 math::Angle 생성 단계에서 std::invalid_argument로 거부되므로 설정 검증 대상이 아니다.
const math::Angle MAX_ANGLE = math::Angle::fromDegrees(MAX_VALUE);

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
        .beamWidth(beam::BeamWidth(6_deg, 6_deg));
}

beam::RequestBeamConfig::Builder requestBuilder()
{
    return beam::RequestBeamConfig::Builder()
        .confirmationBeamWidth(beam::BeamWidth(3_deg, 3_deg))
        .trackingBeamWidth(beam::BeamWidth(3_deg, 3_deg));
}

beam::RequestCoverageConfig::Builder coverageBuilder()
{
    return beam::RequestCoverageConfig::Builder()
        .azimuthRange(math::CircularAngleRange(-60_deg, 60_deg))
        .elevationRange(math::AngleRange(-10_deg, 60_deg));
}

} // namespace

// ================================================================ SearchGridConfig

TEST(SearchGridConfigTest, BuilderRequiresEveryValue)
{
    EXPECT_THROW(beam::SearchGridConfig::Builder().build(), std::logic_error);
}

TEST(SearchGridConfigTest, BuilderOverridesOnlyGivenValues)
{
    const beam::SearchGridConfig config = gridBuilder()
        .azimuthCount(3)
        .elevationCount(2)
        .azimuthStart(math::CircularAngle(-10_deg))
        .azimuthStep(5_deg)
        .elevationStart(math::TiltAngle(1_deg))
        .elevationStep(2_deg)
        .beamWidth(beam::BeamWidth(4_deg, 4_deg))
        .build();

    EXPECT_EQ(config.azimuthCount(), 3u);
    EXPECT_EQ(config.elevationCount(), 2u);
    EXPECT_TRUE(config.azimuthStart().isEquivalentTo(-10_deg));
    EXPECT_EQ(config.azimuthStep(), 5_deg);
    EXPECT_EQ(config.elevationStart(), 1_deg);
    EXPECT_EQ(config.elevationStep(), 2_deg);
    EXPECT_EQ(config.beamWidth().azimuth, 4_deg);
    EXPECT_EQ(config.beamWidth().elevation, 4_deg);

    const beam::SearchGridConfig partial = gridBuilder().azimuthCount(5).build();
    EXPECT_EQ(partial.azimuthCount(), 5u);
    EXPECT_EQ(partial.elevationCount(), 9u);
}

TEST(SearchGridConfigTest, ConstructorValidatesLikeBuilder)
{
    EXPECT_NO_THROW(beam::SearchGridConfig(3, 2, math::CircularAngle(-10_deg), 5_deg, math::TiltAngle(1_deg), 2_deg, beam::BeamWidth(4_deg, 4_deg)));
    EXPECT_THROW(beam::SearchGridConfig(0, 2, math::CircularAngle(-10_deg), 5_deg, math::TiltAngle(1_deg), 2_deg, beam::BeamWidth(4_deg, 4_deg)), std::invalid_argument);
}

TEST(SearchGridConfigTest, ZeroAzimuthCountThrows)
{
    EXPECT_THROW(gridBuilder().azimuthCount(0).build(), std::invalid_argument);
}

TEST(SearchGridConfigTest, ZeroElevationCountThrows)
{
    EXPECT_THROW(gridBuilder().elevationCount(0).build(), std::invalid_argument);
}

TEST(SearchGridConfigTest, BothCountsZeroThrowsWithoutDividingByZero)
{
    EXPECT_THROW(gridBuilder().azimuthCount(0).elevationCount(0).build(), std::invalid_argument);
}

TEST(SearchGridConfigTest, BeamCountBeyondUint32Throws)
{
    EXPECT_THROW(gridBuilder().azimuthCount(70000).elevationCount(70000).build(), std::invalid_argument);
}

TEST(SearchGridConfigTest, NonPositiveBeamWidthThrows)
{
    EXPECT_THROW(gridBuilder().beamWidth(beam::BeamWidth(0_deg, 0_deg)).build(), std::invalid_argument);
    EXPECT_THROW(gridBuilder().beamWidth(beam::BeamWidth(-1_deg, -1_deg)).build(), std::invalid_argument);
}

TEST(SearchGridConfigTest, GridAngleOverflowThrows)
{
    // step * (count - 1)에서 오버플로
    EXPECT_THROW(gridBuilder().azimuthCount(3).azimuthStep(MAX_ANGLE).build(), std::invalid_argument);
    EXPECT_THROW(gridBuilder().elevationCount(3).elevationStep(MAX_ANGLE).build(), std::invalid_argument);

    // start + step * (count - 1)에서 오버플로
    EXPECT_THROW(
        gridBuilder().elevationCount(2).elevationStart(math::TiltAngle(90_deg)).elevationStep(MAX_ANGLE).build(),
        std::invalid_argument);
}

TEST(SearchGridConfigTest, WrapsLargeFiniteAzimuthStep)
{
    EXPECT_NO_THROW(
        gridBuilder().azimuthCount(2).azimuthStart(math::CircularAngle(MAX_ANGLE)).azimuthStep(MAX_ANGLE).build());
}

TEST(SearchGridConfigTest, LargeStepWithSingleBeamPerAxisIsAccepted)
{
    // count가 1이면 step은 쓰이지 않으므로 큰 step도 문제없다.
    EXPECT_NO_THROW(gridBuilder()
        .azimuthCount(1)
        .azimuthStep(MAX_ANGLE)
        .elevationCount(1)
        .elevationStep(MAX_ANGLE)
        .build());
}

TEST(SearchGridConfigTest, ElevationOutsideSphericalRangeThrows)
{
    EXPECT_THROW(gridBuilder().elevationStart(math::TiltAngle(91_deg)).build(), std::invalid_argument);
    EXPECT_THROW(gridBuilder().elevationStart(math::TiltAngle(-91_deg)).build(), std::invalid_argument);

    // 마지막 고각 80 + 6 * 2 = 92도
    EXPECT_THROW(
        gridBuilder().elevationCount(3).elevationStart(math::TiltAngle(80_deg)).elevationStep(6_deg).build(),
        std::invalid_argument);
    // 마지막 고각 0 - 50 * 2 = -100도 (음수 step)
    EXPECT_THROW(
        gridBuilder().elevationCount(3).elevationStart(math::TiltAngle(0_deg)).elevationStep(-50_deg).build(),
        std::invalid_argument);
}

TEST(SearchGridConfigTest, ElevationEquivalentByPeriodToValidRangeThrows)
{
    // 360도 주기로는 범위 안(90도)과 동치이지만 실제 수치는 [-90, 90]도 밖이므로 거부해야 한다.
    // 450 -> 270 -> 90도로 가는 격자는 중간 고각 270도가 SphericalDirection에서 거부된다.
    EXPECT_THROW(
        gridBuilder().elevationCount(3).elevationStart(math::TiltAngle(450_deg)).elevationStep(-180_deg).build(),
        std::invalid_argument);
    EXPECT_THROW(gridBuilder().elevationCount(1).elevationStart(math::TiltAngle(450_deg)).elevationStep(0_deg).build(), std::invalid_argument);
    EXPECT_THROW(gridBuilder().elevationCount(1).elevationStart(math::TiltAngle(-270_deg)).elevationStep(0_deg).build(), std::invalid_argument);
    // 시작점은 범위 안이고 마지막 고각만 90 + 360 = 450도
    EXPECT_THROW(
        gridBuilder().elevationCount(2).elevationStart(math::TiltAngle(90_deg)).elevationStep(360_deg).build(),
        std::invalid_argument);
}

TEST(SearchGridConfigTest, ElevationBoundaryIsAccepted)
{
    // 마지막 고각이 정확히 90도
    EXPECT_NO_THROW(gridBuilder().elevationCount(3).elevationStart(math::TiltAngle(80_deg)).elevationStep(5_deg).build());
    // 음수 step으로 내려가는 격자
    EXPECT_NO_THROW(gridBuilder().elevationCount(3).elevationStart(math::TiltAngle(0_deg)).elevationStep(-5_deg).build());
}

TEST(SearchGridConfigTest, AzimuthBeyondHalfTurnIsAccepted)
{
    // 방위각은 CircularAngle이 정규화하므로 범위를 제한하지 않는다.
    EXPECT_NO_THROW(gridBuilder().azimuthCount(100).azimuthStep(10_deg).build());
    EXPECT_NO_THROW(
        gridBuilder()
            .azimuthCount(1)
            .elevationCount(1)
            .azimuthStart(math::CircularAngle(1e18_deg))
            .build());
}

TEST(SearchGridConfigTest, AngleAtIndexIsStartPlusStepTimesIndex)
{
    const beam::SearchGridConfig config = gridBuilder().build();

    EXPECT_EQ(config.azimuthAt(0), math::CircularAngle(-42_deg));
    EXPECT_EQ(config.azimuthAt(20), math::CircularAngle(-42_deg) + 4.2_deg * 20.0);
    EXPECT_EQ(config.elevationAt(0), math::TiltAngle(3_deg));
    EXPECT_EQ(config.elevationAt(8), math::TiltAngle(3_deg) + 4.4_deg * 8.0);
}

// ================================================================ AttitudeConfig

TEST(AttitudeConfigTest, BuilderRequiresEveryValue)
{
    EXPECT_THROW(beam::AttitudeConfig::Builder().build(), std::logic_error);
}

TEST(AttitudeConfigTest, BuilderOverridesGivenValues)
{
    const math::SquareMatrix mount{{0, -1, 0}, {1, 0, 0}, {0, 0, 1}};
    const beam::AttitudeConfig config = attitudeBuilder()
        .maxAbsPitch(80_deg)
        .mountAntToBodyRotation(mount)
        .mountOrthogonalTolerance(math::Tolerance{1e-3, 2e-3})
        .build();

    EXPECT_EQ(config.maxAbsPitch(), 80_deg);
    EXPECT_EQ(config.mountOrthogonalTolerance().absolute(), 1e-3);
    EXPECT_EQ(config.mountOrthogonalTolerance().relative(), 2e-3);
    EXPECT_EQ(config.mountAntToBodyRotation()(0, 1), -1.0);
    EXPECT_EQ(config.mountAntToBodyRotation()(1, 0), 1.0);
}

TEST(AttitudeConfigTest, AcceptsRotationMount)
{
    EXPECT_NO_THROW(attitudeBuilder()
        .mountAntToBodyRotation(math::SquareMatrix{{0, -1, 0}, {1, 0, 0}, {0, 0, 1}}) // z축 90도 회전
        .build());
}

TEST(AttitudeConfigTest, AcceptsRotationMountWithFloatingPointEntries)
{
    const math::Angle angle = 0.37_rad;
    const double c = std::cos(angle.rad());
    const double s = std::sin(angle.rad());

    EXPECT_NO_THROW(attitudeBuilder()
        .mountAntToBodyRotation(math::SquareMatrix{{c, -s, 0}, {s, c, 0}, {0, 0, 1}})
        .build());
}

TEST(AttitudeConfigTest, NonRotationMountThrows)
{
    EXPECT_THROW(
        attitudeBuilder().mountAntToBodyRotation(math::SquareMatrix{{2, 0, 0}, {0, 1, 0}, {0, 0, 1}}).build(),
        std::invalid_argument); // 스케일
    EXPECT_THROW(
        attitudeBuilder().mountAntToBodyRotation(math::SquareMatrix{{1, 0, 0}, {0, 1, 0}, {0, 0, -1}}).build(),
        std::invalid_argument); // 반사 (det = -1)
    EXPECT_THROW(
        attitudeBuilder().mountAntToBodyRotation(math::SquareMatrix{{1, 0, 0}, {0, 1, 0}, {0, 0, 0}}).build(),
        std::invalid_argument); // 특이 행렬
    EXPECT_THROW(
        attitudeBuilder().mountAntToBodyRotation(math::SquareMatrix::identity(2)).build(),
        std::invalid_argument); // 3x3 이 아님
}

TEST(AttitudeConfigTest, NonFiniteMountEntriesAndOverflowThrow)
{
    for (double value : {NAN_VALUE, INF_VALUE, MAX_VALUE})
    {
        EXPECT_THROW(
            attitudeBuilder().mountAntToBodyRotation(math::SquareMatrix{{value, 0, 0}, {0, 1, 0}, {0, 0, 1}}).build(),
            std::invalid_argument)
            << "value=" << value;
    }
}

TEST(AttitudeConfigTest, MountOrthogonalityUsesConfiguredTolerance)
{
    const math::SquareMatrix skewed{{1, 0.1, 0}, {0, 1, 0}, {0, 0, 1}};

    EXPECT_NO_THROW(attitudeBuilder()
        .mountAntToBodyRotation(skewed)
        .mountOrthogonalTolerance(math::Tolerance{0.2, 0.0})
        .build());
    EXPECT_THROW(
        attitudeBuilder()
            .mountAntToBodyRotation(skewed)
            .mountOrthogonalTolerance(math::Tolerance{0.05, 0.0})
            .build(),
        std::invalid_argument);
}

TEST(AttitudeConfigTest, MountRotationUsesRelativeTolerance)
{
    const math::SquareMatrix slightlyScaled{
        {1.0005, 0, 0},
        {0, 1, 0},
        {0, 0, 1}};

    EXPECT_NO_THROW(attitudeBuilder()
        .mountAntToBodyRotation(slightlyScaled)
        .mountOrthogonalTolerance(math::Tolerance{0.0, 0.002})
        .build());
    EXPECT_THROW(attitudeBuilder()
        .mountAntToBodyRotation(slightlyScaled)
        .mountOrthogonalTolerance(math::Tolerance{0.0, 0.0})
        .build(),
        std::invalid_argument);
}

TEST(AttitudeConfigTest, InvalidMountOrthogonalToleranceThrows)
{
    EXPECT_NO_THROW(
        attitudeBuilder().mountOrthogonalTolerance(math::Tolerance{0.0, 0.0}).build());

    for (double tolerance : {-1.0, NAN_VALUE, INF_VALUE})
    {
        EXPECT_THROW((math::Tolerance{tolerance, 0.0}), std::invalid_argument)
            << "tolerance=" << tolerance;
        EXPECT_THROW((math::Tolerance{0.0, tolerance}), std::invalid_argument)
            << "tolerance=" << tolerance;
    }
}

TEST(AttitudeConfigTest, MaxAbsPitchMustBeInZeroToNinetyDegrees)
{
    EXPECT_NO_THROW(attitudeBuilder().maxAbsPitch(90_deg).build());
    EXPECT_NO_THROW(attitudeBuilder().maxAbsPitch(0.1_deg).build());

    EXPECT_THROW(attitudeBuilder().maxAbsPitch(0_deg).build(), std::invalid_argument);
    EXPECT_THROW(attitudeBuilder().maxAbsPitch(-1_deg).build(), std::invalid_argument);
    EXPECT_THROW(attitudeBuilder().maxAbsPitch(91_deg).build(), std::invalid_argument);
}

TEST(AttitudeConfigTest, MaxAbsPitchRejectsAnglesEquivalentToNinetyDegreesByPeriod)
{
    // 360도 주기로 90도와 동치인 값은 (0, 90] 범위 밖이므로 거부해야 한다.
    EXPECT_THROW(attitudeBuilder().maxAbsPitch(450_deg).build(), std::invalid_argument);
    EXPECT_THROW(attitudeBuilder().maxAbsPitch(810_deg).build(), std::invalid_argument);
    EXPECT_THROW(attitudeBuilder().maxAbsPitch(-270_deg).build(), std::invalid_argument);
    EXPECT_THROW(
        beam::AttitudeConfig(
            450_deg,
            math::SquareMatrix::identity(3),
            math::Tolerance{1e-6, 0.0}),
        std::invalid_argument);
}

TEST(AttitudeConfigTest, ConstructorValidatesLikeBuilder)
{
    EXPECT_NO_THROW(beam::AttitudeConfig(
        89_deg,
        math::SquareMatrix::identity(3),
        math::Tolerance{1e-6, 0.0}));
    EXPECT_THROW(
        beam::AttitudeConfig(
            89_deg,
            math::SquareMatrix::identity(2),
            math::Tolerance{1e-6, 0.0}),
        std::invalid_argument);
}

// ================================================================ RequestBeamConfig

TEST(RequestBeamConfigTest, BuilderRequiresEveryValue)
{
    EXPECT_THROW(beam::RequestBeamConfig::Builder().build(), std::logic_error);
}

TEST(RequestBeamConfigTest, BuilderOverridesGivenValues)
{
    const beam::RequestBeamConfig config = requestBuilder()
        .confirmationBeamWidth(beam::BeamWidth(2_deg, 2_deg))
        .trackingBeamWidth(beam::BeamWidth(4_deg, 4_deg))
        .build();

    EXPECT_EQ(config.confirmationBeamWidth.azimuth, 2_deg);
    EXPECT_EQ(config.confirmationBeamWidth.elevation, 2_deg);
    EXPECT_EQ(config.trackingBeamWidth.azimuth, 4_deg);
    EXPECT_EQ(config.trackingBeamWidth.elevation, 4_deg);
}

TEST(RequestBeamConfigTest, NonPositiveBeamWidthThrows)
{
    EXPECT_THROW(requestBuilder().confirmationBeamWidth(beam::BeamWidth(0_deg, 0_deg)).build(), std::invalid_argument);
    EXPECT_THROW(requestBuilder().confirmationBeamWidth(beam::BeamWidth(-1_deg, -1_deg)).build(), std::invalid_argument);
    EXPECT_THROW(requestBuilder().trackingBeamWidth(beam::BeamWidth(0_deg, 0_deg)).build(), std::invalid_argument);
    EXPECT_THROW(requestBuilder().trackingBeamWidth(beam::BeamWidth(-1_deg, -1_deg)).build(), std::invalid_argument);
}

TEST(RequestBeamConfigTest, ConstructorValidatesLikeBuilder)
{
    EXPECT_NO_THROW(beam::RequestBeamConfig(beam::BeamWidth(2_deg, 2_deg), beam::BeamWidth(4_deg, 4_deg)));
    EXPECT_THROW(beam::RequestBeamConfig(beam::BeamWidth(0_deg, 0_deg), beam::BeamWidth(4_deg, 4_deg)), std::invalid_argument);
}

// ================================================================ RequestCoverageConfig

TEST(RequestCoverageConfigTest, BuilderRequiresEveryValue)
{
    EXPECT_THROW(beam::RequestCoverageConfig::Builder().build(), std::logic_error);
    EXPECT_THROW(beam::RequestCoverageConfig::Builder().azimuthRange(math::CircularAngleRange(-60_deg, 60_deg)).build(), std::logic_error);
}

TEST(RequestCoverageConfigTest, BuilderOverridesGivenValues)
{
    const beam::RequestCoverageConfig config = coverageBuilder()
        .azimuthRange(math::CircularAngleRange(-50_deg, 40_deg))
        .elevationRange(math::AngleRange(-5_deg, 70_deg))
        .build();

    EXPECT_EQ(config.azimuthRange().start(), -50_deg);
    EXPECT_EQ(config.azimuthRange().end(), 40_deg);
    EXPECT_EQ(config.elevationRange().start(), -5_deg);
    EXPECT_EQ(config.elevationRange().end(), 70_deg);
}

TEST(RequestCoverageConfigTest, AcceptsCircularAzimuthRanges)
{
    EXPECT_NO_THROW(coverageBuilder().azimuthRange(math::CircularAngleRange(-180_deg, 180_deg)).build());
    EXPECT_NO_THROW(coverageBuilder().azimuthRange(math::CircularAngleRange(350_deg, 10_deg)).build());
    EXPECT_NO_THROW(coverageBuilder().azimuthRange(math::CircularAngleRange::fullCircle()).build());
    EXPECT_THROW(coverageBuilder().azimuthRange(math::CircularAngleRange(-MAX_ANGLE, 60_deg)).build(), std::invalid_argument);
    EXPECT_THROW(coverageBuilder().azimuthRange(math::CircularAngleRange(-60_deg, MAX_ANGLE)).build(), std::invalid_argument);
}

TEST(RequestCoverageConfigTest, StartNotLessThanEndThrows)
{
    EXPECT_THROW(coverageBuilder().azimuthRange(math::CircularAngleRange(10_deg, 10_deg)).build(), std::invalid_argument);
    EXPECT_NO_THROW(coverageBuilder().azimuthRange(math::CircularAngleRange(20_deg, 10_deg)).build());
    EXPECT_THROW(
        coverageBuilder().elevationRange(math::AngleRange(30_deg, 30_deg)).build(),
        std::invalid_argument);
    EXPECT_THROW(
        coverageBuilder().elevationRange(math::AngleRange(40_deg, 30_deg)).build(),
        std::invalid_argument);
}

TEST(RequestCoverageConfigTest, ElevationOutsideQuarterTurnThrows)
{
    EXPECT_NO_THROW(coverageBuilder().elevationRange(math::AngleRange(-90_deg, 90_deg)).build());
    EXPECT_THROW(math::TiltAngle(91_deg), std::invalid_argument);
    EXPECT_THROW(coverageBuilder().elevationRange(math::AngleRange(-90.001_deg, 60_deg)).build(), std::invalid_argument);
    EXPECT_THROW(coverageBuilder().elevationRange(math::AngleRange(-10_deg, 90.001_deg)).build(), std::invalid_argument);
    EXPECT_THROW(
        beam::RequestCoverageConfig(math::CircularAngleRange(-60_deg, 60_deg), math::AngleRange(-91_deg, 60_deg)),
        std::invalid_argument);
    EXPECT_THROW(
        beam::RequestCoverageConfig(math::CircularAngleRange(-60_deg, 60_deg), math::AngleRange(-10_deg, 91_deg)),
        std::invalid_argument);
}

TEST(RequestCoverageConfigTest, ConstructorValidatesLikeBuilder)
{
    EXPECT_NO_THROW(beam::RequestCoverageConfig(math::CircularAngleRange(-60_deg, 60_deg), math::AngleRange(-10_deg, 60_deg)));
    EXPECT_NO_THROW(
        beam::RequestCoverageConfig(math::CircularAngleRange(60_deg, -60_deg), math::AngleRange(-10_deg, 60_deg)));
    EXPECT_NO_THROW(
        beam::RequestCoverageConfig(math::CircularAngleRange(-181_deg, 60_deg), math::AngleRange(-10_deg, 60_deg)));
    EXPECT_NO_THROW(
        beam::RequestCoverageConfig(math::CircularAngleRange(-60_deg, 181_deg), math::AngleRange(-10_deg, 60_deg)));
    EXPECT_THROW(
        beam::RequestCoverageConfig(math::CircularAngleRange(-60_deg, 60_deg), math::AngleRange(60_deg, -10_deg)),
        std::invalid_argument);
}

// ================================================================ BeamConfig

TEST(BeamConfigTest, BuilderRequiresEveryValue)
{
    EXPECT_THROW(beam::BeamConfig::Builder().build(), std::logic_error);
}

TEST(BeamConfigTest, BuilderReplacesOnlyGivenSubConfigs)
{
    const beam::BeamConfig config = beam::BeamConfig::Builder()
        .attitudeConfig(attitudeBuilder().build())
        .searchGridConfig(gridBuilder().build())
        .requestBeamConfig(requestBuilder().build())
        .requestCoverageConfig(coverageBuilder().build())
        .searchGridConfig(gridBuilder().azimuthCount(5).build())
        .requestBeamConfig(requestBuilder().trackingBeamWidth(beam::BeamWidth(4_deg, 4_deg)).build())
        .build();

    EXPECT_EQ(config.searchGrid.azimuthCount(), 5u);
    EXPECT_EQ(config.requestBeam.trackingBeamWidth.azimuth, 4_deg);
    EXPECT_EQ(config.requestBeam.trackingBeamWidth.elevation, 4_deg);
    EXPECT_EQ(config.requestBeam.confirmationBeamWidth.azimuth, 3_deg);
    EXPECT_EQ(config.requestBeam.confirmationBeamWidth.elevation, 3_deg);
    EXPECT_EQ(config.attitude.maxAbsPitch(), 89_deg);
    EXPECT_EQ(config.requestCoverage.azimuthRange().end(), 60_deg);
}

TEST(BeamConfigTest, BuilderRequiresRequestCoverageConfig)
{
    EXPECT_THROW(
        beam::BeamConfig::Builder()
            .attitudeConfig(attitudeBuilder().build())
            .searchGridConfig(gridBuilder().build())
            .requestBeamConfig(requestBuilder().build())
            .build(),
        std::logic_error);
}

TEST(BeamConfigTest, ConstructorKeepsGivenSubConfigs)
{
    const beam::BeamConfig config(
        attitudeBuilder().maxAbsPitch(70_deg).build(),
        gridBuilder().elevationCount(4).build(),
        requestBuilder().confirmationBeamWidth(beam::BeamWidth(2_deg, 2_deg)).build(),
        coverageBuilder().elevationRange(math::AngleRange(-5_deg, 60_deg)).build());

    EXPECT_EQ(config.attitude.maxAbsPitch(), 70_deg);
    EXPECT_EQ(config.searchGrid.elevationCount(), 4u);
    EXPECT_EQ(config.requestBeam.confirmationBeamWidth.azimuth, 2_deg);
    EXPECT_EQ(config.requestBeam.confirmationBeamWidth.elevation, 2_deg);
    EXPECT_EQ(config.requestCoverage.elevationRange().start(), -5_deg);
}
