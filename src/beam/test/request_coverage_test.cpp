#include "scheduler/request_coverage.hpp"

#include "attitude/ant_enu_transform.hpp"
#include "attitude/radar_attitude.hpp"

#include <gtest/gtest.h>

using namespace math::literals;

namespace
{

beam::AntEnuTransform makeTransform(math::Angle roll, math::Angle pitch, math::Angle yaw)
{
    return beam::AntEnuTransform(
        beam::RadarAttitude(roll, pitch, yaw),
        beam::AttitudeConfig(
            89_deg,
            math::SquareMatrix::identity(3),
            math::Tolerance{1e-6, 0.0}));
}

// 기본 추적 영역: 안테나 기준 방위각 -60~60도, 고각 -10~60도
beam::RequestCoverageConfig makeDefaultConfig()
{
    return beam::RequestCoverageConfig(-60_deg, 60_deg, math::Elevation(-10_deg), math::Elevation(60_deg));
}

beam::RequestCoverage makeCoverage(math::Angle roll, math::Angle pitch, math::Angle yaw)
{
    return beam::RequestCoverage(makeTransform(roll, pitch, yaw), makeDefaultConfig());
}

math::SphericalDirection direction(math::Angle azimuth, math::Angle elevation)
{
    return math::SphericalDirection(math::Azimuth(azimuth), math::Elevation(elevation));
}

} // namespace

// ---------- 영 자세: ENU와 안테나 기준이 같다 ----------

TEST(RequestCoverageTest, ZeroAttitudeIncludesBoundaries)
{
    const beam::RequestCoverage coverage = makeCoverage(0_deg, 0_deg, 0_deg);

    EXPECT_TRUE(coverage.contains(direction(-60_deg, 0_deg)));
    EXPECT_TRUE(coverage.contains(direction(60_deg, 0_deg)));
    EXPECT_TRUE(coverage.contains(direction(0_deg, -10_deg)));
    EXPECT_TRUE(coverage.contains(direction(0_deg, 60_deg)));
    EXPECT_TRUE(coverage.contains(direction(-60_deg, -10_deg)));
    EXPECT_TRUE(coverage.contains(direction(60_deg, 60_deg)));
}

TEST(RequestCoverageTest, ZeroAttitudeRejectsJustOutsideBoundaries)
{
    const beam::RequestCoverage coverage = makeCoverage(0_deg, 0_deg, 0_deg);

    EXPECT_FALSE(coverage.contains(direction(-60.1_deg, 0_deg)));
    EXPECT_FALSE(coverage.contains(direction(60.1_deg, 0_deg)));
    EXPECT_FALSE(coverage.contains(direction(0_deg, -10.1_deg)));
    EXPECT_FALSE(coverage.contains(direction(0_deg, 60.1_deg)));
}

TEST(RequestCoverageTest, AzimuthNearZeroAndFullTurnIsInside)
{
    const beam::RequestCoverage coverage = makeCoverage(0_deg, 0_deg, 0_deg);

    // Azimuth는 [0, 360)으로 정규화되므로 -0.1도는 359.9도로 들어온다.
    EXPECT_TRUE(coverage.contains(direction(0_deg, 10_deg)));
    EXPECT_TRUE(coverage.contains(direction(359.9_deg, 10_deg)));
    EXPECT_TRUE(coverage.contains(direction(-0.1_deg, 10_deg)));
    EXPECT_TRUE(coverage.contains(direction(300_deg, 10_deg)));
    EXPECT_FALSE(coverage.contains(direction(299.9_deg, 10_deg)));
}

TEST(RequestCoverageTest, BehindTheAntennaIsOutside)
{
    const beam::RequestCoverage coverage = makeCoverage(0_deg, 0_deg, 0_deg);

    EXPECT_FALSE(coverage.contains(direction(180_deg, 10_deg)));
    EXPECT_FALSE(coverage.contains(direction(90_deg, 10_deg)));
    EXPECT_FALSE(coverage.contains(direction(270_deg, 10_deg)));
}

TEST(RequestCoverageTest, CoverageAroundZeroAzimuthWorks)
{
    const beam::RequestCoverage coverage(
        makeTransform(0_deg, 0_deg, 0_deg),
        beam::RequestCoverageConfig(-10_deg, 10_deg, math::Elevation(0_deg), math::Elevation(20_deg)));

    EXPECT_TRUE(coverage.contains(direction(350_deg, 5_deg)));
    EXPECT_TRUE(coverage.contains(direction(10_deg, 5_deg)));
    EXPECT_FALSE(coverage.contains(direction(349_deg, 5_deg)));
    EXPECT_FALSE(coverage.contains(direction(11_deg, 5_deg)));
}

TEST(RequestCoverageTest, FullCoverageContainsEveryDirection)
{
    const beam::RequestCoverage coverage(
        makeTransform(10_deg, 20_deg, 30_deg),
        beam::RequestCoverageConfig(-180_deg, 180_deg, math::Elevation(-90_deg), math::Elevation(90_deg)));

    EXPECT_TRUE(coverage.contains(direction(0_deg, 0_deg)));
    EXPECT_TRUE(coverage.contains(direction(180_deg, 0_deg)));
    EXPECT_TRUE(coverage.contains(direction(210_deg, -80_deg)));
    EXPECT_TRUE(coverage.contains(direction(45_deg, 90_deg)));
}

// ---------- 자세가 있으면 ENU가 아닌 안테나 기준으로 판정한다 ----------

TEST(RequestCoverageTest, YawShiftsAzimuthToAntennaFrame)
{
    // 샘플 시나리오 자세(yaw 13.02429도). ENU 방위각에서 yaw를 뺀 값이 안테나 방위각이다(roll·pitch 0, 고각 0).
    const beam::RequestCoverage coverage = makeCoverage(0_deg, 0_deg, 13.02429_deg);

    // ENU 75도 → 안테나 약 61.98도: ENU로는 영역 밖이지만 안테나 기준으로도 밖이다.
    EXPECT_FALSE(coverage.contains(direction(75_deg, 0_deg)));
    // ENU 70도 → 안테나 약 56.98도: ENU 기준(60도 이하)과 안테나 기준 모두 안이다.
    EXPECT_TRUE(coverage.contains(direction(70_deg, 0_deg)));
    // ENU -50도 → 안테나 약 -63.02도: ENU 기준이면 안이지만 안테나 기준이면 밖이다.
    EXPECT_FALSE(coverage.contains(direction(-50_deg, 0_deg)));
    // ENU 72도 → 안테나 약 58.98도: ENU 기준이면 밖이지만 안테나 기준이면 안이다.
    EXPECT_TRUE(coverage.contains(direction(72_deg, 0_deg)));
}

TEST(RequestCoverageTest, PitchShiftsElevationToAntennaFrame)
{
    // pitch 20도(정면이 위로 들림): 정면 방향에서 ENU 고각 - 20도가 안테나 고각이다.
    const beam::RequestCoverage coverage = makeCoverage(0_deg, 20_deg, 0_deg);

    // ENU 고각 5도 → 안테나 -15도: 밖
    EXPECT_FALSE(coverage.contains(direction(0_deg, 5_deg)));
    // ENU 고각 15도 → 안테나 -5도: 안
    EXPECT_TRUE(coverage.contains(direction(0_deg, 15_deg)));
    // ENU 고각 75도 → 안테나 55도: ENU 기준이면 밖이지만 안테나 기준이면 안이다.
    EXPECT_TRUE(coverage.contains(direction(0_deg, 75_deg)));
}
