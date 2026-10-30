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
        beam::RadarAttitude(roll, math::TiltAngle(pitch), yaw),
        beam::AttitudeConfig(
            89_deg,
            math::SquareMatrix::identity(3),
            math::Tolerance{1e-6, 0.0}));
}

// 기본 추적 영역: 안테나 기준 방위각 -60~60도, 고각 -10~60도
beam::RequestCoverageConfig makeDefaultConfig()
{
    return beam::RequestCoverageConfig(math::CircularAngleRange(-60_deg, 60_deg), math::AngleRange(-10_deg, 60_deg));
}

beam::RequestCoverage makeCoverage(math::Angle roll, math::Angle pitch, math::Angle yaw)
{
    return beam::RequestCoverage(makeTransform(roll, pitch, yaw), makeDefaultConfig());
}

math::SphericalDirection direction(math::Angle azimuth, math::Angle elevation)
{
    return math::SphericalDirection(azimuth, elevation);
}

} // namespace

// ---------- 영 자세: ENU와 안테나 기준이 같다 ----------

// 영역 판정은 경계를 가로지르는 범위와 넓은 원형 범위를 구분한다.
TEST(RequestCoverageTest, SupportsWrappedAndWideCircularRanges)
{
    // Given: 검사할 자세와 요청 영역 설정을 준비한다.
    const beam::RequestCoverage narrow(
        makeTransform(0_deg, 0_deg, 0_deg),
        beam::RequestCoverageConfig(math::CircularAngleRange(350_deg, 10_deg), math::AngleRange(-10_deg, 60_deg)));
    const beam::RequestCoverage wide(
        makeTransform(0_deg, 0_deg, 0_deg),
        beam::RequestCoverageConfig(math::CircularAngleRange(60_deg, -60_deg), math::AngleRange(-10_deg, 60_deg)));

    // When: 각 입력 방향이 요청 영역에 포함되는지 판정한다.
    const bool narrowContainsFront = narrow.contains(direction(0_deg, 10_deg));
    const bool narrowContainsRear = narrow.contains(direction(180_deg, 10_deg));

    const bool wideContainsRear = wide.contains(direction(180_deg, 10_deg));
    const bool wideContainsFront = wide.contains(direction(0_deg, 10_deg));

    // Then: 영역 판정은 경계를 가로지르는 범위와 넓은 원형 범위를 구분한다.
    EXPECT_TRUE(narrowContainsFront);
    EXPECT_FALSE(narrowContainsRear);
    EXPECT_TRUE(wideContainsRear);
    EXPECT_FALSE(wideContainsFront);
}

// 영 자세의 영역 판정은 방위각과 고각의 경계를 포함한다.
TEST(RequestCoverageTest, ZeroAttitudeIncludesBoundaries)
{
    // Given: 검사할 자세와 요청 영역 설정을 준비한다.
    const beam::RequestCoverage coverage = makeCoverage(0_deg, 0_deg, 0_deg);

    // When: 각 입력 방향이 요청 영역에 포함되는지 판정한다.
    const bool containsMinAzimuth = coverage.contains(direction(-60_deg, 0_deg));
    const bool containsMaxAzimuth = coverage.contains(direction(60_deg, 0_deg));
    const bool containsMinElevation = coverage.contains(direction(0_deg, -10_deg));
    const bool containsMaxElevation = coverage.contains(direction(0_deg, 60_deg));
    const bool containsLowerCorner = coverage.contains(direction(-60_deg, -10_deg));
    const bool containsUpperCorner = coverage.contains(direction(60_deg, 60_deg));

    // Then: 영 자세의 영역 판정은 방위각과 고각의 경계를 포함한다.
    EXPECT_TRUE(containsMinAzimuth);
    EXPECT_TRUE(containsMaxAzimuth);
    EXPECT_TRUE(containsMinElevation);
    EXPECT_TRUE(containsMaxElevation);
    EXPECT_TRUE(containsLowerCorner);
    EXPECT_TRUE(containsUpperCorner);
}

// 영 자세의 영역 판정은 경계를 조금 벗어난 방향을 거부한다.
TEST(RequestCoverageTest, ZeroAttitudeRejectsJustOutsideBoundaries)
{
    // Given: 검사할 자세와 요청 영역 설정을 준비한다.
    const beam::RequestCoverage coverage = makeCoverage(0_deg, 0_deg, 0_deg);

    // When: 각 입력 방향이 요청 영역에 포함되는지 판정한다.
    const bool containsBelowMinAzimuth = coverage.contains(direction(-60.1_deg, 0_deg));
    const bool containsAboveMaxAzimuth = coverage.contains(direction(60.1_deg, 0_deg));
    const bool containsBelowMinElevation = coverage.contains(direction(0_deg, -10.1_deg));
    const bool containsAboveMaxElevation = coverage.contains(direction(0_deg, 60.1_deg));

    // Then: 영 자세의 영역 판정은 경계를 조금 벗어난 방향을 거부한다.
    EXPECT_FALSE(containsBelowMinAzimuth);
    EXPECT_FALSE(containsAboveMaxAzimuth);
    EXPECT_FALSE(containsBelowMinElevation);
    EXPECT_FALSE(containsAboveMaxElevation);
}

// 영역 판정은 0도 부근과 한 바퀴 경계의 방향을 처리한다.
TEST(RequestCoverageTest, AzimuthNearZeroAndFullTurnIsInside)
{
    // Given: 검사할 자세와 요청 영역 설정을 준비한다.
    const beam::RequestCoverage coverage = makeCoverage(0_deg, 0_deg, 0_deg);

    // CircularAngle은 [0, 360)으로 정규화되므로 -0.1도는 359.9도로 들어온다.

    // When: 각 입력 방향이 요청 영역에 포함되는지 판정한다.
    const bool containsZero = coverage.contains(direction(0_deg, 10_deg));
    const bool containsBelowFullTurn = coverage.contains(direction(359.9_deg, 10_deg));
    const bool containsBelowZero = coverage.contains(direction(-0.1_deg, 10_deg));
    const bool containsWrappedBoundary = coverage.contains(direction(300_deg, 10_deg));
    const bool containsOutsideBoundary = coverage.contains(direction(299.9_deg, 10_deg));

    // Then: 영역 판정은 0도 부근과 한 바퀴 경계의 방향을 처리한다.
    EXPECT_TRUE(containsZero);
    EXPECT_TRUE(containsBelowFullTurn);
    EXPECT_TRUE(containsBelowZero);
    EXPECT_TRUE(containsWrappedBoundary);
    EXPECT_FALSE(containsOutsideBoundary);
}

// 기본 영역 판정은 안테나의 옆과 뒤 방향을 거부한다.
TEST(RequestCoverageTest, BehindTheAntennaIsOutside)
{
    // Given: 검사할 자세와 요청 영역 설정을 준비한다.
    const beam::RequestCoverage coverage = makeCoverage(0_deg, 0_deg, 0_deg);

    // When: 각 입력 방향이 요청 영역에 포함되는지 판정한다.
    const bool containsRear = coverage.contains(direction(180_deg, 10_deg));
    const bool containsRight = coverage.contains(direction(90_deg, 10_deg));
    const bool containsLeft = coverage.contains(direction(270_deg, 10_deg));

    // Then: 기본 영역 판정은 안테나의 옆과 뒤 방향을 거부한다.
    EXPECT_FALSE(containsRear);
    EXPECT_FALSE(containsRight);
    EXPECT_FALSE(containsLeft);
}

// 0도 부근의 좁은 영역은 두 방위각 경계를 포함한다.
TEST(RequestCoverageTest, CoverageAroundZeroAzimuthWorks)
{
    // Given: 검사할 자세와 요청 영역 설정을 준비한다.
    const beam::RequestCoverage coverage(
        makeTransform(0_deg, 0_deg, 0_deg),
        beam::RequestCoverageConfig(math::CircularAngleRange(-10_deg, 10_deg), math::AngleRange(0_deg, 20_deg)));

    // When: 각 입력 방향이 요청 영역에 포함되는지 판정한다.
    const bool containsMinAzimuth = coverage.contains(direction(350_deg, 5_deg));
    const bool containsMaxAzimuth = coverage.contains(direction(10_deg, 5_deg));
    const bool containsBelowMinAzimuth = coverage.contains(direction(349_deg, 5_deg));
    const bool containsAboveMaxAzimuth = coverage.contains(direction(11_deg, 5_deg));

    // Then: 0도 부근의 좁은 영역은 두 방위각 경계를 포함한다.
    EXPECT_TRUE(containsMinAzimuth);
    EXPECT_TRUE(containsMaxAzimuth);
    EXPECT_FALSE(containsBelowMinAzimuth);
    EXPECT_FALSE(containsAboveMaxAzimuth);
}

// 전체 영역은 자세와 무관하게 모든 방향을 포함한다.
TEST(RequestCoverageTest, FullCoverageContainsEveryDirection)
{
    // Given: 검사할 자세와 요청 영역 설정을 준비한다.
    const beam::RequestCoverage coverage(
        makeTransform(10_deg, 20_deg, 30_deg),
        beam::RequestCoverageConfig(math::CircularAngleRange(-180_deg, 180_deg), math::AngleRange(-90_deg, 90_deg)));

    // When: 각 입력 방향이 요청 영역에 포함되는지 판정한다.
    const bool containsFront = coverage.contains(direction(0_deg, 0_deg));
    const bool containsRear = coverage.contains(direction(180_deg, 0_deg));
    const bool containsUp = coverage.contains(direction(210_deg, -80_deg));
    const bool containsDown = coverage.contains(direction(45_deg, 90_deg));

    // Then: 전체 영역은 자세와 무관하게 모든 방향을 포함한다.
    EXPECT_TRUE(containsFront);
    EXPECT_TRUE(containsRear);
    EXPECT_TRUE(containsUp);
    EXPECT_TRUE(containsDown);
}

// ---------- 자세가 있으면 ENU가 아닌 안테나 기준으로 판정한다 ----------

// 영역 판정은 요가 반영된 안테나 좌표에서 방위각을 검사한다.
TEST(RequestCoverageTest, YawShiftsAzimuthToAntennaFrame)
{
    // Given: 검사할 자세와 요청 영역 설정을 준비한다.
    // 샘플 시나리오 자세(yaw 13.02429도). ENU 방위각에서 yaw를 뺀 값이 안테나 방위각이다(roll·pitch 0, 고각 0).
    const beam::RequestCoverage coverage = makeCoverage(0_deg, 0_deg, 13.02429_deg);

    // ENU 75도 → 안테나 약 61.98도: ENU로는 영역 밖이지만 안테나 기준으로도 밖이다.

    // When: 각 입력 방향이 요청 영역에 포함되는지 판정한다.
    const bool containsBeyondRightEdge = coverage.contains(direction(75_deg, 0_deg));
    // ENU 70도 → 안테나 약 56.98도: ENU 기준(60도 이하)과 안테나 기준 모두 안이다.
    const bool containsInsideRightEdge = coverage.contains(direction(70_deg, 0_deg));
    // ENU -50도 → 안테나 약 -63.02도: ENU 기준이면 안이지만 안테나 기준이면 밖이다.
    const bool containsBeyondLeftEdge = coverage.contains(direction(-50_deg, 0_deg));
    // ENU 72도 → 안테나 약 58.98도: ENU 기준이면 밖이지만 안테나 기준이면 안이다.
    const bool containsRotatedInside = coverage.contains(direction(72_deg, 0_deg));

    // Then: 영역 판정은 요가 반영된 안테나 좌표에서 방위각을 검사한다.
    EXPECT_FALSE(containsBeyondRightEdge);
    EXPECT_TRUE(containsInsideRightEdge);
    EXPECT_FALSE(containsBeyondLeftEdge);
    EXPECT_TRUE(containsRotatedInside);
}

// 영역 판정은 피치가 반영된 안테나 좌표에서 고각을 검사한다.
TEST(RequestCoverageTest, PitchShiftsElevationToAntennaFrame)
{
    // Given: 검사할 자세와 요청 영역 설정을 준비한다.
    // pitch 20도(정면이 위로 들림): 정면 방향에서 ENU 고각 - 20도가 안테나 고각이다.
    const beam::RequestCoverage coverage = makeCoverage(0_deg, 20_deg, 0_deg);

    // ENU 고각 5도 → 안테나 -15도: 밖

    // When: 각 입력 방향이 요청 영역에 포함되는지 판정한다.
    const bool containsBelowLowerEdge = coverage.contains(direction(0_deg, 5_deg));
    // ENU 고각 15도 → 안테나 -5도: 안
    const bool containsAboveLowerEdge = coverage.contains(direction(0_deg, 15_deg));
    // ENU 고각 75도 → 안테나 55도: ENU 기준이면 밖이지만 안테나 기준이면 안이다.
    const bool containsRotatedUpperDirection = coverage.contains(direction(0_deg, 75_deg));

    // Then: 영역 판정은 피치가 반영된 안테나 좌표에서 고각을 검사한다.
    EXPECT_FALSE(containsBelowLowerEdge);
    EXPECT_TRUE(containsAboveLowerEdge);
    EXPECT_TRUE(containsRotatedUpperDirection);
}

// ---------- 후방 경계(±180도)는 같은 방향이다 ----------

// 후방에서 끝나는 영역은 동치인 두 후방 방위각을 포함한다.
TEST(RequestCoverageTest, CoverageEndingAtRearBoundaryIncludesHalfTurn)
{
    // Given: 검사할 자세와 요청 영역 설정을 준비한다.
    const beam::RequestCoverage coverage(
        makeTransform(0_deg, 0_deg, 0_deg),
        beam::RequestCoverageConfig(math::CircularAngleRange(170_deg, 180_deg), math::AngleRange(0_deg, 20_deg)));

    // 180도와 -180도는 같은 방향이므로 끝 경계에 포함된다.

    // When: 각 입력 방향이 요청 영역에 포함되는지 판정한다.
    const bool containsPositiveHalfTurn = coverage.contains(direction(180_deg, 5_deg));
    const bool containsNegativeHalfTurn = coverage.contains(direction(-180_deg, 5_deg));
    const bool containsMiddle = coverage.contains(direction(175_deg, 5_deg));
    const bool containsStart = coverage.contains(direction(170_deg, 5_deg));
    const bool containsBeforeStart = coverage.contains(direction(169_deg, 5_deg));
    // 181도는 -179도이므로 영역 밖이다.
    const bool containsAfterEnd = coverage.contains(direction(181_deg, 5_deg));

    // Then: 후방에서 끝나는 영역은 동치인 두 후방 방위각을 포함한다.
    EXPECT_TRUE(containsPositiveHalfTurn);
    EXPECT_TRUE(containsNegativeHalfTurn);
    EXPECT_TRUE(containsMiddle);
    EXPECT_TRUE(containsStart);
    EXPECT_FALSE(containsBeforeStart);
    EXPECT_FALSE(containsAfterEnd);
}

// 후방에서 시작하는 영역은 동치인 두 후방 방위각을 포함한다.
TEST(RequestCoverageTest, CoverageStartingAtRearBoundaryIncludesHalfTurn)
{
    // Given: 검사할 자세와 요청 영역 설정을 준비한다.
    const beam::RequestCoverage coverage(
        makeTransform(0_deg, 0_deg, 0_deg),
        beam::RequestCoverageConfig(math::CircularAngleRange(-180_deg, -170_deg), math::AngleRange(0_deg, 20_deg)));

    // When: 각 입력 방향이 요청 영역에 포함되는지 판정한다.
    const bool containsPositiveHalfTurn = coverage.contains(direction(180_deg, 5_deg));
    const bool containsNegativeHalfTurn = coverage.contains(direction(-180_deg, 5_deg));
    // 185도 = -175도, 190도 = -170도
    const bool containsMiddle = coverage.contains(direction(185_deg, 5_deg));
    const bool containsEnd = coverage.contains(direction(190_deg, 5_deg));
    // 191도 = -169도
    const bool containsAfterEnd = coverage.contains(direction(191_deg, 5_deg));
    const bool containsBeforeStart = coverage.contains(direction(179_deg, 5_deg));

    // Then: 후방에서 시작하는 영역은 동치인 두 후방 방위각을 포함한다.
    EXPECT_TRUE(containsPositiveHalfTurn);
    EXPECT_TRUE(containsNegativeHalfTurn);
    EXPECT_TRUE(containsMiddle);
    EXPECT_TRUE(containsEnd);
    EXPECT_FALSE(containsAfterEnd);
    EXPECT_FALSE(containsBeforeStart);
}

// 후방 경계 판정에도 요를 반영한 안테나 좌표를 사용한다.
TEST(RequestCoverageTest, RearBoundaryUsesAntennaFrameAfterYaw)
{
    // Given: 검사할 자세와 요청 영역 설정을 준비한다.
    // yaw 30도: ENU 방위각 210도가 안테나 기준 180도(후방)이다.
    const beam::RequestCoverage coverage(
        makeTransform(0_deg, 0_deg, 30_deg),
        beam::RequestCoverageConfig(math::CircularAngleRange(170_deg, 180_deg), math::AngleRange(-10_deg, 20_deg)));

    // When: 각 입력 방향이 요청 영역에 포함되는지 판정한다.
    const bool containsRotatedRear = coverage.contains(direction(210_deg, 0_deg));
    const bool containsRotatedMiddle = coverage.contains(direction(205_deg, 0_deg));
    // ENU 211도 = 안테나 181도(-179도): 밖
    const bool containsBeyondRotatedRear = coverage.contains(direction(211_deg, 0_deg));

    // Then: 후방 경계 판정에도 요를 반영한 안테나 좌표를 사용한다.
    EXPECT_TRUE(containsRotatedRear);
    EXPECT_TRUE(containsRotatedMiddle);
    EXPECT_FALSE(containsBeyondRotatedRear);
}
