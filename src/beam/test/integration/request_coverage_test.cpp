#include "request/request_coverage.hpp"

#include <pose/domain/ant_enu_transform.hpp>
#include <pose/domain/factory/radar_attitude_factory.hpp>

#include <gtest/gtest.h>

using namespace math::literals;

namespace
{

pose::AntEnuTransform makeTransform(math::Angle roll, math::Angle pitch, math::Angle yaw)
{
    const pose::RadarAttitudeFactory factory(pose::RadarAttitudeFactoryConfig(89_deg));
    return pose::AntEnuTransform(
        factory.create(roll, math::TiltAngle(pitch), yaw),
        pose::AntEnuTransformConfig(math::SquareMatrix::identity(3)));
}

// 기본 추적 영역: 안테나 기준 방위각 -60~60도, 고각 -10~60도
beam::RequestCoverageConfig::Builder validConfigBuilder()
{
    return beam::RequestCoverageConfig::Builder()
        .azimuthRange(math::CircularAngleRange(-60_deg, 60_deg))
        .elevationRange(math::AngleRange(-10_deg, 60_deg));
}

beam::RequestCoverage makeCoverage(math::Angle roll, math::Angle pitch, math::Angle yaw)
{
    return beam::RequestCoverage(makeTransform(roll, pitch, yaw), validConfigBuilder().build());
}

math::SphericalDirection direction(math::Angle azimuth, math::Angle elevation)
{
    return math::SphericalDirection(azimuth, elevation);
}

} // namespace

// 기본 영역 판정은 안테나의 옆과 뒤 방향을 거부한다.
TEST(RequestCoverageTest, BehindTheAntennaIsOutside)
{
    // Given: 검사할 자세와 요청 영역 설정을 준비한다.
    const auto coverage = makeCoverage(0_deg, 0_deg, 0_deg);

    // When: 각 입력 방향이 요청 영역에 포함되는지 판정한다.
    const auto containsRear = coverage.contains(direction(180_deg, 10_deg));
    const auto containsRight = coverage.contains(direction(90_deg, 10_deg));
    const auto containsLeft = coverage.contains(direction(270_deg, 10_deg));

    // Then: 기본 영역 판정은 안테나의 옆과 뒤 방향을 거부한다.
    EXPECT_FALSE(containsRear);
    EXPECT_FALSE(containsRight);
    EXPECT_FALSE(containsLeft);
}

// 영역 판정은 요가 반영된 안테나 좌표에서 방위각을 검사한다.
TEST(RequestCoverageTest, YawShiftsAzimuthToAntennaFrame)
{
    // Given: 검사할 자세와 요청 영역 설정을 준비한다.
    // 샘플 시나리오 자세(yaw 13.02429도). ENU 방위각에서 yaw를 뺀 값이 안테나 방위각이다(roll·pitch 0, 고각 0).
    const auto coverage = makeCoverage(0_deg, 0_deg, 13.02429_deg);

    // ENU 75도 → 안테나 약 61.98도: ENU로는 영역 밖이지만 안테나 기준으로도 밖이다.

    // When: 각 입력 방향이 요청 영역에 포함되는지 판정한다.
    const auto containsBeyondRightEdge = coverage.contains(direction(75_deg, 0_deg));
    // ENU 70도 → 안테나 약 56.98도: ENU 기준(60도 이하)과 안테나 기준 모두 안이다.
    const auto containsInsideRightEdge = coverage.contains(direction(70_deg, 0_deg));
    // ENU -50도 → 안테나 약 -63.02도: ENU 기준이면 안이지만 안테나 기준이면 밖이다.
    const auto containsBeyondLeftEdge = coverage.contains(direction(-50_deg, 0_deg));
    // ENU 72도 → 안테나 약 58.98도: ENU 기준이면 밖이지만 안테나 기준이면 안이다.
    const auto containsRotatedInside = coverage.contains(direction(72_deg, 0_deg));

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
    const auto coverage = makeCoverage(0_deg, 20_deg, 0_deg);

    // ENU 고각 5도 → 안테나 -15도: 밖

    // When: 각 입력 방향이 요청 영역에 포함되는지 판정한다.
    const auto containsBelowLowerEdge = coverage.contains(direction(0_deg, 5_deg));
    // ENU 고각 15도 → 안테나 -5도: 안
    const auto containsAboveLowerEdge = coverage.contains(direction(0_deg, 15_deg));
    // ENU 고각 75도 → 안테나 55도: ENU 기준이면 밖이지만 안테나 기준이면 안이다.
    const auto containsRotatedUpperDirection = coverage.contains(direction(0_deg, 75_deg));

    // Then: 영역 판정은 피치가 반영된 안테나 좌표에서 고각을 검사한다.
    EXPECT_FALSE(containsBelowLowerEdge);
    EXPECT_TRUE(containsAboveLowerEdge);
    EXPECT_TRUE(containsRotatedUpperDirection);
}

// 후방 경계 판정에도 요를 반영한 안테나 좌표를 사용한다.
TEST(RequestCoverageTest, RearBoundaryUsesAntennaFrameAfterYaw)
{
    // Given: 검사할 자세와 요청 영역 설정을 준비한다.
    // yaw 30도: ENU 방위각 210도가 안테나 기준 180도(후방)이다.
    const auto coverage = beam::RequestCoverage(
        makeTransform(0_deg, 0_deg, 30_deg),
        beam::RequestCoverageConfig(math::CircularAngleRange(170_deg, 180_deg), math::AngleRange(-10_deg, 20_deg)));

    // When: 각 입력 방향이 요청 영역에 포함되는지 판정한다.
    const auto containsRotatedRear = coverage.contains(direction(210_deg, 0_deg));
    const auto containsRotatedMiddle = coverage.contains(direction(205_deg, 0_deg));
    // ENU 211도 = 안테나 181도(-179도): 밖
    const auto containsBeyondRotatedRear = coverage.contains(direction(211_deg, 0_deg));

    // Then: 후방 경계 판정에도 요를 반영한 안테나 좌표를 사용한다.
    EXPECT_TRUE(containsRotatedRear);
    EXPECT_TRUE(containsRotatedMiddle);
    EXPECT_FALSE(containsBeyondRotatedRear);
}
