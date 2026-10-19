#include "search_beam_table.hpp"

#include "attitude/ant_enu_transform.hpp"
#include "attitude/attitude_error.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>

using namespace math::literals;

namespace
{

constexpr std::size_t AZIMUTH_COUNT = 21;
constexpr std::size_t ELEVATION_COUNT = 9;

const beam::SearchGridConfig DEFAULT_GRID_CONFIG{};

beam::AntEnuTransform makeValidTransform(math::Angle roll, math::Angle pitch, math::Angle yaw)
{
    const beam::RadarAttitude attitude(37.0_deg, 127.0_deg, 0.1_km, roll, pitch, yaw);

    return beam::AntEnuTransform(attitude, beam::AttitudeConfig{});
}

math::Angle gridAzimuth(std::size_t beamIndex)
{
    return -42_deg + 4.2_deg * static_cast<double>(beamIndex % AZIMUTH_COUNT);
}
math::Angle gridElevation(std::size_t beamIndex)
{
    return 3_deg + 4.4_deg * static_cast<double>(beamIndex / AZIMUTH_COUNT);
}

} // namespace

TEST(SearchBeamTableTest, DefaultConfigHasExpectedBeamCount)
{
    const beam::SearchBeamTable table(makeValidTransform(0_deg, 0_deg, 0_deg), DEFAULT_GRID_CONFIG);

    EXPECT_EQ(table.size(), 189u);
    EXPECT_EQ(AZIMUTH_COUNT * ELEVATION_COUNT, table.size());
}

TEST(SearchBeamTableTest, BeamIdsAreSequentialFromOne)
{
    const beam::SearchBeamTable table(makeValidTransform(10_deg, 20_deg, 30_deg), DEFAULT_GRID_CONFIG);

    for (std::size_t beamIndex = 0; beamIndex < table.size(); ++beamIndex)
    {
        EXPECT_EQ(table.get(beamIndex).beamId, static_cast<uint32_t>(beamIndex + 1));
    }
}

TEST(SearchBeamTableTest, BeamWidthIsSixDegrees)
{
    const beam::SearchBeamTable table(makeValidTransform(10_deg, 20_deg, 30_deg), DEFAULT_GRID_CONFIG);

    for (std::size_t beamIndex = 0; beamIndex < table.size(); ++beamIndex)
    {
        EXPECT_EQ(table.get(beamIndex).azimuthBeamWidth, 6_deg);
        EXPECT_EQ(table.get(beamIndex).elevationBeamWidth, 6_deg);
    }
}

TEST(SearchBeamTableTest, ZeroAttitudeKeepsFixedGrid)
{
    const beam::SearchBeamTable table(makeValidTransform(0_deg, 0_deg, 0_deg), DEFAULT_GRID_CONFIG);

    for (std::size_t beamIndex = 0; beamIndex < table.size(); ++beamIndex)
    {
        EXPECT_EQ(table.get(beamIndex).direction_ant.azimuth, gridAzimuth(beamIndex)) << "beamIndex=" << beamIndex;
        EXPECT_EQ(table.get(beamIndex).direction_ant.elevation, gridElevation(beamIndex))
            << "beamIndex=" << beamIndex;
    }
}

TEST(SearchBeamTableTest, GridOrderStartsLowElevationLeftAzimuth)
{
    const beam::SearchBeamTable table(makeValidTransform(0_deg, 0_deg, 0_deg), DEFAULT_GRID_CONFIG);

    // 첫 빔: 가장 낮은 고각, 가장 왼쪽 방위각
    EXPECT_EQ(table.get(0).direction_ant.azimuth, -42_deg);
    EXPECT_EQ(table.get(0).direction_ant.elevation, 3_deg);
    // 같은 고각의 마지막 빔: 가장 오른쪽 방위각
    EXPECT_EQ(table.get(20).direction_ant.azimuth, 42_deg);
    EXPECT_EQ(table.get(20).direction_ant.elevation, 3_deg);
    // 다음 고각 줄의 첫 빔
    EXPECT_EQ(table.get(21).direction_ant.azimuth, -42_deg);
    EXPECT_EQ(table.get(21).direction_ant.elevation, 7.4_deg);
    // 마지막 빔: 가장 높은 고각, 가장 오른쪽 방위각
    EXPECT_EQ(table.get(188).direction_ant.azimuth, 42_deg);
    EXPECT_EQ(table.get(188).direction_ant.elevation, 38.2_deg);
}

TEST(SearchBeamTableTest, YawShiftsAzimuthsByOppositeAngle)
{
    // 안테나가 yaw 만큼 돌아가 있으면 고정 그리드는 안테나 기준으로 -yaw 만큼 이동
    const beam::SearchBeamTable table(makeValidTransform(0_deg, 0_deg, 10_deg), DEFAULT_GRID_CONFIG);

    for (std::size_t beamIndex = 0; beamIndex < table.size(); ++beamIndex)
    {
        EXPECT_EQ(table.get(beamIndex).direction_ant.azimuth, gridAzimuth(beamIndex) - 10_deg)
            << "beamIndex=" << beamIndex;
        EXPECT_EQ(table.get(beamIndex).direction_ant.elevation, gridElevation(beamIndex))
            << "beamIndex=" << beamIndex;
    }
}

TEST(SearchBeamTableTest, TableMatchesAnglesConvertedToAntFrame)
{
    const beam::AntEnuTransform transform = makeValidTransform(10_deg, 20_deg, 30_deg);
    const beam::SearchBeamTable table(transform, DEFAULT_GRID_CONFIG);

    for (std::size_t beamIndex = 0; beamIndex < table.size(); ++beamIndex)
    {
        const math::SphericalDirection direction_ant = transform.enuToAnt(
            math::SphericalDirection(gridAzimuth(beamIndex), gridElevation(beamIndex)));
        EXPECT_EQ(table.get(beamIndex).direction_ant.azimuth, direction_ant.azimuth) << "beamIndex=" << beamIndex;
        EXPECT_EQ(table.get(beamIndex).direction_ant.elevation, direction_ant.elevation) << "beamIndex=" << beamIndex;
    }
}

TEST(SearchBeamTableTest, TableConvertedBackToFixedFrameRestoresGrid)
{
    const beam::AntEnuTransform transform = makeValidTransform(10_deg, 20_deg, 30_deg);
    const beam::SearchBeamTable table(transform, DEFAULT_GRID_CONFIG);

    for (std::size_t beamIndex = 0; beamIndex < table.size(); ++beamIndex)
    {
        const math::SphericalDirection direction_enu = transform.antToEnu(table.get(beamIndex).direction_ant);
        EXPECT_EQ(direction_enu.azimuth, math::wrap360(gridAzimuth(beamIndex))) << "beamIndex=" << beamIndex;
        EXPECT_EQ(direction_enu.elevation, gridElevation(beamIndex)) << "beamIndex=" << beamIndex;
    }
}

TEST(SearchBeamTableTest, DifferentAttitudeProducesDifferentTable)
{
    const beam::SearchBeamTable zeroAttitudeTable(makeValidTransform(0_deg, 0_deg, 0_deg), DEFAULT_GRID_CONFIG);
    const beam::SearchBeamTable yawRotatedTable(makeValidTransform(0_deg, 0_deg, 20_deg), DEFAULT_GRID_CONFIG);

    bool anyDifferent = false;
    for (std::size_t beamIndex = 0; beamIndex < zeroAttitudeTable.size(); ++beamIndex)
    {
        if (!zeroAttitudeTable.get(beamIndex).direction_ant.azimuth.equals(yawRotatedTable.get(beamIndex).direction_ant.azimuth))
        {
            anyDifferent = true;
            break;
        }
    }
    EXPECT_TRUE(anyDifferent);
}

TEST(SearchBeamTableTest, GetThrowsOnOutOfRangeIndex)
{
    const beam::SearchBeamTable table(makeValidTransform(0_deg, 0_deg, 0_deg), DEFAULT_GRID_CONFIG);

    EXPECT_NO_THROW(table.get(188));
    EXPECT_THROW(table.get(189), std::out_of_range);
}

// ---------- 설정 반영 ----------

TEST(SearchBeamTableTest, CustomGridChangesBeamCountAndIds)
{
    const beam::SearchGridConfig config = beam::SearchGridConfig::Builder()
        .azimuthCount(3)
        .elevationCount(2)
        .build();

    const beam::SearchBeamTable table(makeValidTransform(0_deg, 0_deg, 0_deg), config);

    ASSERT_EQ(table.size(), 6u);
    for (std::size_t beamIndex = 0; beamIndex < table.size(); ++beamIndex)
    {
        EXPECT_EQ(table.get(beamIndex).beamId, static_cast<uint32_t>(beamIndex + 1));
    }
    EXPECT_THROW(table.get(6), std::out_of_range);
}

TEST(SearchBeamTableTest, CustomGridUsesConfiguredStartStepAndWidth)
{
    const beam::SearchGridConfig config = beam::SearchGridConfig::Builder()
        .azimuthCount(3)
        .elevationCount(2)
        .azimuthStart(-10_deg)
        .azimuthStep(5_deg)
        .elevationStart(1_deg)
        .elevationStep(2_deg)
        .beamWidth(4_deg)
        .build();

    const beam::SearchBeamTable table(makeValidTransform(0_deg, 0_deg, 0_deg), config);

    // 방위각 -10, -5, 0 / 고각 1, 3 (고각 바깥 루프, 방위각 안쪽 루프 순서)
    const math::Angle expectedAzimuth[] = {-10_deg, -5_deg, 0_deg, -10_deg, -5_deg, 0_deg};
    const math::Angle expectedElevation[] = {1_deg, 1_deg, 1_deg, 3_deg, 3_deg, 3_deg};

    ASSERT_EQ(table.size(), 6u);
    for (std::size_t beamIndex = 0; beamIndex < table.size(); ++beamIndex)
    {
        EXPECT_EQ(table.get(beamIndex).direction_ant.azimuth, expectedAzimuth[beamIndex])
            << "beamIndex=" << beamIndex;
        EXPECT_EQ(table.get(beamIndex).direction_ant.elevation, expectedElevation[beamIndex])
            << "beamIndex=" << beamIndex;
        EXPECT_EQ(table.get(beamIndex).azimuthBeamWidth, 4_deg);
        EXPECT_EQ(table.get(beamIndex).elevationBeamWidth, 4_deg);
    }
}

// 설정 값 검증(개수, 빔폭, 유한성, 오버플로 등)은 config_test.cpp 에서 확인한다.

TEST(SearchBeamTableTest, LargeStepWithSingleBeamPerAxisIsAccepted)
{
    const auto maxAngle = math::Angle::fromDegrees(std::numeric_limits<double>::max());

    // count가 1이면 step은 쓰이지 않으므로 큰 step도 문제없다.
    const beam::SearchGridConfig config = beam::SearchGridConfig::Builder()
        .azimuthCount(1)
        .azimuthStep(maxAngle)
        .elevationCount(1)
        .elevationStep(maxAngle)
        .build();

    const beam::SearchBeamTable table(makeValidTransform(0_deg, 0_deg, 0_deg), config);

    ASSERT_EQ(table.size(), 1u);
}

TEST(SearchBeamTableTest, LargeAzimuthStartWithSingleBeamPreservesWrappedDirection)
{
    const beam::SearchGridConfig config = beam::SearchGridConfig::Builder()
        .azimuthCount(1)
        .elevationCount(1)
        .azimuthStart(math::Angle::fromDegrees(1e18))
        .build();

    const beam::SearchBeamTable table(makeValidTransform(0_deg, 0_deg, 0_deg), config);

    ASSERT_EQ(table.size(), 1u);
    EXPECT_EQ(table.get(0).direction_ant.azimuth, -80_deg);
}

// ---------- 탐색 커서 ----------

TEST(SearchBeamTableTest, NextBeamFollowsTableOrderFromTheFirstBeam)
{
    beam::SearchBeamTable table(makeValidTransform(10_deg, 20_deg, 30_deg), DEFAULT_GRID_CONFIG);

    for (std::size_t beamIndex = 0; beamIndex < table.size(); ++beamIndex)
    {
        const beam::SearchBeam& beam = table.nextBeam();

        EXPECT_EQ(beam.beamId, table.get(beamIndex).beamId) << "beamIndex=" << beamIndex;
        EXPECT_EQ(beam.direction_ant.azimuth, table.get(beamIndex).direction_ant.azimuth);
        EXPECT_EQ(beam.direction_ant.elevation, table.get(beamIndex).direction_ant.elevation);
    }
}

TEST(SearchBeamTableTest, NextBeamWrapsToTheFirstBeamAfterTheLastOne)
{
    beam::SearchBeamTable table(makeValidTransform(0_deg, 0_deg, 0_deg), DEFAULT_GRID_CONFIG);
    for (std::size_t beamIndex = 0; beamIndex < table.size(); ++beamIndex)
    {
        table.nextBeam();
    }

    EXPECT_EQ(table.nextBeam().beamId, 1u);
    EXPECT_EQ(table.nextBeam().beamId, 2u);
}

TEST(SearchBeamTableTest, NextBeamCursorDoesNotAffectGetAndSize)
{
    beam::SearchBeamTable table(makeValidTransform(0_deg, 0_deg, 0_deg), DEFAULT_GRID_CONFIG);
    table.nextBeam();
    table.nextBeam();

    EXPECT_EQ(table.size(), 189u);
    EXPECT_EQ(table.get(0).beamId, 1u);
    EXPECT_EQ(table.nextBeam().beamId, 3u);
}

TEST(SearchBeamTableTest, SingleBeamTableReturnsTheSameBeamEveryTime)
{
    const beam::SearchGridConfig config = beam::SearchGridConfig::Builder()
        .azimuthCount(1)
        .elevationCount(1)
        .build();
    beam::SearchBeamTable table(makeValidTransform(0_deg, 0_deg, 0_deg), config);

    EXPECT_EQ(table.nextBeam().beamId, 1u);
    EXPECT_EQ(table.nextBeam().beamId, 1u);
    EXPECT_EQ(table.nextBeam().beamId, 1u);
}

TEST(SearchBeamTableTest, CopiedTableKeepsItsOwnCursor)
{
    beam::SearchBeamTable table(makeValidTransform(0_deg, 0_deg, 0_deg), DEFAULT_GRID_CONFIG);
    table.nextBeam();
    beam::SearchBeamTable copied = table;

    EXPECT_EQ(table.nextBeam().beamId, 2u);
    EXPECT_EQ(table.nextBeam().beamId, 3u);
    EXPECT_EQ(copied.nextBeam().beamId, 2u);
}
