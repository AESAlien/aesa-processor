#include "beam_table.hpp"

#include "attitude/ant_enu_transform.hpp"
#include "attitude/attitude_error.hpp"

#include <gtest/gtest.h>

#include <cstdint>
#include <stdexcept>

using namespace math::literals;

namespace
{

constexpr std::size_t AZIMUTH_COUNT = 21;
constexpr std::size_t ELEVATION_COUNT = 9;

beam::AntEnuTransform makeValidTransform(math::Angle roll, math::Angle pitch, math::Angle yaw)
{
    beam::RadarAttitude attitude{};
    attitude.latitude = 37.0_deg;
    attitude.longitude = 127.0_deg;
    attitude.altitude = 0.1_km;
    attitude.roll = roll;
    attitude.pitch = pitch;
    attitude.yaw = yaw;

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

TEST(BeamTableTest, HasFixedSize)
{
    EXPECT_EQ(beam::BeamTable::size(), 189u);
    EXPECT_EQ(AZIMUTH_COUNT * ELEVATION_COUNT, beam::BeamTable::size());
}

TEST(BeamTableTest, BeamIdsAreSequentialFromOne)
{
    const beam::BeamTable table(makeValidTransform(10_deg, 20_deg, 30_deg));

    for (std::size_t beamIndex = 0; beamIndex < beam::BeamTable::size(); ++beamIndex)
    {
        EXPECT_EQ(table.get(beamIndex).beamId, static_cast<std::uint32_t>(beamIndex + 1));
    }
}

TEST(BeamTableTest, BeamWidthIsSixDegrees)
{
    const beam::BeamTable table(makeValidTransform(10_deg, 20_deg, 30_deg));

    for (std::size_t beamIndex = 0; beamIndex < beam::BeamTable::size(); ++beamIndex)
    {
        EXPECT_DOUBLE_EQ(table.get(beamIndex).azimuthBeamWidth.deg(), 6.0);
        EXPECT_DOUBLE_EQ(table.get(beamIndex).elevationBeamWidth.deg(), 6.0);
    }
}

TEST(BeamTableTest, ZeroAttitudeKeepsFixedGrid)
{
    const beam::BeamTable table(makeValidTransform(0_deg, 0_deg, 0_deg));

    for (std::size_t beamIndex = 0; beamIndex < beam::BeamTable::size(); ++beamIndex)
    {
        EXPECT_NEAR(table.get(beamIndex).azimuth_ant.deg(), gridAzimuth(beamIndex).deg(), 1e-4) << "beamIndex=" << beamIndex;
        EXPECT_NEAR(table.get(beamIndex).elevation_ant.deg(), gridElevation(beamIndex).deg(), 1e-4)
            << "beamIndex=" << beamIndex;
    }
}

TEST(BeamTableTest, GridOrderStartsLowElevationLeftAzimuth)
{
    const beam::BeamTable table(makeValidTransform(0_deg, 0_deg, 0_deg));

    // 첫 빔: 가장 낮은 고각, 가장 왼쪽 방위각
    EXPECT_NEAR(table.get(0).azimuth_ant.deg(), -42.0, 1e-4);
    EXPECT_NEAR(table.get(0).elevation_ant.deg(), 3.0, 1e-4);
    // 같은 고각의 마지막 빔: 가장 오른쪽 방위각
    EXPECT_NEAR(table.get(20).azimuth_ant.deg(), 42.0, 1e-4);
    EXPECT_NEAR(table.get(20).elevation_ant.deg(), 3.0, 1e-4);
    // 다음 고각 줄의 첫 빔
    EXPECT_NEAR(table.get(21).azimuth_ant.deg(), -42.0, 1e-4);
    EXPECT_NEAR(table.get(21).elevation_ant.deg(), 7.4, 1e-4);
    // 마지막 빔: 가장 높은 고각, 가장 오른쪽 방위각
    EXPECT_NEAR(table.get(188).azimuth_ant.deg(), 42.0, 1e-4);
    EXPECT_NEAR(table.get(188).elevation_ant.deg(), 38.2, 1e-4);
}

TEST(BeamTableTest, YawShiftsAzimuthsByOppositeAngle)
{
    // 안테나가 yaw 만큼 돌아가 있으면 고정 그리드는 안테나 기준으로 -yaw 만큼 이동
    const beam::BeamTable table(makeValidTransform(0_deg, 0_deg, 10_deg));

    for (std::size_t beamIndex = 0; beamIndex < beam::BeamTable::size(); ++beamIndex)
    {
        EXPECT_NEAR(table.get(beamIndex).azimuth_ant.deg(), gridAzimuth(beamIndex).deg() - 10.0, 1e-4)
            << "beamIndex=" << beamIndex;
        EXPECT_NEAR(table.get(beamIndex).elevation_ant.deg(), gridElevation(beamIndex).deg(), 1e-4)
            << "beamIndex=" << beamIndex;
    }
}

TEST(BeamTableTest, TableMatchesAnglesConvertedToAntFrame)
{
    const beam::AntEnuTransform transform = makeValidTransform(10_deg, 20_deg, 30_deg);
    const beam::BeamTable table(transform);

    for (std::size_t beamIndex = 0; beamIndex < beam::BeamTable::size(); ++beamIndex)
    {
        const auto [azimuth_ant, elevation_ant] =
            transform.enuToAnt(gridAzimuth(beamIndex), gridElevation(beamIndex));
        EXPECT_NEAR(table.get(beamIndex).azimuth_ant.deg(), azimuth_ant.deg(), 1e-3) << "beamIndex=" << beamIndex;
        EXPECT_NEAR(table.get(beamIndex).elevation_ant.deg(), elevation_ant.deg(), 1e-3) << "beamIndex=" << beamIndex;
    }
}

TEST(BeamTableTest, TableConvertedBackToFixedFrameRestoresGrid)
{
    const beam::AntEnuTransform transform = makeValidTransform(10_deg, 20_deg, 30_deg);
    const beam::BeamTable table(transform);

    for (std::size_t beamIndex = 0; beamIndex < beam::BeamTable::size(); ++beamIndex)
    {
        const auto [azimuth_enu, elevation_enu] =
            transform.antToEnu(table.get(beamIndex).azimuth_ant, table.get(beamIndex).elevation_ant);
        EXPECT_NEAR(azimuth_enu.deg(), gridAzimuth(beamIndex).deg(), 1e-3) << "beamIndex=" << beamIndex;
        EXPECT_NEAR(elevation_enu.deg(), gridElevation(beamIndex).deg(), 1e-3) << "beamIndex=" << beamIndex;
    }
}

TEST(BeamTableTest, DifferentAttitudeProducesDifferentTable)
{
    const beam::BeamTable zeroAttitudeTable(makeValidTransform(0_deg, 0_deg, 0_deg));
    const beam::BeamTable yawRotatedTable(makeValidTransform(0_deg, 0_deg, 20_deg));

    bool anyDifferent = false;
    for (std::size_t beamIndex = 0; beamIndex < beam::BeamTable::size(); ++beamIndex)
    {
        if (zeroAttitudeTable.get(beamIndex).azimuth_ant != yawRotatedTable.get(beamIndex).azimuth_ant)
        {
            anyDifferent = true;
            break;
        }
    }
    EXPECT_TRUE(anyDifferent);
}

TEST(BeamTableTest, GetThrowsOnOutOfRangeIndex)
{
    const beam::BeamTable table(makeValidTransform(0_deg, 0_deg, 0_deg));

    EXPECT_NO_THROW(table.get(188));
    EXPECT_THROW(table.get(189), std::out_of_range);
}
