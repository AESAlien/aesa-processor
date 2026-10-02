#include "beam_table.hpp"

#include "attitude/ant_enu_transform.hpp"
#include "attitude/attitude_error.hpp"

#include <gtest/gtest.h>

#include <cstdint>
#include <stdexcept>

namespace
{

constexpr std::size_t AZIMUTH_COUNT = 21;
constexpr std::size_t ELEVATION_COUNT = 9;

beam::AntEnuTransform makeValidTransform(double roll_deg, double pitch_deg, double yaw_deg)
{
    beam::RadarAttitude attitude{};
    attitude.latitude_deg = 37.0;
    attitude.longitude_deg = 127.0;
    attitude.altitude_km = 0.1;
    attitude.roll_deg = roll_deg;
    attitude.pitch_deg = pitch_deg;
    attitude.yaw_deg = yaw_deg;

    return beam::AntEnuTransform(attitude, beam::AttitudeConfig{});
}

double gridAzimuth(std::size_t beamIndex)
{
    return -42.0 + 4.2 * static_cast<double>(beamIndex % AZIMUTH_COUNT);
}
double gridElevation(std::size_t beamIndex)
{
    return 3.0 + 4.4 * static_cast<double>(beamIndex / AZIMUTH_COUNT);
}

} // namespace

TEST(BeamTableTest, HasFixedSize)
{
    EXPECT_EQ(beam::BeamTable::size(), 189u);
    EXPECT_EQ(AZIMUTH_COUNT * ELEVATION_COUNT, beam::BeamTable::size());
}

TEST(BeamTableTest, BeamIdsAreSequentialFromOne)
{
    const beam::BeamTable table(makeValidTransform(10, 20, 30));

    for (std::size_t beamIndex = 0; beamIndex < beam::BeamTable::size(); ++beamIndex)
    {
        EXPECT_EQ(table.get(beamIndex).beamId, static_cast<std::uint32_t>(beamIndex + 1));
    }
}

TEST(BeamTableTest, BeamWidthIsSixDegrees)
{
    const beam::BeamTable table(makeValidTransform(10, 20, 30));

    for (std::size_t beamIndex = 0; beamIndex < beam::BeamTable::size(); ++beamIndex)
    {
        EXPECT_FLOAT_EQ(table.get(beamIndex).azimuthBeamWidth_deg, 6.0f);
        EXPECT_FLOAT_EQ(table.get(beamIndex).elevationBeamWidth_deg, 6.0f);
    }
}

TEST(BeamTableTest, ZeroAttitudeKeepsFixedGrid)
{
    const beam::BeamTable table(makeValidTransform(0, 0, 0));

    for (std::size_t beamIndex = 0; beamIndex < beam::BeamTable::size(); ++beamIndex)
    {
        EXPECT_NEAR(table.get(beamIndex).azimuth_ant_deg, gridAzimuth(beamIndex), 1e-4) << "beamIndex=" << beamIndex;
        EXPECT_NEAR(table.get(beamIndex).elevation_ant_deg, gridElevation(beamIndex), 1e-4)
            << "beamIndex=" << beamIndex;
    }
}

TEST(BeamTableTest, GridOrderStartsLowElevationLeftAzimuth)
{
    const beam::BeamTable table(makeValidTransform(0, 0, 0));

    // 첫 빔: 가장 낮은 고각, 가장 왼쪽 방위각
    EXPECT_NEAR(table.get(0).azimuth_ant_deg, -42.0, 1e-4);
    EXPECT_NEAR(table.get(0).elevation_ant_deg, 3.0, 1e-4);
    // 같은 고각의 마지막 빔: 가장 오른쪽 방위각
    EXPECT_NEAR(table.get(20).azimuth_ant_deg, 42.0, 1e-4);
    EXPECT_NEAR(table.get(20).elevation_ant_deg, 3.0, 1e-4);
    // 다음 고각 줄의 첫 빔
    EXPECT_NEAR(table.get(21).azimuth_ant_deg, -42.0, 1e-4);
    EXPECT_NEAR(table.get(21).elevation_ant_deg, 7.4, 1e-4);
    // 마지막 빔: 가장 높은 고각, 가장 오른쪽 방위각
    EXPECT_NEAR(table.get(188).azimuth_ant_deg, 42.0, 1e-4);
    EXPECT_NEAR(table.get(188).elevation_ant_deg, 38.2, 1e-4);
}

TEST(BeamTableTest, YawShiftsAzimuthsByOppositeAngle)
{
    // 안테나가 yaw 만큼 돌아가 있으면 고정 그리드는 안테나 기준으로 -yaw 만큼 이동
    const beam::BeamTable table(makeValidTransform(0, 0, 10));

    for (std::size_t beamIndex = 0; beamIndex < beam::BeamTable::size(); ++beamIndex)
    {
        EXPECT_NEAR(table.get(beamIndex).azimuth_ant_deg, gridAzimuth(beamIndex) - 10.0, 1e-4)
            << "beamIndex=" << beamIndex;
        EXPECT_NEAR(table.get(beamIndex).elevation_ant_deg, gridElevation(beamIndex), 1e-4)
            << "beamIndex=" << beamIndex;
    }
}

TEST(BeamTableTest, TableMatchesAnglesConvertedToAntFrame)
{
    const beam::AntEnuTransform transform = makeValidTransform(10, 20, 30);
    const beam::BeamTable table(transform);

    for (std::size_t beamIndex = 0; beamIndex < beam::BeamTable::size(); ++beamIndex)
    {
        const auto [azimuth_ant_deg, elevation_ant_deg] =
            transform.enuToAnt(gridAzimuth(beamIndex), gridElevation(beamIndex));
        EXPECT_NEAR(table.get(beamIndex).azimuth_ant_deg, azimuth_ant_deg, 1e-3) << "beamIndex=" << beamIndex;
        EXPECT_NEAR(table.get(beamIndex).elevation_ant_deg, elevation_ant_deg, 1e-3) << "beamIndex=" << beamIndex;
    }
}

TEST(BeamTableTest, TableConvertedBackToFixedFrameRestoresGrid)
{
    const beam::AntEnuTransform transform = makeValidTransform(10, 20, 30);
    const beam::BeamTable table(transform);

    for (std::size_t beamIndex = 0; beamIndex < beam::BeamTable::size(); ++beamIndex)
    {
        const auto [azimuth_enu_deg, elevation_enu_deg] =
            transform.antToEnu(table.get(beamIndex).azimuth_ant_deg, table.get(beamIndex).elevation_ant_deg);
        EXPECT_NEAR(azimuth_enu_deg, gridAzimuth(beamIndex), 1e-3) << "beamIndex=" << beamIndex;
        EXPECT_NEAR(elevation_enu_deg, gridElevation(beamIndex), 1e-3) << "beamIndex=" << beamIndex;
    }
}

TEST(BeamTableTest, DifferentAttitudeProducesDifferentTable)
{
    const beam::BeamTable zeroAttitudeTable(makeValidTransform(0, 0, 0));
    const beam::BeamTable yawRotatedTable(makeValidTransform(0, 0, 20));

    bool anyDifferent = false;
    for (std::size_t beamIndex = 0; beamIndex < beam::BeamTable::size(); ++beamIndex)
    {
        if (zeroAttitudeTable.get(beamIndex).azimuth_ant_deg != yawRotatedTable.get(beamIndex).azimuth_ant_deg)
        {
            anyDifferent = true;
            break;
        }
    }
    EXPECT_TRUE(anyDifferent);
}

TEST(BeamTableTest, GetThrowsOnOutOfRangeIndex)
{
    const beam::BeamTable table(makeValidTransform(0, 0, 0));

    EXPECT_NO_THROW(table.get(188));
    EXPECT_THROW(table.get(189), std::out_of_range);
}
