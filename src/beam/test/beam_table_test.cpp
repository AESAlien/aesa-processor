#include "beam_table.hpp"

#include <beam/domain/antenna_to_enu.hpp>
#include <beam/error/attitude_error.hpp>

#include <gtest/gtest.h>

#include <cstdint>
#include <stdexcept>

namespace
{

constexpr std::size_t AzCount = 21;
constexpr std::size_t ElCount = 9;

beam::AntennaToEnu MakeValid(double roll, double pitch, double yaw)
{
    beam::RadarAttitude a{};
    a.radarLat_deg = 37.0;
    a.radarLon_deg = 127.0;
    a.radarAlt_km = 0.1;
    a.roll_deg = roll;
    a.pitch_deg = pitch;
    a.yaw_deg = yaw;

    return beam::AntennaToEnu(a, beam::AttitudeConfig{});
}

double GridAz(std::size_t idx)
{
    return -42.0 + 4.2 * static_cast<double>(idx % AzCount);
}
double GridEl(std::size_t idx)
{
    return 3.0 + 4.4 * static_cast<double>(idx / AzCount);
}

} // namespace

TEST(BeamTableTest, HasFixedSize)
{
    EXPECT_EQ(beam::BeamTable::Size(), 189u);
    EXPECT_EQ(AzCount * ElCount, beam::BeamTable::Size());
}

TEST(BeamTableTest, BeamIdsAreSequentialFromOne)
{
    const beam::BeamTable table(MakeValid(10, 20, 30));

    for (std::size_t i = 0; i < beam::BeamTable::Size(); ++i)
    {
        EXPECT_EQ(table.Get(i).beamId, static_cast<std::uint32_t>(i + 1));
    }
}

TEST(BeamTableTest, BeamWidthIsSixDegrees)
{
    const beam::BeamTable table(MakeValid(10, 20, 30));

    for (std::size_t i = 0; i < beam::BeamTable::Size(); ++i)
    {
        EXPECT_FLOAT_EQ(table.Get(i).azWidth_deg, 6.0f);
        EXPECT_FLOAT_EQ(table.Get(i).elWidth_deg, 6.0f);
    }
}

TEST(BeamTableTest, ZeroAttitudeKeepsFixedGrid)
{
    const beam::BeamTable table(MakeValid(0, 0, 0));

    for (std::size_t i = 0; i < beam::BeamTable::Size(); ++i)
    {
        EXPECT_NEAR(table.Get(i).az_deg, GridAz(i), 1e-4) << "idx=" << i;
        EXPECT_NEAR(table.Get(i).el_deg, GridEl(i), 1e-4) << "idx=" << i;
    }
}

TEST(BeamTableTest, GridOrderStartsLowElevationLeftAzimuth)
{
    const beam::BeamTable table(MakeValid(0, 0, 0));

    // 첫 빔: 가장 낮은 고각, 가장 왼쪽 방위각
    EXPECT_NEAR(table.Get(0).az_deg, -42.0, 1e-4);
    EXPECT_NEAR(table.Get(0).el_deg, 3.0, 1e-4);
    // 같은 고각의 마지막 빔: 가장 오른쪽 방위각
    EXPECT_NEAR(table.Get(20).az_deg, 42.0, 1e-4);
    EXPECT_NEAR(table.Get(20).el_deg, 3.0, 1e-4);
    // 다음 고각 줄의 첫 빔
    EXPECT_NEAR(table.Get(21).az_deg, -42.0, 1e-4);
    EXPECT_NEAR(table.Get(21).el_deg, 7.4, 1e-4);
    // 마지막 빔: 가장 높은 고각, 가장 오른쪽 방위각
    EXPECT_NEAR(table.Get(188).az_deg, 42.0, 1e-4);
    EXPECT_NEAR(table.Get(188).el_deg, 38.2, 1e-4);
}

TEST(BeamTableTest, YawShiftsAzimuthsByOppositeAngle)
{
    // 안테나가 yaw 만큼 돌아가 있으면 고정 그리드는 안테나 기준으로 -yaw 만큼 이동
    const beam::BeamTable table(MakeValid(0, 0, 10));

    for (std::size_t i = 0; i < beam::BeamTable::Size(); ++i)
    {
        EXPECT_NEAR(table.Get(i).az_deg, GridAz(i) - 10.0, 1e-4) << "idx=" << i;
        EXPECT_NEAR(table.Get(i).el_deg, GridEl(i), 1e-4) << "idx=" << i;
    }
}

TEST(BeamTableTest, TableMatchesAnglesConvertedToAntennaFrame)
{
    const beam::AntennaToEnu x = MakeValid(10, 20, 30);
    const beam::BeamTable table(x);

    for (std::size_t i = 0; i < beam::BeamTable::Size(); ++i)
    {
        double az = 0, el = 0;
        x.EnuToAntAngle(GridAz(i), GridEl(i), az, el);
        EXPECT_NEAR(table.Get(i).az_deg, az, 1e-3) << "idx=" << i;
        EXPECT_NEAR(table.Get(i).el_deg, el, 1e-3) << "idx=" << i;
    }
}

TEST(BeamTableTest, TableConvertedBackToFixedFrameRestoresGrid)
{
    const beam::AntennaToEnu x = MakeValid(10, 20, 30);
    const beam::BeamTable table(x);

    for (std::size_t i = 0; i < beam::BeamTable::Size(); ++i)
    {
        double az = 0, el = 0;
        x.AntToEnuAngle(table.Get(i).az_deg, table.Get(i).el_deg, az, el);
        EXPECT_NEAR(az, GridAz(i), 1e-3) << "idx=" << i;
        EXPECT_NEAR(el, GridEl(i), 1e-3) << "idx=" << i;
    }
}

TEST(BeamTableTest, DifferentAttitudeProducesDifferentTable)
{
    const beam::BeamTable a(MakeValid(0, 0, 0));
    const beam::BeamTable b(MakeValid(0, 0, 20));

    bool anyDifferent = false;
    for (std::size_t i = 0; i < beam::BeamTable::Size(); ++i)
    {
        if (a.Get(i).az_deg != b.Get(i).az_deg)
        {
            anyDifferent = true;
            break;
        }
    }
    EXPECT_TRUE(anyDifferent);
}

TEST(BeamTableTest, GetThrowsOnOutOfRangeIndex)
{
    const beam::BeamTable table(MakeValid(0, 0, 0));

    EXPECT_NO_THROW(table.Get(188));
    EXPECT_THROW(table.Get(189), std::out_of_range);
}
