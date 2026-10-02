#include "beam_table.hpp"

#include <beam/attitude_transform.hpp>
#include <beam/transform.hpp>

#include <gtest/gtest.h>

#include <cstdint>
#include <stdexcept>

namespace
{

constexpr std::size_t kAzCount = 21;
constexpr std::size_t kElCount = 9;

beam::AntennaToEnu makeValid(double roll, double pitch, double yaw)
{
    beam::RadarAttitude a{};
    a.radar_lat_deg = 37.0;
    a.radar_lon_deg = 127.0;
    a.radar_alt_km  = 0.1;
    a.roll_deg      = roll;
    a.pitch_deg     = pitch;
    a.yaw_deg       = yaw;

    return beam::makeAntennaToEnu(a, beam::AttitudeConfig{});
}

double gridAz(std::size_t idx) { return -42.0 + 4.2 * static_cast<double>(idx % kAzCount); }
double gridEl(std::size_t idx) { return   3.0 + 4.4 * static_cast<double>(idx / kAzCount); }

}   // namespace

TEST(BeamTableTest, HasFixedSize)
{
    EXPECT_EQ(beam::BeamTable::size(), 189u);
    EXPECT_EQ(kAzCount * kElCount, beam::BeamTable::size());
}

TEST(BeamTableTest, BeamIdsAreSequentialFromOne)
{
    const beam::BeamTable table(makeValid(10, 20, 30));

    for(std::size_t i = 0; i < beam::BeamTable::size(); ++i) {
        EXPECT_EQ(table.get(i).beamID, static_cast<std::uint32_t>(i + 1));
    }
}

TEST(BeamTableTest, BeamWidthIsSixDegrees)
{
    const beam::BeamTable table(makeValid(10, 20, 30));

    for(std::size_t i = 0; i < beam::BeamTable::size(); ++i) {
        EXPECT_FLOAT_EQ(table.get(i).az_width_deg, 6.0f);
        EXPECT_FLOAT_EQ(table.get(i).el_width_deg, 6.0f);
    }
}

TEST(BeamTableTest, ZeroAttitudeKeepsFixedGrid)
{
    const beam::BeamTable table(makeValid(0, 0, 0));

    for(std::size_t i = 0; i < beam::BeamTable::size(); ++i) {
        EXPECT_NEAR(table.get(i).az_deg, gridAz(i), 1e-4) << "idx=" << i;
        EXPECT_NEAR(table.get(i).el_deg, gridEl(i), 1e-4) << "idx=" << i;
    }
}

TEST(BeamTableTest, GridOrderStartsLowElevationLeftAzimuth)
{
    const beam::BeamTable table(makeValid(0, 0, 0));

    // 첫 빔: 가장 낮은 고각, 가장 왼쪽 방위각
    EXPECT_NEAR(table.get(0).az_deg, -42.0, 1e-4);
    EXPECT_NEAR(table.get(0).el_deg, 3.0, 1e-4);
    // 같은 고각의 마지막 빔: 가장 오른쪽 방위각
    EXPECT_NEAR(table.get(20).az_deg, 42.0, 1e-4);
    EXPECT_NEAR(table.get(20).el_deg, 3.0, 1e-4);
    // 다음 고각 줄의 첫 빔
    EXPECT_NEAR(table.get(21).az_deg, -42.0, 1e-4);
    EXPECT_NEAR(table.get(21).el_deg, 7.4, 1e-4);
    // 마지막 빔: 가장 높은 고각, 가장 오른쪽 방위각
    EXPECT_NEAR(table.get(188).az_deg, 42.0, 1e-4);
    EXPECT_NEAR(table.get(188).el_deg, 38.2, 1e-4);
}

TEST(BeamTableTest, YawShiftsAzimuthsByOppositeAngle)
{
    // 안테나가 yaw 만큼 돌아가 있으면 고정 그리드는 안테나 기준으로 -yaw 만큼 이동
    const beam::BeamTable table(makeValid(0, 0, 10));

    for(std::size_t i = 0; i < beam::BeamTable::size(); ++i) {
        EXPECT_NEAR(table.get(i).az_deg, gridAz(i) - 10.0, 1e-4) << "idx=" << i;
        EXPECT_NEAR(table.get(i).el_deg, gridEl(i), 1e-4) << "idx=" << i;
    }
}

TEST(BeamTableTest, TableMatchesAnglesConvertedToAntennaFrame)
{
    const beam::AntennaToEnu x = makeValid(10, 20, 30);
    const beam::BeamTable table(x);

    for(std::size_t i = 0; i < beam::BeamTable::size(); ++i) {
        double az = 0, el = 0;
        beam::enuToAntAngle(x, gridAz(i), gridEl(i), az, el);
        EXPECT_NEAR(table.get(i).az_deg, az, 1e-3) << "idx=" << i;
        EXPECT_NEAR(table.get(i).el_deg, el, 1e-3) << "idx=" << i;
    }
}

TEST(BeamTableTest, TableConvertedBackToFixedFrameRestoresGrid)
{
    const beam::AntennaToEnu x = makeValid(10, 20, 30);
    const beam::BeamTable table(x);

    for(std::size_t i = 0; i < beam::BeamTable::size(); ++i) {
        double az = 0, el = 0;
        beam::antToEnuAngle(x, table.get(i).az_deg, table.get(i).el_deg, az, el);
        EXPECT_NEAR(az, gridAz(i), 1e-3) << "idx=" << i;
        EXPECT_NEAR(el, gridEl(i), 1e-3) << "idx=" << i;
    }
}

TEST(BeamTableTest, DifferentAttitudeProducesDifferentTable)
{
    const beam::BeamTable a(makeValid(0, 0, 0));
    const beam::BeamTable b(makeValid(0, 0, 20));

    bool anyDifferent = false;
    for(std::size_t i = 0; i < beam::BeamTable::size(); ++i) {
        if(a.get(i).az_deg != b.get(i).az_deg) { anyDifferent = true; break; }
    }
    EXPECT_TRUE(anyDifferent);
}

TEST(BeamTableTest, ThrowsOnInvalidTransform)
{
    const beam::AntennaToEnu invalid{};   // valid == false
    EXPECT_THROW(beam::BeamTable{invalid}, std::invalid_argument);
}

TEST(BeamTableTest, GetThrowsOnOutOfRangeIndex)
{
    const beam::BeamTable table(makeValid(0, 0, 0));

    EXPECT_NO_THROW(table.get(188));
    EXPECT_THROW(table.get(189), std::out_of_range);
}
