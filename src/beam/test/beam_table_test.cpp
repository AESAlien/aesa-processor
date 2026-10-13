#include "beam_table.hpp"

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

TEST(BeamTableTest, DefaultConfigHasExpectedBeamCount)
{
    const beam::BeamTable table(makeValidTransform(0_deg, 0_deg, 0_deg), DEFAULT_GRID_CONFIG);

    EXPECT_EQ(table.size(), 189u);
    EXPECT_EQ(AZIMUTH_COUNT * ELEVATION_COUNT, table.size());
}

TEST(BeamTableTest, BeamIdsAreSequentialFromOne)
{
    const beam::BeamTable table(makeValidTransform(10_deg, 20_deg, 30_deg), DEFAULT_GRID_CONFIG);

    for (std::size_t beamIndex = 0; beamIndex < table.size(); ++beamIndex)
    {
        EXPECT_EQ(table.get(beamIndex).beamId, static_cast<uint32_t>(beamIndex + 1));
    }
}

TEST(BeamTableTest, BeamWidthIsSixDegrees)
{
    const beam::BeamTable table(makeValidTransform(10_deg, 20_deg, 30_deg), DEFAULT_GRID_CONFIG);

    for (std::size_t beamIndex = 0; beamIndex < table.size(); ++beamIndex)
    {
        EXPECT_TRUE(table.get(beamIndex).azimuthBeamWidth.equals(6_deg, math::QuantityTolerance<math::Angle>{math::Angle::fromDegrees(0.0), 0.0}));
        EXPECT_TRUE(table.get(beamIndex).elevationBeamWidth.equals(6_deg, math::QuantityTolerance<math::Angle>{math::Angle::fromDegrees(0.0), 0.0}));
    }
}

TEST(BeamTableTest, ZeroAttitudeKeepsFixedGrid)
{
    const beam::BeamTable table(makeValidTransform(0_deg, 0_deg, 0_deg), DEFAULT_GRID_CONFIG);

    for (std::size_t beamIndex = 0; beamIndex < table.size(); ++beamIndex)
    {
        EXPECT_TRUE(table.get(beamIndex).azimuth_ant.equals(gridAzimuth(beamIndex), math::QuantityTolerance<math::Angle>{math::Angle::fromDegrees(0.0), 1e-4})) << "beamIndex=" << beamIndex;
        EXPECT_TRUE(table.get(beamIndex).elevation_ant.equals(gridElevation(beamIndex), math::QuantityTolerance<math::Angle>{math::Angle::fromDegrees(0.0), 1e-4}))
            << "beamIndex=" << beamIndex;
    }
}

TEST(BeamTableTest, GridOrderStartsLowElevationLeftAzimuth)
{
    const beam::BeamTable table(makeValidTransform(0_deg, 0_deg, 0_deg), DEFAULT_GRID_CONFIG);

    // 첫 빔: 가장 낮은 고각, 가장 왼쪽 방위각
    EXPECT_TRUE(table.get(0).azimuth_ant.equals(-42_deg, math::QuantityTolerance<math::Angle>{math::Angle::fromDegrees(0.0), 1e-4}));
    EXPECT_TRUE(table.get(0).elevation_ant.equals(3_deg, math::QuantityTolerance<math::Angle>{math::Angle::fromDegrees(0.0), 1e-4}));
    // 같은 고각의 마지막 빔: 가장 오른쪽 방위각
    EXPECT_TRUE(table.get(20).azimuth_ant.equals(42_deg, math::QuantityTolerance<math::Angle>{math::Angle::fromDegrees(0.0), 1e-4}));
    EXPECT_TRUE(table.get(20).elevation_ant.equals(3_deg, math::QuantityTolerance<math::Angle>{math::Angle::fromDegrees(0.0), 1e-4}));
    // 다음 고각 줄의 첫 빔
    EXPECT_TRUE(table.get(21).azimuth_ant.equals(-42_deg, math::QuantityTolerance<math::Angle>{math::Angle::fromDegrees(0.0), 1e-4}));
    EXPECT_TRUE(table.get(21).elevation_ant.equals(7.4_deg, math::QuantityTolerance<math::Angle>{math::Angle::fromDegrees(0.0), 1e-4}));
    // 마지막 빔: 가장 높은 고각, 가장 오른쪽 방위각
    EXPECT_TRUE(table.get(188).azimuth_ant.equals(42_deg, math::QuantityTolerance<math::Angle>{math::Angle::fromDegrees(0.0), 1e-4}));
    EXPECT_TRUE(table.get(188).elevation_ant.equals(38.2_deg, math::QuantityTolerance<math::Angle>{math::Angle::fromDegrees(0.0), 1e-4}));
}

TEST(BeamTableTest, YawShiftsAzimuthsByOppositeAngle)
{
    // 안테나가 yaw 만큼 돌아가 있으면 고정 그리드는 안테나 기준으로 -yaw 만큼 이동
    const beam::BeamTable table(makeValidTransform(0_deg, 0_deg, 10_deg), DEFAULT_GRID_CONFIG);

    for (std::size_t beamIndex = 0; beamIndex < table.size(); ++beamIndex)
    {
        EXPECT_TRUE(table.get(beamIndex).azimuth_ant.equals(gridAzimuth(beamIndex) - 10_deg, math::QuantityTolerance<math::Angle>{math::Angle::fromDegrees(0.0), 1e-4}))
            << "beamIndex=" << beamIndex;
        EXPECT_TRUE(table.get(beamIndex).elevation_ant.equals(gridElevation(beamIndex), math::QuantityTolerance<math::Angle>{math::Angle::fromDegrees(0.0), 1e-4}))
            << "beamIndex=" << beamIndex;
    }
}

TEST(BeamTableTest, TableMatchesAnglesConvertedToAntFrame)
{
    const beam::AntEnuTransform transform = makeValidTransform(10_deg, 20_deg, 30_deg);
    const beam::BeamTable table(transform, DEFAULT_GRID_CONFIG);

    for (std::size_t beamIndex = 0; beamIndex < table.size(); ++beamIndex)
    {
        const math::SphericalDirection direction_ant = transform.enuToAnt(
            math::SphericalDirection(gridAzimuth(beamIndex).wrap180(), gridElevation(beamIndex)));
        EXPECT_TRUE(table.get(beamIndex).azimuth_ant.equals(direction_ant.azimuth(), math::QuantityTolerance<math::Angle>{math::Angle::fromDegrees(0.0), 1e-3})) << "beamIndex=" << beamIndex;
        EXPECT_TRUE(table.get(beamIndex).elevation_ant.equals(direction_ant.elevation(), math::QuantityTolerance<math::Angle>{math::Angle::fromDegrees(0.0), 1e-3})) << "beamIndex=" << beamIndex;
    }
}

TEST(BeamTableTest, TableConvertedBackToFixedFrameRestoresGrid)
{
    const beam::AntEnuTransform transform = makeValidTransform(10_deg, 20_deg, 30_deg);
    const beam::BeamTable table(transform, DEFAULT_GRID_CONFIG);

    for (std::size_t beamIndex = 0; beamIndex < table.size(); ++beamIndex)
    {
        const math::SphericalDirection direction_enu = transform.antToEnu(math::SphericalDirection(
            table.get(beamIndex).azimuth_ant, table.get(beamIndex).elevation_ant));
        EXPECT_TRUE(direction_enu.azimuth().equals(gridAzimuth(beamIndex).wrap180(), math::QuantityTolerance{1e-3_deg, 0.0})) << "beamIndex=" << beamIndex;
        EXPECT_TRUE(direction_enu.elevation().equals(gridElevation(beamIndex), math::QuantityTolerance{1e-3_deg, 0.0})) << "beamIndex=" << beamIndex;
    }
}

TEST(BeamTableTest, DifferentAttitudeProducesDifferentTable)
{
    const beam::BeamTable zeroAttitudeTable(makeValidTransform(0_deg, 0_deg, 0_deg), DEFAULT_GRID_CONFIG);
    const beam::BeamTable yawRotatedTable(makeValidTransform(0_deg, 0_deg, 20_deg), DEFAULT_GRID_CONFIG);

    bool anyDifferent = false;
    for (std::size_t beamIndex = 0; beamIndex < zeroAttitudeTable.size(); ++beamIndex)
    {
        if (!zeroAttitudeTable.get(beamIndex).azimuth_ant.equals(yawRotatedTable.get(beamIndex).azimuth_ant, math::QuantityTolerance{0.001_deg, 0.0}))
        {
            anyDifferent = true;
            break;
        }
    }
    EXPECT_TRUE(anyDifferent);
}

TEST(BeamTableTest, GetThrowsOnOutOfRangeIndex)
{
    const beam::BeamTable table(makeValidTransform(0_deg, 0_deg, 0_deg), DEFAULT_GRID_CONFIG);

    EXPECT_NO_THROW(table.get(188));
    EXPECT_THROW(table.get(189), std::out_of_range);
}

// ---------- 설정 반영 ----------

TEST(BeamTableTest, CustomGridChangesBeamCountAndIds)
{
    beam::SearchGridConfig config;
    config.azimuthCount = 3;
    config.elevationCount = 2;

    const beam::BeamTable table(makeValidTransform(0_deg, 0_deg, 0_deg), config);

    ASSERT_EQ(table.size(), 6u);
    for (std::size_t beamIndex = 0; beamIndex < table.size(); ++beamIndex)
    {
        EXPECT_EQ(table.get(beamIndex).beamId, static_cast<uint32_t>(beamIndex + 1));
    }
    EXPECT_THROW(table.get(6), std::out_of_range);
}

TEST(BeamTableTest, CustomGridUsesConfiguredStartStepAndWidth)
{
    beam::SearchGridConfig config;
    config.azimuthCount = 3;
    config.elevationCount = 2;
    config.azimuthStart = -10_deg;
    config.azimuthStep = 5_deg;
    config.elevationStart = 1_deg;
    config.elevationStep = 2_deg;
    config.beamWidth = 4_deg;

    const beam::BeamTable table(makeValidTransform(0_deg, 0_deg, 0_deg), config);

    // 방위각 -10, -5, 0 / 고각 1, 3 (고각 바깥 루프, 방위각 안쪽 루프 순서)
    const math::Angle expectedAzimuth[] = {-10_deg, -5_deg, 0_deg, -10_deg, -5_deg, 0_deg};
    const math::Angle expectedElevation[] = {1_deg, 1_deg, 1_deg, 3_deg, 3_deg, 3_deg};

    ASSERT_EQ(table.size(), 6u);
    for (std::size_t beamIndex = 0; beamIndex < table.size(); ++beamIndex)
    {
        EXPECT_TRUE(table.get(beamIndex).azimuth_ant.equals(expectedAzimuth[beamIndex], math::QuantityTolerance<math::Angle>{math::Angle::fromDegrees(0.0), 1e-4}))
            << "beamIndex=" << beamIndex;
        EXPECT_TRUE(table.get(beamIndex).elevation_ant.equals(expectedElevation[beamIndex], math::QuantityTolerance<math::Angle>{math::Angle::fromDegrees(0.0), 1e-4}))
            << "beamIndex=" << beamIndex;
        EXPECT_TRUE(table.get(beamIndex).azimuthBeamWidth.equals(4_deg, math::QuantityTolerance<math::Angle>{math::Angle::fromDegrees(0.0), 0.0}));
        EXPECT_TRUE(table.get(beamIndex).elevationBeamWidth.equals(4_deg, math::QuantityTolerance<math::Angle>{math::Angle::fromDegrees(0.0), 0.0}));
    }
}

// ---------- 설정 검증 ----------

TEST(BeamTableTest, ZeroAzimuthCountThrows)
{
    beam::SearchGridConfig config;
    config.azimuthCount = 0;

    EXPECT_THROW(beam::BeamTable(makeValidTransform(0_deg, 0_deg, 0_deg), config), std::invalid_argument);
}

TEST(BeamTableTest, ZeroElevationCountThrows)
{
    beam::SearchGridConfig config;
    config.elevationCount = 0;

    EXPECT_THROW(beam::BeamTable(makeValidTransform(0_deg, 0_deg, 0_deg), config), std::invalid_argument);
}

TEST(BeamTableTest, BeamCountBeyondUint32Throws)
{
    beam::SearchGridConfig config;
    config.azimuthCount = 70000;
    config.elevationCount = 70000;

    EXPECT_THROW(beam::BeamTable(makeValidTransform(0_deg, 0_deg, 0_deg), config), std::invalid_argument);
}

TEST(BeamTableTest, NonPositiveBeamWidthThrows)
{
    beam::SearchGridConfig zeroWidth;
    zeroWidth.beamWidth = 0_deg;
    beam::SearchGridConfig negativeWidth;
    negativeWidth.beamWidth = -1_deg;

    EXPECT_THROW(beam::BeamTable(makeValidTransform(0_deg, 0_deg, 0_deg), zeroWidth), std::invalid_argument);
    EXPECT_THROW(beam::BeamTable(makeValidTransform(0_deg, 0_deg, 0_deg), negativeWidth), std::invalid_argument);
}

TEST(BeamTableTest, NonFiniteAngleThrows)
{
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const double inf = std::numeric_limits<double>::infinity();

    beam::SearchGridConfig nanStart;
    nanStart.azimuthStart = math::Angle::fromDegrees(nan);
    beam::SearchGridConfig infStep;
    infStep.elevationStep = math::Angle::fromDegrees(inf);
    beam::SearchGridConfig nanWidth;
    nanWidth.beamWidth = math::Angle::fromDegrees(nan);

    EXPECT_THROW(beam::BeamTable(makeValidTransform(0_deg, 0_deg, 0_deg), nanStart), std::invalid_argument);
    EXPECT_THROW(beam::BeamTable(makeValidTransform(0_deg, 0_deg, 0_deg), infStep), std::invalid_argument);
    EXPECT_THROW(beam::BeamTable(makeValidTransform(0_deg, 0_deg, 0_deg), nanWidth), std::invalid_argument);
}

TEST(BeamTableTest, GridAngleOverflowThrows)
{
    const auto maxAngle = math::Angle::fromDegrees(std::numeric_limits<double>::max());

    // step * (count - 1)에서 오버플로
    beam::SearchGridConfig azimuthStepOverflow;
    azimuthStepOverflow.azimuthCount = 3;
    azimuthStepOverflow.azimuthStep = maxAngle;
    beam::SearchGridConfig elevationStepOverflow;
    elevationStepOverflow.elevationCount = 3;
    elevationStepOverflow.elevationStep = maxAngle;

    // start + step * (count - 1)에서 오버플로
    beam::SearchGridConfig azimuthSumOverflow;
    azimuthSumOverflow.azimuthCount = 2;
    azimuthSumOverflow.azimuthStart = maxAngle;
    azimuthSumOverflow.azimuthStep = maxAngle;
    beam::SearchGridConfig elevationSumOverflow;
    elevationSumOverflow.elevationCount = 2;
    elevationSumOverflow.elevationStart = maxAngle;
    elevationSumOverflow.elevationStep = maxAngle;

    EXPECT_THROW(beam::BeamTable(makeValidTransform(0_deg, 0_deg, 0_deg), azimuthStepOverflow), std::invalid_argument);
    EXPECT_THROW(beam::BeamTable(makeValidTransform(0_deg, 0_deg, 0_deg), elevationStepOverflow), std::invalid_argument);
    EXPECT_THROW(beam::BeamTable(makeValidTransform(0_deg, 0_deg, 0_deg), azimuthSumOverflow), std::invalid_argument);
    EXPECT_THROW(beam::BeamTable(makeValidTransform(0_deg, 0_deg, 0_deg), elevationSumOverflow), std::invalid_argument);
}

TEST(BeamTableTest, LargeStepWithSingleBeamPerAxisIsAccepted)
{
    const auto maxAngle = math::Angle::fromDegrees(std::numeric_limits<double>::max());

    // count가 1이면 step은 쓰이지 않으므로 큰 step도 문제없다.
    beam::SearchGridConfig config;
    config.azimuthCount = 1;
    config.azimuthStep = maxAngle;
    config.elevationCount = 1;
    config.elevationStep = maxAngle;

    const beam::BeamTable table(makeValidTransform(0_deg, 0_deg, 0_deg), config);

    ASSERT_EQ(table.size(), 1u);
    EXPECT_TRUE(table.get(0).azimuth_ant.isFinite());
    EXPECT_TRUE(table.get(0).elevation_ant.isFinite());
}

TEST(BeamTableTest, LargeAzimuthStartWithSingleBeamPreservesWrappedDirection)
{
    beam::SearchGridConfig config;
    config.azimuthCount = 1;
    config.elevationCount = 1;
    config.azimuthStart = math::Angle::fromDegrees(1e18);

    const beam::BeamTable table(makeValidTransform(0_deg, 0_deg, 0_deg), config);

    ASSERT_EQ(table.size(), 1u);
    EXPECT_NEAR(table.get(0).azimuth_ant.deg(), -80.0, 1e-10);
}
