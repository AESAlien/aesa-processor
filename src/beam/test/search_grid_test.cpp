#include "search_grid.hpp"

#include "attitude/ant_enu_transform.hpp"
#include "attitude/attitude_error.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <vector>

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

// 새 격자를 처음부터 한 바퀴 돌며 모든 빔을 탐색 순서대로 꺼낸다.
std::vector<beam::SearchBeam> takeAllBeams(
    const beam::AntEnuTransform& transform,
    const beam::SearchGridConfig& config = DEFAULT_GRID_CONFIG
) {
    beam::SearchGrid grid(transform, config);
    const std::size_t beamCount = config.azimuthCount() * config.elevationCount();

    std::vector<beam::SearchBeam> beams;
    beams.reserve(beamCount);
    for (std::size_t beamIndex = 0; beamIndex < beamCount; ++beamIndex)
    {
        beams.push_back(grid.nextBeam());
    }
    return beams;
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

TEST(SearchGridTest, DefaultConfigHasExpectedBeamCount)
{
    constexpr std::size_t EXPECTED_BEAM_COUNT = 189;
    EXPECT_EQ(AZIMUTH_COUNT * ELEVATION_COUNT, EXPECTED_BEAM_COUNT);

    beam::SearchGrid grid(makeValidTransform(0_deg, 0_deg, 0_deg), DEFAULT_GRID_CONFIG);
    for (std::size_t beamIndex = 0; beamIndex + 1 < EXPECTED_BEAM_COUNT; ++beamIndex)
    {
        grid.nextBeam();
    }

    // 189번째가 마지막 빔이고, 그 다음에 처음으로 돌아온다.
    EXPECT_EQ(grid.nextBeam().beamId, 189u);
    EXPECT_EQ(grid.nextBeam().beamId, 1u);
}

TEST(SearchGridTest, BeamIdsAreSequentialFromOne)
{
    const std::vector<beam::SearchBeam> beams = takeAllBeams(makeValidTransform(10_deg, 20_deg, 30_deg));

    for (std::size_t beamIndex = 0; beamIndex < beams.size(); ++beamIndex)
    {
        EXPECT_EQ(beams[beamIndex].beamId, static_cast<uint32_t>(beamIndex + 1));
    }
}

TEST(SearchGridTest, BeamWidthIsSixDegrees)
{
    const std::vector<beam::SearchBeam> beams = takeAllBeams(makeValidTransform(10_deg, 20_deg, 30_deg));

    for (const beam::SearchBeam& searchBeam : beams)
    {
        EXPECT_EQ(searchBeam.azimuthBeamWidth, 6_deg);
        EXPECT_EQ(searchBeam.elevationBeamWidth, 6_deg);
    }
}

TEST(SearchGridTest, ZeroAttitudeKeepsFixedGrid)
{
    const std::vector<beam::SearchBeam> beams = takeAllBeams(makeValidTransform(0_deg, 0_deg, 0_deg));

    for (std::size_t beamIndex = 0; beamIndex < beams.size(); ++beamIndex)
    {
        EXPECT_EQ(beams[beamIndex].direction_ant.azimuth, gridAzimuth(beamIndex)) << "beamIndex=" << beamIndex;
        EXPECT_EQ(beams[beamIndex].direction_ant.elevation, gridElevation(beamIndex))
            << "beamIndex=" << beamIndex;
    }
}

TEST(SearchGridTest, GridOrderStartsLowElevationLeftAzimuth)
{
    const std::vector<beam::SearchBeam> beams = takeAllBeams(makeValidTransform(0_deg, 0_deg, 0_deg));

    // 첫 빔: 가장 낮은 고각, 가장 왼쪽 방위각
    EXPECT_EQ(beams[0].direction_ant.azimuth, -42_deg);
    EXPECT_EQ(beams[0].direction_ant.elevation, 3_deg);
    // 같은 고각의 마지막 빔: 가장 오른쪽 방위각
    EXPECT_EQ(beams[20].direction_ant.azimuth, 42_deg);
    EXPECT_EQ(beams[20].direction_ant.elevation, 3_deg);
    // 다음 고각 줄의 첫 빔
    EXPECT_EQ(beams[21].direction_ant.azimuth, -42_deg);
    EXPECT_EQ(beams[21].direction_ant.elevation, 7.4_deg);
    // 마지막 빔: 가장 높은 고각, 가장 오른쪽 방위각
    EXPECT_EQ(beams[188].direction_ant.azimuth, 42_deg);
    EXPECT_EQ(beams[188].direction_ant.elevation, 38.2_deg);
}

TEST(SearchGridTest, YawShiftsAzimuthsByOppositeAngle)
{
    // 안테나가 yaw 만큼 돌아가 있으면 고정 그리드는 안테나 기준으로 -yaw 만큼 이동
    const std::vector<beam::SearchBeam> beams = takeAllBeams(makeValidTransform(0_deg, 0_deg, 10_deg));

    for (std::size_t beamIndex = 0; beamIndex < beams.size(); ++beamIndex)
    {
        EXPECT_EQ(beams[beamIndex].direction_ant.azimuth, gridAzimuth(beamIndex) - 10_deg)
            << "beamIndex=" << beamIndex;
        EXPECT_EQ(beams[beamIndex].direction_ant.elevation, gridElevation(beamIndex))
            << "beamIndex=" << beamIndex;
    }
}

TEST(SearchGridTest, GridMatchesAnglesConvertedToAntFrame)
{
    const beam::AntEnuTransform transform = makeValidTransform(10_deg, 20_deg, 30_deg);
    const std::vector<beam::SearchBeam> beams = takeAllBeams(transform);

    for (std::size_t beamIndex = 0; beamIndex < beams.size(); ++beamIndex)
    {
        const math::SphericalDirection direction_ant = transform.enuToAnt(
            math::SphericalDirection(gridAzimuth(beamIndex), gridElevation(beamIndex)));
        EXPECT_EQ(beams[beamIndex].direction_ant.azimuth, direction_ant.azimuth) << "beamIndex=" << beamIndex;
        EXPECT_EQ(beams[beamIndex].direction_ant.elevation, direction_ant.elevation) << "beamIndex=" << beamIndex;
    }
}

TEST(SearchGridTest, BeamsConvertedBackToFixedFrameRestoreGrid)
{
    const beam::AntEnuTransform transform = makeValidTransform(10_deg, 20_deg, 30_deg);
    const std::vector<beam::SearchBeam> beams = takeAllBeams(transform);

    for (std::size_t beamIndex = 0; beamIndex < beams.size(); ++beamIndex)
    {
        const math::SphericalDirection direction_enu = transform.antToEnu(beams[beamIndex].direction_ant);
        EXPECT_EQ(direction_enu.azimuth, math::wrap360(gridAzimuth(beamIndex))) << "beamIndex=" << beamIndex;
        EXPECT_EQ(direction_enu.elevation, gridElevation(beamIndex)) << "beamIndex=" << beamIndex;
    }
}

TEST(SearchGridTest, DifferentAttitudeProducesDifferentGrid)
{
    const std::vector<beam::SearchBeam> zeroAttitudeBeams = takeAllBeams(makeValidTransform(0_deg, 0_deg, 0_deg));
    const std::vector<beam::SearchBeam> yawRotatedBeams = takeAllBeams(makeValidTransform(0_deg, 0_deg, 20_deg));

    bool anyDifferent = false;
    for (std::size_t beamIndex = 0; beamIndex < zeroAttitudeBeams.size(); ++beamIndex)
    {
        if (!zeroAttitudeBeams[beamIndex].direction_ant.azimuth.equals(yawRotatedBeams[beamIndex].direction_ant.azimuth))
        {
            anyDifferent = true;
            break;
        }
    }
    EXPECT_TRUE(anyDifferent);
}

// ---------- 설정 반영 ----------

TEST(SearchGridTest, CustomGridChangesBeamCountAndIds)
{
    const beam::SearchGridConfig config = beam::SearchGridConfig::Builder()
        .azimuthCount(3)
        .elevationCount(2)
        .build();

    beam::SearchGrid grid(makeValidTransform(0_deg, 0_deg, 0_deg), config);

    for (std::size_t beamIndex = 0; beamIndex < 6; ++beamIndex)
    {
        EXPECT_EQ(grid.nextBeam().beamId, static_cast<uint32_t>(beamIndex + 1));
    }
    // 빔이 6개뿐이므로 7번째는 처음으로 돌아온다.
    EXPECT_EQ(grid.nextBeam().beamId, 1u);
}

TEST(SearchGridTest, CustomGridUsesConfiguredStartStepAndWidth)
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

    const std::vector<beam::SearchBeam> beams = takeAllBeams(makeValidTransform(0_deg, 0_deg, 0_deg), config);

    // 방위각 -10, -5, 0 / 고각 1, 3 (고각 바깥 루프, 방위각 안쪽 루프 순서)
    const math::Angle expectedAzimuth[] = {-10_deg, -5_deg, 0_deg, -10_deg, -5_deg, 0_deg};
    const math::Angle expectedElevation[] = {1_deg, 1_deg, 1_deg, 3_deg, 3_deg, 3_deg};

    ASSERT_EQ(beams.size(), 6u);
    for (std::size_t beamIndex = 0; beamIndex < beams.size(); ++beamIndex)
    {
        EXPECT_EQ(beams[beamIndex].direction_ant.azimuth, expectedAzimuth[beamIndex])
            << "beamIndex=" << beamIndex;
        EXPECT_EQ(beams[beamIndex].direction_ant.elevation, expectedElevation[beamIndex])
            << "beamIndex=" << beamIndex;
        EXPECT_EQ(beams[beamIndex].azimuthBeamWidth, 4_deg);
        EXPECT_EQ(beams[beamIndex].elevationBeamWidth, 4_deg);
    }
}

// 설정 값 검증(개수, 빔폭, 유한성, 오버플로 등)은 config_test.cpp 에서 확인한다.

TEST(SearchGridTest, LargeStepWithSingleBeamPerAxisIsAccepted)
{
    const auto maxAngle = math::Angle::fromDegrees(std::numeric_limits<double>::max());

    // count가 1이면 step은 쓰이지 않으므로 큰 step도 문제없다.
    const beam::SearchGridConfig config = beam::SearchGridConfig::Builder()
        .azimuthCount(1)
        .azimuthStep(maxAngle)
        .elevationCount(1)
        .elevationStep(maxAngle)
        .build();

    beam::SearchGrid grid(makeValidTransform(0_deg, 0_deg, 0_deg), config);

    EXPECT_EQ(grid.nextBeam().beamId, 1u);
    EXPECT_EQ(grid.nextBeam().beamId, 1u);
}

TEST(SearchGridTest, LargeAzimuthStartWithSingleBeamPreservesWrappedDirection)
{
    const beam::SearchGridConfig config = beam::SearchGridConfig::Builder()
        .azimuthCount(1)
        .elevationCount(1)
        .azimuthStart(math::Angle::fromDegrees(1e18))
        .build();

    beam::SearchGrid grid(makeValidTransform(0_deg, 0_deg, 0_deg), config);

    EXPECT_EQ(grid.nextBeam().direction_ant.azimuth, -80_deg);
}

// ---------- 탐색 커서 ----------

TEST(SearchGridTest, NextBeamWrapsToTheFirstBeamAfterTheLastOne)
{
    beam::SearchGrid grid(makeValidTransform(0_deg, 0_deg, 0_deg), DEFAULT_GRID_CONFIG);
    for (std::size_t beamIndex = 0; beamIndex < AZIMUTH_COUNT * ELEVATION_COUNT; ++beamIndex)
    {
        grid.nextBeam();
    }

    EXPECT_EQ(grid.nextBeam().beamId, 1u);
    EXPECT_EQ(grid.nextBeam().beamId, 2u);
}

TEST(SearchGridTest, SingleBeamGridReturnsTheSameBeamEveryTime)
{
    const beam::SearchGridConfig config = beam::SearchGridConfig::Builder()
        .azimuthCount(1)
        .elevationCount(1)
        .build();
    beam::SearchGrid grid(makeValidTransform(0_deg, 0_deg, 0_deg), config);

    EXPECT_EQ(grid.nextBeam().beamId, 1u);
    EXPECT_EQ(grid.nextBeam().beamId, 1u);
    EXPECT_EQ(grid.nextBeam().beamId, 1u);
}

TEST(SearchGridTest, CopiedGridKeepsItsOwnCursor)
{
    beam::SearchGrid grid(makeValidTransform(0_deg, 0_deg, 0_deg), DEFAULT_GRID_CONFIG);
    grid.nextBeam();
    beam::SearchGrid copied = grid;

    EXPECT_EQ(grid.nextBeam().beamId, 2u);
    EXPECT_EQ(grid.nextBeam().beamId, 3u);
    EXPECT_EQ(copied.nextBeam().beamId, 2u);
}
