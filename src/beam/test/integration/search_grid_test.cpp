#include "search/search_grid.hpp"

#include "attitude/ant_enu_transform.hpp"
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

beam::SearchGridConfig::Builder validConfigBuilder()
{
    return beam::SearchGridConfig::Builder()
        .azimuthCount(21)
        .elevationCount(9)
        .azimuthStart(-42_deg)
        .azimuthStep(4.2_deg)
        .elevationStart(3_deg)
        .elevationStep(4.4_deg)
        .beamWidth(beam::BeamWidth(6_deg, 6_deg));
}

beam::AntEnuTransform makeTransform(math::Angle roll, math::Angle pitch, math::Angle yaw)
{
    const auto attitude = beam::RadarAttitude(roll, math::TiltAngle(pitch), yaw);

    const auto config = beam::AttitudeConfig(
        89_deg,
        math::SquareMatrix::identity(3),
        math::Tolerance{1e-6, 0.0}
    );

    return beam::AntEnuTransform(attitude, config);
}

// 새 격자를 처음부터 한 바퀴 돌며 모든 빔을 탐색 순서대로 꺼낸다.
std::vector<beam::SearchBeam> takeAllBeams(
    const beam::AntEnuTransform& transform,
    const beam::SearchGridConfig& config = validConfigBuilder().build()
) {
    auto grid = beam::SearchGrid(transform, config);
    const auto beamCount = config.azimuthCount() * config.elevationCount();

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

// 기본 격자는 189개의 빔을 반환한 뒤 첫 빔부터 순서대로 다시 반환한다.
TEST(SearchGridTest, DefaultGridWrapsAfterExpectedBeamCount)
{
    // Given: 기본 설정의 영 자세 격자를 준비한다.
    auto grid = beam::SearchGrid(makeTransform(0_deg, 0_deg, 0_deg), validConfigBuilder().build());
    constexpr std::size_t EXPECTED_BEAM_COUNT = 189;

    // When: 격자의 모든 빔을 순서대로 꺼낸다.
    std::vector<beam::SearchBeam> beams;
    for (std::size_t index = 0; index < EXPECTED_BEAM_COUNT; ++index)
    {
        beams.push_back(grid.nextBeam());
    }
    const auto wrapped = grid.nextBeam();
    const auto afterWrapped = grid.nextBeam();

    // Then: 마지막 빔의 ID는 189이고 다음 빔은 ID 1, 2 순서로 이어진다.
    EXPECT_EQ(beams.back().beamId, 188u);
    EXPECT_EQ(wrapped.beamId, 0u);
    EXPECT_EQ(afterWrapped.beamId, 1u);
}

// 격자의 빔 ID는 0부터 연속으로 증가한다.
TEST(SearchGridTest, BeamIdsAreSequentialFromZero)
{
    // Given: 격자의 자세 변환을 준비한다.
    const auto beamsTransform = makeTransform(10_deg, 20_deg, 30_deg);

    // When: 주어진 자세와 기본 설정으로 격자를 생성해 빔을 꺼낸다.
    const auto beams = takeAllBeams(beamsTransform);

    for (std::size_t beamIndex = 0; beamIndex < beams.size(); ++beamIndex)
    {

        // Then: 격자의 빔 ID는 0부터 연속으로 증가한다.
        EXPECT_EQ(beams[beamIndex].beamId, static_cast<uint32_t>(beamIndex));
    }
}

// 영 자세 격자는 설정된 각도 간격과 순서를 유지한다.
TEST(SearchGridTest, ZeroAttitudeKeepsFixedGrid)
{
    // Given: 격자의 자세 변환을 준비한다.
    const auto beamsTransform = makeTransform(0_deg, 0_deg, 0_deg);

    // When: 주어진 자세와 기본 설정으로 격자를 생성해 빔을 꺼낸다.
    const auto beams = takeAllBeams(beamsTransform);

    for (std::size_t beamIndex = 0; beamIndex < beams.size(); ++beamIndex)
    {

        // Then: 영 자세 격자는 설정된 각도 간격과 순서를 유지한다.
        EXPECT_TRUE(beams[beamIndex].direction_enu.azimuth.isEquivalentTo(gridAzimuth(beamIndex))) << "beamIndex=" << beamIndex;
        EXPECT_EQ(beams[beamIndex].direction_enu.elevation, gridElevation(beamIndex))
            << "beamIndex=" << beamIndex;
    }
}

// 격자는 낮은 고각의 왼쪽 방위각부터 탐색한다.
TEST(SearchGridTest, GridOrderStartsLowElevationLeftAzimuth)
{
    // Given: 격자의 자세 변환을 준비한다.
    const auto beamsTransform = makeTransform(0_deg, 0_deg, 0_deg);

    // When: 주어진 자세와 기본 설정으로 격자를 생성해 빔을 꺼낸다.
    const auto beams = takeAllBeams(beamsTransform);

    // 첫 빔: 가장 낮은 고각, 가장 왼쪽 방위각

    // Then: 격자는 낮은 고각의 왼쪽 방위각부터 탐색한다.
    EXPECT_TRUE(beams[0].direction_enu.azimuth.isEquivalentTo(-42_deg));
    EXPECT_EQ(beams[0].direction_enu.elevation, 3_deg);
    // 같은 고각의 마지막 빔: 가장 오른쪽 방위각
    EXPECT_TRUE(beams[20].direction_enu.azimuth.isEquivalentTo(42_deg));
    EXPECT_EQ(beams[20].direction_enu.elevation, 3_deg);
    // 다음 고각 줄의 첫 빔
    EXPECT_TRUE(beams[21].direction_enu.azimuth.isEquivalentTo(-42_deg));
    EXPECT_EQ(beams[21].direction_enu.elevation, 7.4_deg);
    // 마지막 빔: 가장 높은 고각, 가장 오른쪽 방위각
    EXPECT_TRUE(beams[188].direction_enu.azimuth.isEquivalentTo(42_deg));
    EXPECT_EQ(beams[188].direction_enu.elevation, 38.2_deg);
}

// 격자의 각 방향은 주어진 요 회전을 반영한다.
TEST(SearchGridTest, YawShiftsAzimuthsBySameAngle)
{
    // Given: 격자의 자세 변환을 준비한다.
    // 격자는 안테나 정면 기준이므로 안테나가 yaw 만큼 돌아가 있으면 ENU 방위각도 +yaw 만큼 이동
    const auto beamsTransform = makeTransform(0_deg, 0_deg, 10_deg);

    // When: 주어진 자세와 기본 설정으로 격자를 생성해 빔을 꺼낸다.
    const auto beams = takeAllBeams(beamsTransform);

    for (std::size_t beamIndex = 0; beamIndex < beams.size(); ++beamIndex)
    {

        // Then: 격자의 각 방향은 주어진 요 회전을 반영한다.
        EXPECT_TRUE(beams[beamIndex].direction_enu.azimuth.isEquivalentTo(gridAzimuth(beamIndex) + 10_deg))
            << "beamIndex=" << beamIndex;
        EXPECT_EQ(beams[beamIndex].direction_enu.elevation, gridElevation(beamIndex))
            << "beamIndex=" << beamIndex;
    }
}

// 격자는 각 셀의 안테나 방향을 ENU 방향으로 변환한다.
TEST(SearchGridTest, GridMatchesAnglesConvertedToEnuFrame)
{
    // Given: 격자의 자세 변환을 준비한다.
    const auto transform = makeTransform(10_deg, 20_deg, 30_deg);

    // When: 주어진 자세와 기본 설정으로 격자를 생성해 빔을 꺼낸다.
    const auto beams = takeAllBeams(transform);

    for (std::size_t beamIndex = 0; beamIndex < beams.size(); ++beamIndex)
    {
        const auto direction_enu = transform.antToEnu(
            math::SphericalDirection(
                gridAzimuth(beamIndex),
                gridElevation(beamIndex)));

        // Then: 격자는 각 셀의 안테나 방향을 ENU 방향으로 변환한다.
        EXPECT_TRUE(beams[beamIndex].direction_enu.azimuth.isEquivalentTo(direction_enu.azimuth)) << "beamIndex=" << beamIndex;
        EXPECT_EQ(beams[beamIndex].direction_enu.elevation, direction_enu.elevation) << "beamIndex=" << beamIndex;
    }
}

// 격자의 ENU 방향을 역변환하면 설정된 셀의 방향과 일치한다.
TEST(SearchGridTest, BeamsConvertedBackToAntennaFrameRestoreGrid)
{
    // Given: 격자의 자세 변환을 준비한다.
    const auto transform = makeTransform(10_deg, 20_deg, 30_deg);

    // When: 주어진 자세와 기본 설정으로 격자를 생성해 빔을 꺼낸다.
    const auto beams = takeAllBeams(transform);

    for (std::size_t beamIndex = 0; beamIndex < beams.size(); ++beamIndex)
    {
        const auto direction_ant = transform.enuToAnt(beams[beamIndex].direction_enu);

        // Then: 격자의 ENU 방향을 역변환하면 설정된 셀의 방향과 일치한다.
        EXPECT_TRUE(direction_ant.azimuth.isEquivalentTo(math::wrap360(gridAzimuth(beamIndex)))) << "beamIndex=" << beamIndex;
        EXPECT_EQ(direction_ant.elevation, gridElevation(beamIndex)) << "beamIndex=" << beamIndex;
    }
}

// 시나리오 자세의 격자는 기준 로그의 탐색 방향과 일치한다.
TEST(SearchGridTest, ScenarioAttitudeMatchesReferenceBeamAngles)
{
    // Given: 격자의 자세 변환을 준비한다.
    // 멘토 코드(DPMS_SIM) 로그의 탐색 빔 각도와 비교한다. 시나리오 자세는 roll 0, pitch 0, yaw 13.02429도이다.
    // 로그 값: BeamID 0 = 331.024292도, 10 = 13.024290도, 20 = 55.024288도 (고각 3도), 188 = 55.024288도 (고각 38.2도)
    // 멘토 BeamID와 우리 beamId는 모두 0부터 시작하므로 번호가 같은 빔끼리 비교한다.
    const auto beamsTransform = makeTransform(0_deg, 0_deg, 13.02429_deg);

    // When: 주어진 자세와 기본 설정으로 격자를 생성해 빔을 꺼낸다.
    const auto beams = takeAllBeams(beamsTransform);

    // Then: 시나리오 자세의 격자는 기준 로그의 탐색 방향과 일치한다.
    EXPECT_TRUE(beams[0].direction_enu.azimuth.isEquivalentTo(331.02429_deg));
    EXPECT_EQ(beams[0].direction_enu.elevation, 3_deg);
    EXPECT_TRUE(beams[10].direction_enu.azimuth.isEquivalentTo(13.02429_deg));
    EXPECT_EQ(beams[10].direction_enu.elevation, 3_deg);
    EXPECT_TRUE(beams[20].direction_enu.azimuth.isEquivalentTo(55.02429_deg));
    EXPECT_EQ(beams[20].direction_enu.elevation, 3_deg);
    EXPECT_TRUE(beams[188].direction_enu.azimuth.isEquivalentTo(55.02429_deg));
    EXPECT_EQ(beams[188].direction_enu.elevation, 38.2_deg);
}

// 사용자 격자는 설정한 빔 개수만큼 ID를 순환한다.
TEST(SearchGridTest, CustomGridChangesBeamCountAndIds)
{
    // Given: 검사할 설정과 탐색 상태를 준비한다.
    const auto config = validConfigBuilder()
        .azimuthCount(3)
        .elevationCount(2)
        .build();

    // When: 사용자 설정으로 격자를 생성하고 빔을 꺼낸다.
    auto grid = beam::SearchGrid(makeTransform(0_deg, 0_deg, 0_deg), config);

    for (std::size_t beamIndex = 0; beamIndex < 6; ++beamIndex)
    {

        // Then: 사용자 격자는 설정한 빔 개수만큼 ID를 순환한다.
        EXPECT_EQ(grid.nextBeam().beamId, static_cast<uint32_t>(beamIndex));
    }
    // 빔이 6개뿐이므로 7번째는 처음으로 돌아온다.
    EXPECT_EQ(grid.nextBeam().beamId, 0u);
}

// 사용자 격자는 설정한 시작각과 간격 및 빔 폭을 반영한다.
TEST(SearchGridTest, CustomGridUsesConfiguredStartStepAndWidth)
{
    // Given: 검사할 설정과 탐색 상태를 준비한다.
    const auto config = validConfigBuilder()
        .azimuthCount(3)
        .elevationCount(2)
        .azimuthStart(-10_deg)
        .azimuthStep(5_deg)
        .elevationStart(1_deg)
        .elevationStep(2_deg)
        .beamWidth(beam::BeamWidth(4_deg, 4_deg))
        .build();

    // When: 사용자 설정의 격자에서 모든 빔을 꺼낸다.
    const auto beams = takeAllBeams(makeTransform(0_deg, 0_deg, 0_deg), config);

    // 방위각 -10, -5, 0 / 고각 1, 3 (고각 바깥 루프, 방위각 안쪽 루프 순서)
    const math::Angle expectedAzimuth[] = {-10_deg, -5_deg, 0_deg, -10_deg, -5_deg, 0_deg};
    const math::Angle expectedElevation[] = {1_deg, 1_deg, 1_deg, 3_deg, 3_deg, 3_deg};

    // Then: 사용자 격자는 설정한 시작각과 간격 및 빔 폭을 반영한다.
    ASSERT_EQ(beams.size(), 6u);
    for (std::size_t beamIndex = 0; beamIndex < beams.size(); ++beamIndex)
    {
        EXPECT_TRUE(beams[beamIndex].direction_enu.azimuth.isEquivalentTo(expectedAzimuth[beamIndex]))
            << "beamIndex=" << beamIndex;
        EXPECT_EQ(beams[beamIndex].direction_enu.elevation, expectedElevation[beamIndex])
            << "beamIndex=" << beamIndex;
        EXPECT_EQ(beams[beamIndex].beamWidth.azimuth, 4_deg);
        EXPECT_EQ(beams[beamIndex].beamWidth.elevation, 4_deg);
    }
}

// 단일 빔 격자는 큰 각도 간격을 사용하지 않고 같은 빔을 순환한다.
TEST(SearchGridTest, LargeStepWithSingleBeamPerAxisIsAccepted)
{
    // Given: 검사할 설정과 탐색 상태를 준비한다.
    const auto maxAngle = math::Angle::fromDegrees(std::numeric_limits<double>::max());

    // count가 1이면 step은 쓰이지 않으므로 큰 step도 문제없다.
    const auto config = validConfigBuilder()
        .azimuthCount(1)
        .azimuthStep(maxAngle)
        .elevationCount(1)
        .elevationStep(maxAngle)
        .build();

    // When: 큰 각도 간격 설정으로 단일 빔 격자를 생성한다.
    auto grid = beam::SearchGrid(makeTransform(0_deg, 0_deg, 0_deg), config);

    // Then: 단일 빔 격자는 큰 각도 간격을 사용하지 않고 같은 빔을 순환한다.
    EXPECT_EQ(grid.nextBeam().beamId, 0u);
    EXPECT_EQ(grid.nextBeam().beamId, 0u);
}

// 단일 빔 격자는 매번 같은 빔을 반환한다.
TEST(SearchGridTest, SingleBeamGridReturnsTheSameBeamEveryTime)
{
    // Given: 검사할 설정과 탐색 상태를 준비한다.
    const auto config = validConfigBuilder()
        .azimuthCount(1)
        .elevationCount(1)
        .build();
    auto grid = beam::SearchGrid(makeTransform(0_deg, 0_deg, 0_deg), config);

    // When: 단일 빔 격자에서 연속으로 빔을 꺼낸다.
    const auto first = grid.nextBeam();
    const auto second = grid.nextBeam();
    const auto third = grid.nextBeam();

    // Then: 단일 빔 격자는 매번 같은 빔을 반환한다.
    EXPECT_EQ(first.beamId, 0u);
    EXPECT_EQ(second.beamId, 0u);
    EXPECT_EQ(third.beamId, 0u);
}

// 복사한 격자의 탐색 커서는 원본과 독립적으로 움직인다.
TEST(SearchGridTest, CopiedGridKeepsItsOwnCursor)
{
    // Given: 첫 빔을 꺼낸 격자를 준비한다.
    auto grid = beam::SearchGrid(makeTransform(0_deg, 0_deg, 0_deg), validConfigBuilder().build());
    grid.nextBeam();

    // When: 격자를 복사하고 원본에서 두 개, 복사본에서 한 개의 빔을 꺼낸다.
    auto copied = grid;
    const auto originalSecond = grid.nextBeam();
    const auto originalThird = grid.nextBeam();
    const auto copiedSecond = copied.nextBeam();

    // Then: 복사본의 커서는 원본의 진행에 영향을 받지 않는다.
    EXPECT_EQ(originalSecond.beamId, 1u);
    EXPECT_EQ(originalThird.beamId, 2u);
    EXPECT_EQ(copiedSecond.beamId, 1u);
}
