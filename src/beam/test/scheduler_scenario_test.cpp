// 시각화 도구(beam_schedule_dump, visualize_schedule.py)가 보여 주는 두 시나리오를 GTest로 고정한다.
// 시나리오 정의와 실행은 schedule_scenario.hpp 를 시각화 도구와 함께 쓴다.

#include "schedule_scenario.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

using namespace std::chrono_literals;
using namespace math::literals;
using schedule_scenario::Result;
using schedule_scenario::Status;
using schedule_scenario::TickRecord;

namespace
{

constexpr std::size_t BEAM_COUNT = 189;

beam::RadarAttitude makeAttitude(math::Angle roll, math::Angle pitch, math::Angle yaw)
{
    beam::RadarAttitude attitude{};
    attitude.latitude = 37.0_deg;
    attitude.longitude = 127.0_deg;
    attitude.altitude = 0.1_km;
    attitude.roll = roll;
    attitude.pitch = pitch;
    attitude.yaw = yaw;
    return attitude;
}

beam::RadarAttitude makeZeroAttitude()
{
    return makeAttitude(0_deg, 0_deg, 0_deg);
}

const TickRecord& tickAt(const Result& result, long long tickMs)
{
    const auto found = std::find_if(result.ticks.begin(), result.ticks.end(),
                                    [tickMs](const TickRecord& tick) { return tick.tickMs == tickMs; });
    EXPECT_NE(found, result.ticks.end()) << "no tick at " << tickMs << "ms";
    return *found;
}

const schedule_scenario::RequestSpec& requestNamed(const Result& result, const std::string& name)
{
    const auto found = std::find_if(result.requests.begin(), result.requests.end(),
                                    [&name](const schedule_scenario::RequestSpec& spec) { return spec.name == name; });
    EXPECT_NE(found, result.requests.end()) << "no request named " << name;
    return *found;
}

std::vector<beam::BeamCommand> sentCommands(const Result& result)
{
    std::vector<beam::BeamCommand> commands;
    for (const TickRecord& tick : result.ticks)
    {
        if (tick.command.has_value())
        {
            commands.push_back(*tick.command);
        }
    }
    return commands;
}

std::vector<std::uint32_t> searchBeamIds(const Result& result)
{
    std::vector<std::uint32_t> ids;
    for (const TickRecord& tick : result.ticks)
    {
        if (tick.command.has_value() && tick.command->beamType == beam::BeamCommand::BeamType::SEARCH)
        {
            ids.push_back(tick.command->beamId);
        }
    }
    return ids;
}

// 설계 규칙: 틱마다 빔은 하나, 명령 카운트는 1부터 연속, 탐색 번호는 1~189를 빠짐없이 순환한다.
void expectCommonRules(const Result& result)
{
    const std::vector<beam::BeamCommand> commands = sentCommands(result);
    for (std::size_t index = 0; index < commands.size(); ++index)
    {
        EXPECT_EQ(commands[index].commandCount, index + 1) << "command #" << index;
    }

    const std::vector<std::uint32_t> ids = searchBeamIds(result);
    for (std::size_t index = 0; index < ids.size(); ++index)
    {
        EXPECT_EQ(ids[index], index % BEAM_COUNT + 1) << "search beam #" << index;
    }

    for (const TickRecord& tick : result.ticks)
    {
        EXPECT_EQ(tick.command.has_value(), tick.status == Status::READY) << "tick " << tick.tickMs << "ms";
        if (!tick.command.has_value())
        {
            continue;
        }

        const beam::BeamCommand& command = *tick.command;
        EXPECT_EQ(command.transmitTime, std::chrono::milliseconds(tick.tickMs));
        if (command.beamType == beam::BeamCommand::BeamType::SEARCH)
        {
            EXPECT_DOUBLE_EQ(command.azimuthBeamWidth.deg(), 6.0) << "tick " << tick.tickMs << "ms";
            EXPECT_DOUBLE_EQ(command.elevationBeamWidth.deg(), 6.0) << "tick " << tick.tickMs << "ms";
        }
        else
        {
            EXPECT_EQ(command.beamId, 0u) << "tick " << tick.tickMs << "ms";
            EXPECT_DOUBLE_EQ(command.azimuthBeamWidth.deg(), 3.0) << "tick " << tick.tickMs << "ms";
            EXPECT_DOUBLE_EQ(command.elevationBeamWidth.deg(), 3.0) << "tick " << tick.tickMs << "ms";
        }
    }

    // 송신된 요청은 원한 시각 이후 종류별 허용 지연(확인 20ms, 추적 30ms) 안에서만 나간다.
    for (const schedule_scenario::RequestSpec& spec : result.requests)
    {
        if (spec.sentTickMs < 0)
        {
            continue;
        }

        const long long delay = spec.sentTickMs - spec.request.transmitTime.count();
        const long long limit = spec.request.beamType == beam::BeamRequest::BeamType::CONFIRMATION ? 20 : 30;
        EXPECT_GE(delay, 0) << spec.name;
        EXPECT_LT(delay, limit) << spec.name;
    }
}

} // namespace

// ---------- mixed: 요청이 끼어드는 순서와 폐기 ----------

TEST(SchedulerScenarioTest, MixedSendsNothingUntilReady)
{
    const Result result = schedule_scenario::run(schedule_scenario::makeMixedScenario(), makeZeroAttitude());

    EXPECT_EQ(tickAt(result, 0).status, Status::OFF);
    EXPECT_EQ(tickAt(result, 10).status, Status::OFF);
    EXPECT_EQ(tickAt(result, 20).status, Status::ON_WAIT_ATTITUDE);
    EXPECT_EQ(tickAt(result, 30).status, Status::ON_WAIT_ATTITUDE);
    for (long long tickMs : {0, 10, 20, 30})
    {
        EXPECT_FALSE(tickAt(result, tickMs).command.has_value()) << "tick " << tickMs << "ms";
    }

    const TickRecord& firstReady = tickAt(result, 40);
    ASSERT_TRUE(firstReady.command.has_value());
    EXPECT_EQ(firstReady.command->beamType, beam::BeamCommand::BeamType::SEARCH);
    EXPECT_EQ(firstReady.command->beamId, 1u);
    EXPECT_EQ(firstReady.command->commandCount, 1u);
}

TEST(SchedulerScenarioTest, MixedSendsRequestBeamsAtTheDesignedTicks)
{
    using Type = beam::BeamCommand::BeamType;
    const Result result = schedule_scenario::run(schedule_scenario::makeMixedScenario(), makeZeroAttitude());

    struct Expected
    {
        long long tickMs;
        Type beamType;
    };
    const std::vector<Expected> expected = {
        {100, Type::CONFIRMATION}, // C1
        {150, Type::TRACKING},     // T1
        {210, Type::TRACKING},     // A
        {220, Type::CONFIRMATION}, // B (같은 시각 요청 중 확인 우선)
        {230, Type::TRACKING},     // C
        {240, Type::SEARCH},       // D는 폐기되어 탐색
        {300, Type::TRACKING},     // E
        {310, Type::TRACKING},     // F
        {320, Type::TRACKING},     // G
        {330, Type::SEARCH},       // H는 폐기되어 탐색
    };
    for (const Expected& item : expected)
    {
        const TickRecord& tick = tickAt(result, item.tickMs);
        ASSERT_TRUE(tick.command.has_value()) << "tick " << item.tickMs << "ms";
        EXPECT_EQ(tick.command->beamType, item.beamType) << "tick " << item.tickMs << "ms";
    }
}

TEST(SchedulerScenarioTest, MixedMatchesEachRequestToItsSentTick)
{
    const Result result = schedule_scenario::run(schedule_scenario::makeMixedScenario(), makeZeroAttitude());

    EXPECT_EQ(requestNamed(result, "C1").sentTickMs, 100);
    EXPECT_EQ(requestNamed(result, "T1").sentTickMs, 150);
    EXPECT_EQ(requestNamed(result, "A").sentTickMs, 210);
    EXPECT_EQ(requestNamed(result, "B").sentTickMs, 220);
    EXPECT_EQ(requestNamed(result, "C").sentTickMs, 230);
    EXPECT_EQ(requestNamed(result, "E").sentTickMs, 300);
    EXPECT_EQ(requestNamed(result, "F").sentTickMs, 310);
    EXPECT_EQ(requestNamed(result, "G").sentTickMs, 320);
}

TEST(SchedulerScenarioTest, MixedDropsOnlyRequestsThatWaitedTooLong)
{
    const Result result = schedule_scenario::run(schedule_scenario::makeMixedScenario(), makeZeroAttitude());

    std::vector<std::string> dropped;
    for (const schedule_scenario::RequestSpec& spec : result.requests)
    {
        if (spec.sentTickMs < 0)
        {
            dropped.push_back(spec.name);
        }
    }

    EXPECT_EQ(dropped, (std::vector<std::string>{"D", "H"}));
}

TEST(SchedulerScenarioTest, MixedSearchSweepContinuesWithoutSkippingAfterInterruptions)
{
    const Result result = schedule_scenario::run(schedule_scenario::makeMixedScenario(), makeZeroAttitude());

    // 37개 빔 중 요청 빔이 8개이므로 탐색 빔은 29개이고, 번호는 1부터 건너뛰지 않고 이어진다.
    const std::vector<std::uint32_t> ids = searchBeamIds(result);
    ASSERT_EQ(ids.size(), 29u);
    for (std::size_t index = 0; index < ids.size(); ++index)
    {
        EXPECT_EQ(ids[index], index + 1);
    }
}

TEST(SchedulerScenarioTest, MixedFollowsCommonRules)
{
    expectCommonRules(schedule_scenario::run(schedule_scenario::makeMixedScenario(), makeZeroAttitude()));
}

// ---------- tracks: 추적 20개 / 1초 주기가 탐색 한 바퀴에 주는 영향 ----------

TEST(SchedulerScenarioTest, TracksSendsEveryRequestWithoutDelay)
{
    const Result result = schedule_scenario::run(schedule_scenario::makeTracksScenario(), makeZeroAttitude());

    // 요청이 50ms 간격이라 같은 틱을 두고 다투지 않으므로 모두 원한 시각 그대로 나가고 폐기는 없다.
    ASSERT_EQ(result.requests.size(), 59u);
    for (const schedule_scenario::RequestSpec& spec : result.requests)
    {
        EXPECT_EQ(spec.sentTickMs, spec.request.transmitTime.count()) << spec.name;
    }
}

TEST(SchedulerScenarioTest, TracksFirstSearchSweepIsLongerByTheNumberOfRequestBeams)
{
    const Result result = schedule_scenario::run(schedule_scenario::makeTracksScenario(), makeZeroAttitude());

    std::vector<long long> sweepStartTicks;
    for (const TickRecord& tick : result.ticks)
    {
        if (tick.command.has_value() && tick.command->beamType == beam::BeamCommand::BeamType::SEARCH &&
            tick.command->beamId == 1)
        {
            sweepStartTicks.push_back(tick.tickMs);
        }
    }
    ASSERT_GE(sweepStartTicks.size(), 2u);

    const long long sweepStart = sweepStartTicks[0];
    const long long nextSweepStart = sweepStartTicks[1];
    EXPECT_EQ(sweepStart, 40);

    // 한 바퀴 구간에서 송신된 요청 빔 수만큼 탐색이 뒤로 밀린다.
    long long requestBeamsInSweep = 0;
    for (const schedule_scenario::RequestSpec& spec : result.requests)
    {
        if (spec.sentTickMs >= sweepStart && spec.sentTickMs < nextSweepStart)
        {
            ++requestBeamsInSweep;
        }
    }
    EXPECT_EQ(nextSweepStart - sweepStart, (static_cast<long long>(BEAM_COUNT) + requestBeamsInSweep) * 10);
    EXPECT_EQ(nextSweepStart - sweepStart, 2350);
    EXPECT_GT(nextSweepStart - sweepStart, static_cast<long long>(BEAM_COUNT) * 10);
}

TEST(SchedulerScenarioTest, TracksFirstSweepVisitsEverySearchBeamExactlyOnce)
{
    const Result result = schedule_scenario::run(schedule_scenario::makeTracksScenario(), makeZeroAttitude());

    const std::vector<std::uint32_t> ids = searchBeamIds(result);
    ASSERT_GE(ids.size(), BEAM_COUNT);
    std::vector<std::uint32_t> firstSweep(ids.begin(), ids.begin() + BEAM_COUNT);
    for (std::size_t index = 0; index < BEAM_COUNT; ++index)
    {
        EXPECT_EQ(firstSweep[index], index + 1);
    }
}

TEST(SchedulerScenarioTest, TracksFollowsCommonRules)
{
    expectCommonRules(schedule_scenario::run(schedule_scenario::makeTracksScenario(), makeZeroAttitude()));
}

// ---------- 자세와의 독립성 ----------

TEST(SchedulerScenarioTest, AttitudeChangesAnglesButNotTheSchedule)
{
    const Result zero = schedule_scenario::run(schedule_scenario::makeMixedScenario(), makeZeroAttitude());
    const Result rotated = schedule_scenario::run(
        schedule_scenario::makeMixedScenario(), makeAttitude(10_deg, 20_deg, 30_deg));

    ASSERT_EQ(zero.ticks.size(), rotated.ticks.size());
    bool anySearchAngleDiffers = false;
    for (std::size_t index = 0; index < zero.ticks.size(); ++index)
    {
        const TickRecord& left = zero.ticks[index];
        const TickRecord& right = rotated.ticks[index];
        ASSERT_EQ(left.command.has_value(), right.command.has_value()) << "tick " << left.tickMs << "ms";
        if (!left.command.has_value())
        {
            continue;
        }

        // 어떤 빔이 나가는지(종류, 번호, 카운트)는 자세와 무관하다.
        EXPECT_EQ(left.command->beamType, right.command->beamType) << "tick " << left.tickMs << "ms";
        EXPECT_EQ(left.command->beamId, right.command->beamId) << "tick " << left.tickMs << "ms";
        EXPECT_EQ(left.command->commandCount, right.command->commandCount) << "tick " << left.tickMs << "ms";

        if (left.command->beamType == beam::BeamCommand::BeamType::SEARCH)
        {
            anySearchAngleDiffers |= !left.command->azimuth_ant.equals(right.command->azimuth_ant, 0.001_deg);
        }
        else
        {
            // 요청 빔의 각도는 요청 그대로 나가므로 자세와 무관하다.
            EXPECT_DOUBLE_EQ(left.command->azimuth_ant.deg(), right.command->azimuth_ant.deg());
            EXPECT_DOUBLE_EQ(left.command->elevation_ant.deg(), right.command->elevation_ant.deg());
        }
    }

    // 탐색 빔의 안테나 각도는 자세에 따라 달라진다.
    EXPECT_TRUE(anySearchAngleDiffers);

    for (std::size_t index = 0; index < zero.requests.size(); ++index)
    {
        EXPECT_EQ(zero.requests[index].sentTickMs, rotated.requests[index].sentTickMs) << zero.requests[index].name;
    }
}
