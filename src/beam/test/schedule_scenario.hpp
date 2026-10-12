// Scheduler를 시나리오로 실행하는 테스트/시각화 공용 헤더
//
// beam_schedule_dump(시각화용 도구)와 scheduler_scenario_test(GTest)가 같은 시나리오와 실행 로직을
// 쓰도록 한 곳에 둔다. 그림으로 본 동작과 테스트가 검증하는 동작이 어긋나지 않게 하기 위함이다.
//
// 시나리오:
//   mixed  : 짧은 구간(400ms)에 확인/추적 요청을 섞어 넣고, 같은 시각에 몰릴 때의 순서와 폐기를 본다.
//   tracks : 추적 20개를 1초 주기(50ms 간격)로 요청하며 3초 동안 돌려, 요청이 탐색 한 바퀴에 주는 영향을 본다.

#pragma once

#include "scheduler/scheduler.hpp"

#include <chrono>
#include <cmath>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace schedule_scenario
{

using namespace std::chrono_literals;

constexpr std::chrono::milliseconds TICK_PERIOD = 10ms;

enum class Status
{
    OFF,
    ON_WAIT_ATTITUDE,
    READY
};

struct RequestSpec
{
    std::string name;
    beam::BeamRequest request;
    // 이 요청 때문에 송신된 빔의 틱(ms). 송신되지 못하고 폐기되었으면 -1
    long long sentTick_ms = -1;
};

struct Scenario
{
    std::chrono::milliseconds duration = 0ms;
    std::chrono::milliseconds onTime = 0ms;
    std::chrono::milliseconds attitudeTime = 0ms;
    std::vector<RequestSpec> requests;
};

struct TickRecord
{
    long long tick_ms = 0;
    Status status = Status::OFF;
    std::optional<beam::BeamCommand> command;
};

struct Result
{
    std::vector<TickRecord> ticks;
    std::vector<RequestSpec> requests;
};

inline RequestSpec makeRequestSpec(
    const std::string& name,
    beam::BeamRequest::BeamType beamType,
    std::chrono::milliseconds transmitTime,
    double azimuth_deg,
    double elevation_deg
) {
    return RequestSpec{
        name,
        beam::BeamRequest::Builder()
            .beamType(beamType)
            .transmitTime(transmitTime)
            .requestId(0)
            .azimuth_ant(math::Angle::fromDegrees(azimuth_deg))
            .elevation_ant(math::Angle::fromDegrees(elevation_deg))
            .build()
    };
}

inline Scenario makeMixedScenario()
{
    using Type = beam::BeamRequest::BeamType;

    Scenario scenario;
    scenario.duration = 400ms;
    scenario.onTime = 20ms;
    scenario.attitudeTime = 40ms;
    scenario.requests = std::vector<RequestSpec>{
        makeRequestSpec("C1", Type::CONFIRMATION, 100ms, -10.0, 12.0),
        makeRequestSpec("T1", Type::TRACKING, 150ms, 20.0, 25.0),
        // 200ms 부근에 네 요청이 몰린다: 송신 순서는 A -> B(확인 우선) -> C, D는 지연이 길어져 폐기.
        // 확인 B를 도착 순서상 가장 나중에 넣어서, 도착 순서가 아니라 종류 때문에 먼저 나가는지 드러나게 한다.
        makeRequestSpec("A", Type::TRACKING, 205ms, -30.0, 8.0),
        makeRequestSpec("C", Type::TRACKING, 210ms, 15.0, 30.0),
        makeRequestSpec("D", Type::TRACKING, 210ms, 25.0, 15.0),
        makeRequestSpec("B", Type::CONFIRMATION, 210ms, 5.0, 20.0),
        // 300ms에 추적 4개가 같은 시각: E, F, G는 지연되어 송신되고 H는 폐기
        makeRequestSpec("E", Type::TRACKING, 300ms, -20.0, 35.0),
        makeRequestSpec("F", Type::TRACKING, 300ms, -15.0, 35.0),
        makeRequestSpec("G", Type::TRACKING, 300ms, -10.0, 35.0),
        makeRequestSpec("H", Type::TRACKING, 300ms, -5.0, 35.0),
    };
    return scenario;
}

inline Scenario makeTracksScenario()
{
    using Type = beam::BeamRequest::BeamType;

    constexpr int TRACK_COUNT = 20;
    constexpr int ROUND_COUNT = 3;

    Scenario scenario;
    scenario.duration = 3000ms;
    scenario.onTime = 20ms;
    scenario.attitudeTime = 40ms;
    for (int round = 0; round < ROUND_COUNT; ++round)
    {
        for (int track = 0; track < TRACK_COUNT; ++track)
        {
            const std::chrono::milliseconds transmitTime(100 + 50 * track + 1000 * round);
            if (transmitTime > scenario.duration)
            {
                continue;
            }

            // 요청과 송신 결과를 짝지을 수 있도록 각도가 요청마다 서로 다르게 정한다.
            const double azimuth_deg = -55.0 + 5.5 * track + 0.25 * round;
            const double elevation_deg = 3.0 + 5.0 * (track % 10) + 0.1 * round;
            scenario.requests.push_back(makeRequestSpec(
                "T" + std::to_string(track + 1) + "." + std::to_string(round + 1),
                Type::TRACKING, transmitTime, azimuth_deg, elevation_deg));
        }
    }
    return scenario;
}

namespace detail
{

inline bool sameAngle(math::Angle left, math::Angle right)
{
    return std::abs(left.deg() - right.deg()) < 1e-9;
}

// 송신된 요청 빔을 보낸 요청과 짝짓는다(종류와 각도가 같은 아직 짝이 없는 요청).
inline void markSent(
    std::vector<RequestSpec>& requests,
    const beam::BeamCommand& command,
    long long tick_ms
) {
    for (RequestSpec& spec : requests)
    {
        const bool sameType = static_cast<int>(spec.request.beamType) == static_cast<int>(command.beamType);
        if (spec.sentTick_ms < 0 && sameType && sameAngle(spec.request.azimuth_ant, command.azimuth_ant) &&
            sameAngle(spec.request.elevation_ant, command.elevation_ant))
        {
            spec.sentTick_ms = tick_ms;
            return;
        }
    }
}

} // namespace detail

// 시나리오를 새 Scheduler로 실행한다. 잘못된 자세정보면 AttitudeError가 전파된다.
inline Result run(Scenario scenario, const beam::RadarAttitude& attitude)
{
    Result result;
    beam::Scheduler scheduler;
    Status status = Status::OFF;

    for (auto time = 0ms; time <= scenario.duration; time += TICK_PERIOD)
    {
        if (time == scenario.onTime)
        {
            scheduler.setOperationState(beam::OperationState::ON);
            status = Status::ON_WAIT_ATTITUDE;
        }
        if (time == scenario.attitudeTime)
        {
            scheduler.setAttitude(attitude);
            for (const RequestSpec& spec : scenario.requests)
            {
                scheduler.addRequest(spec.request);
            }
            status = Status::READY;
        }

        TickRecord record{
            time.count(),
            status,
            scheduler.next(time)
        };
        if (record.command.has_value() && record.command->beamType != beam::BeamCommand::BeamType::SEARCH)
        {
            detail::markSent(scenario.requests, *record.command, record.tick_ms);
        }
        result.ticks.push_back(std::move(record));
    }

    result.requests = std::move(scenario.requests);
    return result;
}

} // namespace schedule_scenario
