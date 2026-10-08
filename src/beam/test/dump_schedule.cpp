// Scheduler를 시나리오로 실행해 틱별 결과를 CSV로 출력하는 시각화용 도구
//
// 사용법: beam_schedule_dump [--scenario mixed|tracks] [roll_deg pitch_deg yaw_deg]
// 출력  : 빈 줄 없이 "# ticks", "# requests", "# grid" 세 구역을 차례로 출력한다.
//   # ticks    tick_ms,status,type,beamID,commandCount,az_deg,el_deg,az_width_deg,el_width_deg
//              (송신할 빔이 없는 틱은 type 이 NONE 이고 나머지 칸이 비어 있다)
//   # requests name,type,timestamp_ms,az_deg,el_deg,sent_tick_ms   (송신되지 못한 요청은 -1)
//   # grid     idx,beamID,az_deg,el_deg   (BeamTable 189개, 안테나 기준 각도)
//
// 시나리오 정의와 실행은 schedule_scenario.hpp 를 scheduler_scenario_test 와 함께 쓴다.
// visualize_schedule.py 가 이 프로그램을 실행해서 결과를 그린다.

#include "schedule_scenario.hpp"

#include "attitude/ant_enu_transform.hpp"
#include "attitude/attitude_error.hpp"

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <exception>
#include <string>

namespace
{

using namespace math::literals;

const char* typeName(beam::BeamCommand::BeamType beamType)
{
    switch (beamType)
    {
    case beam::BeamCommand::BeamType::SEARCH:
        return "SEARCH";

    case beam::BeamCommand::BeamType::CONFIRMATION:
        return "CONFIRMATION";

    case beam::BeamCommand::BeamType::TRACKING:
        return "TRACKING";
    }
    return "UNKNOWN";
}

const char* typeName(beam::BeamRequest::BeamType beamType)
{
    return typeName(static_cast<beam::BeamCommand::BeamType>(beamType));
}

const char* statusName(schedule_scenario::Status status)
{
    switch (status)
    {
    case schedule_scenario::Status::OFF:
        return "OFF";

    case schedule_scenario::Status::ON_WAIT_ATTITUDE:
        return "ON_WAIT_ATTITUDE";

    case schedule_scenario::Status::READY:
        return "READY";
    }
    return "UNKNOWN";
}

int usage(const char* program)
{
    std::fprintf(stderr, "usage: %s [--scenario mixed|tracks] [roll_deg pitch_deg yaw_deg]\n", program);
    return 2;
}

} // namespace

int main(int argc, char** argv)
{
    int first = 1;
    std::string scenarioName = "mixed";
    if (argc > first && std::strcmp(argv[first], "--scenario") == 0)
    {
        if (argc <= first + 1)
        {
            return usage(argv[0]);
        }
        scenarioName = argv[first + 1];
        first += 2;
    }

    auto roll = 0_deg;
    auto pitch = 0_deg;
    auto yaw = 0_deg;
    if (argc - first == 3)
    {
        roll = math::Angle::fromDegrees(std::atof(argv[first]));
        pitch = math::Angle::fromDegrees(std::atof(argv[first + 1]));
        yaw = math::Angle::fromDegrees(std::atof(argv[first + 2]));
    }
    else if (argc - first != 0)
    {
        return usage(argv[0]);
    }

    schedule_scenario::Scenario scenario;
    if (scenarioName == "mixed")
    {
        scenario = schedule_scenario::makeMixedScenario();
    }
    else if (scenarioName == "tracks")
    {
        scenario = schedule_scenario::makeTracksScenario();
    }
    else
    {
        return usage(argv[0]);
    }

    beam::RadarAttitude attitude{};
    attitude.latitude = 37_deg;
    attitude.longitude = 127_deg;
    attitude.altitude = 0.1_km;
    attitude.roll = roll;
    attitude.pitch = pitch;
    attitude.yaw = yaw;

    try
    {
        const schedule_scenario::Result result = schedule_scenario::run(scenario, attitude);

        std::printf("# ticks\n");
        std::printf("tick_ms,status,type,beamID,commandCount,az_deg,el_deg,az_width_deg,el_width_deg\n");
        for (const schedule_scenario::TickRecord& tick : result.ticks)
        {
            if (!tick.command.has_value())
            {
                std::printf("%lld,%s,NONE,,,,,,\n", tick.tickMs, statusName(tick.status));
                continue;
            }

            const beam::BeamCommand& command = *tick.command;
            std::printf(
                "%lld,%s,%s,%u,%u,%.6f,%.6f,%.2f,%.2f\n",
                tick.tickMs,
                statusName(tick.status),
                typeName(command.beamType),
                command.beamId,
                command.commandCount,
                command.azimuth_ant.deg(),
                command.elevation_ant.deg(),
                command.azimuthBeamWidth.deg(),
                command.elevationBeamWidth.deg());
        }

        std::printf("# requests\n");
        std::printf("name,type,timestamp_ms,az_deg,el_deg,sent_tick_ms\n");
        for (const schedule_scenario::RequestSpec& spec : result.requests)
        {
            std::printf(
                "%s,%s,%lld,%.6f,%.6f,%lld\n",
                spec.name.c_str(),
                typeName(spec.request.beamType),
                static_cast<long long>(spec.request.timestamp.count()),
                spec.request.azimuth_ant.deg(),
                spec.request.elevation_ant.deg(),
                spec.sentTickMs);
        }

        std::printf("# grid\n");
        std::printf("idx,beamID,az_deg,el_deg\n");
        const beam::BeamTable table(beam::AntEnuTransform(attitude, beam::AttitudeConfig{}));
        for (std::size_t index = 0; index < beam::BeamTable::size(); ++index)
        {
            const beam::BeamInfo& info = table.get(index);
            std::printf("%zu,%u,%.6f,%.6f\n", index, info.beamId, info.azimuth_ant.deg(), info.elevation_ant.deg());
        }
    }
    catch (const beam::AttitudeError& e)
    {
        std::fprintf(stderr, "attitude failed: code=%d (%s)\n", static_cast<int>(e.code()), e.what());
        return 3;
    }
    catch (const std::exception& e)
    {
        std::fprintf(stderr, "schedule dump failed: %s\n", e.what());
        return 4;
    }
    return 0;
}
