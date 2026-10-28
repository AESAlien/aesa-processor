// Scheduler를 시나리오로 실행해 틱별 결과를 CSV로 출력하는 시각화용 도구
//
// 사용법: beam_schedule_dump [--scenario mixed|tracks] [roll_deg pitch_deg yaw_deg]
// 출력  : 빈 줄 없이 "# ticks", "# requests", "# grid" 세 구역을 차례로 출력한다.
//   # ticks    tick_ms,status,type,beamID,commandCount,az_deg,el_deg,az_width_deg,el_width_deg
//              (송신할 빔이 없는 틱은 type 이 NONE 이고 나머지 칸이 비어 있다)
//   # requests name,type,timestamp_ms,az_deg,el_deg,sent_tick_ms   (시나리오가 끝날 때까지 송신되지 못한 요청은 -1)
//   # grid     idx,beamID,az_deg,el_deg   (SearchGrid 189개, ENU 기준 각도)
//
// 시나리오 정의와 실행은 schedule_scenario.hpp 를 scheduler_scenario_test 와 함께 쓴다.
// visualize_schedule.py 가 이 프로그램을 실행해서 결과를 그린다.

#include "schedule_scenario.hpp"

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <exception>
#include <stdexcept>
#include <string>

namespace
{

using namespace math::literals;

const char* typeName(beam::BeamType beamType)
{
    switch (beamType)
    {
    case beam::BeamType::SEARCH:
        return "SEARCH";

    case beam::BeamType::CONFIRMATION:
        return "CONFIRMATION";

    case beam::BeamType::TRACKING:
        return "TRACKING";
    }
    return "UNKNOWN";
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

    const beam::SearchGrid grid = [&]() {
        try
        {
            // 잘못된 자세정보이면 RadarAttitude 생성자나 AntEnuTransform 생성자가 std::invalid_argument를 던진다.
            return schedule_scenario::makeSearchGrid(roll, pitch, yaw);
        }
        catch (const std::invalid_argument& e)
        {
            std::fprintf(stderr, "attitude failed: %s\n", e.what());
            std::exit(3);
        }
    }();

    try
    {
        const schedule_scenario::Result result = schedule_scenario::run(scenario, grid);

        std::printf("# ticks\n");
        std::printf("tick_ms,status,type,beamID,commandCount,az_deg,el_deg,az_width_deg,el_width_deg\n");
        for (const schedule_scenario::TickRecord& tick : result.ticks)
        {
            if (!tick.command.has_value())
            {
                std::printf("%lld,%s,NONE,,,,,,\n", tick.tick_ms, statusName(tick.status));
                continue;
            }

            const beam::TransmitBeamCommand& command = *tick.command;
            std::printf(
                "%lld,%s,%s,%u,%u,%.6f,%.6f,%.2f,%.2f\n",
                tick.tick_ms,
                statusName(tick.status),
                typeName(command.beamType),
                command.beamId,
                command.commandCount,
                command.direction_ant.azimuth.deg(),
                command.direction_ant.elevation.deg(),
                command.beamWidth.azimuth.deg(),
                command.beamWidth.elevation.deg());
        }

        std::printf("# requests\n");
        std::printf("name,type,timestamp_ms,az_deg,el_deg,sent_tick_ms\n");
        for (const schedule_scenario::RequestSpec& spec : result.requests)
        {
            std::printf(
                "%s,%s,%lld,%.6f,%.6f,%lld\n",
                spec.name.c_str(),
                typeName(spec.request.transmitBeamType()),
                static_cast<long long>(spec.request.transmitTime().count()),
                spec.request.direction_enu().azimuth.deg(),
                spec.request.direction_enu().elevation.deg(),
                spec.sentTick_ms);
        }

        std::printf("# grid\n");
        std::printf("idx,beamID,az_deg,el_deg\n");
        // run()은 grid의 복사본을 쓰므로 여기서는 처음 빔부터 꺼낸다.
        beam::SearchGrid gridToPrint = grid;
        const beam::SearchGridConfig gridConfig = schedule_scenario::makeSearchGridConfig();
        const std::size_t beamCount = gridConfig.azimuthCount() * gridConfig.elevationCount();
        for (std::size_t index = 0; index < beamCount; ++index)
        {
            const beam::SearchBeam& info = gridToPrint.nextBeam();
            std::printf("%zu,%u,%.6f,%.6f\n", index, info.beamId, info.direction_enu.azimuth.deg(), info.direction_enu.elevation.deg());
        }
    }
    catch (const std::exception& e)
    {
        std::fprintf(stderr, "schedule dump failed: %s\n", e.what());
        return 4;
    }
    return 0;
}
