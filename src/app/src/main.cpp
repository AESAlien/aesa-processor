#include <app/worker/worker_error.hpp>

#include <beam/controller/beam_controller.hpp>
#include <target/controller/target_controller.hpp>

#include <chrono>
#include <iostream>
#include <stdexcept>
#include <thread>
#include <utility>
#include <vector>

namespace demo
{

using BeamType = beam::BeamCommand::BeamType;

struct Result
{
    bool searchResultSubmitted = false;

    int confirmationCommands = 0;
    int trackingCommands = 0;
    int reportedTracks = 0;
};

void require(bool condition, const char* message)
{
    if (!condition)
    {
        throw std::runtime_error(message);
    }
}

void printAngle(
    const char* name,
    const math::Angle& angle)
{
    std::cout << ", " << name << '=';
    std::cout << angle.deg();
}

void printTracks(
    const std::vector<target::TrackEvent>& tracks)
{
    std::cout
        << "[TRACKS] count="
        << tracks.size()
        << '\n';

    for (const auto& track : tracks)
    {
        std::cout
            << "[CONSOLE OUTPUT]"
            << " id=" << track.id
            << ", state=" << static_cast<int>(track.state)
            << ", range(m)=" << track.slantRange.m()
            << ", ground(m)=" << track.groundRange.m()
            << ", az(deg)=" << track.azimuth.deg()
            << ", el(deg)=" << track.elevation.deg()
            << ", doppler(m/s)=" << track.dopplerVelocity.mps()
            << ", speed(m/s)=" << track.velocity.mps()
            << ", altitude(m)=" << track.altitude.m()
            << ", timestamp(ms)=" << track.timestamp.count();

        printAngle("latitude(deg)", track.latitude);
        printAngle("longitude(deg)", track.longitude);
        printAngle("heading(deg)", track.heading);
        printAngle("flightPath(deg)", track.flightPathAngle);

        std::cout << '\n';
    }
}

target::DetectionEvent makeStationaryDetection(
    const beam::BeamCommand& command)
{
    std::vector<target::DetectionEvent::Plot> plots;

    // 거리 12km, 방위각 0도, 고각 0도의 정지 표적을
    // 탐지했다고 가정하는 테스트 입력이다.
    //
    // 실제 빔 범위나 모의기의 탐지 과정은 구현하지 않는다.
    plots.emplace_back(
        math::Distance::fromMeters(12000),
        math::Angle::fromDegrees(0),
        math::Angle::fromDegrees(0),
        math::LinearVelocity::fromMetersPerSecond(0),
        30.0f);

    // 확인/추적 결과는 해당 빔 명령의 ID를 그대로 반환한다.
    // 현재 스케줄러는 요청의 requestId를 beamId에 넣는다.
    return target::DetectionEvent(
        command.beamType,
        command.beamId,
        command.transmitTime,
        std::move(plots));
}

void handleResult(
    beam::BeamController& beamController,
    target::TargetController& targetController,
    const beam::BeamCommand& command,
    Result& result)
{
    std::cout
        << "[DETECTION INPUT]"
        << " type=" << static_cast<int>(command.beamType)
        << ", requestId=" << command.beamId
        << ", time(ms)=" << command.transmitTime.count()
        << '\n';

    // 실제 Target 모듈에서 탐지정보를 처리한다.
    auto request = targetController.handleDetection(
        makeStationaryDetection(command));

    if (request)
    {
        std::cout
            << "[TARGET REQUEST]"
            << " type=" << static_cast<int>(request->beamType)
            << ", requestId=" << request->requestId
            << ", transmitTime(ms)="
            << request->transmitTime.count()
            << '\n';

        // Target이 생성한 확인/추적 요청을
        // 실제 Beam 모듈에 전달한다.
        beamController.requestBeam(*request);
    }
    else
    {
        std::cout << "[TARGET REQUEST] none\n";
    }

    // 실제 Target 모듈이 보유한 추적정보를 가져온다.
    const auto tracks = targetController.nextTrackEvents();

    printTracks(tracks);

    result.reportedTracks +=
        static_cast<int>(tracks.size());
}

} // namespace demo

int main()
{
    using namespace std::chrono_literals;

    try
    {
        beam::BeamController beamController;
        target::TargetController targetController;

        demo::Result result;

        // 운용 ON 이전에는 빔이 생성되지 않아야 한다.
        demo::require(
            !beamController.nextBeamCommand(0ms),
            "Beam generated before operation ON");

        beamController.setOperationState(
            beam::SetOperationStateCommand(
                beam::OperationState::ON));

        beamController.setAttitude(
            beam::SetAttitudeCommand(
                math::Angle::fromDegrees(37),
                math::Angle::fromDegrees(127),
                math::Distance::fromMeters(100),
                math::Angle::fromDegrees(0),
                math::Angle::fromDegrees(0),
                math::Angle::fromDegrees(0)));

        const auto start =
            std::chrono::steady_clock::now();

        // 10ms 간격으로 스케줄러를 호출한다.
        // currentTime은 테스트용 논리 시각이다.
        for (
            auto currentTime = 0ms;
            currentTime <= 2200ms;
            currentTime += 10ms)
        {
            std::this_thread::sleep_until(
                start + currentTime);

            const bool success = app::tryWorkerAction(
                "TargetExample",
                [&]
                {
                    const auto command =
                        beamController.nextBeamCommand(
                            currentTime);

                    if (!command)
                    {
                        return;
                    }

                    // 검색빔 출력은 100ms마다 표시한다.
                    // 확인/추적빔은 모두 표시한다.
                    if (
                        command->beamType !=
                            demo::BeamType::SEARCH
                        || currentTime.count() % 100 == 0)
                    {
                        std::cout
                            << "[BEAM OUTPUT]"
                            << " type="
                            << static_cast<int>(
                                command->beamType)
                            << ", beamId="
                            << command->beamId
                            << ", count="
                            << command->commandCount
                            << ", time(ms)="
                            << command->transmitTime.count()
                            << '\n';
                    }

                    if (
                        command->beamType ==
                        demo::BeamType::SEARCH)
                    {
                        // 동일한 표적을 여러 번 신규 등록하지 않도록
                        // 검색 탐지정보는 처음 한 번만 입력한다.
                        if (result.searchResultSubmitted)
                        {
                            return;
                        }

                        result.searchResultSubmitted = true;
                    }
                    else if (
                        command->beamType ==
                        demo::BeamType::CONFIRMATION)
                    {
                        ++result.confirmationCommands;
                    }
                    else if (
                        command->beamType ==
                        demo::BeamType::TRACKING)
                    {
                        ++result.trackingCommands;
                    }

                    demo::handleResult(
                        beamController,
                        targetController,
                        *command,
                        result);
                });

            // 예외는 tryWorkerAction에서 출력한다.
            // 이 테스트에서는 처리 실패를 PASS로 판단하지 않도록
            // 실패 시 테스트를 종료한다.
            demo::require(
                success,
                "Controller processing failed; see error log");
        }

        beamController.setOperationState(
            beam::SetOperationStateCommand(
                beam::OperationState::OFF));

        demo::require(
            !beamController.nextBeamCommand(2210ms),
            "Beam generated after operation OFF");

        demo::require(
            result.confirmationCommands >= 1,
            "Confirmation request was not delivered");

        demo::require(
            result.trackingCommands >= 1
                && result.reportedTracks >= 1,
            "Tracking or track reporting failed");

        std::cout
            << "[PASS] Real TargetController: "
            << "search -> confirmation -> tracking\n";

        return 0;
    }
    catch (const std::exception& error)
    {
        std::cerr
            << "[FAIL] "
            << error.what()
            << '\n';
    }
    catch (...)
    {
        std::cerr << "[FAIL] Unknown error\n";
    }

    return 1;
}