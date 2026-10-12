#include <app/worker/comm_worker.hpp>
#include <app/worker/worker_error.hpp>

#include <chrono>
#include <thread>
#include <utility>
#include <variant>
#include <vector>

namespace app
{
CommWorker::CommWorker(
    CommChannel& commChannel,
    BeamChannel& beamChannel,
    TargetChannel& targetChannel,
    comm::CommController& controller)
    : _commChannel(commChannel),
      _beamChannel(beamChannel),
      _targetChannel(targetChannel),
      _controller(controller)
{
}

void CommWorker::runReceive(
    const std::atomic_bool& stopRequested)
{
    while (!stopRequested.load())
    {
        const bool success = tryWorkerAction("CommReceive", [&]
        {
            auto incoming = _controller.receiveData();

            if (stopRequested.load())
            {
                return;
            }

            bool accepted;

            if (auto* attitude =
                    std::get_if<beam::SetAttitudeCommand>(&incoming))
            {
                accepted = _beamChannel.write(std::move(*attitude));
            }
            else if (auto* state =
                         std::get_if<beam::SetOperationStateCommand>(&incoming))
            {
                accepted = _beamChannel.write(std::move(*state));
            }
            else
            {
                auto& detection =
                    std::get<target::DetectionEvent>(incoming);

                accepted = _targetChannel.write(std::move(detection));
            }

            if (!accepted)
            {
                reportWorkerError(
                    "CommReceive",
                    "Destination closed; message dropped");
            }
        });

        // 수신 오류가 즉시 반복될 때 과도한 재호출을 방지한다.
        if (!success && !stopRequested.load())
        {
            std::this_thread::sleep_for(
                std::chrono::milliseconds(100));
        }
    }
}

void CommWorker::runSendSimulator()
{
    while (auto event = _commChannel.readSimulator())
    {
        tryWorkerAction("CommSendSimulator", [&]
        {
            const auto& command =
                std::get<beam::BeamCommand>(event->message);

            _controller.sendBeamCommand(command);
        });
    }
}

void CommWorker::runSendConsole()
{
    while (auto event = _commChannel.readConsole())
    {
        tryWorkerAction("CommSendConsole", [&]
        {
            const auto& tracks =
                std::get<std::vector<target::TrackEvent>>(
                    event->message);

            _controller.sendTrackEvents(tracks);
        });
    }
}
} // namespace app