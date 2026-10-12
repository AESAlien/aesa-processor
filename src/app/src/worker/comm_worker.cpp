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
        bool idle = false;

        const bool success = tryWorkerAction("CommReceive", [&]
        {
            auto incoming = _controller.receiveData();

            if (stopRequested.load())
            {
                return;
            }

            if (!incoming)
            {
                idle = true;
                return;
            }

            auto& message = *incoming;
            bool accepted;

            if (auto* attitude =
                    std::get_if<beam::SetAttitudeCommand>(&message))
            {
                accepted =
                    _beamChannel.write(std::move(*attitude));
            }
            else if (auto* state =
                         std::get_if<beam::SetOperationStateCommand>(&message))
            {
                accepted =
                    _beamChannel.write(std::move(*state));
            }
            else
            {
                auto& detection =
                    std::get<target::DetectionEvent>(message);

                accepted =
                    _targetChannel.write(std::move(detection));
            }

            if (!accepted)
            {
                reportWorkerError(
                    "CommReceive",
                    "Destination closed; message dropped");
            }
        });

        if (!success && !stopRequested.load())
        {
            std::this_thread::sleep_for(
                std::chrono::milliseconds(100));
        }
        else if (idle && !stopRequested.load())
        {
            // receiveData가 즉시 nullopt를 반복 반환하는 경우
            // 과도한 반복을 방지한다.
            std::this_thread::sleep_for(
                std::chrono::milliseconds(1));
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