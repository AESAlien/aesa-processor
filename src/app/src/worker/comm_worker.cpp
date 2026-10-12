#include <app/worker/comm_worker.hpp>
#include <app/worker/worker_error.hpp>

#include <chrono>
#include <exception>
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

void CommWorker::runReceive()
{
    while (true)
    {
        try
        {
            auto incoming = _controller.receiveData();

            if (!incoming)
            {
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

            continue;
        }
        catch (const std::exception& error)
        {
            reportWorkerError(
                "CommReceive",
                error.what());
        }
        catch (...)
        {
            reportWorkerError(
                "CommReceive",
                "Unknown exception");
        }

        std::this_thread::sleep_for(
            std::chrono::milliseconds(100));
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