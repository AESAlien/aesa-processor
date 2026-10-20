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

            if (auto* attitude =
                    std::get_if<beam::SetAttitudeCommand>(&message))
            {
                _beamChannel.write(std::move(*attitude));
            }
            else if (auto* state =
                         std::get_if<beam::SetOperationStateCommand>(&message))
            {
                _beamChannel.write(std::move(*state));
            }
            else
            {
                auto& detection =
                    std::get<target::DetectionEvent>(message);

                _targetChannel.write(std::move(detection));
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
                std::get<beam::TransmitBeamCommand>(*event);

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
                std::get<std::vector<target::TrackEvent>>(*event);

            _controller.sendTrackEvents(tracks);
        });
    }
}
} // namespace app