#include <app/worker/target_worker.hpp>
#include <app/worker/worker_error.hpp>
#include <utility>
#include <variant>

namespace app
{
TargetWorker::TargetWorker(
    TargetChannel& targetChannel,
    BeamChannel& beamChannel,
    CommChannel& commChannel,
    target::TargetController& controller)
    : _targetChannel(targetChannel),
      _beamChannel(beamChannel),
      _commChannel(commChannel),
      _controller(controller)
{
}

void TargetWorker::run()
{
    while (auto event = _targetChannel.read())
    {
        tryWorkerAction("TargetWorker", [&]
        {
            const auto& detection =
                std::get<target::DetectionEvent>(event->message);

            auto request = _controller.handleDetection(detection);

            if (request && !_beamChannel.write(std::move(*request)))
            {
                reportWorkerError(
                    "TargetWorker",
                    "BeamChannel closed; request dropped");
            }

            auto tracks = _controller.nextTrackEvents();

            if (!tracks.empty() &&
                !_commChannel.write(std::move(tracks)))
            {
                reportWorkerError(
                    "TargetWorker",
                    "CommChannel closed; tracks dropped");
            }
        });
    }
}
} // namespace app