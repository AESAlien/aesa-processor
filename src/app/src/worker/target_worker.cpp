#include <app/worker/target_worker.hpp>
#include <app/worker/worker_error.hpp>

#include <utility>

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
    while (auto detection = _targetChannel.read())
    {
        tryWorkerAction("TargetWorker", [&]
        {
            auto request =
                _controller.handleDetection(*detection);

            if (request)
            {
                _beamChannel.write(std::move(*request));
            }

            auto tracks = _controller.nextTrackEvents();

            if (!tracks.empty())
            {
                _commChannel.write(std::move(tracks));
            }
        });
    }
}
} // namespace app