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

            // 마지막 표적이 삭제된 경우도 빈 목록으로 콘솔에 전달한다.
            _commChannel.write(std::move(tracks));
        });
    }
}
} // namespace app
