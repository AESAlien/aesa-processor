#include <app/worker/beam_worker.hpp>
#include <app/worker/worker_error.hpp>

#include <optional>
#include <utility>
#include <variant>

namespace app
{

BeamWorker::BeamWorker(
    BeamChannel& beamChannel,
    CommChannel& commChannel,
    beam::BeamController& controller)
    : _beamChannel(beamChannel),
      _commChannel(commChannel),
      _controller(controller)
{
}

void BeamWorker::run()
{
    // OFF 중 수신한 자세를 다음 ON에서 적용하기 위해 보관한다.
    std::optional<beam::SetAttitudeCommand> latestAttitude;

    while (auto event = _beamChannel.read())
    {
        tryWorkerAction("BeamWorker", [&]
        {
            auto& message = *event;

            if (auto* request =
                    std::get_if<beam::RequestBeamCommand>(&message))
            {
                _controller.requestBeam(*request);
            }
            else if (auto* state =
                         std::get_if<beam::SetOperationStateCommand>(&message))
            {
                _controller.setOperationState(*state);

                if (state->powerState == beam::OperationState::OFF)
                {
                    // 이전 운용에서 사용한 자세를 폐기한다.
                    latestAttitude.reset();
                }
                else if (latestAttitude)
                {
                    // OFF 이후 새로 받은 자세만 다음 운용에 적용한다.
                    _controller.setAttitude(*latestAttitude);

                    latestAttitude.reset();
                }
            }
            else if (auto* attitude =
                         std::get_if<beam::SetAttitudeCommand>(&message))
            {
                // DTO의 멤버가 const이므로 emplace로 교체한다.
                latestAttitude.emplace(*attitude);

                _controller.setAttitude(*attitude);
            }
            else if (auto* tick =
                         std::get_if<NextBeamCommandEvent>(&message))
            {
                auto command =
                    _controller.nextBeamCommand(tick->currentTime);

                if (command)
                {
                    _commChannel.write(std::move(*command));
                }
            }
        });
    }
}

} // namespace app