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
                    latestAttitude.reset();
                }
                else if (latestAttitude)
                {
                    _controller.setAttitude(*latestAttitude);

                    latestAttitude.reset();
                }
            }
            else if (auto* attitude =
                         std::get_if<beam::SetAttitudeCommand>(&message))
            {
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