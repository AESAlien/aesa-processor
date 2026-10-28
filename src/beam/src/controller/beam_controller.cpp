#include <beam/controller/beam_controller.hpp>

#include "attitude/radar_attitude.hpp"
#include "scheduler/beam_request_factory.hpp"
#include "scheduler/scheduler.hpp"

namespace beam
{

BeamController::BeamController(const BeamConfig& config)
    : _scheduler(std::make_unique<Scheduler>(config)),
      _requestFactory(std::make_unique<BeamRequestFactory>(config.requestBeam))
{
}

BeamController::~BeamController() = default;

BeamController::BeamController(BeamController&&) noexcept = default;

BeamController& BeamController::operator=(BeamController&&) noexcept = default;

void BeamController::setOperationState(SetOperationStateCommand command)
{
    _scheduler->setOperationState(command.powerState);
}

void BeamController::setAttitude(SetAttitudeCommand command)
{
    // 잘못된 자세정보이면 RadarAttitude 생성자가 std::invalid_argument를 던진다.
    _scheduler->setAttitude(RadarAttitude(
        command.latitude,
        command.longitude,
        command.altitude,
        command.roll,
        command.pitch,
        command.yaw));
}

void BeamController::requestBeam(RequestBeamCommand command)
{
    // 잘못된 요청이면 팩토리가 std::invalid_argument를 던진다.
    _scheduler->addRequest(_requestFactory->create(command));
}

std::optional<TransmitBeamCommand> BeamController::nextBeamCommand(std::chrono::milliseconds currentTime)
{
    return _scheduler->next(currentTime);
}

} // namespace beam
