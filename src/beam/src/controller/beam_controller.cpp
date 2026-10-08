#include <beam/controller/beam_controller.hpp>

#include "attitude/radar_attitude.hpp"
#include "scheduler/scheduler.hpp"

namespace beam
{

BeamController::BeamController() : _scheduler(std::make_unique<Scheduler>())
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
    RadarAttitude attitude{};
    attitude.latitude = command.latitude;
    attitude.longitude = command.longitude;
    attitude.altitude = command.altitude;
    attitude.roll = command.roll;
    attitude.pitch = command.pitch;
    attitude.yaw = command.yaw;

    _scheduler->setAttitude(attitude);
}

void BeamController::requestBeam(BeamRequest beamRequest)
{
    _scheduler->addRequest(beamRequest);
}

std::optional<BeamCommand> BeamController::nextBeamCommand(std::chrono::milliseconds currentTime)
{
    return _scheduler->next(currentTime);
}

} // namespace beam
