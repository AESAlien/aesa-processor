#include <beam/controller/beam_controller.hpp>
#include <beam/dto/log/beam_event_logger.hpp>

#include "scheduler/attitude_frame.hpp"
#include "request/factory/beam_request_factory.hpp"
#include "scheduler/beam_scheduler.hpp"

#include <utility>

namespace beam
{

BeamController::BeamController(
    const BeamConfig& config,
    std::shared_ptr<BeamEventLogger> eventLogger,
    std::shared_ptr<spdlog::logger> messageLogger
) : _config(config),
    _scheduler(std::make_unique<BeamScheduler>()),
    _requestFactory(std::make_unique<BeamRequestFactory>(config.requestBeam)),
    _eventLogger(std::move(eventLogger)),
    _messageLogger(std::move(messageLogger))
{
}

BeamController::~BeamController() = default;

BeamController::BeamController(BeamController&&) noexcept = default;

BeamController& BeamController::operator=(BeamController&&) noexcept = default;

void BeamController::setOperationState(SetOperationStateCommand command)
{
    const SetOperationStateStatus status = _scheduler->setOperationState(command.powerState);

    // 같은 상태의 중복 수신(UNCHANGED)은 기록하지 않는다.
    if (_eventLogger && status == SetOperationStateStatus::CHANGED)
    {
        _eventLogger->logOperationStateChanged(command.powerState);
    }
}

void BeamController::setAntEnuTransform(const pose::SetAntEnuTransformCommand& command)
{
    const AttitudeFrame frame(command.transform, _config.searchGrid, _config.requestCoverage);
    _scheduler->setAttitudeFrame(frame);
    if (_eventLogger)
    {
        _eventLogger->logAttitudeApplied();
    }
}

RequestBeamResult BeamController::requestBeam(RequestBeamCommand command)
{
    // 잘못된 요청이면 팩토리가 std::invalid_argument를 던지며, 이때는 결과를 만들지 않는다.
    const RequestBeamStatus status = _scheduler->addRequest(_requestFactory->from(command));
    const RequestBeamResult result(status, command.requestId);

    // 수락은 표적 수에 비례해 반복되므로 기록하지 않고, 버려진 요청만 기록한다.
    if (_eventLogger && result.status != RequestBeamStatus::ACCEPTED)
    {
        _eventLogger->logRequestBeamRejected(result);
    }

    return result;
}

std::optional<TransmitBeamCommand> BeamController::nextBeamCommand(std::chrono::milliseconds currentTime)
{
    return _scheduler->next(currentTime);
}

} // namespace beam
