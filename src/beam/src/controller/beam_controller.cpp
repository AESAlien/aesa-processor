#include <beam/controller/beam_controller.hpp>
#include <beam/dto/log/beam_event_logger.hpp>

#include "attitude/ant_enu_transform.hpp"
#include "attitude/radar_attitude.hpp"
#include "scheduler/attitude_frame.hpp"
#include "request/factory/beam_request_factory.hpp"
#include "scheduler/beam_scheduler.hpp"

#include <utility>

namespace beam
{

BeamController::BeamController(
    const BeamConfig& config,
    std::shared_ptr<BeamEventLogger> eventLogger
) : _config(config),
    _scheduler(std::make_unique<BeamScheduler>()),
    _requestFactory(std::make_unique<BeamRequestFactory>(config.requestBeam)),
    _eventLogger(std::move(eventLogger))
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

void BeamController::setAttitude(SetAttitudeCommand command)
{
    // 잘못된 자세정보이면 RadarAttitude 생성자(roll/yaw 범위)나 AntEnuTransform 생성자(짐벌락)가 std::invalid_argument를 던진다.
    // 검증과 AttitudeFrame(탐색 격자·요청 빔 영역 판정) 생성은 운용 상태와 관계없이 하고, 받아들일지는 BeamScheduler가 정한다.
    // 예외가 나면 BeamScheduler는 그대로다.
    // 위도·경도·고도는 빔 모듈에서 쓰지 않으므로 넘기지 않는다.
    const AntEnuTransform transform(RadarAttitude(command.roll, command.pitch, command.yaw), _config.attitude);
    const SetAttitudeStatus status =
        _scheduler->setAttitudeFrame(AttitudeFrame(transform, _config.searchGrid, _config.requestCoverage));

    if (!_eventLogger)
    {
        return;
    }

    switch (status)
    {
    case SetAttitudeStatus::APPLIED:
        _eventLogger->logAttitudeApplied(command);
        break;

    case SetAttitudeStatus::IGNORED_NOT_ON:
        _eventLogger->logAttitudeIgnoredWhileOff(command);
        break;

    case SetAttitudeStatus::IGNORED_ALREADY_SET:
        _eventLogger->logAttitudeIgnoredAlreadySet(command);
        break;
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
