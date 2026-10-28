#include <beam/controller/beam_controller.hpp>

#include "attitude/ant_enu_transform.hpp"
#include "attitude/radar_attitude.hpp"
#include "scheduler/beam_request_factory.hpp"
#include "scheduler/scheduler.hpp"
#include "scheduler/search_grid.hpp"

namespace beam
{

BeamController::BeamController(const BeamConfig& config)
    : _config(config),
      _scheduler(std::make_unique<Scheduler>()),
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
    // 잘못된 자세정보이면 RadarAttitude 생성자(roll/yaw 범위)나 AntEnuTransform 생성자(짐벌락)가 std::invalid_argument를 던진다.
    // 검증과 탐색 격자 생성은 운용 상태와 관계없이 하고, 격자를 받아들일지는 Scheduler가 정한다. 예외가 나면 Scheduler는 그대로다.
    // 위도·경도·고도는 빔 모듈에서 쓰지 않으므로 넘기지 않는다.
    const AntEnuTransform transform(RadarAttitude(command.roll, command.pitch, command.yaw), _config.attitude);
    _scheduler->setSearchGrid(SearchGrid(transform, _config.searchGrid));
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
