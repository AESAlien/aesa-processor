#include "scheduler/scheduler.hpp"

#include "attitude/ant_enu_transform.hpp"

#include <stdexcept>

namespace beam
{
namespace
{
BeamType toBeamType(RequestBeamType beamType)
{
    switch (beamType)
    {
    case RequestBeamType::CONFIRMATION:
        return BeamType::CONFIRMATION;

    case RequestBeamType::TRACKING:
        return BeamType::TRACKING;
    }

    throw std::invalid_argument("Unknown beam request type");
}

math::Angle requestBeamWidth(const RequestBeamConfig& config, RequestBeamType beamType)
{
    switch (beamType)
    {
    case RequestBeamType::CONFIRMATION:
        return config.confirmationBeamWidth();
    case RequestBeamType::TRACKING:
        return config.trackingBeamWidth();
    }

    throw std::invalid_argument("Unkown beam request type");
}

TransmitBeamCommand makeSearchCommand(
    const SearchBeam& searchBeam,
    std::chrono::milliseconds currentTime,
    std::uint32_t commandCount
) {
    return TransmitBeamCommand::Builder()
        .beamType(BeamType::SEARCH)
        .transmitTime(currentTime)
        .beamId(searchBeam.beamId)
        .commandCount(commandCount)
        .direction_ant(searchBeam.direction_ant)
        .azimuthBeamWidth(searchBeam.azimuthBeamWidth)
        .elevationBeamWidth(searchBeam.elevationBeamWidth)
        .build();
}

TransmitBeamCommand makeRequestCommand(
    const BeamRequest& request,
    const RequestBeamConfig& config,
    std::chrono::milliseconds currentTime,
    std::uint32_t commandCount
) {
    math::Angle width = requestBeamWidth(config, request.beamType());
    return TransmitBeamCommand::Builder()
        .beamType(toBeamType(request.beamType()))
        .transmitTime(currentTime)
        .beamId(request.requestId())
        .commandCount(commandCount)
        .direction_ant(request.direction_ant())
        .azimuthBeamWidth(width)
        .elevationBeamWidth(width)
        .build();
}
} // namespace

Scheduler::Scheduler(const BeamConfig& config) : _config(config)
{
}

void Scheduler::setOperationState(OperationState state)
{
    if (state == _operationState)
    {
        return;
    }

    if (state == OperationState::ON)
    {
        reset();
    }
    _operationState = state;
}

void Scheduler::setAttitude(const RadarAttitude& attitude)
{
    // 설정에 의존하는 검증(짐벌락)도 운용 상태와 관계없이 수행해야 하므로 상태 확인보다 먼저 변환을 만든다.
    // 예외가 발생해도 상태는 변하지 않는다.
    const AntEnuTransform transform(attitude, _config.attitude());

    if (_operationState != OperationState::ON || _searchBeamTable.has_value())
    {
        return;
    }

    _searchBeamTable.emplace(transform, _config.searchGrid());
}

void Scheduler::addRequest(const BeamRequest& request)
{
    if (!isReady())
    {
        return;
    }

    _requestQueue.add(request);
}

std::optional<TransmitBeamCommand> Scheduler::next(std::chrono::milliseconds currentTime)
{
    if (!isReady())
    {
        return std::nullopt;
    }

    ++_commandCount;

    if (const std::optional<BeamRequest> request = _requestQueue.popNextDue(currentTime))
    {
        return makeRequestCommand(*request, _config.requestBeam(), currentTime, _commandCount);
    }

    return makeSearchCommand(_searchBeamTable->nextBeam(), currentTime, _commandCount);
}

bool Scheduler::isReady() const
{
    return _operationState == OperationState::ON && _searchBeamTable.has_value();
}

void Scheduler::reset()
{
    _searchBeamTable.reset();
    _requestQueue.clear();
    _commandCount = 0;
}

} // namespace beam
