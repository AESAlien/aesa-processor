#include "scheduler/scheduler.hpp"

#include "attitude/ant_enu_transform.hpp"

#include <stdexcept>

namespace beam
{
namespace
{
void validate(const BeamRequest& request)
{
    if (request.beamType != BeamRequest::BeamType::CONFIRMATION &&
        request.beamType != BeamRequest::BeamType::TRACKING)
    {
        throw std::invalid_argument("Unknown beam request type");
    }
}

BeamCommand::BeamType toCommandBeamType(BeamRequest::BeamType beamType)
{
    switch (beamType)
    {
    case BeamRequest::BeamType::CONFIRMATION:
        return BeamCommand::BeamType::CONFIRMATION;

    case BeamRequest::BeamType::TRACKING:
        return BeamCommand::BeamType::TRACKING;
    }

    throw std::invalid_argument("Unknown beam request type");
}

math::Angle requestBeamWidth(const RequestBeamConfig& config, BeamRequest::BeamType beamType)
{
    switch (beamType)
    {
    case BeamRequest::BeamType::CONFIRMATION:
        return config.confirmationBeamWidth();
    case BeamRequest::BeamType::TRACKING:
        return config.trackingBeamWidth();
    }

    throw std::invalid_argument("Unkown beam request type");
}

BeamCommand makeSearchCommand(
    const SearchBeam& searchBeam,
    std::chrono::milliseconds currentTime,
    std::uint32_t commandCount
) {
    return BeamCommand::Builder()
        .beamType(BeamCommand::BeamType::SEARCH)
        .transmitTime(currentTime)
        .beamId(searchBeam.beamId)
        .commandCount(commandCount)
        .azimuth_ant(searchBeam.direction_ant.azimuth())
        .elevation_ant(searchBeam.direction_ant.elevation())
        .azimuthBeamWidth(searchBeam.azimuthBeamWidth)
        .elevationBeamWidth(searchBeam.elevationBeamWidth)
        .build();
}

BeamCommand makeRequestCommand(
    const BeamRequest& request,
    const RequestBeamConfig& config,
    std::chrono::milliseconds currentTime,
    std::uint32_t commandCount
) {
    math::Angle width = requestBeamWidth(config, request.beamType);
    return BeamCommand::Builder()
        .beamType(toCommandBeamType(request.beamType))
        .transmitTime(currentTime)
        .beamId(request.requestId)
        .commandCount(commandCount)
        .azimuth_ant(request.azimuth_ant)
        .elevation_ant(request.elevation_ant)
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
    if (_operationState != OperationState::ON || _searchBeamTable.has_value())
    {
        return;
    }

    // 생성 중 예외가 발생하면 _searchBeamTable은 비어 있는 채로 유지된다.
    const AntEnuTransform transform(attitude, _config.attitude());
    _searchBeamTable.emplace(transform, _config.searchGrid());
}

void Scheduler::addRequest(const BeamRequest& request)
{
    if (!isReady())
    {
        return;
    }

    validate(request);
    _requestQueue.add(request);
}

std::optional<BeamCommand> Scheduler::next(std::chrono::milliseconds currentTime)
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
