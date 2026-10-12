#include "scheduler/scheduler.hpp"

#include "attitude/ant_enu_transform.hpp"
#include "beam/config/beam_config.hpp"

#include <cmath>
#include <stdexcept>

namespace beam
{
namespace
{
using namespace math::literals;

bool isFinite(math::Angle angle)
{
    return std::isfinite(angle.deg());
}

void validate(const RequestBeamConfig& config)
{
    // NaN은 크기 비교가 모두 false이므로 유한성을 먼저 확인한다.
    if (!isFinite(config.confirmationBeamWidth) || !isFinite(config.trackingBeamWidth))
    {
        throw std::invalid_argument("RequestBeamConfig: beam width must be finite");
    }

    if (config.confirmationBeamWidth <= 0_deg || config.trackingBeamWidth <= 0_deg)
    {
        throw std::invalid_argument("RequestBeamConfig: beam width must be greater than 0");
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
        return config.confirmationBeamWidth;
    case BeamRequest::BeamType::TRACKING:
        return config .trackingBeamWidth;
    }

    throw std::invalid_argument("Unkown beam request type");
}

BeamCommand makeSearchCommand(
    const BeamInfo& beamInfo,
    std::chrono::milliseconds currentTime,
    std::uint32_t commandCount
) {
    return BeamCommand::Builder()
        .beamType(BeamCommand::BeamType::SEARCH)
        .transmitTime(currentTime)
        .beamId(beamInfo.beamId)
        .commandCount(commandCount)
        .azimuth_ant(beamInfo.azimuth_ant)
        .elevation_ant(beamInfo.elevation_ant)
        .azimuthBeamWidth(beamInfo.azimuthBeamWidth)
        .elevationBeamWidth(beamInfo.elevationBeamWidth)
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

Scheduler::Scheduler(const BeamConfig& config)
    : _config(config), _requestQueue(config.requestQueue)
{
    validate(_config.requestBeam);
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
    if (_operationState != OperationState::ON || _beamTable.has_value())
    {
        return;
    }

    // 생성 중 예외가 발생하면 _beamTable은 비어 있는 채로 유지된다.
    const AntEnuTransform transform(attitude, _config.attitude);
    _beamTable.emplace(transform, _config.searchGrid);
}

void Scheduler::addRequest(const BeamRequest& request)
{
    if (!isReady())
    {
        return;
    }

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
        return makeRequestCommand(*request, _config.requestBeam, currentTime, _commandCount);
    }

    const BeamInfo& beamInfo = _beamTable->get(_searchCursor);
    _searchCursor = (_searchCursor + 1) % _beamTable->size();
    return makeSearchCommand(beamInfo, currentTime, _commandCount);
}

bool Scheduler::isReady() const
{
    return _operationState == OperationState::ON && _beamTable.has_value();
}

void Scheduler::reset()
{
    _beamTable.reset();
    _requestQueue.clear();
    _searchCursor = 0;
    _commandCount = 0;
}

} // namespace beam
