#include "scheduler/scheduler.hpp"

#include "attitude/ant_enu_transform.hpp"
#include "attitude/attitude_config.hpp"

#include <stdexcept>

namespace beam
{
namespace
{
using namespace math::literals;

// 확인/추적 빔의 beamId. 탐색 그리드 번호가 1부터 시작하므로 0은 그리드 빔이 아님을 뜻한다.
constexpr std::uint32_t NON_GRID_BEAM_ID = 0;

const math::Angle REQUEST_BEAM_WIDTH = 3_deg;

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

BeamCommand makeSearchCommand(const BeamInfo& beamInfo, std::chrono::milliseconds currentTime, std::uint32_t commandCount)
{
    BeamCommand command;
    command.beamType = BeamCommand::BeamType::SEARCH;
    command.timestamp = currentTime;
    command.beamId = beamInfo.beamId;
    command.commandCount = commandCount;
    command.azimuth_ant = beamInfo.azimuth_ant;
    command.elevation_ant = beamInfo.elevation_ant;
    command.azimuthBeamWidth = beamInfo.azimuthBeamWidth;
    command.elevationBeamWidth = beamInfo.elevationBeamWidth;
    return command;
}

BeamCommand makeRequestCommand(const BeamRequest& request, std::chrono::milliseconds currentTime, std::uint32_t commandCount)
{
    BeamCommand command;
    command.beamType = toCommandBeamType(request.beamType);
    command.timestamp = currentTime;
    command.beamId = NON_GRID_BEAM_ID;
    command.commandCount = commandCount;
    command.azimuth_ant = request.azimuth_ant;
    command.elevation_ant = request.elevation_ant;
    command.azimuthBeamWidth = REQUEST_BEAM_WIDTH;
    command.elevationBeamWidth = REQUEST_BEAM_WIDTH;
    return command;
}
} // namespace

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
    const AntEnuTransform transform(attitude, AttitudeConfig{});
    _beamTable.emplace(transform);
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
        return makeRequestCommand(*request, currentTime, _commandCount);
    }

    const BeamInfo& beamInfo = _beamTable->get(_searchCursor);
    _searchCursor = (_searchCursor + 1) % BeamTable::size();
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
