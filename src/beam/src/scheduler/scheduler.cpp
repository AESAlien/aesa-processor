#include "scheduler/scheduler.hpp"

#include <utility>

namespace beam
{
namespace
{
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
        .direction_ant(searchBeam.direction_enu)
        .beamWidth(searchBeam.beamWidth)
        .build();
}

TransmitBeamCommand makeRequestCommand(
    const BeamRequest& request,
    std::chrono::milliseconds currentTime,
    std::uint32_t commandCount
) {
    return TransmitBeamCommand::Builder()
        .beamType(request.transmitBeamType())
        .transmitTime(currentTime)
        .beamId(request.requestId())
        .commandCount(commandCount)
        .direction_ant(request.direction_enu())
        .beamWidth(request.beamWidth())
        .build();
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

void Scheduler::setSearchGrid(SearchGrid searchGrid)
{
    if (_operationState != OperationState::ON || _searchGrid.has_value())
    {
        return;
    }

    _searchGrid.emplace(std::move(searchGrid));
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
        return makeRequestCommand(*request, currentTime, _commandCount);
    }

    return makeSearchCommand(_searchGrid->nextBeam(), currentTime, _commandCount);
}

bool Scheduler::isReady() const
{
    return _operationState == OperationState::ON && _searchGrid.has_value();
}

void Scheduler::reset()
{
    _searchGrid.reset();
    _requestQueue.clear();
    _commandCount = 0;
}

} // namespace beam
