#include "detection_handler.hpp"

#include "repository/track_repository.hpp"
#include "domain/selector/tentative_selector.hpp"
#include "domain/selector/track_selector.hpp"

#include <stdexcept>
#include <utility>

namespace target
{

DetectionHandler::DetectionHandler(
    TrackManagementConfig config,
    std::shared_ptr<TrackRepository> repository,
    TrackFactory trackFactory,
    std::unique_ptr<const TentativeSelector> tentativeSelector,
    std::unique_ptr<const TrackSelector> trackSelector
) : _config(std::move(config)),
    _trackRepository(std::move(repository)),
    _trackFactory(std::move(trackFactory)),
    _tentativeSelector(std::move(tentativeSelector)),
    _trackSelector(std::move(trackSelector))
{
    if (!_trackRepository || !_tentativeSelector || !_trackSelector)
    {
        throw std::invalid_argument("Detection handler requires a repository and both selectors");
    }
}

DetectionHandler::~DetectionHandler() = default;

std::optional<beam::RequestBeamCommand> DetectionHandler::handle(const DetectionResult& result)
{
    return std::visit(
        [this](const auto& value)
        {
            return handle(value);
        },
        result
    );
}

std::optional<beam::RequestBeamCommand> DetectionHandler::handle(
    const SearchDetectionResult& result
) {
    const auto previous = _lastSearchResultTimes.find(result.requestId);
    if (previous != _lastSearchResultTimes.end() && result.transmitTime <= previous->second)
    {
        return std::nullopt;
    }

    const auto candidate = _tentativeSelector->selectNewCandidate(result.observations, _trackRepository->tracks());
    if (!candidate)
    {
        // 후보가 없는 결과도 시각을 기록해 같은 요청에서 중복되거나 오래된 결과의 재처리를 막는다.
        _lastSearchResultTimes.insert_or_assign(result.requestId, result.transmitTime);
        return std::nullopt;
    }

    const auto transmitTime = result.transmitTime + _config.confirmationDelay();
    const auto id = _trackRepository->add(_trackFactory.createTentative(_trackRepository->nextId(), *candidate));
    _lastSearchResultTimes.insert_or_assign(result.requestId, result.transmitTime);

    return beam::RequestBeamCommand(
        beam::RequestBeamType::CONFIRMATION,
        transmitTime,
        id,
        candidate->sphericalPosition().direction
    );
}

std::optional<beam::RequestBeamCommand> DetectionHandler::handle(
    const ConfirmationDetectionResult& result
) {
    const auto* tentativeTrack = _trackRepository->findTentativeTrack(result.requestId);
    if (!tentativeTrack || result.transmitTime <= tentativeTrack->lastResultTime())
    {
        return std::nullopt;
    }

    const auto observation = _trackSelector->selectConfirmationObservation(
        *tentativeTrack,
        result.transmitTime,
        result.observations
    );
    if (!observation)
    {
        _trackRepository->remove(result.requestId);
        return std::nullopt;
    }

    const auto transmitTime = result.transmitTime + _config.trackingInterval();
    auto confirmedTrack = tentativeTrack->confirm(*observation);
    if (!confirmedTrack)
    {
        _trackRepository->remove(result.requestId);
        return std::nullopt;
    }
    // 기존 저장 위치에서 잠정 트랙을 확정 트랙으로 바꿔 ID와 등록 순서를 보존한다.
    if (!_trackRepository->replace(std::move(*confirmedTrack)))
    {
        return std::nullopt;
    }

    const auto* storedTrack = _trackRepository->findConfirmedTrack(result.requestId);
    // 확인 관측으로 초기화한 상태를 다음 추적 빔의 송신 시각까지 예측해 조향 방향을 구한다.
    const auto prediction = storedTrack->predict(transmitTime);
    if (!prediction)
    {
        _trackRepository->remove(result.requestId);
        return std::nullopt;
    }

    return beam::RequestBeamCommand(
        beam::RequestBeamType::TRACKING,
        prediction->timestamp(),
        result.requestId,
        prediction->sphericalPosition().direction
    );
}

std::optional<beam::RequestBeamCommand> DetectionHandler::handle(
    const TrackingDetectionResult& result
) {
    auto* confirmedTrack = _trackRepository->findConfirmedTrack(result.requestId);
    if (!confirmedTrack || result.transmitTime <= confirmedTrack->lastResultTime())
    {
        return std::nullopt;
    }

    const auto observation = _trackSelector->selectTrackingObservation(
        *confirmedTrack,
        result.transmitTime,
        result.observations
    );
    // 연관된 관측이 없고 이번 미탐으로 한도에 도달하면 다음 빔을 계산하기 전에 만료한다.
    if (!observation &&
        confirmedTrack->consecutiveMissCount() >= _config.maxConsecutiveTrackingMisses() - 1)
    {
        _trackRepository->remove(result.requestId);
        return std::nullopt;
    }

    const auto transmitTime = result.transmitTime + _config.trackingInterval();
    const auto updated = observation && confirmedTrack->update(*observation);
    if (!updated)
    {
        // 마지막 유효 상태는 보존하고 결과 시각과 연속 미탐 횟수를 갱신한다.
        confirmedTrack->recordTrackingMiss(result.transmitTime);
        if (confirmedTrack->consecutiveMissCount() >= _config.maxConsecutiveTrackingMisses())
        {
            _trackRepository->remove(result.requestId);
            return std::nullopt;
        }
    }

    // 미탐을 허용하는 동안에도 마지막 유효 상태에서 다음 빔 방향을 예측해 추적을 이어간다.
    const auto prediction = confirmedTrack->predict(transmitTime);
    if (!prediction)
    {
        _trackRepository->remove(result.requestId);
        return std::nullopt;
    }

    return beam::RequestBeamCommand(
        beam::RequestBeamType::TRACKING,
        prediction->timestamp(),
        result.requestId,
        prediction->sphericalPosition().direction
    );
}

} // namespace target
