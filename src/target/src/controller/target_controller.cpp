#include <target/controller/target_controller.hpp>

#include "candidate/candidate_selector.hpp"
#include "repository/track_repository.hpp"

#include <cmath>
#include <stdexcept>
#include <utility>

namespace target
{
namespace
{

void validateSearchDetection(const DetectionEvent& detection)
{
    const double range = detection.slantRange.m();
    const double azimuth = detection.azimuth.deg();
    const double elevation = detection.elevation.deg();
    if (!std::isfinite(range) || range <= 0.0 || !std::isfinite(azimuth)
        || !std::isfinite(elevation) || std::abs(elevation) > 90.0)
    {
        throw std::invalid_argument("Invalid search detection position");
    }
}

} // namespace

TargetController::TargetController()
    : TargetController(std::make_shared<TrackRepository>())
{
}

TargetController::TargetController(std::shared_ptr<TrackRepository> trackRepository)
    : _trackRepository(std::move(trackRepository))
{
    if (!_trackRepository)
    {
        throw std::invalid_argument("Track repository must not be null");
    }
}

std::optional<beam::BeamRequest> TargetController::handleDetection(DetectionEvent detection)
{
    if (detection.beamType != beam::BeamCommand::BeamType::SEARCH)
    {
        return std::nullopt;
    }

    validateSearchDetection(detection);

    const CandidateSelector selector;
    if (!selector.isNewCandidate(detection, *_trackRepository))
    {
        return std::nullopt;
    }

    beam::BeamRequest request;
    request.beamType = beam::BeamRequest::BeamType::CONFIRMATION;
    request.azimuth_ant = detection.azimuth;
    request.elevation_ant = detection.elevation;
    // Preserve the existing timestamp default; DetectionEvent has no time field.
    return request;
}

std::vector<TrackEvent> TargetController::nextTrackEvents() const
{
    // Return a full copy; reading reports does not consume the stored tracks.
    return _trackRepository->tracks();
}

} // namespace target
