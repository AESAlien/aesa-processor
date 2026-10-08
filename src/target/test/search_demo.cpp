#include <target/controller/target_controller.hpp>

#include "repository/track_repository.hpp"

#include <iostream>
#include <memory>

int main()
{
    // Synthetic report supplied by the tracking side, not a computed track.
    const target::TrackEvent track{
        7,
        target::TrackEvent::TrackState::TRACKING,
        math::Distance::fromMeters(1000.0),
        math::Distance::fromMeters(900.0),
        math::Angle::fromDegrees(30.0),
        math::Angle::fromDegrees(5.0),
        10.0F,
        math::Angle::fromDegrees(0.0),
        math::Angle::fromDegrees(0.0),
        math::Distance::fromMeters(0.0),
        0.0F,
        math::Angle::fromDegrees(0.0),
        math::Angle::fromDegrees(0.0),
        std::chrono::milliseconds{1234}};
    auto repository = std::make_shared<target::TrackRepository>();
    repository->upsert(track);
    target::TargetController controller(repository);

    target::DetectionEvent detection{
        beam::BeamCommand::BeamType::SEARCH,
        math::Distance::fromMeters(1000.0),
        math::Angle::fromDegrees(30.0),
        math::Angle::fromDegrees(5.0),
        10.0F,
        20.0F};

    const auto existingRequest = controller.handleDetection(detection);
    if (existingRequest)
    {
        return 1;
    }
    std::cout << "Existing SEARCH: no confirmation request\n";

    detection.slantRange = math::Distance::fromMeters(1500.0);
    const auto request = controller.handleDetection(detection);
    if (!request)
    {
        return 1;
    }
    std::cout << "CONFIRMATION request: az=" << request->azimuth_ant.deg()
              << " deg, el=" << request->elevation_ant.deg()
              << " deg, timestamp=" << request->timestamp.count() << " ms\n";

    const auto reports = controller.nextTrackEvents();
    if (reports.size() != 1 || reports.front().id != track.id)
    {
        return 1;
    }
    std::cout << "Console tracks: " << reports.size()
              << ", id=" << reports.front().id
              << ", range=" << reports.front().slantRange.m() << " m\n";
    return 0;
}
