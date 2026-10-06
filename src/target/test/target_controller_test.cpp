#include <target/controller/target_controller.hpp>

#include "repository/track_repository.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <limits>
#include <memory>
#include <stdexcept>
#include <vector>

namespace target::test
{
namespace
{

DetectionEvent makeSearchDetection()
{
    return DetectionEvent{
        beam::BeamCommand::BeamType::SEARCH,
        math::Distance::fromMeters(1000.0),
        math::Angle::fromDegrees(30.0),
        math::Angle::fromDegrees(5.0),
        10.0F,
        20.0F};
}

TrackEvent makeTrackEvent(std::uint16_t id)
{
    // Synthetic report values; no tracker or coordinate calculation is implied.
    return TrackEvent{
        id,
        TrackEvent::TrackState::TRACKING,
        math::Distance::fromMeters(1000.0),
        math::Distance::fromMeters(800.0),
        math::Angle::fromDegrees(30.0),
        math::Angle::fromDegrees(5.0),
        10.0F,
        math::Angle::fromDegrees(35.0),
        math::Angle::fromDegrees(127.0),
        math::Distance::fromMeters(100.0),
        120.0F,
        math::Angle::fromDegrees(45.0),
        math::Angle::fromDegrees(2.0),
        std::chrono::milliseconds{2500}};
}

const TrackEvent* findTrack(const std::vector<TrackEvent>& tracks, std::uint16_t id)
{
    const auto entry = std::find_if(tracks.begin(), tracks.end(), [id](const TrackEvent& track)
    {
        return track.id == id;
    });
    return entry == tracks.end() ? nullptr : &*entry;
}

void expectSameReport(const TrackEvent& actual, const TrackEvent& expected)
{
    EXPECT_EQ(actual.id, expected.id);
    EXPECT_EQ(actual.state, expected.state);
    EXPECT_DOUBLE_EQ(actual.slantRange.m(), expected.slantRange.m());
    EXPECT_DOUBLE_EQ(actual.groundRange.m(), expected.groundRange.m());
    EXPECT_DOUBLE_EQ(actual.azimuth.deg(), expected.azimuth.deg());
    EXPECT_DOUBLE_EQ(actual.elevation.deg(), expected.elevation.deg());
    EXPECT_FLOAT_EQ(actual.dopplerVelocity_mps, expected.dopplerVelocity_mps);
    EXPECT_DOUBLE_EQ(actual.latitude.deg(), expected.latitude.deg());
    EXPECT_DOUBLE_EQ(actual.longitude.deg(), expected.longitude.deg());
    EXPECT_DOUBLE_EQ(actual.altitude.m(), expected.altitude.m());
    EXPECT_FLOAT_EQ(actual.velocity_mps, expected.velocity_mps);
    EXPECT_DOUBLE_EQ(actual.heading.deg(), expected.heading.deg());
    EXPECT_DOUBLE_EQ(actual.flightPathAngle.deg(), expected.flightPathAngle.deg());
    EXPECT_EQ(actual.timestamp, expected.timestamp);
}

} // namespace

TEST(TargetController, EmptyRepositoryCreatesConfirmationRequest)
{
    TargetController controller;
    const auto detection = makeSearchDetection();
    const auto request = controller.handleDetection(detection);

    ASSERT_TRUE(request);
    EXPECT_EQ(request->beamType, beam::BeamRequest::BeamType::CONFIRMATION);
    EXPECT_DOUBLE_EQ(request->azimuth_ant.deg(), detection.azimuth.deg());
    EXPECT_DOUBLE_EQ(request->elevation_ant.deg(), detection.elevation.deg());
    EXPECT_EQ(request->timestamp, std::chrono::milliseconds{0});
    EXPECT_TRUE(controller.nextTrackEvents().empty());
}

TEST(TargetController, SamePositionDoesNotCreateRequestRegardlessOfDopplerAndPower)
{
    auto repository = std::make_shared<TrackRepository>();
    repository->upsert(makeTrackEvent(7));
    TargetController controller(repository);
    auto detection = makeSearchDetection();
    detection.dopplerVelocity_mps = -50.0F;
    detection.power_db = 40.0F;

    EXPECT_FALSE(controller.handleDetection(detection));
}

TEST(TargetController, DifferentRangeAzimuthOrElevationCreatesRequest)
{
    auto repository = std::make_shared<TrackRepository>();
    repository->upsert(makeTrackEvent(7));
    TargetController controller(repository);

    auto detection = makeSearchDetection();
    detection.slantRange = math::Distance::fromMeters(1001.0);
    EXPECT_TRUE(controller.handleDetection(detection));

    detection = makeSearchDetection();
    detection.azimuth = math::Angle::fromDegrees(31.0);
    EXPECT_TRUE(controller.handleDetection(detection));

    detection = makeSearchDetection();
    detection.elevation = math::Angle::fromDegrees(6.0);
    EXPECT_TRUE(controller.handleDetection(detection));
}

TEST(TargetController, ChecksLaterTracksForMatchingPosition)
{
    auto repository = std::make_shared<TrackRepository>();
    auto different = makeTrackEvent(1);
    different.slantRange = math::Distance::fromMeters(2000.0);
    repository->upsert(different);
    repository->upsert(makeTrackEvent(2));
    TargetController controller(repository);

    EXPECT_FALSE(controller.handleDetection(makeSearchDetection()));
}

TEST(TargetController, TrackEventsReturnsWholeIndependentSnapshotOnEveryCall)
{
    auto repository = std::make_shared<TrackRepository>();
    auto first = makeTrackEvent(40);
    first.state = TrackEvent::TrackState::INIT;
    const auto second = makeTrackEvent(5);
    repository->upsert(first);
    repository->upsert(second);
    TargetController controller(repository);

    auto snapshot = controller.nextTrackEvents();
    ASSERT_EQ(snapshot.size(), 2U);
    const auto* firstReport = findTrack(snapshot, first.id);
    const auto* secondReport = findTrack(snapshot, second.id);
    ASSERT_NE(firstReport, nullptr);
    ASSERT_NE(secondReport, nullptr);
    expectSameReport(*firstReport, first);
    expectSameReport(*secondReport, second);

    // Returned reports are copies; querying does not consume stored reports.
    snapshot.front().slantRange = math::Distance::fromMeters(9999.0);
    snapshot.clear();
    const auto repeated = controller.nextTrackEvents();
    ASSERT_EQ(repeated.size(), 2U);
    firstReport = findTrack(repeated, first.id);
    secondReport = findTrack(repeated, second.id);
    ASSERT_NE(firstReport, nullptr);
    ASSERT_NE(secondReport, nullptr);
    expectSameReport(*firstReport, first);
    expectSameReport(*secondReport, second);
}

TEST(TargetController, UpdatedTrackAppearsOnceWithLatestReport)
{
    auto repository = std::make_shared<TrackRepository>();
    auto report = makeTrackEvent(7);
    repository->upsert(report);
    TargetController controller(repository);
    const auto previous = controller.nextTrackEvents();

    report.state = TrackEvent::TrackState::INIT;
    report.slantRange = math::Distance::fromMeters(1500.0);
    report.azimuth = math::Angle::fromDegrees(45.0);
    report.elevation = math::Angle::fromDegrees(10.0);
    report.velocity_mps = 160.0F;
    report.timestamp = std::chrono::milliseconds{3000};
    repository->upsert(report);

    const auto latest = controller.nextTrackEvents();
    ASSERT_EQ(latest.size(), 1U);
    expectSameReport(latest.front(), report);
    ASSERT_EQ(previous.size(), 1U);
    expectSameReport(previous.front(), makeTrackEvent(7));

    // Comparison must use the updated track, rather than a stale copy.
    EXPECT_TRUE(controller.handleDetection(makeSearchDetection()));
    auto detection = makeSearchDetection();
    detection.slantRange = report.slantRange;
    detection.azimuth = report.azimuth;
    detection.elevation = report.elevation;
    EXPECT_FALSE(controller.handleDetection(detection));
}

TEST(TargetController, RemovedTrackDisappearsFromSnapshotAndComparison)
{
    auto repository = std::make_shared<TrackRepository>();
    repository->upsert(makeTrackEvent(7));
    TargetController controller(repository);
    EXPECT_FALSE(controller.handleDetection(makeSearchDetection()));

    EXPECT_TRUE(repository->remove(7));
    EXPECT_FALSE(repository->remove(7));
    EXPECT_TRUE(controller.nextTrackEvents().empty());
    EXPECT_TRUE(controller.handleDetection(makeSearchDetection()));
}

TEST(TargetController, DetectionHandlingDoesNotChangeStoredTrackReports)
{
    auto repository = std::make_shared<TrackRepository>();
    const auto report = makeTrackEvent(7);
    repository->upsert(report);
    TargetController controller(repository);

    EXPECT_FALSE(controller.handleDetection(makeSearchDetection()));
    auto detection = makeSearchDetection();
    detection.slantRange = math::Distance::fromMeters(2000.0);
    EXPECT_TRUE(controller.handleDetection(detection));

    detection.beamType = beam::BeamCommand::BeamType::CONFIRMATION;
    EXPECT_FALSE(controller.handleDetection(detection));
    detection.beamType = beam::BeamCommand::BeamType::TRACKING;
    EXPECT_FALSE(controller.handleDetection(detection));

    const auto snapshot = controller.nextTrackEvents();
    ASSERT_EQ(snapshot.size(), 1U);
    expectSameReport(snapshot.front(), report);
}

TEST(TargetController, InvalidSearchRangeIsRejected)
{
    TargetController controller;
    auto detection = makeSearchDetection();
    detection.slantRange = math::Distance::fromMeters(0.0);
    EXPECT_THROW(controller.handleDetection(detection), std::invalid_argument);
    detection.slantRange = math::Distance::fromMeters(-1.0);
    EXPECT_THROW(controller.handleDetection(detection), std::invalid_argument);
    detection.slantRange = math::Distance::fromMeters(std::numeric_limits<double>::infinity());
    EXPECT_THROW(controller.handleDetection(detection), std::invalid_argument);
    detection.slantRange = math::Distance::fromMeters(std::numeric_limits<double>::quiet_NaN());
    EXPECT_THROW(controller.handleDetection(detection), std::invalid_argument);
    EXPECT_TRUE(controller.nextTrackEvents().empty());
}

TEST(TargetController, InvalidSearchDirectionIsRejectedAndValidElevationBoundaryIsAccepted)
{
    auto repository = std::make_shared<TrackRepository>();
    const auto report = makeTrackEvent(7);
    repository->upsert(report);
    TargetController controller(repository);
    auto detection = makeSearchDetection();
    detection.azimuth = math::Angle::fromDegrees(std::numeric_limits<double>::quiet_NaN());
    EXPECT_THROW(controller.handleDetection(detection), std::invalid_argument);
    detection = makeSearchDetection();
    detection.elevation = math::Angle::fromDegrees(std::numeric_limits<double>::quiet_NaN());
    EXPECT_THROW(controller.handleDetection(detection), std::invalid_argument);
    detection.elevation = math::Angle::fromDegrees(91.0);
    EXPECT_THROW(controller.handleDetection(detection), std::invalid_argument);
    detection.elevation = math::Angle::fromDegrees(-91.0);
    EXPECT_THROW(controller.handleDetection(detection), std::invalid_argument);

    detection.elevation = math::Angle::fromDegrees(90.0);
    EXPECT_TRUE(controller.handleDetection(detection));
    detection.elevation = math::Angle::fromDegrees(-90.0);
    EXPECT_TRUE(controller.handleDetection(detection));

    const auto snapshot = controller.nextTrackEvents();
    ASSERT_EQ(snapshot.size(), 1U);
    expectSameReport(snapshot.front(), report);
}

TEST(TargetController, NullRepositoryIsRejected)
{
    EXPECT_THROW((TargetController{std::shared_ptr<TrackRepository>{}}), std::invalid_argument);
}

} // namespace target::test
