#include "algorithm/constant_velocity_algorithm.hpp"
#include "domain/factory/detection_result_factory.hpp"
#include "domain/factory/track_factory.hpp"
#include "repository/track_repository.hpp"
#include "service/detection_matcher.hpp"
#include "service/detection_handler.hpp"
#include "service/tentative_selector.hpp"
#include "service/track_selector.hpp"

#include <gtest/gtest.h>

#include <chrono>
#include <cstdint>
#include <limits>
#include <memory>
#include <stdexcept>
#include <utility>
#include <vector>

namespace target
{
namespace
{

using namespace math::literals;
using namespace std::chrono_literals;
using Time = std::chrono::milliseconds;
using BeamType = beam::BeamType;

DetectionResult result(BeamType type, std::uint32_t id, Time time, math::Length range = 1000_m, bool detected = true)
{
    std::vector<DetectionEvent::Plot> plots;
    if (detected)
    {
        plots.emplace_back(
            math::Distance(range),
            math::CircularAngle(0_deg),
            math::TiltAngle(0_deg),
            0_mps,
            -10.0f
        );
    }
    return DetectionResultFactory::from(DetectionEvent(type, id, time, std::move(plots)));
}

class DetectionHandlerTest : public ::testing::Test
{
protected:
    TrackRepository _repository;
    TrackFactory _factory = TrackFactory(std::make_shared<ConstantVelocityAlgorithm>());
    DetectionMatcher _matcher = DetectionMatcher(AssociationConfig(1000_mps, 30_mps, 11.345), MeasurementConfig(7_m));
    TentativeSelector _tentativeSelector = TentativeSelector(_repository, _matcher);
    TrackSelector _trackSelector = TrackSelector(_repository, _matcher);
    DetectionHandler _handler = DetectionHandler(
        TrackManagementConfig(100ms, 1000ms, 3, 100),
        _repository,
        _factory,
        _tentativeSelector,
        _trackSelector
    );
};

TEST_F(DetectionHandlerTest, ProcessesLifecycleAndExpiresOnlyTheRequestedTrack)
{
    const auto first = _handler.handle(result(BeamType::SEARCH, 50, 0ms));
    const auto second = _handler.handle(result(BeamType::SEARCH, 51, 0ms, 2000_m));
    ASSERT_TRUE(first);
    ASSERT_TRUE(second);
    EXPECT_EQ(first->beamType, beam::RequestBeamType::CONFIRMATION);
    EXPECT_EQ(first->transmitTime, 100ms);
    EXPECT_NE(first->requestId, second->requestId);
    EXPECT_NE(_repository.findTentative(first->requestId), nullptr);

    const auto tracking = _handler.handle(result(BeamType::CONFIRMATION, first->requestId, 100ms));
    ASSERT_TRUE(tracking);
    EXPECT_EQ(tracking->beamType, beam::RequestBeamType::TRACKING);
    EXPECT_EQ(tracking->transmitTime, 1100ms);
    ASSERT_TRUE(_handler.handle(result(BeamType::CONFIRMATION, second->requestId, 100ms, 2000_m)));
    ASSERT_TRUE(_handler.handle(result(BeamType::TRACKING, first->requestId, 1100ms, 1010_m)));
    EXPECT_EQ(_repository.findTrack(first->requestId)->lastObservationTime(), 1100ms);
    EXPECT_EQ(_repository.findTrack(second->requestId)->lastObservationTime(), 100ms);

    for (int miss = 0; miss < 3; ++miss)
    {
        const auto retry = _handler.handle(
            result(
                BeamType::TRACKING,
                first->requestId,
                2100ms + Time(1000 * miss),
                1000_m,
                false
            )
        );
        if (miss < 2)
        {
            ASSERT_TRUE(retry);
            EXPECT_EQ(retry->requestId, first->requestId);
            EXPECT_EQ(retry->transmitTime, 3100ms + Time(1000 * miss));
        }
        else
        {
            EXPECT_FALSE(retry);
        }
    }
    EXPECT_EQ(_repository.findTrack(first->requestId), nullptr);
    ASSERT_NE(_repository.findTrack(second->requestId), nullptr);
    EXPECT_EQ(_repository.findTrack(second->requestId)->lastObservationTime(), 100ms);
    const auto next = _handler.handle(result(BeamType::SEARCH, 52, 4200ms, 3000_m));
    ASSERT_TRUE(next);
    EXPECT_GT(next->requestId, second->requestId);
}

TEST_F(DetectionHandlerTest, IgnoresWrongPhaseAndStaleResults)
{
    const auto confirmation = _handler.handle(result(BeamType::SEARCH, 5, 0ms));
    ASSERT_TRUE(confirmation);
    const auto id = confirmation->requestId;
    const auto* tentative = _repository.findTentative(id);
    EXPECT_FALSE(_handler.handle(result(BeamType::TRACKING, id, 100ms)));
    EXPECT_FALSE(_handler.handle(result(BeamType::CONFIRMATION, id, 0ms)));
    EXPECT_FALSE(_handler.handle(result(BeamType::CONFIRMATION, 999, 100ms)));
    EXPECT_FALSE(_handler.handle(result(BeamType::SEARCH, 5, 0ms, 2000_m)));
    EXPECT_EQ(_repository.findTentative(id), tentative);
    EXPECT_EQ(_repository.targets().size(), 1u);

    ASSERT_TRUE(_handler.handle(result(BeamType::CONFIRMATION, id, 100ms)));
    const auto* track = _repository.findTrack(id);
    const auto state = track->state();
    EXPECT_FALSE(_handler.handle(result(BeamType::CONFIRMATION, id, 200ms)));
    EXPECT_FALSE(_handler.handle(result(BeamType::TRACKING, id, 100ms)));
    EXPECT_EQ(_repository.findTrack(id), track);
    EXPECT_EQ(track->state(), state);
    EXPECT_EQ(track->lastResultTime(), 100ms);
    EXPECT_EQ(track->consecutiveMissCount(), 0u);
}

TEST_F(DetectionHandlerTest, TimeOverflowPreservesStateUntilTheTerminalMiss)
{
    const Time maximum(std::numeric_limits<Time::rep>::max());
    EXPECT_THROW(_handler.handle(result(BeamType::SEARCH, 5, maximum)), std::overflow_error);
    EXPECT_TRUE(_repository.targets().empty());

    const auto confirmation = _handler.handle(result(BeamType::SEARCH, 5, 0ms));
    ASSERT_TRUE(confirmation);
    const auto id = confirmation->requestId;
    EXPECT_EQ(id, 1u);
    const auto* tentative = _repository.findTentative(id);
    EXPECT_THROW(_handler.handle(result(BeamType::CONFIRMATION, id, maximum)), std::overflow_error);
    EXPECT_EQ(_repository.findTentative(id), tentative);

    ASSERT_TRUE(_handler.handle(result(BeamType::CONFIRMATION, id, 100ms)));
    auto* track = _repository.findTrack(id);
    const auto state = track->state();
    EXPECT_THROW(
        _handler.handle(result(BeamType::TRACKING, id, maximum, 1000_m, false)),
        std::overflow_error
    );
    EXPECT_EQ(track->state(), state);
    EXPECT_EQ(track->lastResultTime(), 100ms);
    EXPECT_EQ(track->consecutiveMissCount(), 0u);

    ASSERT_TRUE(track->recordTrackingMiss(1100ms));
    ASSERT_TRUE(track->recordTrackingMiss(2100ms));
    EXPECT_FALSE(_handler.handle(result(BeamType::TRACKING, id, maximum, 1000_m, false)));
    EXPECT_EQ(_repository.findTrack(id), nullptr);
}

TEST_F(DetectionHandlerTest, EmptySearchesAdvanceHistoryWithoutConsumingIdentity)
{
    EXPECT_FALSE(_handler.handle(result(BeamType::SEARCH, 5, 200ms, 1000_m, false)));
    EXPECT_FALSE(_handler.handle(result(BeamType::SEARCH, 5, 200ms)));
    EXPECT_FALSE(_handler.handle(result(BeamType::SEARCH, 5, 100ms)));
    EXPECT_TRUE(_repository.targets().empty());

    const auto first = _handler.handle(result(BeamType::SEARCH, 6, 100ms));
    ASSERT_TRUE(first);
    EXPECT_EQ(first->requestId, 1u);
    const auto second = _handler.handle(result(BeamType::SEARCH, 5, 300ms, 2000_m));
    ASSERT_TRUE(second);
    EXPECT_EQ(second->requestId, 2u);
    EXPECT_FALSE(_handler.handle(result(BeamType::SEARCH, 5, 300ms, 3000_m)));
    EXPECT_FALSE(_handler.handle(result(BeamType::SEARCH, 5, 250ms, 3000_m)));
    const auto third = _handler.handle(result(BeamType::SEARCH, 5, 400ms, 3000_m));
    ASSERT_TRUE(third);
    EXPECT_EQ(third->requestId, 3u);
}

TEST_F(DetectionHandlerTest, PreexistingTargetsDoNotCollideWithNewRegistrations)
{
    const Observation existing(math::SphericalPosition(9000_m, 0_deg, 0_deg), 0_mps, -10.0f, 0ms);
    ASSERT_EQ(_repository.add(_factory.from(_repository.nextId(), existing)), 1u);
    const auto* original = _repository.findTentative(1);

    const auto registered = _handler.handle(result(BeamType::SEARCH, 5, 0ms));
    ASSERT_TRUE(registered);
    EXPECT_EQ(registered->requestId, 2u);
    EXPECT_EQ(_repository.findTentative(1), original);
    EXPECT_EQ(_repository.targets().size(), 2u);

    ASSERT_TRUE(_repository.remove(1));
    EXPECT_FALSE(_handler.handle(result(BeamType::SEARCH, 5, 0ms)));
    const auto next = _handler.handle(result(BeamType::SEARCH, 6, 0ms, 3000_m));
    ASSERT_TRUE(next);
    EXPECT_EQ(next->requestId, 3u);
    EXPECT_EQ(_repository.targets().size(), 2u);
}

} // namespace
} // namespace target
