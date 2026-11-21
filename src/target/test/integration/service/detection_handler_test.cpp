#include "track_state_assertions.hpp"
#include "domain/association/nds_algorithm.hpp"
#include "domain/tracking/constant_velocity_algorithm.hpp"
#include "domain/factory/detection_result_factory.hpp"
#include "domain/factory/track_factory.hpp"
#include "repository/track_repository.hpp"
#include "service/detection_handler.hpp"
#include "domain/association/tentative_selector.hpp"
#include "domain/association/track_selector.hpp"

#include <gtest/gtest.h>

#include <chrono>
#include <cstdint>
#include <memory>
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
    std::shared_ptr<TrackRepository> _repository = std::make_shared<TrackRepository>();
    DetectionHandler _handler = DetectionHandler(
        TrackLifecycleConfig(100ms, 1000ms, 3),
        _repository,
        TrackFactory(std::make_shared<ConstantVelocityAlgorithm>(), TrackLifecycleConfig(100ms, 1000ms, 3)),
        std::make_unique<TentativeSelector>(
            AssociationConfig(1000_mps, 30_mps, 11.345),
            std::make_unique<NdsAlgorithm>(MeasurementConfig(7_m))
        ),
        std::make_unique<TrackSelector>(
            AssociationConfig(1000_mps, 30_mps, 11.345),
            std::make_unique<NdsAlgorithm>(MeasurementConfig(7_m))
        )
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
    EXPECT_NE(_repository->findTentativeById(first->requestId), nullptr);

    const auto tracking = _handler.handle(result(BeamType::CONFIRMATION, first->requestId, 100ms));
    ASSERT_TRUE(tracking);
    EXPECT_EQ(tracking->beamType, beam::RequestBeamType::TRACKING);
    EXPECT_EQ(tracking->transmitTime, 1100ms);
    ASSERT_TRUE(_handler.handle(result(BeamType::CONFIRMATION, second->requestId, 100ms, 2000_m)));
    ASSERT_TRUE(_handler.handle(result(BeamType::TRACKING, first->requestId, 1100ms, 1010_m)));
    EXPECT_EQ(_repository->findConfirmedById(first->requestId)->state().timestamp(), 1100ms);
    EXPECT_EQ(_repository->findConfirmedById(second->requestId)->state().timestamp(), 100ms);

    for (auto miss = 0; miss < 3; ++miss)
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
    EXPECT_EQ(_repository->findConfirmedById(first->requestId), nullptr);
    ASSERT_NE(_repository->findConfirmedById(second->requestId), nullptr);
    EXPECT_EQ(_repository->findConfirmedById(second->requestId)->state().timestamp(), 100ms);
    const auto next = _handler.handle(result(BeamType::SEARCH, 52, 4200ms, 3000_m));
    ASSERT_TRUE(next);
    EXPECT_GT(next->requestId, second->requestId);
}

TEST_F(DetectionHandlerTest, IgnoresWrongPhaseAndStaleResults)
{
    const auto confirmation = _handler.handle(result(BeamType::SEARCH, 5, 0ms));
    ASSERT_TRUE(confirmation);
    const auto id = confirmation->requestId;
    const auto* tentative = _repository->findTentativeById(id);
    EXPECT_FALSE(_handler.handle(result(BeamType::TRACKING, id, 100ms)));
    EXPECT_FALSE(_handler.handle(result(BeamType::CONFIRMATION, id, 0ms)));
    EXPECT_FALSE(_handler.handle(result(BeamType::CONFIRMATION, 999, 100ms)));
    EXPECT_FALSE(_handler.handle(result(BeamType::SEARCH, 5, 0ms, 2000_m)));
    EXPECT_EQ(_repository->findTentativeById(id), tentative);
    EXPECT_EQ(_repository->tracks().size(), 1u);

    ASSERT_TRUE(_handler.handle(result(BeamType::CONFIRMATION, id, 100ms)));
    const auto* track = _repository->findConfirmedById(id);
    const auto state = track->state();
    EXPECT_FALSE(_handler.handle(result(BeamType::CONFIRMATION, id, 200ms)));
    EXPECT_FALSE(_handler.handle(result(BeamType::TRACKING, id, 100ms)));
    EXPECT_EQ(_repository->findConfirmedById(id), track);
    test_support::expectTrackStateEquals(track->state(), state);
    EXPECT_FALSE(track->canProcessResult(100ms));
    EXPECT_TRUE(track->canProcessResult(101ms));
    EXPECT_FALSE(track->willExpireOnNextMiss());
    EXPECT_FALSE(track->hasReachedMissLimit());
}

TEST_F(DetectionHandlerTest, EmptySearchesAdvanceHistoryWithoutConsumingIdentity)
{
    EXPECT_FALSE(_handler.handle(result(BeamType::SEARCH, 5, 200ms, 1000_m, false)));
    EXPECT_FALSE(_handler.handle(result(BeamType::SEARCH, 5, 200ms)));
    EXPECT_FALSE(_handler.handle(result(BeamType::SEARCH, 5, 100ms)));
    EXPECT_TRUE(_repository->tracks().empty());

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
    const auto existing = _handler.handle(result(BeamType::SEARCH, 4, 0ms, 9000_m));
    ASSERT_TRUE(existing);
    ASSERT_EQ(existing->requestId, 1u);
    const auto* original = _repository->findTentativeById(1);

    const auto registered = _handler.handle(result(BeamType::SEARCH, 5, 0ms));
    ASSERT_TRUE(registered);
    EXPECT_EQ(registered->requestId, 2u);
    EXPECT_EQ(_repository->findTentativeById(1), original);
    EXPECT_EQ(_repository->tracks().size(), 2u);

    ASSERT_TRUE(_repository->removeById(1));
    EXPECT_FALSE(_handler.handle(result(BeamType::SEARCH, 5, 0ms)));
    const auto next = _handler.handle(result(BeamType::SEARCH, 6, 0ms, 3000_m));
    ASSERT_TRUE(next);
    EXPECT_EQ(next->requestId, 3u);
    EXPECT_EQ(_repository->tracks().size(), 2u);
}

// 운용을 다시 시작하면 이전 탐색 결과 시각이 새 탐색 결과를 막지 않아야 한다.
TEST_F(DetectionHandlerTest, ClearingSearchHistoryForgetsPreviousSearchResultTimes)
{
    EXPECT_FALSE(_handler.handle(result(BeamType::SEARCH, 50, 500ms, 1000_m, false)));
    EXPECT_FALSE(_handler.handle(result(BeamType::SEARCH, 50, 400ms)));
    EXPECT_TRUE(_repository->tracks().empty());

    _handler.clearSearchHistory();

    const auto confirmation = _handler.handle(result(BeamType::SEARCH, 50, 400ms));
    ASSERT_TRUE(confirmation);
    EXPECT_EQ(confirmation->beamType, beam::RequestBeamType::CONFIRMATION);
    EXPECT_NE(_repository->findTentativeById(confirmation->requestId), nullptr);
}

} // namespace
} // namespace target
