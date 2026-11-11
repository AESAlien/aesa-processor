#include "domain/association/nds_algorithm.hpp"
#include "domain/tracking/constant_velocity_algorithm.hpp"
#include "domain/factory/detection_result_factory.hpp"
#include "domain/track_factory.hpp"
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
    TrackFactory _factory{std::make_shared<ConstantVelocityAlgorithm>()};
    DetectionHandler _handler = DetectionHandler(
        TrackManagementConfig(100ms, 1000ms, 3, 100),
        _repository,
        _factory,
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
    EXPECT_NE(_repository->findTentativeTrack(first->requestId), nullptr);

    const auto tracking = _handler.handle(result(BeamType::CONFIRMATION, first->requestId, 100ms));
    ASSERT_TRUE(tracking);
    EXPECT_EQ(tracking->beamType, beam::RequestBeamType::TRACKING);
    EXPECT_EQ(tracking->transmitTime, 1100ms);
    ASSERT_TRUE(_handler.handle(result(BeamType::CONFIRMATION, second->requestId, 100ms, 2000_m)));
    ASSERT_TRUE(_handler.handle(result(BeamType::TRACKING, first->requestId, 1100ms, 1010_m)));
    EXPECT_EQ(_repository->findConfirmedTrack(first->requestId)->lastObservationTime(), 1100ms);
    EXPECT_EQ(_repository->findConfirmedTrack(second->requestId)->lastObservationTime(), 100ms);

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
    EXPECT_EQ(_repository->findConfirmedTrack(first->requestId), nullptr);
    ASSERT_NE(_repository->findConfirmedTrack(second->requestId), nullptr);
    EXPECT_EQ(_repository->findConfirmedTrack(second->requestId)->lastObservationTime(), 100ms);
    const auto next = _handler.handle(result(BeamType::SEARCH, 52, 4200ms, 3000_m));
    ASSERT_TRUE(next);
    EXPECT_GT(next->requestId, second->requestId);
}

TEST_F(DetectionHandlerTest, IgnoresWrongPhaseAndStaleResults)
{
    const auto confirmation = _handler.handle(result(BeamType::SEARCH, 5, 0ms));
    ASSERT_TRUE(confirmation);
    const auto id = confirmation->requestId;
    const auto* tentative = _repository->findTentativeTrack(id);
    EXPECT_FALSE(_handler.handle(result(BeamType::TRACKING, id, 100ms)));
    EXPECT_FALSE(_handler.handle(result(BeamType::CONFIRMATION, id, 0ms)));
    EXPECT_FALSE(_handler.handle(result(BeamType::CONFIRMATION, 999, 100ms)));
    EXPECT_FALSE(_handler.handle(result(BeamType::SEARCH, 5, 0ms, 2000_m)));
    EXPECT_EQ(_repository->findTentativeTrack(id), tentative);
    EXPECT_EQ(_repository->tracks().size(), 1u);

    ASSERT_TRUE(_handler.handle(result(BeamType::CONFIRMATION, id, 100ms)));
    const auto* track = _repository->findConfirmedTrack(id);
    const auto state = track->state();
    EXPECT_FALSE(_handler.handle(result(BeamType::CONFIRMATION, id, 200ms)));
    EXPECT_FALSE(_handler.handle(result(BeamType::TRACKING, id, 100ms)));
    EXPECT_EQ(_repository->findConfirmedTrack(id), track);
    EXPECT_EQ(track->state(), state);
    EXPECT_EQ(track->lastResultTime(), 100ms);
    EXPECT_EQ(track->consecutiveMissCount(), 0u);
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
    const auto existing = Observation(math::SphericalPosition(9000_m, 0_deg, 0_deg), 0_mps, -10.0f, 0ms);
    ASSERT_EQ(_repository->add(_factory.createTentative(_repository->nextId(), existing)), 1u);
    const auto* original = _repository->findTentativeTrack(1);

    const auto registered = _handler.handle(result(BeamType::SEARCH, 5, 0ms));
    ASSERT_TRUE(registered);
    EXPECT_EQ(registered->requestId, 2u);
    EXPECT_EQ(_repository->findTentativeTrack(1), original);
    EXPECT_EQ(_repository->tracks().size(), 2u);

    ASSERT_TRUE(_repository->remove(1));
    EXPECT_FALSE(_handler.handle(result(BeamType::SEARCH, 5, 0ms)));
    const auto next = _handler.handle(result(BeamType::SEARCH, 6, 0ms, 3000_m));
    ASSERT_TRUE(next);
    EXPECT_EQ(next->requestId, 3u);
    EXPECT_EQ(_repository->tracks().size(), 2u);
}

TEST_F(DetectionHandlerTest, RemovesConfirmedTrackWhoseTrackingResultNeverArrives)
{
    // 준비: 1100ms에 송신할 추적 빔의 결과가 오지 않은 확정 트랙을 만든다.
    const auto confirmation = _handler.handle(result(BeamType::SEARCH, 5, 0ms));
    ASSERT_TRUE(confirmation);
    const auto id = confirmation->requestId;
    ASSERT_TRUE(_handler.handle(result(BeamType::CONFIRMATION, id, 100ms)));

    // 실행: 응답 대기 한도(추적 주기 x 연속 미탐 한도)까지는 같은 위치의 탐지를 받는다.
    const auto beforeTimeout = _handler.handle(result(BeamType::SEARCH, 6, 4100ms));

    // 검증: 한도 안에서는 기존 트랙과 연관되어 새 트랙을 만들지 않는다.
    EXPECT_FALSE(beforeTimeout);
    EXPECT_NE(_repository->findConfirmedTrack(id), nullptr);

    // 실행: 한도를 넘긴 뒤 같은 위치의 탐지를 받는다.
    const auto afterTimeout = _handler.handle(result(BeamType::SEARCH, 6, 4110ms));

    // 검증: 응답 없는 트랙을 삭제하고 같은 위치에서 새 트랙의 확인 빔을 요청한다.
    EXPECT_EQ(_repository->findConfirmedTrack(id), nullptr);
    ASSERT_TRUE(afterTimeout);
    EXPECT_EQ(afterTimeout->beamType, beam::RequestBeamType::CONFIRMATION);
    EXPECT_GT(afterTimeout->requestId, id);
    EXPECT_EQ(_repository->tracks().size(), 1u);
}

TEST_F(DetectionHandlerTest, RemovesTentativeTrackWhoseConfirmationResultNeverArrives)
{
    // 준비: 100ms에 송신할 확인 빔의 결과가 오지 않은 잠정 트랙을 만든다.
    const auto confirmation = _handler.handle(result(BeamType::SEARCH, 5, 0ms));
    ASSERT_TRUE(confirmation);
    const auto id = confirmation->requestId;

    // 실행: 응답 대기 한도 직전과 직후에 같은 위치의 탐지를 받는다.
    const auto beforeTimeout = _handler.handle(result(BeamType::SEARCH, 6, 3100ms));
    const bool keptBeforeTimeout = _repository->findTentativeTrack(id) != nullptr;
    const auto afterTimeout = _handler.handle(result(BeamType::SEARCH, 7, 3110ms));

    // 검증: 한도 전에는 잠정 트랙을 유지하고, 한도 후에는 삭제한 뒤 새 확인 빔을 요청한다.
    EXPECT_FALSE(beforeTimeout);
    EXPECT_TRUE(keptBeforeTimeout);
    EXPECT_EQ(_repository->findTentativeTrack(id), nullptr);
    ASSERT_TRUE(afterTimeout);
    EXPECT_EQ(afterTimeout->beamType, beam::RequestBeamType::CONFIRMATION);
    EXPECT_GT(afterTimeout->requestId, id);
}

} // namespace
} // namespace target
