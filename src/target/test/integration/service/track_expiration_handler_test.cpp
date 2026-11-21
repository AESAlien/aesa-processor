#include "domain/association/nds_algorithm.hpp"
#include "domain/tracking/constant_velocity_algorithm.hpp"
#include "domain/factory/detection_result_factory.hpp"
#include "domain/factory/track_factory.hpp"
#include "repository/track_repository.hpp"
#include "service/detection_handler.hpp"
#include "service/track_expiration_handler.hpp"
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
using Ids = std::vector<std::uint32_t>;

DetectionResult result(BeamType type, std::uint32_t id, Time time, math::Length range = 1000_m)
{
    std::vector<DetectionEvent::Plot> plots;
    plots.emplace_back(
        math::Distance(range),
        math::CircularAngle(0_deg),
        math::TiltAngle(0_deg),
        0_mps,
        -10.0f
    );
    return DetectionResultFactory::from(DetectionEvent(type, id, time, std::move(plots)));
}

class TrackExpirationHandlerTest : public ::testing::Test
{
protected:
    // 확인 지연 100ms, 추적 주기 1000ms, 연속 미탐 한도 3 → 응답 대기 한도 3000ms
    TrackLifecycleConfig _config = TrackLifecycleConfig(100ms, 1000ms, 3);
    std::shared_ptr<TrackRepository> _repository = std::make_shared<TrackRepository>();
    DetectionHandler _detectionHandler = DetectionHandler(
        _config,
        _repository,
        TrackFactory(std::make_shared<ConstantVelocityAlgorithm>(), _config),
        std::make_unique<TentativeSelector>(
            AssociationConfig(1000_mps, 30_mps, 11.345),
            std::make_unique<NdsAlgorithm>(MeasurementConfig(7_m))
        ),
        std::make_unique<TrackSelector>(
            AssociationConfig(1000_mps, 30_mps, 11.345),
            std::make_unique<NdsAlgorithm>(MeasurementConfig(7_m))
        )
    );
    TrackExpirationHandler _expirationHandler = TrackExpirationHandler(_repository);
};

TEST_F(TrackExpirationHandlerTest, RemovesConfirmedTrackWhoseTrackingResultNeverArrives)
{
    // 준비: 1100ms에 송신할 추적 빔의 결과가 오지 않은 확정 트랙을 만든다.
    const auto confirmation = _detectionHandler.handle(result(BeamType::SEARCH, 5, 0ms));
    ASSERT_TRUE(confirmation);
    const auto id = confirmation->requestId;
    ASSERT_TRUE(_detectionHandler.handle(result(BeamType::CONFIRMATION, id, 100ms)));

    // 실행·검증: 예정 결과 시각(1100ms) + 응답 대기 한도(3000ms)까지는 유지한다.
    EXPECT_TRUE(_expirationHandler.removeExpiredTracks(4100ms).empty());
    EXPECT_NE(_repository->findConfirmedById(id), nullptr);

    // 실행·검증: 한도를 넘기면 삭제하고 삭제한 ID를 반환한다.
    EXPECT_EQ(_expirationHandler.removeExpiredTracks(4110ms), Ids{id});
    EXPECT_EQ(_repository->findConfirmedById(id), nullptr);

    // 검증: 같은 위치의 탐지가 더 이상 막히지 않고 새 트랙의 확인 빔을 요청한다.
    const auto next = _detectionHandler.handle(result(BeamType::SEARCH, 6, 4110ms));
    ASSERT_TRUE(next);
    EXPECT_EQ(next->beamType, beam::RequestBeamType::CONFIRMATION);
    EXPECT_GT(next->requestId, id);
}

TEST_F(TrackExpirationHandlerTest, RemovesTentativeTrackWhoseConfirmationResultNeverArrives)
{
    // 준비: 100ms에 송신할 확인 빔의 결과가 오지 않은 잠정 트랙을 만든다.
    const auto confirmation = _detectionHandler.handle(result(BeamType::SEARCH, 5, 0ms));
    ASSERT_TRUE(confirmation);
    const auto id = confirmation->requestId;

    // 실행·검증: 예정 결과 시각(100ms) + 응답 대기 한도(3000ms)까지는 유지한다.
    EXPECT_TRUE(_expirationHandler.removeExpiredTracks(3100ms).empty());
    EXPECT_NE(_repository->findTentativeById(id), nullptr);

    // 실행·검증: 한도를 넘기면 삭제한다.
    EXPECT_EQ(_expirationHandler.removeExpiredTracks(3110ms), Ids{id});
    EXPECT_EQ(_repository->findTentativeById(id), nullptr);
}

TEST_F(TrackExpirationHandlerTest, KeepsTrackThatReceivesScheduledResults)
{
    // 준비: 1100ms 추적 결과까지 받은 확정 트랙을 만든다. 다음 예정 결과 시각은 2100ms다.
    const auto confirmation = _detectionHandler.handle(result(BeamType::SEARCH, 5, 0ms));
    ASSERT_TRUE(confirmation);
    const auto id = confirmation->requestId;
    ASSERT_TRUE(_detectionHandler.handle(result(BeamType::CONFIRMATION, id, 100ms)));
    ASSERT_TRUE(_detectionHandler.handle(result(BeamType::TRACKING, id, 1100ms)));

    // 실행·검증: 결과를 받지 못했다면 삭제되었을 시각(4110ms)에도 유지한다.
    EXPECT_TRUE(_expirationHandler.removeExpiredTracks(5100ms).empty());
    EXPECT_NE(_repository->findConfirmedById(id), nullptr);
}

} // namespace
} // namespace target
