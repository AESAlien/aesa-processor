#include "track_state_assertions.hpp"
#include "domain/tracking/constant_velocity_algorithm.hpp"
#include "domain/track_state.hpp"
#include "domain/factory/track_factory.hpp"
#include "repository/track_repository.hpp"

#include <target/config/track_lifecycle_config.hpp>

#include <gtest/gtest.h>

#include <chrono>
#include <limits>
#include <memory>
#include <optional>
#include <stdexcept>
#include <type_traits>
#include <utility>

namespace target
{
namespace
{

using namespace math::literals;
using namespace std::chrono_literals;
using Time = std::chrono::milliseconds;

static_assert(std::is_copy_assignable_v<TrackState>);
static_assert(std::is_nothrow_move_constructible_v<ConfirmedTrack>);

math::SphericalPosition position(math::Length range = 1000_m)
{
    return math::SphericalPosition(range, 0_deg, 0_deg);
}

Observation observation(math::Length range, Time time)
{
    return Observation(position(range), 0_mps, -10.0f, time);
}

class EnrichedAlgorithm final : public ITrackingAlgorithm
{
public:
    enum class Fault
    {
        NONE,
        NO_RESULT,
        INVALID_DATA,
        WRONG_TIME
    };

    std::optional<TrackingResult> initialize(const Observation& first, const Observation& second) const override
    {
        return enrich(_algorithm.initialize(first, second), false);
    }

    std::optional<TrackingResult> update(const ConfirmedTrack& current, const Observation& incoming) const override
    {
        return enrich(_algorithm.update(current, incoming), true);
    }

    std::optional<TrackPrediction> predict(const ConfirmedTrack& current, Time time) const override
    {
        return _algorithm.predict(current, time);
    }

    Fault fault = Fault::NONE;

    static int data(const ConfirmedTrack& track)
    {
        return std::any_cast<int>(algorithmState(track));
    }

private:
    std::optional<TrackingResult> enrich(
        const std::optional<TrackingResult>& calculated,
        bool updating
    ) const {
        if (!calculated)
        {
            return std::nullopt;
        }
        const auto acceleration = updating
            ? math::CartesianAcceleration(4_mps2, 5_mps2, 6_mps2)
            : math::CartesianAcceleration(1_mps2, 2_mps2, 3_mps2);
        const auto data = updating ? 50 : 25;
        if (fault == Fault::INVALID_DATA)
        {
            throw std::invalid_argument("Invalid strategy data");
        }
        if (fault == Fault::NO_RESULT)
        {
            return std::nullopt;
        }
        const auto& [state, algorithmData] = *calculated;
        const auto timestamp = state.timestamp() + (fault == Fault::WRONG_TIME ? 1ms : 0ms);
        auto nextState = TrackState(
            state.sphericalPosition(),
            state.dopplerVelocity(),
            state.power_db(),
            timestamp,
            state.position(),
            state.velocity(),
            acceleration
        );
        return TrackingResult{std::move(nextState), data};
    }

    ConstantVelocityAlgorithm _algorithm;
};

TEST(TrackResultProcessingTest, TentativeTrackAcceptsOnlyResultsAfterInitialObservation)
{
    // 준비: 최초 관측 시각이 100ms인 잠정 트랙을 만든다.
    const TentativeTrack track(1, observation(1000_m, 100ms), TrackLifecycleConfig(100ms, 1000ms, 3), std::make_shared<ConstantVelocityAlgorithm>());

    // 실행 및 검증: 같은 시각과 이전 시각은 거부하고, 새로운 시각만 처리한다.
    EXPECT_FALSE(track.canProcessResult(99ms));
    EXPECT_FALSE(track.canProcessResult(100ms));
    EXPECT_TRUE(track.canProcessResult(101ms));
}

TEST(TrackResultProcessingTest, ConfirmedTrackRejectsResultsAlreadyCoveredByATrackingMiss)
{
    // 준비: 100ms에 확정한 트랙에 1100ms 미탐 결과를 기록한다.
    const TentativeTrack tentative(1, observation(1000_m, 0ms), TrackLifecycleConfig(100ms, 1000ms, 3), std::make_shared<ConstantVelocityAlgorithm>());
    auto track = tentative.confirm(observation(1010_m, 100ms));
    ASSERT_TRUE(track);
    EXPECT_FALSE(track->canProcessResult(99ms));
    EXPECT_FALSE(track->canProcessResult(100ms));
    EXPECT_TRUE(track->canProcessResult(101ms));
    ASSERT_TRUE(track->recordTrackingMiss(1100ms));
    const auto savedState = track->state();

    // 실행 및 검증: 마지막 관측보다 새로워도 미탐 시각 이하인 결과는 다시 처리하지 않는다.
    EXPECT_FALSE(track->canProcessResult(1000ms));
    EXPECT_FALSE(track->canProcessResult(1100ms));
    EXPECT_TRUE(track->canProcessResult(1101ms));
    EXPECT_FALSE(track->update(observation(1020_m, 1000ms)));
    EXPECT_FALSE(track->recordTrackingMiss(1100ms));
    test_support::expectTrackStateEquals(track->state(), savedState);
    EXPECT_EQ(track->state().timestamp(), 100ms);
    EXPECT_FALSE(track->willExpireOnNextMiss());
    EXPECT_FALSE(track->hasReachedMissLimit());
    EXPECT_FALSE(track->canProcessResult(1100ms));
    EXPECT_TRUE(track->canProcessResult(1101ms));
    ASSERT_TRUE(track->recordTrackingMiss(2100ms));
    EXPECT_TRUE(track->willExpireOnNextMiss());
    EXPECT_FALSE(track->hasReachedMissLimit());
}

TEST(TrackMissLimitTest, DistinguishesNextMissFromAnAlreadyReachedLimit)
{
    // 준비: 연속 미탐 한도가 3인 확정 트랙을 만든다.
    const TrackLifecycleConfig config(100ms, 1000ms, 3);
    const TentativeTrack tentative(1, observation(1000_m, 0ms), config, std::make_shared<ConstantVelocityAlgorithm>());
    auto track = tentative.confirm(observation(1010_m, 100ms));
    ASSERT_TRUE(track);
    EXPECT_FALSE(track->willExpireOnNextMiss());
    EXPECT_FALSE(track->hasReachedMissLimit());

    // 실행 및 검증: 한도 직전에는 다음 미탐만 만료를 유발하고, 기록 후에는 한도 도달로 판단한다.
    ASSERT_TRUE(track->recordTrackingMiss(1100ms));
    EXPECT_FALSE(track->willExpireOnNextMiss());
    EXPECT_FALSE(track->hasReachedMissLimit());
    ASSERT_TRUE(track->recordTrackingMiss(2100ms));
    EXPECT_TRUE(track->willExpireOnNextMiss());
    EXPECT_FALSE(track->hasReachedMissLimit());
    ASSERT_TRUE(track->recordTrackingMiss(3100ms));
    EXPECT_TRUE(track->willExpireOnNextMiss());
    EXPECT_TRUE(track->hasReachedMissLimit());
    ASSERT_TRUE(track->recordTrackingMiss(4100ms));
    EXPECT_TRUE(track->hasReachedMissLimit());
}

TEST(TrackMissLimitTest, SingleMissLimitExpiresOnTheFirstMiss)
{
    // 준비: 연속 미탐 한도가 1인 확정 트랙을 만든다.
    const TrackLifecycleConfig config(100ms, 1000ms, 1);
    const TentativeTrack tentative(1, observation(1000_m, 0ms), config, std::make_shared<ConstantVelocityAlgorithm>());
    auto track = tentative.confirm(observation(1010_m, 100ms));
    ASSERT_TRUE(track);

    // 실행 및 검증: 첫 미탐 전후의 판단을 구분한다.
    EXPECT_TRUE(track->willExpireOnNextMiss());
    EXPECT_FALSE(track->hasReachedMissLimit());
    ASSERT_TRUE(track->recordTrackingMiss(1100ms));
    EXPECT_TRUE(track->hasReachedMissLimit());
}

TEST(TrackMissLimitTest, SuccessfulUpdateResetsMissLimitJudgments)
{
    // 준비: 두 번 미탐해 다음 미탐으로 만료될 확정 트랙을 만든다.
    const TrackLifecycleConfig config(100ms, 1000ms, 3);
    const TentativeTrack tentative(1, observation(1000_m, 0ms), config, std::make_shared<ConstantVelocityAlgorithm>());
    auto track = tentative.confirm(observation(1010_m, 100ms));
    ASSERT_TRUE(track);
    ASSERT_TRUE(track->recordTrackingMiss(1100ms));
    ASSERT_TRUE(track->recordTrackingMiss(2100ms));
    ASSERT_TRUE(track->willExpireOnNextMiss());

    // 실행 및 검증: 유효 관측으로 갱신하면 연속 미탐 이력이 초기화된다.
    ASSERT_TRUE(track->update(observation(1310_m, 3100ms)));
    EXPECT_FALSE(track->willExpireOnNextMiss());
    EXPECT_FALSE(track->hasReachedMissLimit());
    ASSERT_TRUE(track->recordTrackingMiss(4100ms));
    EXPECT_FALSE(track->willExpireOnNextMiss());
    ASSERT_TRUE(track->recordTrackingMiss(5100ms));
    EXPECT_TRUE(track->willExpireOnNextMiss());
    EXPECT_FALSE(track->hasReachedMissLimit());
    ASSERT_TRUE(track->recordTrackingMiss(6100ms));
    EXPECT_TRUE(track->hasReachedMissLimit());
}

TEST(TrackMissLimitTest, FactoryAndTracksKeepTheLifecycleSnapshotFromConstruction)
{
    // 준비: 팩터리를 생성한 뒤 원본 설정의 시간과 미탐 한도를 모두 교체한다.
    auto config = TrackLifecycleConfig(100ms, 1000ms, 1);
    TrackFactory factory(std::make_shared<ConstantVelocityAlgorithm>(), config);
    config = TrackLifecycleConfig(500ms, 5000ms, 3);
    const auto tentative = factory.createTentative(observation(1000_m, 500ms));
    const auto confirmed = tentative.confirm(observation(1010_m, 600ms));
    ASSERT_TRUE(confirmed);

    // 실행 및 검증: 잠정과 확정 트랙 모두 원래 설정으로 판단하고, 확정 시 미탐 정책도 유지한다.
    EXPECT_FALSE(tentative.isExpired(1600ms));
    EXPECT_TRUE(tentative.isExpired(1601ms));
    EXPECT_FALSE(confirmed->isExpired(2600ms));
    EXPECT_TRUE(confirmed->isExpired(2601ms));
    EXPECT_TRUE(confirmed->willExpireOnNextMiss());
}

TEST(TrackExpirationTest, TentativeTrackExpiresOnlyAfterConfirmationResponseDeadline)
{
    // 준비: 500ms 탐색 결과와 100ms 확인 지연, 3000ms 응답 대기 한도를 사용한다.
    const TrackLifecycleConfig config(100ms, 1000ms, 3);
    const TentativeTrack track(1, observation(1000_m, 500ms), config, std::make_shared<ConstantVelocityAlgorithm>());

    // 실행 및 검증: 3600ms 경계까지 유지하고, 이를 넘으면 만료한다.
    EXPECT_FALSE(track.isExpired(3599ms));
    EXPECT_FALSE(track.isExpired(3600ms));
    EXPECT_TRUE(track.isExpired(3601ms));
}

TEST(TrackExpirationTest, ConfirmedTrackUsesTrackingIntervalInsteadOfConfirmationDelay)
{
    // 준비: 600ms에 확정하고 다음 추적 결과를 1600ms에 기다리는 트랙을 만든다.
    const TrackLifecycleConfig config(100ms, 1000ms, 3);
    const TentativeTrack tentative(1, observation(1000_m, 500ms), config, std::make_shared<ConstantVelocityAlgorithm>());
    const auto track = tentative.confirm(observation(1010_m, 600ms));
    ASSERT_TRUE(track);

    // 실행 및 검증: 잠정 트랙의 한도를 지난 뒤에도 유지하고 4600ms를 넘으면 만료한다.
    EXPECT_FALSE(track->isExpired(3601ms));
    EXPECT_FALSE(track->isExpired(4599ms));
    EXPECT_FALSE(track->isExpired(4600ms));
    EXPECT_TRUE(track->isExpired(4601ms));
}

TEST(TrackExpirationTest, SuccessfulUpdateMovesResponseDeadline)
{
    // 준비: 확정 후 1600ms의 추적 결과로 갱신한다.
    const TrackLifecycleConfig config(100ms, 1000ms, 3);
    const TentativeTrack tentative(1, observation(1000_m, 500ms), config, std::make_shared<ConstantVelocityAlgorithm>());
    auto track = tentative.confirm(observation(1010_m, 600ms));
    ASSERT_TRUE(track);
    ASSERT_TRUE(track->update(observation(1110_m, 1600ms)));

    // 실행 및 검증: 이전 한도 이후에도 유지하고 새 한도인 5600ms를 넘으면 만료한다.
    EXPECT_FALSE(track->isExpired(4601ms));
    EXPECT_FALSE(track->isExpired(5600ms));
    EXPECT_TRUE(track->isExpired(5601ms));
}

TEST(TrackExpirationTest, TrackingMissMovesDeadlineWithoutChangingLastObservation)
{
    // 준비: 새 관측 없이 1600ms의 미탐 결과만 기록한다.
    const TrackLifecycleConfig config(100ms, 1000ms, 3);
    const TentativeTrack tentative(1, observation(1000_m, 500ms), config, std::make_shared<ConstantVelocityAlgorithm>());
    auto track = tentative.confirm(observation(1010_m, 600ms));
    ASSERT_TRUE(track);
    const auto savedState = track->state();
    ASSERT_TRUE(track->recordTrackingMiss(1600ms));

    // 실행 및 검증: 관측 상태는 보존하고 응답 대기 한도는 미탐 결과 시각을 기준으로 갱신한다.
    test_support::expectTrackStateEquals(track->state(), savedState);
    EXPECT_EQ(track->state().timestamp(), 600ms);
    EXPECT_FALSE(track->isExpired(4601ms));
    EXPECT_FALSE(track->isExpired(5600ms));
    EXPECT_TRUE(track->isExpired(5601ms));
}

TEST(TrackingStateTest, ExistingConstructorAndConstantVelocityUseZeroAcceleration)
{
    const auto legacy = TrackState(
        position(),
        0_mps,
        -10.0f,
        100ms,
        math::CartesianPosition(0_m, 1000_m, 0_m),
        math::CartesianVelocity(0_mps, 0_mps, 0_mps)
    );
    EXPECT_EQ(legacy.acceleration().acceleration(), 0_mps2);

    TrackRepository repository;
    const auto id = repository.add(
        TrackFactory(std::make_shared<ConstantVelocityAlgorithm>(), TrackLifecycleConfig(100ms, 1000ms, 3)).createTentative(observation(1000_m, 0ms))
    );
    auto confirmedTrack = repository.findTentativeById(id)->confirm(observation(1010_m, 100ms));
    ASSERT_TRUE(confirmedTrack);
    ASSERT_TRUE(repository.replace(std::move(confirmedTrack).value()));
    auto& target = *repository.findConfirmedById(id);
    EXPECT_EQ(target.state().acceleration().acceleration(), 0_mps2);
    ASSERT_TRUE(target.update(observation(1020_m, 200ms)));
    EXPECT_EQ(target.state().acceleration().acceleration(), 0_mps2);
}

TEST(ConstantVelocityAlgorithmTest, TreatsVelocityWithinDefaultToleranceAsStationary)
{
    const auto track = ConfirmedTrack(
        1,
        0ms,
        TrackState(
            position(),
            0_mps,
            -10.0f,
            100ms,
            math::CartesianPosition(0_m, 1000_m, 0_m),
            math::CartesianVelocity(0_mps, 5e-13_mps, 0_mps)
        ),
        TrackLifecycleConfig(100ms, 1000ms, 3),
        std::make_shared<ConstantVelocityAlgorithm>()
    );

    const auto predicted = track.predict(10100ms);
    ASSERT_TRUE(predicted);
    EXPECT_DOUBLE_EQ(predicted->sphericalPosition().radius.m(), 1000.0);
}

TEST(TrackingStateTest, RejectsNonfiniteAccelerationComponents)
{
    for (auto invalid : {std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::infinity(), -std::numeric_limits<double>::infinity()})
    {
        EXPECT_THROW(
            math::CartesianAcceleration(
                0_mps2,
                math::LinearAcceleration::fromMetersPerSecondSquared(invalid),
                0_mps2
            ),
            std::invalid_argument
        );
    }
}

TEST(TrackingStateTest, StoresAlgorithmStateAcrossUpdatesAndPreservesItDuringMisses)
{
    const auto algorithm = std::make_shared<EnrichedAlgorithm>();
    TrackRepository repository;
    const auto id = repository.add(TrackFactory(algorithm, TrackLifecycleConfig(100ms, 1000ms, 3)).createTentative(observation(1000_m, 0ms)));
    auto confirmedTrack = repository.findTentativeById(id)->confirm(observation(1010_m, 100ms));
    ASSERT_TRUE(confirmedTrack);
    ASSERT_TRUE(repository.replace(std::move(confirmedTrack).value()));
    auto& target = *repository.findConfirmedById(id);
    const auto initialTrack = target;
    const auto initialSnapshot = target.state();
    EXPECT_EQ(initialSnapshot.acceleration().x, 1_mps2);
    EXPECT_EQ(initialSnapshot.acceleration().y, 2_mps2);
    EXPECT_EQ(initialSnapshot.acceleration().z, 3_mps2);
    EXPECT_EQ(EnrichedAlgorithm::data(target), 25);

    ASSERT_TRUE(target.update(observation(1020_m, 200ms)));
    const auto updatedSnapshot = target.state();
    EXPECT_NE(updatedSnapshot.timestamp(), initialSnapshot.timestamp());
    EXPECT_EQ(updatedSnapshot.acceleration().x, 4_mps2);
    EXPECT_EQ(updatedSnapshot.acceleration().y, 5_mps2);
    EXPECT_EQ(updatedSnapshot.acceleration().z, 6_mps2);
    EXPECT_EQ(EnrichedAlgorithm::data(target), 50);
    EXPECT_EQ(initialSnapshot.acceleration().x, 1_mps2);
    EXPECT_EQ(EnrichedAlgorithm::data(initialTrack), 25);
    test_support::expectTrackStateEquals(initialTrack.state(), initialSnapshot);
    EXPECT_TRUE(initialTrack.canProcessResult(200ms));
    EXPECT_FALSE(target.canProcessResult(200ms));

    ASSERT_TRUE(target.recordTrackingMiss(1200ms));
    ASSERT_TRUE(target.recordTrackingMiss(2200ms));
    test_support::expectTrackStateEquals(target.state(), updatedSnapshot);
    EXPECT_EQ(EnrichedAlgorithm::data(target), 50);
    EXPECT_EQ(target.state().timestamp(), 200ms);
    EXPECT_FALSE(target.canProcessResult(2200ms));
    EXPECT_TRUE(target.canProcessResult(2201ms));
    EXPECT_TRUE(target.willExpireOnNextMiss());
    EXPECT_FALSE(target.hasReachedMissLimit());
}

TEST(TrackingStateTest, RejectedUpdatesPreserveAccelerationStrategyDataAndMissHistory)
{
    const auto algorithm = std::make_shared<EnrichedAlgorithm>();
    TrackRepository repository;
    const auto id = repository.add(TrackFactory(algorithm, TrackLifecycleConfig(100ms, 1000ms, 3)).createTentative(observation(1000_m, 0ms)));
    auto confirmedTrack = repository.findTentativeById(id)->confirm(observation(1010_m, 100ms));
    ASSERT_TRUE(confirmedTrack);
    ASSERT_TRUE(repository.replace(std::move(confirmedTrack).value()));
    auto& target = *repository.findConfirmedById(id);
    ASSERT_TRUE(target.recordTrackingMiss(1100ms));
    ASSERT_TRUE(target.recordTrackingMiss(1200ms));
    const auto saved = target.state();

    algorithm->fault = EnrichedAlgorithm::Fault::NO_RESULT;
    EXPECT_FALSE(target.update(observation(1020_m, 2100ms)));
    test_support::expectTrackStateEquals(target.state(), saved);
    EXPECT_EQ(EnrichedAlgorithm::data(target), 25);
    EXPECT_TRUE(target.willExpireOnNextMiss());
    EXPECT_FALSE(target.hasReachedMissLimit());

    for (auto fault : {EnrichedAlgorithm::Fault::INVALID_DATA, EnrichedAlgorithm::Fault::WRONG_TIME})
    {
        algorithm->fault = fault;
        EXPECT_THROW(target.update(observation(1020_m, 2100ms)), std::invalid_argument);
        test_support::expectTrackStateEquals(target.state(), saved);
        EXPECT_EQ(target.state().acceleration().x, 1_mps2);
        EXPECT_EQ(target.state().acceleration().y, 2_mps2);
        EXPECT_EQ(target.state().acceleration().z, 3_mps2);
        EXPECT_EQ(EnrichedAlgorithm::data(target), 25);
        EXPECT_EQ(target.state().timestamp(), 100ms);
        EXPECT_FALSE(target.canProcessResult(1200ms));
        EXPECT_TRUE(target.canProcessResult(1201ms));
        EXPECT_TRUE(target.willExpireOnNextMiss());
        EXPECT_FALSE(target.hasReachedMissLimit());
    }
    algorithm->fault = EnrichedAlgorithm::Fault::NONE;
    ASSERT_TRUE(target.update(observation(1020_m, 2100ms)));
    EXPECT_EQ(target.state().acceleration().x, 4_mps2);
    EXPECT_EQ(target.state().acceleration().y, 5_mps2);
    EXPECT_EQ(target.state().acceleration().z, 6_mps2);
    EXPECT_EQ(EnrichedAlgorithm::data(target), 50);
    EXPECT_FALSE(target.willExpireOnNextMiss());
    EXPECT_FALSE(target.hasReachedMissLimit());
}

TEST(TrackingStateTest, RemovingTracksPreservesRemainingAccelerationAndStrategyData)
{
    const auto algorithm = std::make_shared<EnrichedAlgorithm>();
    auto factory = TrackFactory(algorithm, TrackLifecycleConfig(100ms, 1000ms, 3));
    TrackRepository repository;
    const auto firstId = repository.add(
        factory.createTentative(
            observation(1000_m, 0ms)
        )
    );
    const auto middleId = repository.add(
        factory.createTentative(
            observation(2000_m, 0ms)
        )
    );
    const auto lastId = repository.add(
        factory.createTentative(
            observation(3000_m, 0ms)
        )
    );
    auto confirmedFirstTrack = repository.findTentativeById(firstId)->confirm(observation(1001_m, 100ms));
    ASSERT_TRUE(confirmedFirstTrack);
    ASSERT_TRUE(repository.replace(std::move(confirmedFirstTrack).value()));
    auto confirmedMiddleTrack = repository.findTentativeById(middleId)->confirm(observation(2001_m, 100ms));
    ASSERT_TRUE(confirmedMiddleTrack);
    ASSERT_TRUE(repository.replace(std::move(confirmedMiddleTrack).value()));
    auto confirmedLastTrack = repository.findTentativeById(lastId)->confirm(observation(3001_m, 100ms));
    ASSERT_TRUE(confirmedLastTrack);
    ASSERT_TRUE(repository.replace(std::move(confirmedLastTrack).value()));
    ASSERT_TRUE(repository.findConfirmedById(lastId)->update(observation(3002_m, 200ms)));
    const auto saved = repository.findConfirmedById(lastId)->state();

    ASSERT_TRUE(repository.removeById(middleId));
    ASSERT_EQ(repository.tracks().size(), 2u);
    EXPECT_EQ(std::get<ConfirmedTrack>(repository.tracks().front()).id(), firstId);
    EXPECT_EQ(std::get<ConfirmedTrack>(repository.tracks().back()).id(), lastId);
    EXPECT_EQ(repository.findConfirmedById(firstId)->state().acceleration().x, 1_mps2);
    EXPECT_EQ(repository.findConfirmedById(firstId)->state().acceleration().y, 2_mps2);
    EXPECT_EQ(repository.findConfirmedById(firstId)->state().acceleration().z, 3_mps2);
    EXPECT_EQ(EnrichedAlgorithm::data(*repository.findConfirmedById(firstId)), 25);
    test_support::expectTrackStateEquals(repository.findConfirmedById(lastId)->state(), saved);
    EXPECT_EQ(saved.acceleration().x, 4_mps2);
    EXPECT_EQ(saved.acceleration().y, 5_mps2);
    EXPECT_EQ(saved.acceleration().z, 6_mps2);
    EXPECT_EQ(EnrichedAlgorithm::data(*repository.findConfirmedById(lastId)), 50);
}

} // namespace
} // namespace target
