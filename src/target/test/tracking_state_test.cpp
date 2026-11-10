#include "domain/tracking/constant_velocity_algorithm.hpp"
#include "domain/track_state.hpp"
#include "domain/track_factory.hpp"
#include "repository/track_repository.hpp"

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
        TrackFactory(std::make_shared<ConstantVelocityAlgorithm>()).createTentative(repository.nextId(), observation(1000_m, 0ms))
    );
    auto confirmedTrack = repository.findTentativeTrack(id)->confirm(observation(1010_m, 100ms));
    ASSERT_TRUE(confirmedTrack);
    ASSERT_TRUE(repository.replace(std::move(confirmedTrack).value()));
    auto& target = *repository.findConfirmedTrack(id);
    EXPECT_EQ(target.state()->acceleration().acceleration(), 0_mps2);
    ASSERT_TRUE(target.update(observation(1020_m, 200ms)));
    EXPECT_EQ(target.state()->acceleration().acceleration(), 0_mps2);
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
        std::make_shared<ConstantVelocityAlgorithm>()
    );

    const auto predicted = track.predict(10100ms);
    ASSERT_TRUE(predicted);
    EXPECT_DOUBLE_EQ(predicted->sphericalPosition().radius.m(), 1000.0);
}

TEST(TrackingStateTest, RejectsNonfiniteAccelerationComponents)
{
    for (auto invalid : {std::numeric_limits<double>::quiet_NaN(),
             std::numeric_limits<double>::infinity(),
             -std::numeric_limits<double>::infinity()})
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
    const auto id = repository.add(TrackFactory(algorithm).createTentative(repository.nextId(), observation(1000_m, 0ms)));
    auto confirmedTrack = repository.findTentativeTrack(id)->confirm(observation(1010_m, 100ms));
    ASSERT_TRUE(confirmedTrack);
    ASSERT_TRUE(repository.replace(std::move(confirmedTrack).value()));
    auto& target = *repository.findConfirmedTrack(id);
    const auto initialTrack = target;
    const auto initialSnapshot = target.state();
    EXPECT_EQ(initialSnapshot->acceleration().x, 1_mps2);
    EXPECT_EQ(initialSnapshot->acceleration().y, 2_mps2);
    EXPECT_EQ(initialSnapshot->acceleration().z, 3_mps2);
    EXPECT_EQ(EnrichedAlgorithm::data(target), 25);

    ASSERT_TRUE(target.update(observation(1020_m, 200ms)));
    const auto updatedSnapshot = target.state();
    EXPECT_NE(updatedSnapshot.get(), initialSnapshot.get());
    EXPECT_EQ(updatedSnapshot->acceleration().x, 4_mps2);
    EXPECT_EQ(updatedSnapshot->acceleration().y, 5_mps2);
    EXPECT_EQ(updatedSnapshot->acceleration().z, 6_mps2);
    EXPECT_EQ(EnrichedAlgorithm::data(target), 50);
    EXPECT_EQ(initialSnapshot->acceleration().x, 1_mps2);
    EXPECT_EQ(EnrichedAlgorithm::data(initialTrack), 25);

    ASSERT_TRUE(target.recordTrackingMiss(1200ms));
    ASSERT_TRUE(target.recordTrackingMiss(2200ms));
    EXPECT_EQ(target.state().get(), updatedSnapshot.get());
    EXPECT_EQ(EnrichedAlgorithm::data(target), 50);
    EXPECT_EQ(target.lastObservationTime(), 200ms);
    EXPECT_EQ(target.lastResultTime(), 2200ms);
    EXPECT_EQ(target.consecutiveMissCount(), 2u);
}

TEST(TrackingStateTest, RejectedUpdatesPreserveAccelerationStrategyDataAndMissHistory)
{
    const auto algorithm = std::make_shared<EnrichedAlgorithm>();
    TrackRepository repository;
    const auto id = repository.add(TrackFactory(algorithm).createTentative(repository.nextId(), observation(1000_m, 0ms)));
    auto confirmedTrack = repository.findTentativeTrack(id)->confirm(observation(1010_m, 100ms));
    ASSERT_TRUE(confirmedTrack);
    ASSERT_TRUE(repository.replace(std::move(confirmedTrack).value()));
    auto& target = *repository.findConfirmedTrack(id);
    ASSERT_TRUE(target.recordTrackingMiss(1100ms));
    const auto saved = target.state();

    algorithm->fault = EnrichedAlgorithm::Fault::NO_RESULT;
    EXPECT_FALSE(target.update(observation(1020_m, 2100ms)));
    EXPECT_EQ(target.state(), saved);
    EXPECT_EQ(EnrichedAlgorithm::data(target), 25);

    for (auto fault : {EnrichedAlgorithm::Fault::INVALID_DATA,
             EnrichedAlgorithm::Fault::WRONG_TIME})
    {
        algorithm->fault = fault;
        EXPECT_THROW(target.update(observation(1020_m, 2100ms)), std::invalid_argument);
        EXPECT_EQ(target.state().get(), saved.get());
        EXPECT_EQ(target.state()->acceleration().x, 1_mps2);
        EXPECT_EQ(target.state()->acceleration().y, 2_mps2);
        EXPECT_EQ(target.state()->acceleration().z, 3_mps2);
        EXPECT_EQ(EnrichedAlgorithm::data(target), 25);
        EXPECT_EQ(target.lastObservationTime(), 100ms);
        EXPECT_EQ(target.lastResultTime(), 1100ms);
        EXPECT_EQ(target.consecutiveMissCount(), 1u);
    }
    algorithm->fault = EnrichedAlgorithm::Fault::NONE;
    ASSERT_TRUE(target.update(observation(1020_m, 2100ms)));
    EXPECT_EQ(target.state()->acceleration().x, 4_mps2);
    EXPECT_EQ(target.state()->acceleration().y, 5_mps2);
    EXPECT_EQ(target.state()->acceleration().z, 6_mps2);
    EXPECT_EQ(EnrichedAlgorithm::data(target), 50);
    EXPECT_EQ(target.consecutiveMissCount(), 0u);
}

TEST(TrackingStateTest, RemovingTracksPreservesRemainingAccelerationAndStrategyData)
{
    const auto algorithm = std::make_shared<EnrichedAlgorithm>();
    auto factory = TrackFactory(algorithm);
    TrackRepository repository;
    const auto firstId = repository.add(
        factory.createTentative(
            repository.nextId(),
            observation(1000_m, 0ms)
        )
    );
    const auto middleId = repository.add(
        factory.createTentative(
            repository.nextId(),
            observation(2000_m, 0ms)
        )
    );
    const auto lastId = repository.add(
        factory.createTentative(
            repository.nextId(),
            observation(3000_m, 0ms)
        )
    );
    auto confirmedFirstTrack = repository.findTentativeTrack(firstId)->confirm(observation(1001_m, 100ms));
    ASSERT_TRUE(confirmedFirstTrack);
    ASSERT_TRUE(repository.replace(std::move(confirmedFirstTrack).value()));
    auto confirmedMiddleTrack = repository.findTentativeTrack(middleId)->confirm(observation(2001_m, 100ms));
    ASSERT_TRUE(confirmedMiddleTrack);
    ASSERT_TRUE(repository.replace(std::move(confirmedMiddleTrack).value()));
    auto confirmedLastTrack = repository.findTentativeTrack(lastId)->confirm(observation(3001_m, 100ms));
    ASSERT_TRUE(confirmedLastTrack);
    ASSERT_TRUE(repository.replace(std::move(confirmedLastTrack).value()));
    ASSERT_TRUE(repository.findConfirmedTrack(lastId)->update(observation(3002_m, 200ms)));
    const auto saved = repository.findConfirmedTrack(lastId)->state();

    ASSERT_TRUE(repository.remove(middleId));
    ASSERT_EQ(repository.tracks().size(), 2u);
    EXPECT_EQ(std::get<ConfirmedTrack>(repository.tracks().front()).id(), firstId);
    EXPECT_EQ(std::get<ConfirmedTrack>(repository.tracks().back()).id(), lastId);
    EXPECT_EQ(repository.findConfirmedTrack(firstId)->state()->acceleration().x, 1_mps2);
    EXPECT_EQ(repository.findConfirmedTrack(firstId)->state()->acceleration().y, 2_mps2);
    EXPECT_EQ(repository.findConfirmedTrack(firstId)->state()->acceleration().z, 3_mps2);
    EXPECT_EQ(EnrichedAlgorithm::data(*repository.findConfirmedTrack(firstId)), 25);
    EXPECT_EQ(repository.findConfirmedTrack(lastId)->state().get(), saved.get());
    EXPECT_EQ(saved->acceleration().x, 4_mps2);
    EXPECT_EQ(saved->acceleration().y, 5_mps2);
    EXPECT_EQ(saved->acceleration().z, 6_mps2);
    EXPECT_EQ(EnrichedAlgorithm::data(*repository.findConfirmedTrack(lastId)), 50);
}

} // namespace
} // namespace target
