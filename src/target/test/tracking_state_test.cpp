#include "domain/algorithm/constant_velocity_algorithm.hpp"
#include "domain/track_state.hpp"
#include "domain/factory/track_factory.hpp"
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

TrackState state(
    math::CartesianAcceleration acceleration = math::CartesianAcceleration(0_mps2, 0_mps2, 0_mps2),
    std::optional<math::SquareMatrix> covariance = std::nullopt
) {
    return TrackState(
        position(),
        0_mps,
        -10.0f,
        100ms,
        math::CartesianPosition(0_m, 1000_m, 0_m),
        math::CartesianVelocity(0_mps, 0_mps, 0_mps),
        std::move(acceleration),
        std::move(covariance)
    );
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
        INVALID_COVARIANCE,
        WRONG_TIME
    };

    std::optional<TrackState> initialize(const Observation& first, const Observation& second) const override
    {
        return enrich(_algorithm.initialize(first, second), false);
    }

    std::optional<TrackState> update(const ConfirmedTrack& current, const Observation& incoming) const override
    {
        return enrich(_algorithm.update(current, incoming), true);
    }

    std::optional<TargetPrediction> predict(const ConfirmedTrack& current, Time time) const override
    {
        return _algorithm.predict(current, time);
    }

    Fault fault = Fault::NONE;

private:
    std::optional<TrackState> enrich(
        const std::optional<TrackState>& calculated,
        bool updating
    ) const {
        if (!calculated)
        {
            return std::nullopt;
        }
        const auto acceleration = updating
            ? math::CartesianAcceleration(4_mps2, 5_mps2, 6_mps2)
            : math::CartesianAcceleration(1_mps2, 2_mps2, 3_mps2);
        auto covariance = math::SquareMatrix::identity(TrackState::STATE_DIMENSION)
            * (updating ? 50.0 : 25.0);
        if (fault == Fault::INVALID_COVARIANCE)
        {
            covariance(0, 0) = -1.0;
        }
        const auto timestamp = calculated->timestamp() + (fault == Fault::WRONG_TIME ? 1ms : 0ms);
        return TrackState(
            calculated->sphericalPosition(),
            calculated->dopplerVelocity(),
            calculated->power_db(),
            timestamp,
            calculated->position(),
            calculated->velocity(),
            acceleration,
            std::move(covariance)
        );
    }

    ConstantVelocityAlgorithm _algorithm;
};

TEST(TrackingStateTest, ExistingConstructorAndConstantVelocityLeaveCovarianceUnavailable)
{
    const TrackState legacy(
        position(),
        0_mps,
        -10.0f,
        100ms,
        math::CartesianPosition(0_m, 1000_m, 0_m),
        math::CartesianVelocity(0_mps, 0_mps, 0_mps)
    );
    EXPECT_EQ(legacy.acceleration().acceleration(), 0_mps2);
    EXPECT_FALSE(legacy.errorCovariance());

    TrackRepository repository;
    const auto id = repository.add(
        TrackFactory(std::make_shared<ConstantVelocityAlgorithm>()).createTentative(repository.nextId(), observation(1000_m, 0ms))
    );
    auto confirmedTrack = repository.findTentative(id)->confirm(observation(1010_m, 100ms));
    ASSERT_TRUE(confirmedTrack);
    ASSERT_TRUE(repository.replace(std::move(confirmedTrack).value()));
    auto& target = *repository.findTrack(id);
    EXPECT_EQ(target.state()->acceleration().acceleration(), 0_mps2);
    EXPECT_FALSE(target.state()->errorCovariance());
    ASSERT_TRUE(target.update(observation(1020_m, 200ms)));
    EXPECT_EQ(target.state()->acceleration().acceleration(), 0_mps2);
    EXPECT_FALSE(target.state()->errorCovariance());
}

TEST(TrackingStateTest, CopiesAccelerationAndCovarianceIndependentlyOfCallerLifetime)
{
    std::shared_ptr<const TrackState> stored;
    {
        const math::CartesianAcceleration acceleration(1_mps2, 2_mps2, 3_mps2);
        auto covariance = math::SquareMatrix::identity(TrackState::STATE_DIMENSION) * 25.0;
        covariance(0, 3) = 2.0;
        covariance(3, 0) = 2.0;
        stored = std::make_shared<const TrackState>(state(acceleration, covariance));
        covariance(0, 0) = 99.0;
    }
    const TrackState copied(*stored);
    stored.reset();
    EXPECT_EQ(copied.acceleration().x, 1_mps2);
    EXPECT_EQ(copied.acceleration().y, 2_mps2);
    EXPECT_EQ(copied.acceleration().z, 3_mps2);
    ASSERT_TRUE(copied.errorCovariance());
    EXPECT_EQ(copied.errorCovariance()->size(), 9u);
    EXPECT_EQ((*copied.errorCovariance())(0, 0), 25.0);
    EXPECT_EQ((*copied.errorCovariance())(0, 3), 2.0);
    EXPECT_EQ((*copied.errorCovariance())(3, 0), 2.0);
}

TEST(TrackingStateTest, RejectsNonfiniteAccelerationComponents)
{
    for (double invalid : {std::numeric_limits<double>::quiet_NaN(),
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

TEST(TrackingStateTest, RejectsCovarianceWithWrongDimension)
{
    for (std::size_t size : {0u, 3u, 8u, 10u})
    {
        SCOPED_TRACE(size);
        EXPECT_THROW(
            state(
                math::CartesianAcceleration(0_mps2, 0_mps2, 0_mps2),
                math::SquareMatrix(size)
            ),
            std::invalid_argument
        );
    }
}

TEST(TrackingStateTest, RejectsNonfiniteOrAsymmetricCovariance)
{
    for (double invalid : {std::numeric_limits<double>::quiet_NaN(),
             std::numeric_limits<double>::infinity(),
             -std::numeric_limits<double>::infinity()})
    {
        auto covariance = math::SquareMatrix::identity(TrackState::STATE_DIMENSION);
        covariance(0, 0) = invalid;
        EXPECT_THROW(
            state(math::CartesianAcceleration(0_mps2, 0_mps2, 0_mps2), covariance),
            std::invalid_argument
        );
    }
    auto asymmetric = math::SquareMatrix::identity(TrackState::STATE_DIMENSION);
    asymmetric(0, 1) = std::numeric_limits<double>::epsilon();
    EXPECT_THROW(
        state(math::CartesianAcceleration(0_mps2, 0_mps2, 0_mps2), asymmetric),
        std::invalid_argument
    );
}

TEST(TrackingStateTest, RejectsCovarianceThatIsNotPositiveDefinite)
{
    auto negative = math::SquareMatrix::identity(TrackState::STATE_DIMENSION);
    negative(0, 0) = -1.0;
    EXPECT_THROW(
        state(math::CartesianAcceleration(0_mps2, 0_mps2, 0_mps2), negative),
        std::invalid_argument
    );

    const math::SquareMatrix singular(TrackState::STATE_DIMENSION);
    EXPECT_THROW(
        state(math::CartesianAcceleration(0_mps2, 0_mps2, 0_mps2), singular),
        std::invalid_argument
    );
}

TEST(TrackingStateTest, StoresAlgorithmStateAcrossUpdatesAndPreservesItDuringMisses)
{
    const auto algorithm = std::make_shared<EnrichedAlgorithm>();
    TrackRepository repository;
    const auto id = repository.add(TrackFactory(algorithm).createTentative(repository.nextId(), observation(1000_m, 0ms)));
    auto confirmedTrack = repository.findTentative(id)->confirm(observation(1010_m, 100ms));
    ASSERT_TRUE(confirmedTrack);
    ASSERT_TRUE(repository.replace(std::move(confirmedTrack).value()));
    auto& target = *repository.findTrack(id);
    const auto initialSnapshot = target.state();
    EXPECT_EQ(initialSnapshot->acceleration().x, 1_mps2);
    EXPECT_EQ(initialSnapshot->acceleration().y, 2_mps2);
    EXPECT_EQ(initialSnapshot->acceleration().z, 3_mps2);
    ASSERT_TRUE(initialSnapshot->errorCovariance());
    EXPECT_EQ((*initialSnapshot->errorCovariance())(0, 0), 25.0);

    ASSERT_TRUE(target.update(observation(1020_m, 200ms)));
    const auto updatedSnapshot = target.state();
    EXPECT_NE(updatedSnapshot.get(), initialSnapshot.get());
    EXPECT_EQ(updatedSnapshot->acceleration().x, 4_mps2);
    EXPECT_EQ(updatedSnapshot->acceleration().y, 5_mps2);
    EXPECT_EQ(updatedSnapshot->acceleration().z, 6_mps2);
    ASSERT_TRUE(updatedSnapshot->errorCovariance());
    EXPECT_EQ((*updatedSnapshot->errorCovariance())(0, 0), 50.0);
    EXPECT_EQ(initialSnapshot->acceleration().x, 1_mps2);
    EXPECT_EQ((*initialSnapshot->errorCovariance())(0, 0), 25.0);

    ASSERT_TRUE(target.recordTrackingMiss(1200ms));
    ASSERT_TRUE(target.recordTrackingMiss(2200ms));
    EXPECT_EQ(target.state().get(), updatedSnapshot.get());
    EXPECT_EQ(target.lastObservationTime(), 200ms);
    EXPECT_EQ(target.lastResultTime(), 2200ms);
    EXPECT_EQ(target.consecutiveMissCount(), 2u);
}

TEST(TrackingStateTest, RejectedAlgorithmStatesPreserveAccelerationCovarianceAndMissHistory)
{
    const auto algorithm = std::make_shared<EnrichedAlgorithm>();
    TrackRepository repository;
    const auto id = repository.add(TrackFactory(algorithm).createTentative(repository.nextId(), observation(1000_m, 0ms)));
    auto confirmedTrack = repository.findTentative(id)->confirm(observation(1010_m, 100ms));
    ASSERT_TRUE(confirmedTrack);
    ASSERT_TRUE(repository.replace(std::move(confirmedTrack).value()));
    auto& target = *repository.findTrack(id);
    ASSERT_TRUE(target.recordTrackingMiss(1100ms));
    const auto saved = target.state();

    for (auto fault : {EnrichedAlgorithm::Fault::INVALID_COVARIANCE,
             EnrichedAlgorithm::Fault::WRONG_TIME})
    {
        algorithm->fault = fault;
        EXPECT_THROW(target.update(observation(1020_m, 2100ms)), std::invalid_argument);
        EXPECT_EQ(target.state().get(), saved.get());
        EXPECT_EQ(target.state()->acceleration().x, 1_mps2);
        EXPECT_EQ(target.state()->acceleration().y, 2_mps2);
        EXPECT_EQ(target.state()->acceleration().z, 3_mps2);
        EXPECT_EQ((*target.state()->errorCovariance())(0, 0), 25.0);
        EXPECT_EQ(target.lastObservationTime(), 100ms);
        EXPECT_EQ(target.lastResultTime(), 1100ms);
        EXPECT_EQ(target.consecutiveMissCount(), 1u);
    }
    algorithm->fault = EnrichedAlgorithm::Fault::NONE;
    ASSERT_TRUE(target.update(observation(1020_m, 2100ms)));
    EXPECT_EQ(target.state()->acceleration().x, 4_mps2);
    EXPECT_EQ(target.state()->acceleration().y, 5_mps2);
    EXPECT_EQ(target.state()->acceleration().z, 6_mps2);
    EXPECT_EQ((*target.state()->errorCovariance())(0, 0), 50.0);
    EXPECT_EQ(target.consecutiveMissCount(), 0u);
}

TEST(TrackingStateTest, RemovingTargetsPreservesRemainingAccelerationAndCovariance)
{
    const auto algorithm = std::make_shared<EnrichedAlgorithm>();
    TrackFactory factory(algorithm);
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
    auto confirmedFirstTrack = repository.findTentative(firstId)->confirm(observation(1001_m, 100ms));
    ASSERT_TRUE(confirmedFirstTrack);
    ASSERT_TRUE(repository.replace(std::move(confirmedFirstTrack).value()));
    auto confirmedMiddleTrack = repository.findTentative(middleId)->confirm(observation(2001_m, 100ms));
    ASSERT_TRUE(confirmedMiddleTrack);
    ASSERT_TRUE(repository.replace(std::move(confirmedMiddleTrack).value()));
    auto confirmedLastTrack = repository.findTentative(lastId)->confirm(observation(3001_m, 100ms));
    ASSERT_TRUE(confirmedLastTrack);
    ASSERT_TRUE(repository.replace(std::move(confirmedLastTrack).value()));
    ASSERT_TRUE(repository.findTrack(lastId)->update(observation(3002_m, 200ms)));
    const auto saved = repository.findTrack(lastId)->state();

    ASSERT_TRUE(repository.remove(middleId));
    ASSERT_EQ(repository.targets().size(), 2u);
    EXPECT_EQ(std::get<ConfirmedTrack>(repository.targets().front()).id(), firstId);
    EXPECT_EQ(std::get<ConfirmedTrack>(repository.targets().back()).id(), lastId);
    EXPECT_EQ(repository.findTrack(firstId)->state()->acceleration().x, 1_mps2);
    EXPECT_EQ(repository.findTrack(firstId)->state()->acceleration().y, 2_mps2);
    EXPECT_EQ(repository.findTrack(firstId)->state()->acceleration().z, 3_mps2);
    EXPECT_EQ((*repository.findTrack(firstId)->state()->errorCovariance())(0, 0), 25.0);
    EXPECT_EQ(repository.findTrack(lastId)->state().get(), saved.get());
    EXPECT_EQ(saved->acceleration().x, 4_mps2);
    EXPECT_EQ(saved->acceleration().y, 5_mps2);
    EXPECT_EQ(saved->acceleration().z, 6_mps2);
    EXPECT_EQ((*saved->errorCovariance())(0, 0), 50.0);
}

} // namespace
} // namespace target
