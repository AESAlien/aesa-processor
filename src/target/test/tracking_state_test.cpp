#include "calculation/tracking/constant_velocity_algorithm.hpp"
#include "domain/tracking_state.hpp"
#include "repository/target_repository.hpp"

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

static_assert(std::is_const_v<decltype(TargetTrackingState::acceleration)>);
static_assert(std::is_const_v<decltype(TargetTrackingState::errorCovariance)>);
static_assert(!std::is_copy_assignable_v<TargetTrackingState>);
static_assert(std::is_nothrow_move_constructible_v<Target>);

math::SphericalPosition position(math::Length range = 1000_m)
{
    return math::SphericalPosition(range, math::SphericalDirection(math::Azimuth(0_deg), math::Elevation(0_deg)));
}

TargetTrackingState state(
    math::CartesianAcceleration acceleration = math::CartesianAcceleration(0_mps2, 0_mps2, 0_mps2),
    std::optional<math::SquareMatrix> covariance = std::nullopt)
{
    return TargetTrackingState(position(), 0_mps, -10.0f, 100ms,
        math::CartesianPosition(0_m, 1000_m, 0_m),
        math::CartesianVelocity(0_mps, 0_mps, 0_mps),
        std::move(acceleration), std::move(covariance));
}

Target observation(
    math::Length range,
    Time time,
    std::shared_ptr<const ITrackingAlgorithm> algorithm = std::make_shared<ConstantVelocityAlgorithm>())
{
    return Target(position(range), 0_mps, -10.0f, time, std::move(algorithm));
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

    std::optional<TargetTrackingState> initialize(const Target& first, const Target& second) const override
    {
        return enrich(_algorithm.initialize(first, second), false);
    }

    std::optional<TargetTrackingState> update(const Target& current, const Target& incoming) const override
    {
        return enrich(_algorithm.update(current, incoming), true);
    }

    std::optional<TargetPrediction> predict(const Target& current, Time time) const override
    {
        return _algorithm.predict(current, time);
    }

    Fault fault = Fault::NONE;

private:
    std::optional<TargetTrackingState> enrich(
        const std::optional<TargetTrackingState>& calculated,
        bool updating) const
    {
        if (!calculated)
        {
            return std::nullopt;
        }
        const auto acceleration = updating
            ? math::CartesianAcceleration(4_mps2, 5_mps2, 6_mps2)
            : math::CartesianAcceleration(1_mps2, 2_mps2, 3_mps2);
        auto covariance = math::SquareMatrix::identity(TargetTrackingState::STATE_DIMENSION)
            * (updating ? 50.0 : 25.0);
        if (fault == Fault::INVALID_COVARIANCE)
        {
            covariance(0, 0) = -1.0;
        }
        const auto timestamp = calculated->timestamp + (fault == Fault::WRONG_TIME ? 1ms : 0ms);
        return TargetTrackingState(calculated->sphericalPosition, calculated->dopplerVelocity,
            calculated->power_db, timestamp, calculated->position, calculated->velocity,
            acceleration, std::move(covariance));
    }

    ConstantVelocityAlgorithm _algorithm;
};

TEST(TrackingStateTest, ExistingConstructorAndConstantVelocityLeaveCovarianceUnavailable)
{
    const TargetTrackingState legacy(position(), 0_mps, -10.0f, 100ms,
        math::CartesianPosition(0_m, 1000_m, 0_m),
        math::CartesianVelocity(0_mps, 0_mps, 0_mps));
    EXPECT_EQ(legacy.acceleration.acceleration(), 0_mps2);
    EXPECT_FALSE(legacy.errorCovariance);

    TargetRepository repository;
    const auto id = repository.add(observation(1000_m, 0ms));
    auto& target = *repository.find(id);
    ASSERT_TRUE(target.confirm(observation(1010_m, 100ms)));
    EXPECT_EQ(target.trackingState()->acceleration.acceleration(), 0_mps2);
    EXPECT_FALSE(target.trackingState()->errorCovariance);
    ASSERT_TRUE(target.update(observation(1020_m, 200ms)));
    EXPECT_EQ(target.trackingState()->acceleration.acceleration(), 0_mps2);
    EXPECT_FALSE(target.trackingState()->errorCovariance);
}

TEST(TrackingStateTest, CopiesAccelerationAndCovarianceIndependentlyOfCallerLifetime)
{
    std::shared_ptr<const TargetTrackingState> stored;
    {
        const math::CartesianAcceleration acceleration(1_mps2, 2_mps2, 3_mps2);
        auto covariance = math::SquareMatrix::identity(TargetTrackingState::STATE_DIMENSION) * 25.0;
        covariance(0, 3) = 2.0;
        covariance(3, 0) = 2.0;
        stored = std::make_shared<const TargetTrackingState>(state(acceleration, covariance));
        covariance(0, 0) = 99.0;
    }
    const TargetTrackingState copied(*stored);
    stored.reset();
    EXPECT_EQ(copied.acceleration.x, 1_mps2);
    EXPECT_EQ(copied.acceleration.y, 2_mps2);
    EXPECT_EQ(copied.acceleration.z, 3_mps2);
    ASSERT_TRUE(copied.errorCovariance);
    EXPECT_EQ(copied.errorCovariance->size(), 9u);
    EXPECT_EQ((*copied.errorCovariance)(0, 0), 25.0);
    EXPECT_EQ((*copied.errorCovariance)(0, 3), 2.0);
    EXPECT_EQ((*copied.errorCovariance)(3, 0), 2.0);
}

TEST(TrackingStateTest, RejectsNonfiniteAccelerationComponents)
{
    for (double invalid : {std::numeric_limits<double>::quiet_NaN(),
             std::numeric_limits<double>::infinity(), -std::numeric_limits<double>::infinity()})
    {
        EXPECT_THROW(math::CartesianAcceleration(
            0_mps2,
            math::LinearAcceleration::fromMetersPerSecondSquared(invalid),
            0_mps2), std::invalid_argument);
    }
}

TEST(TrackingStateTest, RejectsCovarianceWithWrongDimension)
{
    for (std::size_t size : {0u, 3u, 8u, 10u})
    {
        SCOPED_TRACE(size);
        EXPECT_THROW(state(math::CartesianAcceleration(0_mps2, 0_mps2, 0_mps2),
            math::SquareMatrix(size)), std::invalid_argument);
    }
}

TEST(TrackingStateTest, RejectsNonfiniteOrAsymmetricCovariance)
{
    for (double invalid : {std::numeric_limits<double>::quiet_NaN(),
             std::numeric_limits<double>::infinity(), -std::numeric_limits<double>::infinity()})
    {
        auto covariance = math::SquareMatrix::identity(TargetTrackingState::STATE_DIMENSION);
        covariance(0, 0) = invalid;
        EXPECT_THROW(state(math::CartesianAcceleration(0_mps2, 0_mps2, 0_mps2), covariance),
            std::invalid_argument);
    }
    auto asymmetric = math::SquareMatrix::identity(TargetTrackingState::STATE_DIMENSION);
    asymmetric(0, 1) = std::numeric_limits<double>::epsilon();
    EXPECT_THROW(state(math::CartesianAcceleration(0_mps2, 0_mps2, 0_mps2), asymmetric),
        std::invalid_argument);
}

TEST(TrackingStateTest, RejectsNegativeDiagonalButAllowsExplicitSingularZeroCovariance)
{
    auto negative = math::SquareMatrix::identity(TargetTrackingState::STATE_DIMENSION);
    negative(0, 0) = -1.0;
    EXPECT_THROW(state(math::CartesianAcceleration(0_mps2, 0_mps2, 0_mps2), negative),
        std::invalid_argument);

    const auto explicitZero = state(math::CartesianAcceleration(0_mps2, 0_mps2, 0_mps2),
        math::SquareMatrix(TargetTrackingState::STATE_DIMENSION));
    ASSERT_TRUE(explicitZero.errorCovariance);
    EXPECT_EQ(*explicitZero.errorCovariance, math::SquareMatrix(9));
    // These are structural checks; they do not establish positive semidefiniteness for arbitrary matrices.
}

TEST(TrackingStateTest, StoresAlgorithmStateAcrossUpdatesAndPreservesItDuringMisses)
{
    const auto algorithm = std::make_shared<EnrichedAlgorithm>();
    TargetRepository repository;
    const auto id = repository.add(observation(1000_m, 0ms, algorithm));
    auto& target = *repository.find(id);
    ASSERT_TRUE(target.confirm(observation(1010_m, 100ms)));
    const auto initialSnapshot = target.trackingState();
    EXPECT_EQ(initialSnapshot->acceleration.x, 1_mps2);
    EXPECT_EQ(initialSnapshot->acceleration.y, 2_mps2);
    EXPECT_EQ(initialSnapshot->acceleration.z, 3_mps2);
    ASSERT_TRUE(initialSnapshot->errorCovariance);
    EXPECT_EQ((*initialSnapshot->errorCovariance)(0, 0), 25.0);

    ASSERT_TRUE(target.update(observation(1020_m, 200ms)));
    const auto updatedSnapshot = target.trackingState();
    EXPECT_NE(updatedSnapshot.get(), initialSnapshot.get());
    EXPECT_EQ(updatedSnapshot->acceleration.x, 4_mps2);
    EXPECT_EQ(updatedSnapshot->acceleration.y, 5_mps2);
    EXPECT_EQ(updatedSnapshot->acceleration.z, 6_mps2);
    ASSERT_TRUE(updatedSnapshot->errorCovariance);
    EXPECT_EQ((*updatedSnapshot->errorCovariance)(0, 0), 50.0);
    EXPECT_EQ(initialSnapshot->acceleration.x, 1_mps2);
    EXPECT_EQ((*initialSnapshot->errorCovariance)(0, 0), 25.0);

    ASSERT_TRUE(target.recordTrackingMiss(1200ms));
    ASSERT_TRUE(target.recordTrackingMiss(2200ms));
    EXPECT_EQ(target.trackingState().get(), updatedSnapshot.get());
    EXPECT_EQ(target.lastObservationTime(), 200ms);
    EXPECT_EQ(target.lastResultTime(), 2200ms);
    EXPECT_EQ(target.consecutiveMissCount(), 2u);
}

TEST(TrackingStateTest, RejectedAlgorithmStatesPreserveAccelerationCovarianceAndMissHistory)
{
    const auto algorithm = std::make_shared<EnrichedAlgorithm>();
    TargetRepository repository;
    const auto id = repository.add(observation(1000_m, 0ms, algorithm));
    auto& target = *repository.find(id);
    ASSERT_TRUE(target.confirm(observation(1010_m, 100ms)));
    ASSERT_TRUE(target.recordTrackingMiss(1100ms));
    const auto saved = target.trackingState();

    for (auto fault : {EnrichedAlgorithm::Fault::INVALID_COVARIANCE,
             EnrichedAlgorithm::Fault::WRONG_TIME})
    {
        algorithm->fault = fault;
        EXPECT_THROW(target.update(observation(1020_m, 2100ms)), std::invalid_argument);
        EXPECT_EQ(target.trackingState().get(), saved.get());
        EXPECT_EQ(target.trackingState()->acceleration.x, 1_mps2);
        EXPECT_EQ(target.trackingState()->acceleration.y, 2_mps2);
        EXPECT_EQ(target.trackingState()->acceleration.z, 3_mps2);
        EXPECT_EQ((*target.trackingState()->errorCovariance)(0, 0), 25.0);
        EXPECT_EQ(target.lastObservationTime(), 100ms);
        EXPECT_EQ(target.lastResultTime(), 1100ms);
        EXPECT_EQ(target.consecutiveMissCount(), 1u);
    }
    algorithm->fault = EnrichedAlgorithm::Fault::NONE;
    ASSERT_TRUE(target.update(observation(1020_m, 2100ms)));
    EXPECT_EQ(target.trackingState()->acceleration.x, 4_mps2);
    EXPECT_EQ(target.trackingState()->acceleration.y, 5_mps2);
    EXPECT_EQ(target.trackingState()->acceleration.z, 6_mps2);
    EXPECT_EQ((*target.trackingState()->errorCovariance)(0, 0), 50.0);
    EXPECT_EQ(target.consecutiveMissCount(), 0u);
}

TEST(TrackingStateTest, RemovingTargetsPreservesRemainingAccelerationAndCovariance)
{
    const auto algorithm = std::make_shared<EnrichedAlgorithm>();
    TargetRepository repository;
    const auto firstId = repository.add(observation(1000_m, 0ms, algorithm));
    const auto middleId = repository.add(observation(2000_m, 0ms, algorithm));
    const auto lastId = repository.add(observation(3000_m, 0ms, algorithm));
    ASSERT_TRUE(repository.find(firstId)->confirm(observation(1001_m, 100ms)));
    ASSERT_TRUE(repository.find(middleId)->confirm(observation(2001_m, 100ms)));
    ASSERT_TRUE(repository.find(lastId)->confirm(observation(3001_m, 100ms)));
    ASSERT_TRUE(repository.find(lastId)->update(observation(3002_m, 200ms)));
    const auto saved = repository.find(lastId)->trackingState();

    ASSERT_TRUE(repository.remove(middleId));
    ASSERT_EQ(repository.targets().size(), 2u);
    EXPECT_EQ(repository.targets()[0].id(), firstId);
    EXPECT_EQ(repository.targets()[1].id(), lastId);
    EXPECT_EQ(repository.find(firstId)->trackingState()->acceleration.x, 1_mps2);
    EXPECT_EQ(repository.find(firstId)->trackingState()->acceleration.y, 2_mps2);
    EXPECT_EQ(repository.find(firstId)->trackingState()->acceleration.z, 3_mps2);
    EXPECT_EQ((*repository.find(firstId)->trackingState()->errorCovariance)(0, 0), 25.0);
    EXPECT_EQ(repository.find(lastId)->trackingState().get(), saved.get());
    EXPECT_EQ(saved->acceleration.x, 4_mps2);
    EXPECT_EQ(saved->acceleration.y, 5_mps2);
    EXPECT_EQ(saved->acceleration.z, 6_mps2);
    EXPECT_EQ((*saved->errorCovariance)(0, 0), 50.0);
}

} // namespace
} // namespace target
