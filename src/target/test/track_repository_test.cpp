#include "algorithm/constant_velocity_algorithm.hpp"
#include "domain/factory/track_factory.hpp"
#include "repository/track_repository.hpp"

#include <gtest/gtest.h>

#include <chrono>
#include <cstdint>
#include <limits>
#include <memory>
#include <stdexcept>

namespace target
{
namespace
{

using namespace math::literals;
using namespace std::chrono_literals;

Observation observation()
{
    return Observation(math::SphericalPosition(1000_m, 0_deg, 0_deg), 0_mps, -10.0f, 0ms);
}

TEST(TrackRepositoryTest, AdvancesIdentityOnlyAfterSuccessfulRegistration)
{
    TrackRepository repository;
    const TrackFactory factory(std::make_shared<ConstantVelocityAlgorithm>());
    EXPECT_EQ(repository.nextId(), 1u);
    const auto tentative = factory.from(repository.nextId(), observation());
    EXPECT_EQ(repository.nextId(), 1u);

    EXPECT_EQ(repository.add(tentative), 1u);
    EXPECT_EQ(repository.nextId(), 2u);
    EXPECT_THROW(repository.add(tentative), std::invalid_argument);
    EXPECT_EQ(repository.nextId(), 2u);
    EXPECT_EQ(repository.targets().size(), 1u);
}

TEST(TrackRepositoryTest, ExhaustionDoesNotWrapOrReuseDeletedIds)
{
    TrackRepository repository;
    const TrackFactory factory(std::make_shared<ConstantVelocityAlgorithm>());
    const auto maximum = std::numeric_limits<std::uint16_t>::max();
    for (std::uint32_t id = 1; id <= maximum; ++id)
    {
        ASSERT_EQ(repository.nextId(), id);
        ASSERT_EQ(repository.add(factory.from(repository.nextId(), observation())), id);
        ASSERT_TRUE(repository.remove(id));
    }
    EXPECT_TRUE(repository.targets().empty());
    EXPECT_THROW(repository.nextId(), std::overflow_error);
    EXPECT_THROW(repository.nextId(), std::overflow_error);
}

TEST(TrackRepositoryTest, ConfirmationPreservesIdentityWithoutConsumingAnotherId)
{
    TrackRepository repository;
    const TrackFactory factory(std::make_shared<ConstantVelocityAlgorithm>());
    const auto tentative = factory.from(repository.nextId(), observation());
    repository.add(tentative);

    const Observation confirmation(math::SphericalPosition(1001_m, 0_deg, 0_deg), 0_mps, -10.0f, 100ms);
    const auto track = TrackFactory::from(tentative, confirmation);

    ASSERT_TRUE(track);
    EXPECT_EQ(track->id(), tentative.id());
    ASSERT_TRUE(repository.replace(*track));
    EXPECT_EQ(repository.nextId(), 2u);
}

TEST(TrackRepositoryTest, PreparedIdentitiesAdvanceTheSequenceWithoutMovingItBackward)
{
    TrackRepository repository;
    const TrackFactory factory(std::make_shared<ConstantVelocityAlgorithm>());
    EXPECT_EQ(repository.add(factory.from(42, observation())), 42u);
    EXPECT_EQ(repository.nextId(), 43u);

    const auto tentative = factory.from(7, observation());
    const Observation confirmation(math::SphericalPosition(1001_m, 0_deg, 0_deg), 0_mps, -10.0f, 100ms);
    const auto track = TrackFactory::from(tentative, confirmation);
    ASSERT_TRUE(track);
    EXPECT_EQ(repository.add(*track), 7u);
    EXPECT_EQ(repository.nextId(), 43u);
    ASSERT_TRUE(repository.remove(42));
    EXPECT_EQ(repository.add(factory.from(repository.nextId(), observation())), 43u);
}

TEST(TrackRepositoryTest, RejectsUnreportableIdentitiesWithoutConsumingAnId)
{
    TrackRepository repository;
    const TrackFactory factory(std::make_shared<ConstantVelocityAlgorithm>());
    const auto invalid = std::uint32_t{std::numeric_limits<std::uint16_t>::max()} + 1;
    const auto tentative = factory.from(invalid, observation());
    EXPECT_THROW(repository.add(tentative), std::overflow_error);
    EXPECT_EQ(repository.nextId(), 1u);

    const Observation confirmation(math::SphericalPosition(1001_m, 0_deg, 0_deg), 0_mps, -10.0f, 100ms);
    const auto track = TrackFactory::from(tentative, confirmation);
    ASSERT_TRUE(track);
    EXPECT_THROW(repository.add(*track), std::overflow_error);
    EXPECT_TRUE(repository.targets().empty());
    EXPECT_EQ(repository.nextId(), 1u);
    EXPECT_EQ(repository.add(factory.from(repository.nextId(), observation())), 1u);
}

} // namespace
} // namespace target
