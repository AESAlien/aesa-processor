#include "domain/tracking/constant_velocity_algorithm.hpp"
#include "domain/track_factory.hpp"
#include "repository/track_repository.hpp"

#include <gtest/gtest.h>

#include <chrono>
#include <cstdint>
#include <limits>
#include <memory>
#include <stdexcept>
#include <vector>

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
    const auto factory = TrackFactory(std::make_shared<ConstantVelocityAlgorithm>());
    EXPECT_EQ(repository.nextId(), 1u);
    const auto tentative = factory.createTentative(repository.nextId(), observation());
    EXPECT_EQ(repository.nextId(), 1u);

    EXPECT_EQ(repository.add(tentative), 1u);
    EXPECT_EQ(repository.nextId(), 2u);
    EXPECT_THROW(repository.add(tentative), std::invalid_argument);
    EXPECT_EQ(repository.nextId(), 2u);
    EXPECT_EQ(repository.tracks().size(), 1u);
}

TEST(TrackRepositoryTest, ExhaustionDoesNotWrapOrReuseDeletedIds)
{
    TrackRepository repository;
    const auto factory = TrackFactory(std::make_shared<ConstantVelocityAlgorithm>());
    const auto maximum = std::numeric_limits<std::uint16_t>::max();
    for (std::uint32_t id = 1; id <= maximum; ++id)
    {
        ASSERT_EQ(repository.nextId(), id);
        ASSERT_EQ(repository.add(factory.createTentative(repository.nextId(), observation())), id);
        ASSERT_TRUE(repository.remove(id));
    }
    EXPECT_TRUE(repository.tracks().empty());
    EXPECT_THROW(repository.nextId(), std::overflow_error);
    EXPECT_THROW(repository.nextId(), std::overflow_error);
}

TEST(TrackRepositoryTest, ConfirmationPreservesIdentityWithoutConsumingAnotherId)
{
    TrackRepository repository;
    const auto factory = TrackFactory(std::make_shared<ConstantVelocityAlgorithm>());
    const auto tentative = factory.createTentative(repository.nextId(), observation());
    repository.add(tentative);

    const auto confirmation = Observation(math::SphericalPosition(1001_m, 0_deg, 0_deg), 0_mps, -10.0f, 100ms);
    const auto track = tentative.confirm(confirmation);

    ASSERT_TRUE(track);
    EXPECT_EQ(track->id(), tentative.id());
    ASSERT_TRUE(repository.replace(*track));
    EXPECT_EQ(repository.nextId(), 2u);
}

TEST(TrackRepositoryTest, QueriesTrackTypesInRegistrationOrderWithIndependentLimits)
{
    TrackRepository repository;
    const auto factory = TrackFactory(std::make_shared<ConstantVelocityAlgorithm>());
    const auto earlier = factory.createTentative(7, observation());
    const auto later = factory.createTentative(3, observation());
    repository.add(factory.createTentative(42, observation()));
    repository.add(earlier);
    repository.add(factory.createTentative(20, observation()));
    repository.add(later);

    const auto confirmation = Observation(math::SphericalPosition(1001_m, 0_deg, 0_deg), 0_mps, -10.0f, 100ms);
    const auto laterTrack = later.confirm(confirmation);
    const auto earlierTrack = earlier.confirm(confirmation);
    ASSERT_TRUE(laterTrack);
    ASSERT_TRUE(earlierTrack);
    ASSERT_TRUE(repository.replace(*laterTrack));
    ASSERT_TRUE(repository.replace(*earlierTrack));

    const auto confirmed = std::vector<const ConfirmedTrack*>{repository.findConfirmedTrack(7), repository.findConfirmedTrack(3)};
    const auto tentative = std::vector<const TentativeTrack*>{repository.findTentativeTrack(42), repository.findTentativeTrack(20)};
    EXPECT_EQ(repository.confirmedTracks(), confirmed);
    EXPECT_EQ(repository.tentativeTracks(), tentative);
    EXPECT_EQ(repository.confirmedTracks(1), (std::vector<const ConfirmedTrack*>{confirmed.front()}));
    EXPECT_EQ(repository.tentativeTracks(1), (std::vector<const TentativeTrack*>{tentative.front()}));
    EXPECT_TRUE(repository.confirmedTracks(0).empty());
    EXPECT_TRUE(repository.tentativeTracks(0).empty());
}

TEST(TrackRepositoryTest, PreparedIdentitiesAdvanceTheSequenceWithoutMovingItBackward)
{
    TrackRepository repository;
    const auto factory = TrackFactory(std::make_shared<ConstantVelocityAlgorithm>());
    EXPECT_EQ(repository.add(factory.createTentative(42, observation())), 42u);
    EXPECT_EQ(repository.nextId(), 43u);

    const auto tentative = factory.createTentative(7, observation());
    const auto confirmation = Observation(math::SphericalPosition(1001_m, 0_deg, 0_deg), 0_mps, -10.0f, 100ms);
    const auto track = tentative.confirm(confirmation);
    ASSERT_TRUE(track);
    EXPECT_EQ(repository.add(*track), 7u);
    EXPECT_EQ(repository.nextId(), 43u);
    ASSERT_TRUE(repository.remove(42));
    EXPECT_EQ(repository.add(factory.createTentative(repository.nextId(), observation())), 43u);
}

TEST(TrackRepositoryTest, RejectsUnreportableIdentitiesWithoutConsumingAnId)
{
    TrackRepository repository;
    const auto factory = TrackFactory(std::make_shared<ConstantVelocityAlgorithm>());
    const auto invalid = std::uint32_t{std::numeric_limits<std::uint16_t>::max()} + 1;
    const auto tentative = factory.createTentative(invalid, observation());
    EXPECT_THROW(repository.add(tentative), std::overflow_error);
    EXPECT_EQ(repository.nextId(), 1u);

    const auto confirmation = Observation(math::SphericalPosition(1001_m, 0_deg, 0_deg), 0_mps, -10.0f, 100ms);
    const auto track = tentative.confirm(confirmation);
    ASSERT_TRUE(track);
    EXPECT_THROW(repository.add(*track), std::overflow_error);
    EXPECT_TRUE(repository.tracks().empty());
    EXPECT_EQ(repository.nextId(), 1u);
    EXPECT_EQ(repository.add(factory.createTentative(repository.nextId(), observation())), 1u);
}

} // namespace
} // namespace target
