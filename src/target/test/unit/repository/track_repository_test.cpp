#include "domain/tracking/constant_velocity_algorithm.hpp"
#include "domain/factory/track_factory.hpp"
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

TEST(TrackRepositoryTest, StoresFactoryAssignedIdentitiesAndRejectsDuplicates)
{
    TrackRepository repository;
    auto factory = TrackFactory(std::make_shared<ConstantVelocityAlgorithm>(), TrackLifecycleConfig(100ms, 1000ms, 3));
    const auto tentative = factory.createTentative(observation());
    EXPECT_EQ(tentative.id(), 1u);
    EXPECT_TRUE(repository.tracks().empty());

    EXPECT_EQ(repository.add(tentative), 1u);
    EXPECT_THROW(repository.add(tentative), std::invalid_argument);
    EXPECT_EQ(repository.tracks().size(), 1u);
    EXPECT_EQ(repository.add(factory.createTentative(observation())), 2u);
}

TEST(TrackRepositoryTest, ConfirmationPreservesIdentityWithoutConsumingAnotherId)
{
    TrackRepository repository;
    auto factory = TrackFactory(std::make_shared<ConstantVelocityAlgorithm>(), TrackLifecycleConfig(100ms, 1000ms, 3));
    const auto tentative = factory.createTentative(observation());
    repository.add(tentative);

    const auto confirmation = Observation(math::SphericalPosition(1001_m, 0_deg, 0_deg), 0_mps, -10.0f, 100ms);
    const auto track = tentative.confirm(confirmation);

    ASSERT_TRUE(track);
    EXPECT_EQ(track->id(), tentative.id());
    ASSERT_TRUE(repository.replace(*track));
    EXPECT_EQ(factory.createTentative(observation()).id(), 2u);
}

TEST(TrackRepositoryTest, QueriesTrackTypesInRegistrationOrderWithIndependentLimits)
{
    TrackRepository repository;
    const auto algorithm = std::make_shared<ConstantVelocityAlgorithm>();
    const auto earlier = TentativeTrack(7, observation(), TrackLifecycleConfig(100ms, 1000ms, 3), algorithm);
    const auto later = TentativeTrack(3, observation(), TrackLifecycleConfig(100ms, 1000ms, 3), algorithm);
    repository.add(TentativeTrack(42, observation(), TrackLifecycleConfig(100ms, 1000ms, 3), algorithm));
    repository.add(earlier);
    repository.add(TentativeTrack(20, observation(), TrackLifecycleConfig(100ms, 1000ms, 3), algorithm));
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

TEST(TrackRepositoryTest, StoresPreparedIdentitiesWithoutChangingThem)
{
    TrackRepository repository;
    const auto algorithm = std::make_shared<ConstantVelocityAlgorithm>();
    EXPECT_EQ(repository.add(TentativeTrack(42, observation(), TrackLifecycleConfig(100ms, 1000ms, 3), algorithm)), 42u);

    const auto tentative = TentativeTrack(7, observation(), TrackLifecycleConfig(100ms, 1000ms, 3), algorithm);
    const auto confirmation = Observation(math::SphericalPosition(1001_m, 0_deg, 0_deg), 0_mps, -10.0f, 100ms);
    const auto track = tentative.confirm(confirmation);
    ASSERT_TRUE(track);
    EXPECT_EQ(repository.add(*track), 7u);
    EXPECT_EQ(repository.findConfirmedTrack(7)->id(), 7u);
    ASSERT_TRUE(repository.removeById(42));
    EXPECT_EQ(repository.findTentativeTrack(42), nullptr);
}

TEST(TrackRepositoryTest, RejectsUnreportableIdentitiesWithoutStoringTracks)
{
    TrackRepository repository;
    auto factory = TrackFactory(std::make_shared<ConstantVelocityAlgorithm>(), TrackLifecycleConfig(100ms, 1000ms, 3));
    const auto invalid = std::uint32_t{std::numeric_limits<std::uint16_t>::max()} + 1;
    const auto tentative = TentativeTrack(invalid, observation(), TrackLifecycleConfig(100ms, 1000ms, 3), std::make_shared<ConstantVelocityAlgorithm>());
    EXPECT_THROW(repository.add(tentative), std::overflow_error);
    EXPECT_TRUE(repository.tracks().empty());

    const auto confirmation = Observation(math::SphericalPosition(1001_m, 0_deg, 0_deg), 0_mps, -10.0f, 100ms);
    const auto track = tentative.confirm(confirmation);
    ASSERT_TRUE(track);
    EXPECT_THROW(repository.add(*track), std::overflow_error);
    EXPECT_TRUE(repository.tracks().empty());
    EXPECT_EQ(repository.add(factory.createTentative(observation())), 1u);
}

// 운용 OFF 시 모든 트랙을 지우되, 늦게 도착한 결과가 새 트랙에 붙지 않도록 ID는 재사용하지 않는다.
TEST(TrackRepositoryTest, ClearRemovesAllTrackTypesWithoutReusingIds)
{
    TrackRepository repository;
    auto factory = TrackFactory(std::make_shared<ConstantVelocityAlgorithm>(), TrackLifecycleConfig(100ms, 1000ms, 3));
    const auto tentative = factory.createTentative(observation());
    repository.add(tentative);
    repository.add(factory.createTentative(observation()));
    const auto confirmation = Observation(math::SphericalPosition(1001_m, 0_deg, 0_deg), 0_mps, -10.0f, 100ms);
    const auto confirmed = tentative.confirm(confirmation);
    ASSERT_TRUE(confirmed);
    ASSERT_TRUE(repository.replace(*confirmed));

    EXPECT_EQ(repository.clear(), 2u);

    EXPECT_TRUE(repository.tracks().empty());
    EXPECT_TRUE(repository.confirmedTracks().empty());
    EXPECT_TRUE(repository.tentativeTracks().empty());
    EXPECT_EQ(repository.findConfirmedTrack(1), nullptr);
    EXPECT_EQ(repository.findTentativeTrack(2), nullptr);
    EXPECT_EQ(repository.add(factory.createTentative(observation())), 3u);
}

TEST(TrackRepositoryTest, ClearIsRepeatableOnEmptyRepository)
{
    TrackRepository repository;
    auto factory = TrackFactory(std::make_shared<ConstantVelocityAlgorithm>(), TrackLifecycleConfig(100ms, 1000ms, 3));
    repository.add(factory.createTentative(observation()));

    EXPECT_EQ(repository.clear(), 1u);
    EXPECT_EQ(repository.clear(), 0u);
    EXPECT_TRUE(repository.tracks().empty());
    EXPECT_EQ(repository.add(factory.createTentative(observation())), 2u);
}

} // namespace
} // namespace target
