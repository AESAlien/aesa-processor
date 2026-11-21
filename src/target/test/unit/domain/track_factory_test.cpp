#include "domain/track_factory.hpp"
#include "domain/tracking/constant_velocity_algorithm.hpp"

#include <gtest/gtest.h>

#include <chrono>
#include <cstdint>
#include <limits>
#include <memory>
#include <stdexcept>
#include <type_traits>
#include <utility>

namespace target
{
namespace
{

using namespace math::literals;
using namespace std::chrono_literals;

static_assert(!std::is_copy_constructible_v<TrackFactory>);
static_assert(std::is_nothrow_move_constructible_v<TrackFactory>);

Observation observation(std::chrono::milliseconds time = 0ms)
{
    return Observation(math::SphericalPosition(1000_m, 0_deg, 0_deg), 0_mps, -10.0f, time);
}

TEST(TrackFactoryTest, AssignsIdsAtCreationWithoutReusingDiscardedTracks)
{
    // 준비: 저장소 없이 팩터리만 생성한다.
    TrackFactory factory(std::make_shared<ConstantVelocityAlgorithm>(), TrackLifecycleConfig(100ms, 1000ms, 3));

    // 실행 및 검증: 생성 순간 ID를 결정하고, 저장하지 않은 트랙의 ID도 재사용하지 않는다.
    {
        const auto first = factory.createTentative(observation());
        EXPECT_EQ(first.id(), 1u);
    }
    const auto second = factory.createTentative(observation());
    const auto third = factory.createTentative(observation());
    EXPECT_EQ(second.id(), 2u);
    EXPECT_EQ(third.id(), 3u);
}

TEST(TrackFactoryTest, MovingFactoryPreservesIssuedIdentitySequence)
{
    // 준비: ID 하나를 발급한 팩터리를 이동한다.
    TrackFactory factory(std::make_shared<ConstantVelocityAlgorithm>(), TrackLifecycleConfig(100ms, 1000ms, 3));
    ASSERT_EQ(factory.createTentative(observation()).id(), 1u);
    auto movedFactory = std::move(factory);

    // 실행 및 검증: 이동 후에도 다음 순번을 이어서 발급한다.
    EXPECT_EQ(movedFactory.createTentative(observation()).id(), 2u);
}

TEST(TrackFactoryTest, ConfirmationPreservesIdentityWithoutIssuingAnotherId)
{
    // 준비: 팩터리가 생성한 잠정 트랙을 확정한다.
    TrackFactory factory(std::make_shared<ConstantVelocityAlgorithm>(), TrackLifecycleConfig(100ms, 1000ms, 3));
    const auto tentative = factory.createTentative(observation());
    const auto confirmed = tentative.confirm(observation(100ms));
    ASSERT_TRUE(confirmed);

    // 실행 및 검증: 확정은 기존 ID를 유지하고 새 생성만 다음 ID를 발급한다.
    EXPECT_EQ(confirmed->id(), tentative.id());
    EXPECT_EQ(factory.createTentative(observation()).id(), 2u);
}

TEST(TrackFactoryTest, ConfirmationRetainsTheFactoryMissLimit)
{
    // 준비: 첫 미탐에 만료하도록 설정한 팩터리에서 트랙을 생성한다.
    TrackFactory factory(std::make_shared<ConstantVelocityAlgorithm>(), TrackLifecycleConfig(100ms, 1000ms, 1));
    const auto tentative = factory.createTentative(observation());
    auto confirmed = tentative.confirm(observation(100ms));
    ASSERT_TRUE(confirmed);

    // 실행 및 검증: 확정 전환 후에도 생성 시 지정한 미탐 한도를 유지한다.
    EXPECT_TRUE(confirmed->willExpireOnNextMiss());
    EXPECT_FALSE(confirmed->hasReachedMissLimit());
    ASSERT_TRUE(confirmed->recordTrackingMiss(1100ms));
    EXPECT_TRUE(confirmed->hasReachedMissLimit());
}

TEST(TrackFactoryTest, ExhaustionDoesNotWrapOrReuseIssuedIds)
{
    // 준비: 전시기 프로토콜이 허용하는 최대 ID까지 발급한다.
    TrackFactory factory(std::make_shared<ConstantVelocityAlgorithm>(), TrackLifecycleConfig(100ms, 1000ms, 3));
    const auto maximum = std::numeric_limits<std::uint16_t>::max();
    for (std::uint32_t id = 1; id <= maximum; ++id)
    {
        ASSERT_EQ(factory.createTentative(observation()).id(), id);
    }

    // 실행 및 검증: 한도를 넘으면 반복 호출에도 순환하거나 기존 ID를 재사용하지 않는다.
    EXPECT_THROW(factory.createTentative(observation()), std::overflow_error);
    EXPECT_THROW(factory.createTentative(observation()), std::overflow_error);
}

} // namespace
} // namespace target
