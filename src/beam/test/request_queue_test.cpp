#include "scheduler/request_queue.hpp"
#include "scheduler/beam_request_factory.hpp"

#include <gtest/gtest.h>

#include <chrono>

using namespace std::chrono_literals;
using namespace math::literals;

namespace
{

beam::BeamRequest makeRequest(
    beam::RequestBeamType beamType,
    std::chrono::milliseconds transmitTime,
    math::Angle azimuth = 0_deg
) {
    static const beam::BeamRequestFactory factory{
        beam::RequestBeamConfig(
            beam::BeamWidth(math::AngularWidth(3_deg), math::AngularWidth(3_deg)),
            beam::BeamWidth(math::AngularWidth(3_deg), math::AngularWidth(3_deg)))};
    return factory.create(beam::RequestBeamCommand(
        beamType, transmitTime, 1, math::SphericalDirection(math::CircularAngle(azimuth), math::TiltAngle(0_deg))));
}

beam::BeamRequest makeConfirmation(std::chrono::milliseconds transmitTime, math::Angle azimuth = 0_deg)
{
    return makeRequest(beam::RequestBeamType::CONFIRMATION, transmitTime, azimuth);
}

beam::BeamRequest makeTracking(std::chrono::milliseconds transmitTime, math::Angle azimuth = 0_deg)
{
    return makeRequest(beam::RequestBeamType::TRACKING, transmitTime, azimuth);
}

} // namespace

TEST(RequestQueueTest, EmptyQueueReturnsNothing)
{
    beam::RequestQueue queue;

    EXPECT_FALSE(queue.popNextDue(1000ms).has_value());
}

TEST(RequestQueueTest, FutureRequestIsNotPoppedAndStaysQueued)
{
    beam::RequestQueue queue;
    queue.add(makeTracking(1010ms));

    EXPECT_FALSE(queue.popNextDue(1000ms).has_value());

    // 시각이 되면 그대로 남아 있던 요청이 꺼내진다.
    const auto request = queue.popNextDue(1010ms);
    ASSERT_TRUE(request.has_value());
    EXPECT_EQ(request->transmitTime(), 1010ms);
}

TEST(RequestQueueTest, RequestExactlyAtCurrentTimeIsDue)
{
    beam::RequestQueue queue;
    queue.add(makeTracking(1000ms));

    const auto request = queue.popNextDue(1000ms);

    ASSERT_TRUE(request.has_value());
    EXPECT_EQ(request->transmitTime(), 1000ms);
}

TEST(RequestQueueTest, PoppedRequestIsRemovedFromQueue)
{
    beam::RequestQueue queue;
    queue.add(makeTracking(1000ms));

    ASSERT_TRUE(queue.popNextDue(1000ms).has_value());
    EXPECT_FALSE(queue.popNextDue(1000ms).has_value());
}

TEST(RequestQueueTest, EarliestDueRequestComesFirstRegardlessOfInsertionOrder)
{
    beam::RequestQueue queue;
    queue.add(makeTracking(1010ms, 3_deg));
    queue.add(makeTracking(1000ms, 1_deg));
    queue.add(makeTracking(1005ms, 2_deg));

    EXPECT_TRUE(queue.popNextDue(1010ms)->direction_enu().azimuth.isEquivalentTo(1_deg));
    EXPECT_TRUE(queue.popNextDue(1010ms)->direction_enu().azimuth.isEquivalentTo(2_deg));
    EXPECT_TRUE(queue.popNextDue(1010ms)->direction_enu().azimuth.isEquivalentTo(3_deg));
}

TEST(RequestQueueTest, SameTransmitTimeConfirmationComesBeforeTracking)
{
    beam::RequestQueue queue;
    queue.add(makeTracking(1000ms, 1_deg));
    queue.add(makeConfirmation(1000ms, 2_deg));

    const auto first = queue.popNextDue(1000ms);
    const auto second = queue.popNextDue(1010ms);

    ASSERT_TRUE(first.has_value());
    ASSERT_TRUE(second.has_value());
    EXPECT_EQ(first->transmitBeamType(), beam::BeamType::CONFIRMATION);
    EXPECT_EQ(second->transmitBeamType(), beam::BeamType::TRACKING);
}

TEST(RequestQueueTest, SameTransmitTimeAndTypeKeepsInsertionOrder)
{
    beam::RequestQueue queue;
    queue.add(makeTracking(1000ms, 1_deg));
    queue.add(makeTracking(1000ms, 2_deg));
    queue.add(makeTracking(1000ms, 3_deg));

    EXPECT_TRUE(queue.popNextDue(1000ms)->direction_enu().azimuth.isEquivalentTo(1_deg));
    EXPECT_TRUE(queue.popNextDue(1000ms)->direction_enu().azimuth.isEquivalentTo(2_deg));
    EXPECT_TRUE(queue.popNextDue(1000ms)->direction_enu().azimuth.isEquivalentTo(3_deg));
}

TEST(RequestQueueTest, EarlierTrackingStillComesBeforeLaterConfirmation)
{
    beam::RequestQueue queue;
    queue.add(makeConfirmation(1010ms));
    queue.add(makeTracking(1000ms));

    const auto request = queue.popNextDue(1010ms);

    ASSERT_TRUE(request.has_value());
    EXPECT_EQ(request->transmitBeamType(), beam::BeamType::TRACKING);
}

TEST(RequestQueueTest, VeryOldRequestIsStillPopped)
{
    // 요청은 만료로 폐기되지 않는다. 아무리 오래 기다렸어도 꺼낼 수 있다.
    beam::RequestQueue queue;
    queue.add(makeConfirmation(1000ms, 1_deg));
    queue.add(makeTracking(1000ms, 2_deg));

    const auto first = queue.popNextDue(100000ms);
    const auto second = queue.popNextDue(100000ms);

    ASSERT_TRUE(first.has_value());
    EXPECT_EQ(first->transmitBeamType(), beam::BeamType::CONFIRMATION);
    ASSERT_TRUE(second.has_value());
    EXPECT_EQ(second->transmitBeamType(), beam::BeamType::TRACKING);
    EXPECT_FALSE(queue.popNextDue(100000ms).has_value());
}

TEST(RequestQueueTest, CrowdedRequestsAreAllPoppedInTimeOrderWithoutDrop)
{
    // 방위각으로 기대 순서를 표시한다. 일부러 시각 순서와 다르게 넣는다.
    beam::RequestQueue queue;
    queue.add(makeTracking(1020ms, 5_deg));
    queue.add(makeConfirmation(1010ms, 2_deg));
    queue.add(makeTracking(1010ms, 3_deg));
    queue.add(makeTracking(1000ms, 1_deg));
    queue.add(makeConfirmation(1020ms, 4_deg));

    // 모든 요청이 한참 지난 시각에 꺼내도 하나도 사라지지 않고 시각 순(같은 시각이면 확인 빔 먼저)으로 나온다.
    for (double expectedAzimuth = 1.0; expectedAzimuth <= 5.0; expectedAzimuth += 1.0)
    {
        const auto request = queue.popNextDue(5000ms);
        ASSERT_TRUE(request.has_value()) << "expected azimuth " << expectedAzimuth;
        EXPECT_TRUE(request->direction_enu().azimuth.isEquivalentTo(math::Angle::fromDegrees(expectedAzimuth)));
    }

    EXPECT_FALSE(queue.popNextDue(5000ms).has_value());
}

TEST(RequestQueueTest, ClearRemovesAllRequests)
{
    beam::RequestQueue queue;
    queue.add(makeTracking(1000ms));
    queue.add(makeConfirmation(2000ms));

    queue.clear();

    EXPECT_FALSE(queue.popNextDue(5000ms).has_value());
}
