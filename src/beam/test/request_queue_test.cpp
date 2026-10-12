#include "scheduler/request_queue.hpp"

#include <gtest/gtest.h>

#include <chrono>

using namespace std::chrono_literals;
using namespace math::literals;

namespace
{

beam::BeamRequest makeRequest(
    beam::BeamRequest::BeamType beamType,
    std::chrono::milliseconds transmitTime,
    double azimuth_deg = 0.0
) {
    return beam::BeamRequest::Builder()
        .beamType(beamType)
        .transmitTime(transmitTime)
        .requestId(0)
        .azimuth_ant(math::Angle::fromDegrees(azimuth_deg))
        .elevation_ant(0_deg)
        .build();
}

beam::BeamRequest makeConfirmation(std::chrono::milliseconds transmitTime, double azimuth_deg = 0.0)
{
    return makeRequest(beam::BeamRequest::BeamType::CONFIRMATION, transmitTime, azimuth_deg);
}

beam::BeamRequest makeTracking(std::chrono::milliseconds transmitTime, double azimuth_deg = 0.0)
{
    return makeRequest(beam::BeamRequest::BeamType::TRACKING, transmitTime, azimuth_deg);
}

} // namespace

TEST(RequestQueueTest, EmptyQueueReturnsNothing)
{
    beam::RequestQueue queue;

    EXPECT_TRUE(queue.empty());
    EXPECT_FALSE(queue.popNextDue(1000ms).has_value());
}

TEST(RequestQueueTest, FutureRequestIsNotPoppedAndStaysQueued)
{
    beam::RequestQueue queue;
    queue.add(makeTracking(1010ms));

    EXPECT_FALSE(queue.popNextDue(1000ms).has_value());
    EXPECT_EQ(queue.size(), 1u);
}

TEST(RequestQueueTest, RequestExactlyAtCurrentTimeIsDue)
{
    beam::RequestQueue queue;
    queue.add(makeTracking(1000ms));

    const auto request = queue.popNextDue(1000ms);

    ASSERT_TRUE(request.has_value());
    EXPECT_EQ(request->transmitTime, 1000ms);
}

TEST(RequestQueueTest, PoppedRequestIsRemovedFromQueue)
{
    beam::RequestQueue queue;
    queue.add(makeTracking(1000ms));

    ASSERT_TRUE(queue.popNextDue(1000ms).has_value());
    EXPECT_TRUE(queue.empty());
    EXPECT_FALSE(queue.popNextDue(1000ms).has_value());
}

TEST(RequestQueueTest, EarliestDueRequestComesFirstRegardlessOfInsertionOrder)
{
    beam::RequestQueue queue;
    queue.add(makeTracking(1010ms, 3.0));
    queue.add(makeTracking(1000ms, 1.0));
    queue.add(makeTracking(1005ms, 2.0));

    EXPECT_DOUBLE_EQ(queue.popNextDue(1010ms)->azimuth_ant.deg(), 1.0);
    EXPECT_DOUBLE_EQ(queue.popNextDue(1010ms)->azimuth_ant.deg(), 2.0);
    EXPECT_DOUBLE_EQ(queue.popNextDue(1010ms)->azimuth_ant.deg(), 3.0);
}

TEST(RequestQueueTest, SameTransmitTimeConfirmationComesBeforeTracking)
{
    beam::RequestQueue queue;
    queue.add(makeTracking(1000ms, 1.0));
    queue.add(makeConfirmation(1000ms, 2.0));

    const auto first = queue.popNextDue(1000ms);
    const auto second = queue.popNextDue(1010ms);

    ASSERT_TRUE(first.has_value());
    ASSERT_TRUE(second.has_value());
    EXPECT_EQ(first->beamType, beam::BeamRequest::BeamType::CONFIRMATION);
    EXPECT_EQ(second->beamType, beam::BeamRequest::BeamType::TRACKING);
}

TEST(RequestQueueTest, SameTransmitTimeAndTypeKeepsInsertionOrder)
{
    beam::RequestQueue queue;
    queue.add(makeTracking(1000ms, 1.0));
    queue.add(makeTracking(1000ms, 2.0));
    queue.add(makeTracking(1000ms, 3.0));

    EXPECT_DOUBLE_EQ(queue.popNextDue(1000ms)->azimuth_ant.deg(), 1.0);
    EXPECT_DOUBLE_EQ(queue.popNextDue(1000ms)->azimuth_ant.deg(), 2.0);
    EXPECT_DOUBLE_EQ(queue.popNextDue(1000ms)->azimuth_ant.deg(), 3.0);
}

TEST(RequestQueueTest, EarlierTrackingStillComesBeforeLaterConfirmation)
{
    beam::RequestQueue queue;
    queue.add(makeConfirmation(1010ms));
    queue.add(makeTracking(1000ms));

    const auto request = queue.popNextDue(1010ms);

    ASSERT_TRUE(request.has_value());
    EXPECT_EQ(request->beamType, beam::BeamRequest::BeamType::TRACKING);
}

TEST(RequestQueueTest, ConfirmationExpiresAtTwentyMilliseconds)
{
    beam::RequestQueue valid;
    valid.add(makeConfirmation(1000ms));
    EXPECT_TRUE(valid.popNextDue(1019ms).has_value());

    beam::RequestQueue expired;
    expired.add(makeConfirmation(1000ms));
    EXPECT_FALSE(expired.popNextDue(1020ms).has_value());
    EXPECT_TRUE(expired.empty());
}

TEST(RequestQueueTest, TrackingExpiresAtThirtyMilliseconds)
{
    beam::RequestQueue valid;
    valid.add(makeTracking(1000ms));
    EXPECT_TRUE(valid.popNextDue(1029ms).has_value());

    beam::RequestQueue expired;
    expired.add(makeTracking(1000ms));
    EXPECT_FALSE(expired.popNextDue(1030ms).has_value());
    EXPECT_TRUE(expired.empty());
}

TEST(RequestQueueTest, ExpiredRequestIsSkippedAndNextValidRequestIsReturned)
{
    beam::RequestQueue queue;
    queue.add(makeTracking(1000ms, 1.0));
    queue.add(makeTracking(1025ms, 2.0));

    const auto request = queue.popNextDue(1030ms);

    ASSERT_TRUE(request.has_value());
    EXPECT_DOUBLE_EQ(request->azimuth_ant.deg(), 2.0);
    EXPECT_TRUE(queue.empty());
}

TEST(RequestQueueTest, ExpiryCheckAppliesPerBeamType)
{
    // 1000ms 시점 요청이 1025ms 틱에 평가될 때 확인(20ms)은 만료, 추적(30ms)은 유효하다.
    beam::RequestQueue queue;
    queue.add(makeConfirmation(1000ms, 1.0));
    queue.add(makeTracking(1000ms, 2.0));

    const auto request = queue.popNextDue(1025ms);

    ASSERT_TRUE(request.has_value());
    EXPECT_EQ(request->beamType, beam::BeamRequest::BeamType::TRACKING);
    EXPECT_TRUE(queue.empty());
}

TEST(RequestQueueTest, ExpiryDoesNotAffectFutureRequests)
{
    beam::RequestQueue queue;
    queue.add(makeTracking(1000ms));
    queue.add(makeTracking(2000ms));

    EXPECT_FALSE(queue.popNextDue(1100ms).has_value());
    EXPECT_EQ(queue.size(), 1u);

    const auto request = queue.popNextDue(2000ms);
    ASSERT_TRUE(request.has_value());
    EXPECT_EQ(request->transmitTime, 2000ms);
}

TEST(RequestQueueTest, UnknownBeamTypeIsDropped)
{
    beam::RequestQueue queue;
    queue.add(makeRequest(static_cast<beam::BeamRequest::BeamType>(0), 1000ms));

    EXPECT_FALSE(queue.popNextDue(1000ms).has_value());
    EXPECT_TRUE(queue.empty());
}

TEST(RequestQueueTest, ClearRemovesAllRequests)
{
    beam::RequestQueue queue;
    queue.add(makeTracking(1000ms));
    queue.add(makeConfirmation(2000ms));

    queue.clear();

    EXPECT_TRUE(queue.empty());
    EXPECT_EQ(queue.size(), 0u);
    EXPECT_FALSE(queue.popNextDue(5000ms).has_value());
}
