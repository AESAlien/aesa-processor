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
    math::Angle azimuth = 0_deg
) {
    return beam::BeamRequest::Builder()
        .beamType(beamType)
        .transmitTime(transmitTime)
        .requestId(0)
        .azimuth_ant(azimuth)
        .elevation_ant(0_deg)
        .build();
}

beam::BeamRequest makeConfirmation(std::chrono::milliseconds transmitTime, math::Angle azimuth = 0_deg)
{
    return makeRequest(beam::BeamRequest::BeamType::CONFIRMATION, transmitTime, azimuth);
}

beam::BeamRequest makeTracking(std::chrono::milliseconds transmitTime, math::Angle azimuth = 0_deg)
{
    return makeRequest(beam::BeamRequest::BeamType::TRACKING, transmitTime, azimuth);
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
    queue.add(makeTracking(1010ms, 3_deg));
    queue.add(makeTracking(1000ms, 1_deg));
    queue.add(makeTracking(1005ms, 2_deg));

    EXPECT_TRUE(queue.popNextDue(1010ms)->azimuth_ant.equals(1_deg, math::QuantityTolerance<math::Angle>{math::Angle::fromDegrees(0.0), 0.0}));
    EXPECT_TRUE(queue.popNextDue(1010ms)->azimuth_ant.equals(2_deg, math::QuantityTolerance<math::Angle>{math::Angle::fromDegrees(0.0), 0.0}));
    EXPECT_TRUE(queue.popNextDue(1010ms)->azimuth_ant.equals(3_deg, math::QuantityTolerance<math::Angle>{math::Angle::fromDegrees(0.0), 0.0}));
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
    EXPECT_EQ(first->beamType, beam::BeamRequest::BeamType::CONFIRMATION);
    EXPECT_EQ(second->beamType, beam::BeamRequest::BeamType::TRACKING);
}

TEST(RequestQueueTest, SameTransmitTimeAndTypeKeepsInsertionOrder)
{
    beam::RequestQueue queue;
    queue.add(makeTracking(1000ms, 1_deg));
    queue.add(makeTracking(1000ms, 2_deg));
    queue.add(makeTracking(1000ms, 3_deg));

    EXPECT_TRUE(queue.popNextDue(1000ms)->azimuth_ant.equals(1_deg, math::QuantityTolerance<math::Angle>{math::Angle::fromDegrees(0.0), 0.0}));
    EXPECT_TRUE(queue.popNextDue(1000ms)->azimuth_ant.equals(2_deg, math::QuantityTolerance<math::Angle>{math::Angle::fromDegrees(0.0), 0.0}));
    EXPECT_TRUE(queue.popNextDue(1000ms)->azimuth_ant.equals(3_deg, math::QuantityTolerance<math::Angle>{math::Angle::fromDegrees(0.0), 0.0}));
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

TEST(RequestQueueTest, VeryOldRequestIsStillPopped)
{
    // 요청은 만료로 폐기되지 않는다. 아무리 오래 기다렸어도 꺼낼 수 있다.
    beam::RequestQueue queue;
    queue.add(makeConfirmation(1000ms, 1_deg));
    queue.add(makeTracking(1000ms, 2_deg));

    const auto first = queue.popNextDue(100000ms);
    const auto second = queue.popNextDue(100000ms);

    ASSERT_TRUE(first.has_value());
    EXPECT_EQ(first->beamType, beam::BeamRequest::BeamType::CONFIRMATION);
    ASSERT_TRUE(second.has_value());
    EXPECT_EQ(second->beamType, beam::BeamRequest::BeamType::TRACKING);
    EXPECT_TRUE(queue.empty());
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
        EXPECT_DOUBLE_EQ(request->azimuth_ant.deg(), expectedAzimuth);
    }

    EXPECT_FALSE(queue.popNextDue(5000ms).has_value());
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
