#include "scheduler/request_queue.hpp"

#include <gtest/gtest.h>

#include <chrono>

using namespace std::chrono_literals;
using namespace math::literals;

namespace
{

beam::BeamRequest makeRequest(
    beam::BeamType beamType,
    std::chrono::milliseconds transmitTime,
    math::Angle azimuth = 0_deg
) {
    const int priority = beamType == beam::BeamType::CONFIRMATION ? 1 : 0;
    return beam::BeamRequest(
        beamType, beam::BeamWidth(3_deg, 3_deg), priority,
        transmitTime, 1, math::SphericalDirection(azimuth, 0_deg));
}

beam::BeamRequest makeConfirmation(std::chrono::milliseconds transmitTime, math::Angle azimuth = 0_deg)
{
    return makeRequest(beam::BeamType::CONFIRMATION, transmitTime, azimuth);
}

beam::BeamRequest makeTracking(std::chrono::milliseconds transmitTime, math::Angle azimuth = 0_deg)
{
    return makeRequest(beam::BeamType::TRACKING, transmitTime, azimuth);
}

} // namespace

// 빈 요청 큐에서는 꺼낼 요청이 없다.
TEST(RequestQueueTest, EmptyQueueReturnsNothing)
{
    // Given: 요청이 없는 큐를 준비한다.
    beam::RequestQueue queue;

    // When: 현재 시각에 송신할 요청을 꺼낸다.
    const auto request = queue.popNextDue(1000ms);

    // Then: 큐는 빈 결과를 반환한다.
    EXPECT_FALSE(request.has_value());
}

// 미래 요청은 예정 시각 전까지 큐에 남는다.
TEST(RequestQueueTest, FutureRequestIsNotPoppedAndStaysQueued)
{
    // Given: 미래에 송신할 요청이 있는 큐를 준비한다.
    beam::RequestQueue queue;
    queue.add(makeTracking(1010ms));

    // When: 예정 시각 전과 예정 시각에 각각 요청을 꺼낸다.
    const auto beforeDue = queue.popNextDue(1000ms);
    const auto due = queue.popNextDue(1010ms);

    // Then: 예정 시각 전에 요청이 소모되지 않고 시각이 되면 반환된다.
    EXPECT_FALSE(beforeDue.has_value());
    ASSERT_TRUE(due.has_value());
    EXPECT_EQ(due->transmitTime(), 1010ms);
}

// 예정 시각이 현재 시각인 요청은 바로 반환된다.
TEST(RequestQueueTest, RequestExactlyAtCurrentTimeIsDue)
{
    // Given: 검사할 시각과 우선순위를 가진 요청들을 큐에 넣는다.
    beam::RequestQueue queue;
    queue.add(makeTracking(1000ms));

    // When: 요청의 예정 시각에 요청을 꺼낸다.
    const auto request = queue.popNextDue(1000ms);

    // Then: 예정 시각이 현재 시각인 요청은 바로 반환된다.
    ASSERT_TRUE(request.has_value());
    EXPECT_EQ(request->transmitTime(), 1000ms);
}

// 꺼낸 요청은 큐에서 제거된다.
TEST(RequestQueueTest, PoppedRequestIsRemovedFromQueue)
{
    // Given: 송신할 요청 하나를 큐에 넣는다.
    beam::RequestQueue queue;
    queue.add(makeTracking(1000ms));

    // When: 같은 시각으로 요청을 두 번 꺼낸다.
    const auto first = queue.popNextDue(1000ms);
    const auto second = queue.popNextDue(1000ms);

    // Then: 첫 요청만 반환되고 같은 요청을 다시 반환하지 않는다.
    ASSERT_TRUE(first.has_value());
    EXPECT_FALSE(second.has_value());
}

// 큐는 삽입 순서와 관계없이 송신 시각 순으로 요청을 반환한다.
TEST(RequestQueueTest, EarliestDueRequestComesFirstRegardlessOfInsertionOrder)
{
    // Given: 순서를 구분할 수 있는 여러 요청을 큐에 넣는다.
    beam::RequestQueue queue;
    queue.add(makeTracking(1010ms, 3_deg));
    queue.add(makeTracking(1000ms, 1_deg));
    queue.add(makeTracking(1005ms, 2_deg));

    // When: 송신 시각이 된 요청을 차례로 꺼낸다.
    const auto request1 = queue.popNextDue(1010ms);
    const auto request2 = queue.popNextDue(1010ms);
    const auto request3 = queue.popNextDue(1010ms);

    // Then: 큐는 삽입 순서와 관계없이 송신 시각 순으로 요청을 반환한다.
    ASSERT_TRUE(request1.has_value());
    EXPECT_TRUE(request1->direction_enu().azimuth.isEquivalentTo(1_deg));
    ASSERT_TRUE(request2.has_value());
    EXPECT_TRUE(request2->direction_enu().azimuth.isEquivalentTo(2_deg));
    ASSERT_TRUE(request3.has_value());
    EXPECT_TRUE(request3->direction_enu().azimuth.isEquivalentTo(3_deg));
}

// 송신 시각이 같으면 빔 종류와 관계없이 우선순위가 높은 요청을 먼저 반환한다.
TEST(RequestQueueTest, SameTransmitTimeHigherPriorityComesFirst)
{
    // Given: 같은 시각에 송신할 요청들의 우선순위를 빔 종류와 독립적으로 지정한다.
    beam::RequestQueue queue;
    queue.add(beam::BeamRequest(
        beam::BeamType::CONFIRMATION, beam::BeamWidth(3_deg, 3_deg), 0,
        1000ms, 1, math::SphericalDirection(1_deg, 0_deg)));
    queue.add(beam::BeamRequest(
        beam::BeamType::TRACKING, beam::BeamWidth(3_deg, 3_deg), 2,
        1000ms, 2, math::SphericalDirection(2_deg, 0_deg)));

    // When: 같은 예정 시각의 두 요청을 꺼낸다.
    const auto first = queue.popNextDue(1000ms);
    const auto second = queue.popNextDue(1010ms);

    // Then: 우선순위가 높은 추적 요청이 확인 요청보다 먼저 반환된다.
    ASSERT_TRUE(first.has_value());
    ASSERT_TRUE(second.has_value());
    EXPECT_EQ(first->requestId(), 2u);
    EXPECT_EQ(second->requestId(), 1u);
}

// 송신 시각과 우선순위가 같은 요청은 삽입 순서대로 반환된다.
TEST(RequestQueueTest, SameTransmitTimeAndTypeKeepsInsertionOrder)
{
    // Given: 순서를 구분할 수 있는 여러 요청을 큐에 넣는다.
    beam::RequestQueue queue;
    queue.add(makeTracking(1000ms, 1_deg));
    queue.add(makeTracking(1000ms, 2_deg));
    queue.add(makeTracking(1000ms, 3_deg));

    // When: 송신 시각이 된 요청을 차례로 꺼낸다.
    const auto request1 = queue.popNextDue(1000ms);
    const auto request2 = queue.popNextDue(1000ms);
    const auto request3 = queue.popNextDue(1000ms);

    // Then: 송신 시각과 우선순위가 같은 요청은 삽입 순서대로 반환된다.
    ASSERT_TRUE(request1.has_value());
    EXPECT_TRUE(request1->direction_enu().azimuth.isEquivalentTo(1_deg));
    ASSERT_TRUE(request2.has_value());
    EXPECT_TRUE(request2->direction_enu().azimuth.isEquivalentTo(2_deg));
    ASSERT_TRUE(request3.has_value());
    EXPECT_TRUE(request3->direction_enu().azimuth.isEquivalentTo(3_deg));
}

// 우선순위보다 이른 송신 시각을 먼저 적용한다.
TEST(RequestQueueTest, EarlierTrackingStillComesBeforeLaterConfirmation)
{
    // Given: 검사할 시각과 우선순위를 가진 요청들을 큐에 넣는다.
    beam::RequestQueue queue;
    queue.add(makeConfirmation(1010ms));
    queue.add(makeTracking(1000ms));

    // When: 두 요청의 예정 시각이 지난 뒤 요청을 꺼낸다.
    const auto request = queue.popNextDue(1010ms);

    // Then: 우선순위보다 이른 송신 시각을 먼저 적용한다.
    ASSERT_TRUE(request.has_value());
    EXPECT_EQ(request->transmitBeamType(), beam::BeamType::TRACKING);
}

// 오래 기다린 요청도 큐에서 만료되지 않는다.
TEST(RequestQueueTest, VeryOldRequestIsStillPopped)
{
    // Given: 검사할 시각과 우선순위를 가진 요청들을 큐에 넣는다.
    // 요청은 만료로 폐기되지 않는다. 아무리 오래 기다렸어도 꺼낼 수 있다.
    beam::RequestQueue queue;
    queue.add(makeConfirmation(1000ms, 1_deg));
    queue.add(makeTracking(1000ms, 2_deg));

    // When: 예정 시각이 한참 지난 뒤 요청을 꺼낸다.
    const auto first = queue.popNextDue(100000ms);
    const auto second = queue.popNextDue(100000ms);

    // Then: 오래 기다린 요청도 큐에서 만료되지 않는다.
    ASSERT_TRUE(first.has_value());
    EXPECT_EQ(first->transmitBeamType(), beam::BeamType::CONFIRMATION);
    ASSERT_TRUE(second.has_value());
    EXPECT_EQ(second->transmitBeamType(), beam::BeamType::TRACKING);
    EXPECT_FALSE(queue.popNextDue(100000ms).has_value());
}

// 몰린 요청도 손실 없이 시각과 우선순위 순서로 반환된다.
TEST(RequestQueueTest, CrowdedRequestsAreAllPoppedInTimeOrderWithoutDrop)
{
    // Given: 검사할 시각과 우선순위를 가진 요청들을 큐에 넣는다.
    // 방위각으로 기대 순서를 표시한다. 일부러 시각 순서와 다르게 넣는다.
    beam::RequestQueue queue;
    queue.add(makeTracking(1020ms, 5_deg));
    queue.add(makeConfirmation(1010ms, 2_deg));
    queue.add(makeTracking(1010ms, 3_deg));
    queue.add(makeTracking(1000ms, 1_deg));
    queue.add(makeConfirmation(1020ms, 4_deg));

    // 모든 요청이 한참 지난 시각에 꺼내도 하나도 사라지지 않고 시각 순(같은 시각이면 확인 빔 먼저)으로 나온다.

    // When: 모든 요청의 예정 시각이 지난 뒤 차례로 꺼낸다.
    for (double expectedAzimuth = 1.0; expectedAzimuth <= 5.0; expectedAzimuth += 1.0)
    {
        const auto request = queue.popNextDue(5000ms);

        // Then: 몰린 요청도 손실 없이 시각과 우선순위 순서로 반환된다.
        ASSERT_TRUE(request.has_value()) << "expected azimuth " << expectedAzimuth;
        EXPECT_TRUE(request->direction_enu().azimuth.isEquivalentTo(math::Angle::fromDegrees(expectedAzimuth)));
    }

    EXPECT_FALSE(queue.popNextDue(5000ms).has_value());
}

// 큐 초기화는 현재 요청과 미래 요청을 모두 제거한다.
TEST(RequestQueueTest, ClearRemovesAllRequests)
{
    // Given: 검사할 시각과 우선순위를 가진 요청들을 큐에 넣는다.
    beam::RequestQueue queue;
    queue.add(makeTracking(1000ms));
    queue.add(makeConfirmation(2000ms));

    // When: 큐를 초기화한다.
    queue.clear();

    // Then: 큐 초기화는 현재 요청과 미래 요청을 모두 제거한다.
    EXPECT_FALSE(queue.popNextDue(5000ms).has_value());
}
