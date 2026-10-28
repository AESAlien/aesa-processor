#include "scheduler/beam_request.hpp"
#include "scheduler/request_beam_policy_factory.hpp"

#include <gtest/gtest.h>

#include <chrono>
#include <cstdint>
#include <limits>
#include <memory>
#include <stdexcept>

using namespace std::chrono_literals;
using namespace math::literals;

namespace
{

math::SphericalDirection makeDirection()
{
    return math::SphericalDirection(math::Azimuth(12.5_deg), math::Elevation(20.5_deg));
}

// 종류와 축마다 빔 폭을 다르게 두어 값이 뒤바뀌면 테스트가 실패하게 한다.
std::shared_ptr<const beam::IRequestBeamPolicy> makePolicy(beam::RequestBeamType beamType)
{
    const beam::RequestBeamPolicyFactory factory(
        beam::RequestBeamConfig::Builder()
            .confirmationBeamWidth(beam::BeamWidth(math::AngularWidth(3_deg), math::AngularWidth(3_deg)))
            .trackingBeamWidth(beam::BeamWidth(math::AngularWidth(3_deg), math::AngularWidth(3_deg)))
            .confirmationBeamWidth(beam::BeamWidth(
                math::AngularWidth(4_deg),
                math::AngularWidth(5_deg)))
            .trackingBeamWidth(beam::BeamWidth(
                math::AngularWidth(2_deg),
                math::AngularWidth(3_deg)))
            .build());
    return factory.create(beamType);
}

} // namespace

TEST(BeamRequestTest, KeepsGivenValues)
{
    const beam::BeamRequest request(makePolicy(beam::RequestBeamType::TRACKING), 1500ms, 7, makeDirection());

    EXPECT_EQ(request.transmitTime(), 1500ms);
    EXPECT_EQ(request.requestId(), 7u);
    EXPECT_TRUE(request.direction_enu().azimuth.isEquivalentTo(12.5_deg));
    EXPECT_EQ(request.direction_enu().elevation, 20.5_deg);
}

TEST(BeamRequestTest, DelegatesToInjectedPolicy)
{
    const beam::BeamRequest confirmation(makePolicy(beam::RequestBeamType::CONFIRMATION), 0ms, 1, makeDirection());
    const beam::BeamRequest tracking(makePolicy(beam::RequestBeamType::TRACKING), 0ms, 2, makeDirection());

    EXPECT_EQ(confirmation.transmitBeamType(), beam::BeamType::CONFIRMATION);
    EXPECT_EQ(confirmation.beamWidth().azimuth, 4_deg);
    EXPECT_EQ(confirmation.beamWidth().elevation, 5_deg);
    EXPECT_EQ(tracking.transmitBeamType(), beam::BeamType::TRACKING);
    EXPECT_EQ(tracking.beamWidth().azimuth, 2_deg);
    EXPECT_EQ(tracking.beamWidth().elevation, 3_deg);
    EXPECT_GT(confirmation.priority(), tracking.priority());
}

TEST(BeamRequestTest, CopyKeepsSamePolicyBehavior)
{
    const beam::BeamRequest request(makePolicy(beam::RequestBeamType::CONFIRMATION), 100ms, 3, makeDirection());
    const beam::BeamRequest copy = request;

    EXPECT_EQ(copy.transmitBeamType(), request.transmitBeamType());
    EXPECT_EQ(copy.beamWidth(), request.beamWidth());
    EXPECT_EQ(copy.priority(), request.priority());
    EXPECT_EQ(copy.requestId(), request.requestId());
}

TEST(BeamRequestTest, AcceptsBoundaryValues)
{
    // transmitTime 0과 가장 큰 requestId는 올바른 값이다.
    EXPECT_NO_THROW(beam::BeamRequest(
        makePolicy(beam::RequestBeamType::TRACKING),
        0ms,
        std::numeric_limits<std::uint32_t>::max(),
        makeDirection()));
}

TEST(BeamRequestTest, RejectsNullPolicy)
{
    EXPECT_THROW(beam::BeamRequest(nullptr, 0ms, 1, makeDirection()), std::invalid_argument);
}

TEST(BeamRequestTest, RejectsNegativeTransmitTime)
{
    EXPECT_THROW(
        beam::BeamRequest(makePolicy(beam::RequestBeamType::TRACKING), -1ms, 1, makeDirection()),
        std::invalid_argument);
}

TEST(BeamRequestTest, RejectsZeroRequestId)
{
    EXPECT_THROW(
        beam::BeamRequest(makePolicy(beam::RequestBeamType::CONFIRMATION), 0ms, 0, makeDirection()),
        std::invalid_argument);
}

TEST(BeamTypeTest, RequestBeamTypeValuesMatchBeamType)
{
    // 와이어로 나가는 값이므로 숫자 자체도 고정한다. 두 enum의 값 일치는 beam_type.hpp 의 static_assert 가 보장한다.
    EXPECT_EQ(static_cast<int>(beam::BeamType::SEARCH), 1);
    EXPECT_EQ(static_cast<int>(beam::BeamType::CONFIRMATION), 2);
    EXPECT_EQ(static_cast<int>(beam::BeamType::TRACKING), 3);
    EXPECT_EQ(static_cast<int>(beam::RequestBeamType::CONFIRMATION), 2);
    EXPECT_EQ(static_cast<int>(beam::RequestBeamType::TRACKING), 3);
}
