#include "scheduler/request_beam_policy_factory.hpp"

#include <gtest/gtest.h>

#include <stdexcept>

using namespace math::literals;

namespace
{

// 확인 빔 폭과 추적 빔 폭을 서로 다른 값으로 두어 두 값이 뒤바뀌면 테스트가 실패하게 한다.
beam::RequestBeamPolicyFactory makeFactory()
{
    return beam::RequestBeamPolicyFactory(
        beam::RequestBeamConfig::Builder()
            .confirmationBeamWidth(3_deg)
            .trackingBeamWidth(3_deg)
            .confirmationBeamWidth(4_deg)
            .trackingBeamWidth(2_deg)
            .build());
}

} // namespace

TEST(RequestBeamPolicyTest, ConfirmationTransmitsConfirmationBeam)
{
    const auto policy = makeFactory().create(beam::RequestBeamType::CONFIRMATION);

    EXPECT_EQ(policy->transmitBeamType(), beam::BeamType::CONFIRMATION);
}

TEST(RequestBeamPolicyTest, TrackingTransmitsTrackingBeam)
{
    const auto policy = makeFactory().create(beam::RequestBeamType::TRACKING);

    EXPECT_EQ(policy->transmitBeamType(), beam::BeamType::TRACKING);
}

TEST(RequestBeamPolicyTest, ConfirmationUsesInjectedConfirmationBeamWidth)
{
    const auto policy = makeFactory().create(beam::RequestBeamType::CONFIRMATION);

    EXPECT_EQ(policy->beamWidth(), 4_deg);
}

TEST(RequestBeamPolicyTest, TrackingUsesInjectedTrackingBeamWidth)
{
    const auto policy = makeFactory().create(beam::RequestBeamType::TRACKING);

    EXPECT_EQ(policy->beamWidth(), 2_deg);
}

TEST(RequestBeamPolicyTest, DefaultConfigGivesDefaultBeamWidths)
{
    const beam::RequestBeamPolicyFactory factory{
        beam::RequestBeamConfig(3_deg, 3_deg)};

    EXPECT_EQ(factory.create(beam::RequestBeamType::CONFIRMATION)->beamWidth(), 3_deg);
    EXPECT_EQ(factory.create(beam::RequestBeamType::TRACKING)->beamWidth(), 3_deg);
}

TEST(RequestBeamPolicyTest, ConfirmationHasHigherPriorityThanTracking)
{
    const beam::RequestBeamPolicyFactory factory = makeFactory();
    const auto confirmation = factory.create(beam::RequestBeamType::CONFIRMATION);
    const auto tracking = factory.create(beam::RequestBeamType::TRACKING);

    EXPECT_GT(confirmation->priority(), tracking->priority());
}

TEST(RequestBeamPolicyTest, SameTypeReturnsSamePolicy)
{
    const beam::RequestBeamPolicyFactory factory = makeFactory();

    EXPECT_EQ(
        factory.create(beam::RequestBeamType::TRACKING),
        factory.create(beam::RequestBeamType::TRACKING));
}

TEST(RequestBeamPolicyTest, DifferentTypesReturnDifferentPolicies)
{
    const beam::RequestBeamPolicyFactory factory = makeFactory();

    EXPECT_NE(
        factory.create(beam::RequestBeamType::CONFIRMATION),
        factory.create(beam::RequestBeamType::TRACKING));
}

TEST(RequestBeamPolicyTest, UnknownTypeThrows)
{
    const beam::RequestBeamPolicyFactory factory = makeFactory();

    EXPECT_THROW(factory.create(static_cast<beam::RequestBeamType>(0)), std::invalid_argument);
    EXPECT_THROW(factory.create(static_cast<beam::RequestBeamType>(99)), std::invalid_argument);

    // 요청할 수 없는 탐색 빔 값(BeamType::SEARCH = 1)도 거부한다.
    EXPECT_THROW(
        factory.create(static_cast<beam::RequestBeamType>(beam::BeamType::SEARCH)),
        std::invalid_argument);
}
