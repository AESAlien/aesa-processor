#include "scheduler/request_beam_policy.hpp"

#include <gtest/gtest.h>

#include <stdexcept>

using namespace math::literals;

namespace
{

// 확인 빔 폭과 추적 빔 폭을 서로 다른 값으로 두어 두 값이 뒤바뀌면 테스트가 실패하게 한다.
beam::RequestBeamConfig makeConfig()
{
    return beam::RequestBeamConfig::Builder()
        .confirmationBeamWidth(4_deg)
        .trackingBeamWidth(2_deg)
        .build();
}

} // namespace

TEST(RequestBeamPolicyTest, ConfirmationTransmitsConfirmationBeam)
{
    const beam::IRequestBeamPolicy& policy = beam::requestBeamPolicyOf(beam::RequestBeamType::CONFIRMATION);

    EXPECT_EQ(policy.transmitBeamType(), beam::BeamType::CONFIRMATION);
}

TEST(RequestBeamPolicyTest, TrackingTransmitsTrackingBeam)
{
    const beam::IRequestBeamPolicy& policy = beam::requestBeamPolicyOf(beam::RequestBeamType::TRACKING);

    EXPECT_EQ(policy.transmitBeamType(), beam::BeamType::TRACKING);
}

TEST(RequestBeamPolicyTest, ConfirmationUsesConfirmationBeamWidth)
{
    const beam::IRequestBeamPolicy& policy = beam::requestBeamPolicyOf(beam::RequestBeamType::CONFIRMATION);

    EXPECT_EQ(policy.beamWidth(makeConfig()), 4_deg);
}

TEST(RequestBeamPolicyTest, TrackingUsesTrackingBeamWidth)
{
    const beam::IRequestBeamPolicy& policy = beam::requestBeamPolicyOf(beam::RequestBeamType::TRACKING);

    EXPECT_EQ(policy.beamWidth(makeConfig()), 2_deg);
}

TEST(RequestBeamPolicyTest, ConfirmationHasHigherPriorityThanTracking)
{
    const beam::IRequestBeamPolicy& confirmation = beam::requestBeamPolicyOf(beam::RequestBeamType::CONFIRMATION);
    const beam::IRequestBeamPolicy& tracking = beam::requestBeamPolicyOf(beam::RequestBeamType::TRACKING);

    EXPECT_GT(confirmation.priority(), tracking.priority());
}

TEST(RequestBeamPolicyTest, SameTypeReturnsSamePolicy)
{
    EXPECT_EQ(
        &beam::requestBeamPolicyOf(beam::RequestBeamType::TRACKING),
        &beam::requestBeamPolicyOf(beam::RequestBeamType::TRACKING));
}

TEST(RequestBeamPolicyTest, UnknownTypeThrows)
{
    EXPECT_THROW(
        beam::requestBeamPolicyOf(static_cast<beam::RequestBeamType>(99)),
        std::invalid_argument);
}
