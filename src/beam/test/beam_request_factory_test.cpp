#include "scheduler/beam_request_factory.hpp"

#include <gtest/gtest.h>

#include <chrono>
#include <stdexcept>

using namespace std::chrono_literals;
using namespace aesa_processor::math::literals;

namespace
{

aesa_processor::math::SphericalDirection makeDirection()
{
    return aesa_processor::math::SphericalDirection(12.5_deg, 20.5_deg);
}

// 확인 빔 폭과 추적 빔 폭을 서로 다른 값으로 두어 두 값이 뒤바뀌면 테스트가 실패하게 한다.
aesa_processor::beam::BeamRequestFactory makeFactory()
{
    return aesa_processor::beam::BeamRequestFactory(
        aesa_processor::beam::RequestBeamConfig::Builder()
            .confirmationBeamWidth(4_deg)
            .trackingBeamWidth(2_deg)
            .build());
}

} // namespace

TEST(BeamRequestFactoryTest, ConvertsFromRequestBeamCommand)
{
    const aesa_processor::beam::RequestBeamCommand command = aesa_processor::beam::RequestBeamCommand::Builder()
        .beamType(aesa_processor::beam::RequestBeamType::CONFIRMATION)
        .transmitTime(100ms)
        .requestId(20)
        .direction_ant(makeDirection())
        .build();

    const aesa_processor::beam::BeamRequest request = makeFactory().create(command);

    EXPECT_EQ(request.transmitBeamType(), aesa_processor::beam::BeamType::CONFIRMATION);
    EXPECT_EQ(request.transmitTime(), 100ms);
    EXPECT_EQ(request.requestId(), 20u);
    EXPECT_TRUE(request.direction_enu().azimuth.isEquivalentTo(12.5_deg));
    EXPECT_EQ(request.direction_enu().elevation, 20.5_deg);
}

TEST(BeamRequestFactoryTest, InjectsPolicyMatchingBeamType)
{
    const aesa_processor::beam::BeamRequestFactory factory = makeFactory();

    const aesa_processor::beam::BeamRequest confirmation = factory.create(
        aesa_processor::beam::RequestBeamCommand(aesa_processor::beam::RequestBeamType::CONFIRMATION, 0ms, 1, makeDirection()));
    const aesa_processor::beam::BeamRequest tracking = factory.create(
        aesa_processor::beam::RequestBeamCommand(aesa_processor::beam::RequestBeamType::TRACKING, 0ms, 2, makeDirection()));

    EXPECT_EQ(confirmation.transmitBeamType(), aesa_processor::beam::BeamType::CONFIRMATION);
    EXPECT_EQ(confirmation.beamWidth(), 4_deg);
    EXPECT_EQ(tracking.transmitBeamType(), aesa_processor::beam::BeamType::TRACKING);
    EXPECT_EQ(tracking.beamWidth(), 2_deg);
    EXPECT_GT(confirmation.priority(), tracking.priority());
}

TEST(BeamRequestFactoryTest, RejectsInvalidCommand)
{
    const aesa_processor::beam::BeamRequestFactory factory = makeFactory();

    const aesa_processor::beam::RequestBeamCommand zeroId(aesa_processor::beam::RequestBeamType::TRACKING, 0ms, 0, makeDirection());
    const aesa_processor::beam::RequestBeamCommand negativeTime(aesa_processor::beam::RequestBeamType::TRACKING, -10ms, 1, makeDirection());
    const aesa_processor::beam::RequestBeamCommand unknownType(static_cast<aesa_processor::beam::RequestBeamType>(9), 0ms, 1, makeDirection());
    const aesa_processor::beam::RequestBeamCommand searchType(
        static_cast<aesa_processor::beam::RequestBeamType>(aesa_processor::beam::BeamType::SEARCH), 0ms, 1, makeDirection());

    EXPECT_THROW(factory.create(zeroId), std::invalid_argument);
    EXPECT_THROW(factory.create(negativeTime), std::invalid_argument);
    EXPECT_THROW(factory.create(unknownType), std::invalid_argument);
    EXPECT_THROW(factory.create(searchType), std::invalid_argument);
}
