#include "scheduler/beam_request_factory.hpp"

#include <gtest/gtest.h>

#include <chrono>
#include <stdexcept>

using namespace std::chrono_literals;
using namespace math::literals;

namespace
{

math::SphericalDirection makeDirection()
{
    return math::SphericalDirection(12.5_deg, 20.5_deg);
}

// 확인 빔 폭과 추적 빔 폭을 서로 다른 값으로 두어 두 값이 뒤바뀌면 테스트가 실패하게 한다.
beam::BeamRequestFactory makeFactory()
{
    return beam::BeamRequestFactory(
        beam::RequestBeamConfig::Builder()
            .confirmationBeamWidth(4_deg)
            .trackingBeamWidth(2_deg)
            .build());
}

} // namespace

TEST(BeamRequestFactoryTest, ConvertsFromRequestBeamCommand)
{
    const beam::RequestBeamCommand command = beam::RequestBeamCommand::Builder()
        .beamType(beam::RequestBeamType::CONFIRMATION)
        .transmitTime(100ms)
        .requestId(20)
        .direction_ant(makeDirection())
        .build();

    const beam::BeamRequest request = makeFactory().create(command);

    EXPECT_EQ(request.transmitBeamType(), beam::BeamType::CONFIRMATION);
    EXPECT_EQ(request.transmitTime(), 100ms);
    EXPECT_EQ(request.requestId(), 20u);
    EXPECT_TRUE(request.direction_ant().azimuth.isEquivalentTo(12.5_deg));
    EXPECT_EQ(request.direction_ant().elevation, 20.5_deg);
}

TEST(BeamRequestFactoryTest, InjectsPolicyMatchingBeamType)
{
    const beam::BeamRequestFactory factory = makeFactory();

    const beam::BeamRequest confirmation = factory.create(
        beam::RequestBeamCommand(beam::RequestBeamType::CONFIRMATION, 0ms, 1, makeDirection()));
    const beam::BeamRequest tracking = factory.create(
        beam::RequestBeamCommand(beam::RequestBeamType::TRACKING, 0ms, 2, makeDirection()));

    EXPECT_EQ(confirmation.transmitBeamType(), beam::BeamType::CONFIRMATION);
    EXPECT_EQ(confirmation.beamWidth(), 4_deg);
    EXPECT_EQ(tracking.transmitBeamType(), beam::BeamType::TRACKING);
    EXPECT_EQ(tracking.beamWidth(), 2_deg);
    EXPECT_GT(confirmation.priority(), tracking.priority());
}

TEST(BeamRequestFactoryTest, RejectsInvalidCommand)
{
    const beam::BeamRequestFactory factory = makeFactory();

    const beam::RequestBeamCommand zeroId(beam::RequestBeamType::TRACKING, 0ms, 0, makeDirection());
    const beam::RequestBeamCommand negativeTime(beam::RequestBeamType::TRACKING, -10ms, 1, makeDirection());
    const beam::RequestBeamCommand unknownType(static_cast<beam::RequestBeamType>(9), 0ms, 1, makeDirection());
    const beam::RequestBeamCommand searchType(
        static_cast<beam::RequestBeamType>(beam::BeamType::SEARCH), 0ms, 1, makeDirection());

    EXPECT_THROW(factory.create(zeroId), std::invalid_argument);
    EXPECT_THROW(factory.create(negativeTime), std::invalid_argument);
    EXPECT_THROW(factory.create(unknownType), std::invalid_argument);
    EXPECT_THROW(factory.create(searchType), std::invalid_argument);
}
