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
            .confirmationBeamWidth(beam::BeamWidth(3_deg, 3_deg))
            .trackingBeamWidth(beam::BeamWidth(3_deg, 3_deg))
            .confirmationBeamWidth(beam::BeamWidth(4_deg, 4_deg))
            .trackingBeamWidth(beam::BeamWidth(2_deg, 2_deg))
            .build());
}

} // namespace

// 요청 팩토리는 명령의 송신 시각과 요청 ID 및 방향을 전달한다.
TEST(BeamRequestFactoryTest, ConvertsFromRequestBeamCommand)
{
    // Given: 확인 빔 요청 명령을 준비한다.
    const beam::RequestBeamCommand command = beam::RequestBeamCommand::Builder()
        .beamType(beam::RequestBeamType::CONFIRMATION)
        .transmitTime(100ms)
        .requestId(20)
        .direction_ant(makeDirection())
        .build();

    // When: 팩토리로 요청 명령을 변환한다.
    const beam::BeamRequest request = makeFactory().create(command);

    // Then: 요청 팩토리는 명령의 송신 시각과 요청 ID 및 방향을 전달한다.
    EXPECT_EQ(request.transmitBeamType(), beam::BeamType::CONFIRMATION);
    EXPECT_EQ(request.transmitTime(), 100ms);
    EXPECT_EQ(request.requestId(), 20u);
    EXPECT_TRUE(request.direction_enu().azimuth.isEquivalentTo(12.5_deg));
    EXPECT_EQ(request.direction_enu().elevation, 20.5_deg);
}

// 요청 팩토리는 빔 종류에 맞는 폭과 우선순위를 지정한다.
TEST(BeamRequestFactoryTest, CreatesRequestMatchingBeamType)
{
    // Given: 확인 및 추적 빔 폭이 서로 다른 팩토리를 준비한다.
    const beam::BeamRequestFactory factory = makeFactory();

    // When: 각 종류의 요청을 생성한다.
    const beam::BeamRequest confirmation = factory.create(
        beam::RequestBeamCommand(beam::RequestBeamType::CONFIRMATION, 0ms, 1, makeDirection()));
    const beam::BeamRequest tracking = factory.create(
        beam::RequestBeamCommand(beam::RequestBeamType::TRACKING, 0ms, 2, makeDirection()));

    // Then: 요청 팩토리는 빔 종류에 맞는 폭과 우선순위를 지정한다.
    EXPECT_EQ(confirmation.transmitBeamType(), beam::BeamType::CONFIRMATION);
    EXPECT_EQ(confirmation.beamWidth().azimuth, 4_deg);
    EXPECT_EQ(confirmation.beamWidth().elevation, 4_deg);
    EXPECT_EQ(tracking.transmitBeamType(), beam::BeamType::TRACKING);
    EXPECT_EQ(tracking.beamWidth().azimuth, 2_deg);
    EXPECT_EQ(tracking.beamWidth().elevation, 2_deg);
    EXPECT_GT(confirmation.priority(), tracking.priority());
}

// 요청 팩토리는 잘못된 ID와 시각 및 빔 종류를 거부한다.
TEST(BeamRequestFactoryTest, RejectsInvalidCommand)
{
    // Given: 잘못된 값을 하나씩 가진 요청 명령들을 준비한다.
    const beam::BeamRequestFactory factory = makeFactory();

    const beam::RequestBeamCommand zeroId(beam::RequestBeamType::TRACKING, 0ms, 0, makeDirection());
    const beam::RequestBeamCommand negativeTime(beam::RequestBeamType::TRACKING, -10ms, 1, makeDirection());
    const beam::RequestBeamCommand zeroType(static_cast<beam::RequestBeamType>(0), 0ms, 1, makeDirection());
    const beam::RequestBeamCommand unknownType(static_cast<beam::RequestBeamType>(9), 0ms, 1, makeDirection());
    const beam::RequestBeamCommand searchType(
        static_cast<beam::RequestBeamType>(beam::BeamType::SEARCH), 0ms, 1, makeDirection());

    // When: 각 명령으로 요청 생성을 시도한다.
    const auto create = [&](const beam::RequestBeamCommand& command) { return factory.create(command); };

    // Then: 요청 팩토리는 잘못된 ID와 시각 및 빔 종류를 거부한다.
    EXPECT_THROW(create(zeroId), std::invalid_argument);
    EXPECT_THROW(create(negativeTime), std::invalid_argument);
    EXPECT_THROW(create(zeroType), std::invalid_argument);
    EXPECT_THROW(create(unknownType), std::invalid_argument);
    EXPECT_THROW(create(searchType), std::invalid_argument);
}
