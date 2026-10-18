#include "scheduler/beam_request.hpp"

#include <gtest/gtest.h>

#include <chrono>
#include <cstdint>
#include <limits>
#include <stdexcept>

using namespace std::chrono_literals;
using namespace math::literals;

namespace
{

math::SphericalDirection makeDirection()
{
    return math::SphericalDirection(12.5_deg, 20.5_deg);
}

} // namespace

TEST(BeamRequestTest, KeepsGivenValues)
{
    const beam::BeamRequest request(beam::RequestBeamType::TRACKING, 1500ms, 7, makeDirection());

    EXPECT_EQ(request.beamType(), beam::RequestBeamType::TRACKING);
    EXPECT_EQ(request.transmitTime(), 1500ms);
    EXPECT_EQ(request.requestId(), 7u);
    EXPECT_EQ(request.direction_ant().azimuth, 12.5_deg);
    EXPECT_EQ(request.direction_ant().elevation, 20.5_deg);
}

TEST(BeamRequestTest, AcceptsBothRequestBeamTypes)
{
    EXPECT_NO_THROW(beam::BeamRequest(beam::RequestBeamType::CONFIRMATION, 0ms, 1, makeDirection()));
    EXPECT_NO_THROW(beam::BeamRequest(beam::RequestBeamType::TRACKING, 0ms, 1, makeDirection()));
}

TEST(BeamRequestTest, AcceptsBoundaryValues)
{
    // transmitTime 0과 가장 큰 requestId는 올바른 값이다.
    EXPECT_NO_THROW(beam::BeamRequest(
        beam::RequestBeamType::TRACKING, 0ms, std::numeric_limits<std::uint32_t>::max(), makeDirection()));
}

TEST(BeamRequestTest, RejectsUnknownBeamType)
{
    EXPECT_THROW(
        beam::BeamRequest(static_cast<beam::RequestBeamType>(0), 0ms, 1, makeDirection()),
        std::invalid_argument);

    // 요청할 수 없는 탐색 빔 값(BeamType::SEARCH = 1)도 거부한다.
    EXPECT_THROW(
        beam::BeamRequest(static_cast<beam::RequestBeamType>(beam::BeamType::SEARCH), 0ms, 1, makeDirection()),
        std::invalid_argument);
}

TEST(BeamRequestTest, RejectsNegativeTransmitTime)
{
    EXPECT_THROW(
        beam::BeamRequest(beam::RequestBeamType::TRACKING, -1ms, 1, makeDirection()),
        std::invalid_argument);
}

TEST(BeamRequestTest, RejectsZeroRequestId)
{
    EXPECT_THROW(
        beam::BeamRequest(beam::RequestBeamType::CONFIRMATION, 0ms, 0, makeDirection()),
        std::invalid_argument);
}

TEST(BeamRequestTest, ConvertsFromRequestBeamCommand)
{
    const beam::RequestBeamCommand command = beam::RequestBeamCommand::Builder()
        .beamType(beam::RequestBeamType::CONFIRMATION)
        .transmitTime(100ms)
        .requestId(20)
        .direction_ant(makeDirection())
        .build();

    const beam::BeamRequest request(command);

    EXPECT_EQ(request.beamType(), beam::RequestBeamType::CONFIRMATION);
    EXPECT_EQ(request.transmitTime(), 100ms);
    EXPECT_EQ(request.requestId(), 20u);
    EXPECT_EQ(request.direction_ant().azimuth, 12.5_deg);
    EXPECT_EQ(request.direction_ant().elevation, 20.5_deg);
}

TEST(BeamRequestTest, ConversionFromCommandAppliesSameValidation)
{
    const beam::RequestBeamCommand zeroId(beam::RequestBeamType::TRACKING, 0ms, 0, makeDirection());
    const beam::RequestBeamCommand negativeTime(beam::RequestBeamType::TRACKING, -10ms, 1, makeDirection());
    const beam::RequestBeamCommand unknownType(static_cast<beam::RequestBeamType>(9), 0ms, 1, makeDirection());

    EXPECT_THROW(beam::BeamRequest{zeroId}, std::invalid_argument);
    EXPECT_THROW(beam::BeamRequest{negativeTime}, std::invalid_argument);
    EXPECT_THROW(beam::BeamRequest{unknownType}, std::invalid_argument);
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
