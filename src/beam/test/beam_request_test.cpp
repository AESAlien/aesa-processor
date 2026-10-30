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

beam::BeamWidth makeBeamWidth(math::Angle azimuth, math::Angle elevation)
{
    return beam::BeamWidth(azimuth, elevation);
}

} // namespace

TEST(BeamRequestTest, KeepsGivenValues)
{
    const beam::BeamRequest request(beam::BeamType::TRACKING, makeBeamWidth(2_deg, 3_deg), 0, 1500ms, 7, makeDirection());

    EXPECT_EQ(request.transmitBeamType(), beam::BeamType::TRACKING);
    EXPECT_EQ(request.beamWidth().azimuth, 2_deg);
    EXPECT_EQ(request.beamWidth().elevation, 3_deg);
    EXPECT_EQ(request.priority(), 0);
    EXPECT_EQ(request.transmitTime(), 1500ms);
    EXPECT_EQ(request.requestId(), 7u);
    EXPECT_TRUE(request.direction_enu().azimuth.isEquivalentTo(12.5_deg));
    EXPECT_EQ(request.direction_enu().elevation, 20.5_deg);
}

TEST(BeamRequestTest, CopyKeepsSameValues)
{
    const beam::BeamRequest request(beam::BeamType::CONFIRMATION, makeBeamWidth(4_deg, 5_deg), 1, 100ms, 3, makeDirection());
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
        beam::BeamType::TRACKING,
        makeBeamWidth(3_deg, 3_deg),
        0,
        0ms,
        std::numeric_limits<std::uint32_t>::max(),
        makeDirection()));
}

TEST(BeamRequestTest, RejectsNegativeTransmitTime)
{
    EXPECT_THROW(
        beam::BeamRequest(beam::BeamType::TRACKING, makeBeamWidth(3_deg, 3_deg), 0, -1ms, 1, makeDirection()),
        std::invalid_argument);
}

TEST(BeamRequestTest, RejectsZeroRequestId)
{
    EXPECT_THROW(
        beam::BeamRequest(beam::BeamType::CONFIRMATION, makeBeamWidth(3_deg, 3_deg), 1, 0ms, 0, makeDirection()),
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
