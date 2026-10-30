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

// 빔 요청은 전달받은 종류와 폭 및 식별 정보와 방향을 보관한다.
TEST(BeamRequestTest, KeepsGivenValues)
{
    // Given: 요청의 빔 폭과 방향을 준비한다.
    const auto width = makeBeamWidth(2_deg, 3_deg);
    const auto direction = makeDirection();

    // When: 각 요청 속성을 지정해 빔 요청을 생성한다.
    const beam::BeamRequest request(beam::BeamType::TRACKING, width, 0, 1500ms, 7, direction);

    // Then: 빔 요청은 전달받은 종류와 폭 및 식별 정보와 방향을 보관한다.
    EXPECT_EQ(request.transmitBeamType(), beam::BeamType::TRACKING);
    EXPECT_EQ(request.beamWidth().azimuth, 2_deg);
    EXPECT_EQ(request.beamWidth().elevation, 3_deg);
    EXPECT_EQ(request.priority(), 0);
    EXPECT_EQ(request.transmitTime(), 1500ms);
    EXPECT_EQ(request.requestId(), 7u);
    EXPECT_TRUE(request.direction_enu().azimuth.isEquivalentTo(12.5_deg));
    EXPECT_EQ(request.direction_enu().elevation, 20.5_deg);
}

// 복사된 빔 요청은 원래 요청의 값을 유지한다.
TEST(BeamRequestTest, CopyKeepsSameValues)
{
    // Given: 복사할 확인 빔 요청을 준비한다.
    const beam::BeamRequest request(beam::BeamType::CONFIRMATION, makeBeamWidth(4_deg, 5_deg), 1, 100ms, 3, makeDirection());

    // When: 요청 객체를 복사한다.
    const beam::BeamRequest copy = request;

    // Then: 복사된 빔 요청은 원래 요청의 값을 유지한다.
    EXPECT_EQ(copy.transmitBeamType(), request.transmitBeamType());
    EXPECT_EQ(copy.beamWidth(), request.beamWidth());
    EXPECT_EQ(copy.priority(), request.priority());
    EXPECT_EQ(copy.requestId(), request.requestId());
}

// 빔 요청은 송신 시각 0과 최대 요청 ID를 허용한다.
TEST(BeamRequestTest, AcceptsBoundaryValues)
{
    // Given: 생성자에 전달할 입력을 준비한다.
    const auto width = makeBeamWidth(3_deg, 3_deg);
    const auto direction = makeDirection();
    const auto transmitTime = 0ms;
    const std::uint32_t requestId = std::numeric_limits<std::uint32_t>::max();

    // When: 준비한 값으로 대상 객체 생성을 시도한다.
    const auto construct = [&] {
        return beam::BeamRequest(beam::BeamType::TRACKING, width, 0, transmitTime, requestId, direction);
    };

    // Then: 빔 요청은 송신 시각 0과 최대 요청 ID를 허용한다.
    EXPECT_NO_THROW(construct());
}

// 빔 요청은 음수 송신 시각을 거부한다.
TEST(BeamRequestTest, RejectsNegativeTransmitTime)
{
    // Given: 생성자에 전달할 입력을 준비한다.
    const auto width = makeBeamWidth(3_deg, 3_deg);
    const auto direction = makeDirection();
    const auto transmitTime = -1ms;
    const std::uint32_t requestId = 1;

    // When: 준비한 값으로 대상 객체 생성을 시도한다.
    const auto construct = [&] {
        return beam::BeamRequest(beam::BeamType::TRACKING, width, 0, transmitTime, requestId, direction);
    };

    // Then: 빔 요청은 음수 송신 시각을 거부한다.
    EXPECT_THROW(construct(), std::invalid_argument);
}

// 빔 요청은 요청 ID 0을 거부한다.
TEST(BeamRequestTest, RejectsZeroRequestId)
{
    // Given: 생성자에 전달할 입력을 준비한다.
    const auto width = makeBeamWidth(3_deg, 3_deg);
    const auto direction = makeDirection();
    const auto transmitTime = 0ms;
    const std::uint32_t requestId = 0;

    // When: 준비한 값으로 대상 객체 생성을 시도한다.
    const auto construct = [&] {
        return beam::BeamRequest(beam::BeamType::TRACKING, width, 0, transmitTime, requestId, direction);
    };

    // Then: 빔 요청은 요청 ID 0을 거부한다.
    EXPECT_THROW(construct(), std::invalid_argument);
}
