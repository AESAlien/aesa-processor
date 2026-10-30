#include <beam/dto/beam_type.hpp>

#include <gtest/gtest.h>

// 빔 종류의 와이어 값과 요청 빔 종류의 값이 일치한다.
TEST(BeamTypeTest, RequestBeamTypeValuesMatchBeamType)
{
    // Given: 송신 빔과 요청 빔의 각 종류를 준비한다.
    const beam::BeamType types[] = {
        beam::BeamType::SEARCH, beam::BeamType::CONFIRMATION, beam::BeamType::TRACKING};
    const beam::RequestBeamType requestTypes[] = {
        beam::RequestBeamType::CONFIRMATION, beam::RequestBeamType::TRACKING};

    // When: 각 종류를 와이어로 전송하는 정수 값으로 변환한다.
    const int search = static_cast<int>(types[0]);
    const int confirmation = static_cast<int>(types[1]);
    const int tracking = static_cast<int>(types[2]);
    const int requestConfirmation = static_cast<int>(requestTypes[0]);
    const int requestTracking = static_cast<int>(requestTypes[1]);

    // Then: 약속된 와이어 값과 요청 빔의 값이 일치한다.
    EXPECT_EQ(search, 1);
    EXPECT_EQ(confirmation, 2);
    EXPECT_EQ(tracking, 3);
    EXPECT_EQ(requestConfirmation, confirmation);
    EXPECT_EQ(requestTracking, tracking);
}
