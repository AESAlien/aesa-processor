#pragma once
#include <attitude/dto/AttitudeDto.hpp>
#include <cstdint>
#include <array>

namespace attitude
{

struct AttTransformDto
{
    dto::AttitudeDto att{};
    std::int64_t rx_time_ns = 0;

    // 안테나 -> ENU 회전 행렬 (3x3, 행 우선: 인덱스 = 행*3 + 열)
    // v_enu = R * v_ant (열벡터 오른쪽 곱), 역변환은 전치
    std::array<double,9> rot_ant_to_enu{};

    // valid == false 이면 무의미. 축, 각도 규약은 <문서 위치> 참조
    bool valid = false;
    std::uint32_t update_seq = 0;
};

}   // namespace attitude