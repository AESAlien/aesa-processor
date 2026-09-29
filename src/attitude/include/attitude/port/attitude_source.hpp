#pragma once
#include <attitude/dto/attitude_dto.hpp>
#include <chrono>
#include <cstdint>

namespace attitude
{

enum class AttQueStatus : std::uint8_t { Success, Timeout, Closed };

class AttitudePort
{
public:
    virtual ~AttitudePort() = default;
    
    virtual AttQueStatus read(AttitudeDto& out_msg, 
        std::int64_t& out_rx_time_ns, std::chrono::milliseconds timeout) = 0;
};

}   // namespace attitude