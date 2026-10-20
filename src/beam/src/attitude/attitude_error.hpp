#pragma once

#include <cstdint>
#include <stdexcept>
#include <string>

namespace aesa_processor::beam
{

enum class AttitudeErrorCode : uint8_t
{
    OUT_OF_RANGE,
    GIMBAL_LOCK
};

class AttitudeError : public std::invalid_argument
{
public:
    AttitudeError(AttitudeErrorCode code, const std::string& message) : std::invalid_argument(message), _code(code)
    {
    }

    AttitudeErrorCode code() const noexcept
    {
        return _code;
    }

private:
    AttitudeErrorCode _code;
};

} // namespace aesa_processor::beam
