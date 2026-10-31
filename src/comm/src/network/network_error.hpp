#pragma once

#include <string>

namespace comm
{

// Throws Error with a message such as
// "<message>: operation=bind, systemError=10048".
template <typename Error>
[[noreturn]] void throwSystemError(
    const std::string& message,
    const char* operation,
    int errorCode
)
{
    throw Error(
        message +
        ": operation=" + operation +
        ", systemError=" + std::to_string(errorCode));
}

} // namespace comm
