#pragma once

#include <string>

namespace comm
{

// 다음과 같은 메시지를 담은 Error를 던진다.
// "<message>: operation=bind, systemError=10048".
template <typename Error>
[[noreturn]] void throwSystemError(
    const std::string& message,
    const char* operation,
    int errorCode
) {
    throw Error(
        message +
        ": operation=" + operation +
        ", systemError=" + std::to_string(errorCode)
    );
}

} // namespace comm
