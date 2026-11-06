#pragma once

#include <system_error>

namespace comm
{

// A snapshot of one native socket error. Queries never read the native error
// state again. Call captureLast() immediately after a failed socket operation.
class SocketError
{
public:
    static SocketError captureLast() noexcept;
    explicit SocketError(int errorCode) noexcept;

    bool isInterrupted() const noexcept;
    bool isWouldBlock() const noexcept;
    std::error_code code() const noexcept;

private:
    std::error_code _code;
};

} // namespace comm
