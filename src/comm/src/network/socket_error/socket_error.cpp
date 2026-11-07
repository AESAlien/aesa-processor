#include <network/socket_error/socket_error.hpp>

#include <network/native_socket/native_socket.hpp>

namespace comm
{

SocketError SocketError::captureLast() noexcept
{
    return SocketError(native_socket::lastError());
}

SocketError::SocketError(int errorCode) noexcept
  : _code(errorCode, std::system_category())
{
}

bool SocketError::isInterrupted() const noexcept
{
    return native_socket::isInterrupted(_code.value());
}

bool SocketError::isWouldBlock() const noexcept
{
    return native_socket::isWouldBlock(_code.value());
}

std::error_code SocketError::code() const noexcept
{
    return _code;
}

} // namespace comm
