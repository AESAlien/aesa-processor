#include <network/socket_error/socket_error.hpp>

#ifdef _WIN32
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif

#ifndef NOMINMAX
#define NOMINMAX
#endif

#include <winsock2.h>
#else
#include <cerrno>
#endif

namespace comm
{

SocketError SocketError::captureLast() noexcept
{
#ifdef _WIN32
    return SocketError(WSAGetLastError());
#else
    return SocketError(errno);
#endif
}

SocketError::SocketError(int errorCode) noexcept
  : _code(errorCode, std::system_category())
{
}

bool SocketError::isInterrupted() const noexcept
{
#ifdef _WIN32
    return _code.value() == WSAEINTR;
#else
    return _code.value() == EINTR;
#endif
}

bool SocketError::isWouldBlock() const noexcept
{
#ifdef _WIN32
    return _code.value() == WSAEWOULDBLOCK;
#else
    return _code.value() == EAGAIN || _code.value() == EWOULDBLOCK;
#endif
}

std::error_code SocketError::code() const noexcept
{
    return _code;
}

} // namespace comm
