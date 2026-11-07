#include <network/socket/socket.hpp>

#include <network/socket_error/socket_error.hpp>

#include <system_error>
#include <utility>

namespace comm
{

Socket::Socket()
{
    try
    {
        initializeApi();
        _handle = native_socket::createTcpSocket();
        validate();
    }
    catch (...)
    {
        close();
        cleanupApi();
        throw;
    }
}

Socket::Socket(SocketHandle handle)
  : _handle(handle)
{
    try
    {
        initializeApi();
        validate();
    }
    catch (...)
    {
        close();
        cleanupApi();
        throw;
    }
}

Socket::~Socket()
{
    close();
    cleanupApi();
}

Socket::Socket(Socket&& other) noexcept
  : _handle(std::exchange(other._handle, INVALID_SOCKET_HANDLE)),
    _apiInitialized(std::exchange(other._apiInitialized, false))
{
}

Socket& Socket::operator=(Socket&& other) noexcept
{
    if (this != &other)
    {
        close();
        cleanupApi();
        _handle = std::exchange(other._handle, INVALID_SOCKET_HANDLE);
        _apiInitialized = std::exchange(other._apiInitialized, false);
    }
    return *this;
}

SocketHandle Socket::getHandle() const noexcept
{
    return _handle;
}

void Socket::close() noexcept
{
    if (_handle == INVALID_SOCKET_HANDLE)
    {
        return;
    }

    const SocketHandle handle = std::exchange(_handle, INVALID_SOCKET_HANDLE);
    native_socket::closeSocket(handle);
}

void Socket::initializeApi()
{
    const int errorCode = native_socket::startup();
    if (errorCode != 0)
    {
        const SocketError error(errorCode);
        throw std::system_error(error.code(), "socket-api-startup");
    }
    _apiInitialized = true;
}

void Socket::cleanupApi() noexcept
{
    if (std::exchange(_apiInitialized, false))
    {
        native_socket::cleanup();
    }
}

void Socket::validate() const
{
    if (_handle == INVALID_SOCKET_HANDLE)
    {
        const SocketError error = SocketError::captureLast();
        throw std::system_error(error.code(), "create-socket");
    }
}

} // namespace comm
