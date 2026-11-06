#include <network/socket_resource/socket_resource.hpp>

#include <network/socket_error/socket_error.hpp>

#include <system_error>
#include <utility>

namespace comm
{

SocketResource::SocketResource()
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

SocketResource::SocketResource(SocketHandle handle)
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

SocketResource::~SocketResource()
{
    close();
    cleanupApi();
}

SocketResource::SocketResource(SocketResource&& other) noexcept
    : _handle(std::exchange(other._handle, INVALID_SOCKET_HANDLE))
{
    _apiInitialized = std::exchange(other._apiInitialized, false);
}

SocketResource& SocketResource::operator=(SocketResource&& other) noexcept
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

SocketHandle SocketResource::getHandle() const noexcept
{
    return _handle;
}

void SocketResource::initializeApi()
{
    const int errorCode = native_socket::startup();
    if (errorCode != 0)
    {
        const SocketError error(errorCode);
        throw std::system_error(error.code(), "socket-api-startup");
    }
    _apiInitialized = true;
}

void SocketResource::cleanupApi() noexcept
{
    if (std::exchange(_apiInitialized, false))
    {
        native_socket::cleanup();
    }
}

void SocketResource::validate() const
{
    if (_handle == INVALID_SOCKET_HANDLE)
    {
        const SocketError error = SocketError::captureLast();
        throw std::system_error(error.code(), "create-socket");
    }
}

void SocketResource::close() noexcept
{
    if (_handle == INVALID_SOCKET_HANDLE)
    {
        return;
    }

    const SocketHandle handle = std::exchange(_handle, INVALID_SOCKET_HANDLE);
    native_socket::closeSocket(handle);
}

} // namespace comm
