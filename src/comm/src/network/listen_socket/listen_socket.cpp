#include <network/listen_socket/listen_socket.hpp>

#include <comm/error/comm_error.hpp>
#include <network/socket_error/socket_error.hpp>

#include <string>
#include <system_error>

namespace comm
{
namespace
{

void checkResult(int result, const char* operation)
{
    if (result < 0)
    {
        const SocketError error = SocketError::captureLast();
        throw std::system_error(error.code(), operation);
    }
}

} // namespace

ListenSocket::ListenSocket(const CommEndpoint& endpoint)
  : ListenSocket(endpoint, native_socket::defaultBacklog())
{
}

ListenSocket::ListenSocket(const CommEndpoint& endpoint, int backlog)
try
{
    checkResult(native_socket::setReuseAddress(_handle), "set-reuse-address");
    checkResult(native_socket::bindSocket(
        _handle, endpoint.getIntAddress(), endpoint.getPort()
    ), "bind"
    );
    checkResult(native_socket::listenSocket(_handle, backlog), "listen");
    checkResult(native_socket::setNonBlocking(_handle), "set-non-blocking");
}
catch (const std::system_error& error)
{
    throw ServerOpenError(
        std::string("Communication server could not be opened: ") + error.what() +
        ", systemError=" + std::to_string(error.code().value())
    );
}

std::optional<ConnectedSocket> ListenSocket::accept()
{
    try
    {
        uint32_t remoteAddress = 0;
        uint16_t remotePort = 0;
        const SocketHandle handle = native_socket::acceptSocket(_handle, remoteAddress, remotePort);
        if (handle == INVALID_SOCKET_HANDLE)
        {
            const SocketError error = SocketError::captureLast();
            if (error.isWouldBlock() || error.isInterrupted())
            {
                return std::nullopt;
            }
            throw std::system_error(error.code(), "accept");
        }
        return ConnectedSocket(handle, remoteAddress, remotePort);
    }
    catch (const std::system_error& error)
    {
        throw ConnectionError(
            std::string("Communication connection could not be established: ") + error.what() +
            ", systemError=" + std::to_string(error.code().value())
        );
    }
}

} // namespace comm
