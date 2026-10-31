#include <network/tcp_server/tcp_server.hpp>

#include <comm/error/comm_error.hpp>
#include <network/network_error.hpp>

namespace comm
{
namespace
{

constexpr int LISTEN_BACKLOG = 2;
constexpr char OPEN_FAILURE[] = "Communication server could not be opened";
constexpr char ACCEPT_FAILURE[] = "Communication connection could not be established";

} // namespace

TcpServer::TcpServer(const Ipv4Endpoint& endpoint)
{
    const int startupError = native_socket::startup();
    if (startupError != 0)
    {
        throwSystemError<ServerOpenError>(
            OPEN_FAILURE,
            "socket-api-startup",
            startupError);
    }

    _listenSocket = native_socket::createTcpSocket();
    if (_listenSocket == INVALID_SOCKET_HANDLE)
    {
        failOpen("create-socket");
    }

    if (native_socket::setReuseAddress(_listenSocket) < 0)
    {
        failOpen("set-socket-option");
    }

    if (native_socket::bindSocket(_listenSocket, endpoint.address, endpoint.port) < 0)
    {
        failOpen("bind");
    }

    if (native_socket::listenSocket(_listenSocket, LISTEN_BACKLOG) < 0)
    {
        failOpen("listen");
    }

    if (native_socket::setNonBlocking(_listenSocket) < 0)
    {
        failOpen("set-socket-option");
    }
}

TcpServer::~TcpServer()
{
    native_socket::closeSocket(_listenSocket);
    native_socket::cleanup();
}

SocketHandle TcpServer::socket() const noexcept
{
    return _listenSocket;
}

std::optional<AcceptedSocket> TcpServer::accept()
{
    AcceptedSocket accepted{};
    accepted.socket = native_socket::acceptSocket(
        _listenSocket,
        accepted.address,
        accepted.port);

    if (accepted.socket == INVALID_SOCKET_HANDLE)
    {
        const int errorCode = native_socket::lastError();
        if (native_socket::isWouldBlock(errorCode))
        {
            return std::nullopt;
        }

        throwSystemError<ConnectionError>(ACCEPT_FAILURE, "accept", errorCode);
    }

    if (native_socket::setNonBlocking(accepted.socket) < 0)
    {
        const int errorCode = native_socket::lastError();
        native_socket::closeSocket(accepted.socket);
        throwSystemError<ConnectionError>(
            ACCEPT_FAILURE,
            "set-socket-option",
            errorCode);
    }

    return accepted;
}

// The destructor does not run when the constructor throws,
// so release what the constructor acquired before throwing.
void TcpServer::failOpen(const char* operation)
{
    const int errorCode = native_socket::lastError();
    if (_listenSocket != INVALID_SOCKET_HANDLE)
    {
        native_socket::closeSocket(_listenSocket);
    }
    native_socket::cleanup();

    throwSystemError<ServerOpenError>(OPEN_FAILURE, operation, errorCode);
}

} // namespace comm
