#include <network/tcp_server/tcp_server.hpp>

#include <comm/error/comm_error.hpp>

#include <string>
#include <system_error>

namespace comm
{
namespace
{

constexpr char OPEN_FAILURE[] = "Communication server could not be opened";
constexpr char ACCEPT_FAILURE[] = "Communication connection could not be established";

} // namespace

TcpServer::TcpServer(const CommEndpoint& endpoint)
try
{
    _listenSocket.setReuseAddress();
    _listenSocket.bind(endpoint);
    _listenSocket.listen();
    _listenSocket.setNonBlocking();
}
catch (const std::system_error& error)
{
    throw ServerOpenError(
        std::string(OPEN_FAILURE) + ": " + error.what() +
        ", systemError=" + std::to_string(error.code().value()));
}

const TcpSocket& TcpServer::getSocket() const noexcept
{
    return _listenSocket;
}

std::optional<TcpSocket> TcpServer::accept()
{
    try
    {
        std::optional<TcpSocket> accepted = _listenSocket.accept();
        if (accepted)
        {
            accepted->setNonBlocking();
        }
        return accepted;
    }
    catch (const std::system_error& error)
    {
        throw ConnectionError(
            std::string(ACCEPT_FAILURE) + ": " + error.what() +
            ", systemError=" + std::to_string(error.code().value()));
    }
}

} // namespace comm
