#include <network/tcp_server/tcp_server.hpp>

#include <comm/error/comm_error.hpp>
#include <network/socket_selector/socket_selector.hpp>

#include <algorithm>
#include <optional>
#include <string>
#include <system_error>
#include <utility>

namespace comm
{

TcpServer::TcpServer(const CommEndpoint& endpoint)
  : _listenSocket(endpoint)
{
}

ServerEvent TcpServer::poll(
    const std::vector<std::shared_ptr<Session>>& sessions,
    std::chrono::milliseconds timeout
) {
    // 인덱스 0은 리스닝 소켓이고, 나머지는 같은 인덱스의 세션을 가리킨다.
    std::vector<const Socket*> sockets{&_listenSocket};
    std::vector<std::shared_ptr<Session>> watchedSessions{nullptr};
    for (const std::shared_ptr<Session>& session : sessions)
    {
        if (session)
        {
            sockets.push_back(&session->getSocket());
            watchedSessions.push_back(session);
        }
    }

    std::vector<const Socket*> readySockets;
    try
    {
        readySockets = SocketSelector(sockets).waitReadable(timeout);
    }
    catch (const std::system_error& error)
    {
        throw ReceiveError(
            std::string("Communication data could not be received: operation=wait-readable, systemError=") +
            std::to_string(error.code().value())
        );
    }
    if (readySockets.empty())
    {
        return {ServerEventKind::NONE, nullptr};
    }

    const std::size_t startIndex = _nextReadyIndex % sockets.size();
    for (std::size_t offset = 0; offset < sockets.size(); ++offset)
    {
        const std::size_t index = (startIndex + offset) % sockets.size();
        if (std::find(readySockets.begin(), readySockets.end(), sockets[index]) != readySockets.end())
        {
            _nextReadyIndex = (index + 1) % sockets.size();
            if (watchedSessions[index])
            {
                return {ServerEventKind::READABLE, watchedSessions[index]};
            }
            return accept();
        }
    }
    return {ServerEventKind::NONE, nullptr};
}

ServerEvent TcpServer::accept()
{
    try
    {
        std::optional<ConnectedSocket> socket = _listenSocket.accept();
        if (!socket)
        {
            return {ServerEventKind::NONE, nullptr};
        }
        return {ServerEventKind::NEW_SESSION, std::make_shared<Session>(std::move(*socket))};
    }
    catch (const std::system_error& error)
    {
        throw ConnectionError(
            std::string("Communication connection could not be established: ") +
            error.what() + ", systemError=" + std::to_string(error.code().value())
        );
    }
}

} // namespace comm
