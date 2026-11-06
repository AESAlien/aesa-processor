#include <network/session_listener/session_listener.hpp>

#include <comm/error/comm_error.hpp>
#include <network/socket_selector/socket_selector.hpp>

#include <algorithm>
#include <string>
#include <system_error>
#include <utility>
#include <vector>

namespace comm
{

SessionListener::SessionListener(const CommEndpoint& endpoint)
  : _listenSocket(endpoint)
{
}

std::shared_ptr<Session> SessionListener::accept()
{
    try
    {
        auto socket = _listenSocket.accept();
        if (!socket)
        {
            return nullptr;
        }
        auto session = std::make_shared<Session>(std::move(*socket));
        _sessions.push_back(session);
        return session;
    }
    catch (const std::system_error& error)
    {
        throw ConnectionError(
            std::string("Communication connection could not be established: ") +
            error.what() + ", systemError=" + std::to_string(error.code().value()));
    }
}

std::shared_ptr<Session> SessionListener::poll(std::chrono::milliseconds timeout)
{
    struct WatchedSocket
    {
        const Socket* socket;
        std::shared_ptr<Session> session;
    };

    std::vector<WatchedSocket> watchedSockets;
    std::vector<const Socket*> sockets;
    watchedSockets.reserve(_sessions.size() + 1);
    sockets.reserve(_sessions.size() + 1);
    watchedSockets.push_back({&_listenSocket, nullptr});
    sockets.push_back(&_listenSocket);

    auto session = _sessions.begin();
    while (session != _sessions.end())
    {
        if (auto snapshot = session->lock())
        {
            const Socket* socket = &snapshot->getSocket();
            watchedSockets.push_back({socket, std::move(snapshot)});
            sockets.push_back(socket);
            ++session;
        }
        else
        {
            session = _sessions.erase(session);
        }
    }

    std::vector<const Socket*> ready;
    try
    {
        const SocketSelector selector(std::move(sockets));
        ready = selector.waitReadable(timeout);
        if (ready.empty())
        {
            return nullptr;
        }
    }
    catch (const std::system_error& error)
    {
        throw ReceiveError(
            std::string("Communication data could not be received: operation=wait-readable, systemError=") +
            std::to_string(error.code().value()));
    }

    const std::size_t startIndex = _nextReadyIndex % watchedSockets.size();
    for (std::size_t offset = 0; offset < watchedSockets.size(); ++offset)
    {
        const std::size_t index = (startIndex + offset) % watchedSockets.size();
        const WatchedSocket& watched = watchedSockets[index];
        if (std::find(ready.begin(), ready.end(), watched.socket) != ready.end())
        {
            _nextReadyIndex = (index + 1) % watchedSockets.size();
            return watched.session ? watched.session : accept();
        }
    }
    return nullptr;
}

} // namespace comm
