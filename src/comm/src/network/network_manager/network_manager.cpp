#include <network/network_manager/network_manager.hpp>

#include <comm/error/comm_error.hpp>
#include <network/network_error.hpp>

#include <algorithm>
#include <iostream>
#include <stdexcept>
#include <utility>

namespace comm
{
namespace
{

constexpr char RECEIVE_FAILURE[] = "Communication data could not be received";

// Throws std::invalid_argument for an invalid endpoint.
Ipv4Endpoint parseEndpoint(const NetworkEndpoint& endpoint);

// Returns nullopt for an endpoint with neither address nor port, which
// disables the peer. Throws std::invalid_argument for an invalid endpoint.
std::optional<Ipv4Endpoint> parsePeerEndpoint(const NetworkEndpoint& endpoint);

bool matches(
    const std::optional<Ipv4Endpoint>& endpoint,
    const AcceptedSocket& accepted
);

bool contains(
    const std::vector<SocketHandle>& sockets,
    SocketHandle socket
);

void reportRejectedConnection(const AcceptedSocket& accepted);

} // namespace


/********************************network manager 구현부****************************************** */

NetworkManager::NetworkManager(
    NetworkEndpoint serverEndpoint,
    NetworkEndpoint operatorConsoleEndpoint,
    NetworkEndpoint scenarioSimulatorEndpoint
) : _serverEndpoint(parseEndpoint(serverEndpoint)),
    _operatorConsoleEndpoint(parsePeerEndpoint(operatorConsoleEndpoint)),
    _scenarioSimulatorEndpoint(parsePeerEndpoint(scenarioSimulatorEndpoint))
{
    validatePeers();
}

void NetworkManager::start()
{
    if (_server)
    {
        return;
    }

    _server = std::make_unique<TcpServer>(_serverEndpoint);
}

std::optional<ReceivedFrames> NetworkManager::receiveFrames(
    std::chrono::milliseconds timeout
)
{
    if (!_server)
    {
        throw ReceiveError(
            std::string(RECEIVE_FAILURE) + ": server is not started");
    }

    if (timeout.count() < 0)
    {
        throw std::invalid_argument("Receive timeout must not be negative");
    }

    
    //session pointer는 락을 잡고 접근(sender receive 공통)
    const std::shared_ptr<Session> operatorConsole =
        getSession(NetworkPeer::OPERATOR_CONSOLE);
    const std::shared_ptr<Session> scenarioSimulator =
        getSession(NetworkPeer::SCENARIO_SIMULATOR);

    std::vector<SocketHandle> sockets{_server->socket()};
    if (operatorConsole)
    {
        sockets.push_back(operatorConsole->socket());
    }
    if (scenarioSimulator)
    {
        sockets.push_back(scenarioSimulator->socket());
    }

    std::vector<SocketHandle> readySockets;
    const int selectedCount =
        native_socket::waitReadable(sockets, timeout, readySockets); //select

    if (selectedCount < 0)
    {
        const int errorCode = native_socket::lastError();
        if (native_socket::isInterrupted(errorCode))
        {
            return std::nullopt;
        }

        throwSystemError<ReceiveError>(RECEIVE_FAILURE, "select", errorCode);
    }

    if (contains(readySockets, _server->socket()))
    {
        acceptSession();
        return std::nullopt;
    }

    const bool operatorConsoleReady =
        operatorConsole && contains(readySockets, operatorConsole->socket());
    const bool scenarioSimulatorReady =
        scenarioSimulator && contains(readySockets, scenarioSimulator->socket());

    //console, simulator 가 모두 ready일 때, 이전에 선택하지 않은 소켓을 선택
    if (operatorConsoleReady &&
        (!scenarioSimulatorReady ||
         _nextPeer == NetworkPeer::OPERATOR_CONSOLE))
    {
        _nextPeer = NetworkPeer::SCENARIO_SIMULATOR;
        return receiveFrom(NetworkPeer::OPERATOR_CONSOLE, operatorConsole);
    }

    if (scenarioSimulatorReady)
    {
        _nextPeer = NetworkPeer::OPERATOR_CONSOLE;
        return receiveFrom(NetworkPeer::SCENARIO_SIMULATOR, scenarioSimulator);
    }

    return std::nullopt;
}

void NetworkManager::send(
    NetworkPeer peer,
    const std::vector<uint8_t>& data,
    std::chrono::milliseconds timeout
)
{
    if (data.empty() || timeout.count() < 0)
    {
        throw std::invalid_argument(
            "Send requires data and a non-negative timeout");
    }

    // The copy keeps the session alive even if it is replaced meanwhile.
    const std::shared_ptr<Session> session = getSession(peer);
    if (!session)
    {
        return;
    }

    session->send(data, timeout);
}

void NetworkManager::validatePeers() const
{
    if (!_operatorConsoleEndpoint.has_value() && !_scenarioSimulatorEndpoint.has_value())
    {
        throw std::invalid_argument("At least one peer must be enabled");
    }

    if (_operatorConsoleEndpoint.has_value() &&
        _scenarioSimulatorEndpoint.has_value() &&
        _operatorConsoleEndpoint->address == _scenarioSimulatorEndpoint->address &&
        _operatorConsoleEndpoint->port == _scenarioSimulatorEndpoint->port)
    {
        throw std::invalid_argument("Peer endpoints must be distinct");
    }
}

std::optional<NetworkPeer> NetworkManager::identifyPeer(
    const AcceptedSocket& accepted
) const
{
    if (matches(_operatorConsoleEndpoint, accepted))
    {
        return NetworkPeer::OPERATOR_CONSOLE;
    }

    if (matches(_scenarioSimulatorEndpoint, accepted))
    {
        return NetworkPeer::SCENARIO_SIMULATOR;
    }

    return std::nullopt;
}

void NetworkManager::acceptSession()
{
    const std::optional<AcceptedSocket> accepted = _server->accept();
    if (!accepted.has_value())
    {
        return;
    }

    const std::optional<NetworkPeer> peer = identifyPeer(*accepted);
    if (!peer.has_value())
    {
        reportRejectedConnection(*accepted);
        native_socket::closeSocket(accepted->socket);
        return;
    }

    // A reconnection replaces the previous session, which closes its socket
    // once no send() is using it anymore.
    setSession(*peer, std::make_shared<Session>(accepted->socket, *peer));
}

std::optional<ReceivedFrames> NetworkManager::receiveFrom(
    NetworkPeer peer,
    const std::shared_ptr<Session>& session
)
{
    ReceivedFrames received{peer, {}};
    try
    {
        received.frames = session->receive();
    }
    //recv < 0 receive error 또는 framing error
    catch (const ReceiveError&)
    {
        if (!session->isOpen())
        {
            setSession(peer, nullptr);
        }
        throw;
    }

    //recv = 0 상대방 정상 연결 종료
    if (!session->isOpen())
    {
        setSession(peer, nullptr);
    }

    //recv = 0 또는 수신 byte < frame size 또는 would block
    if (received.frames.empty())
    {
        return std::nullopt;
    }

    return received;
}

std::shared_ptr<Session> NetworkManager::getSession(NetworkPeer peer) const
{
    std::lock_guard<std::mutex> lock(_sessionMutex);
    switch (peer)
    {
    case NetworkPeer::OPERATOR_CONSOLE:
        return _operatorConsoleSession;

    case NetworkPeer::SCENARIO_SIMULATOR:
        return _scenarioSimulatorSession;
    }

    return nullptr;
}

// peer의 Session을 교체한다. 재연결이면 새 Session, 연결 종료면 nullptr를 넘긴다.
void NetworkManager::setSession(
    NetworkPeer peer,
    std::shared_ptr<Session> session
)
{
    // 교체되어 나가는 이전 Session을 락 밖에서 해제하기 위해 잠시 보관한다.
    std::shared_ptr<Session> previousSession;
    {
        // send()의 getSession()이 포인터를 복사하는 것과 겹치지 않도록 보호한다.
        std::lock_guard<std::mutex> lock(_sessionMutex);

        // std::exchange: 멤버에 새 Session을 넣고, 원래 있던 Session을 반환한다.
        switch (peer)
        {
        case NetworkPeer::OPERATOR_CONSOLE:
            previousSession = std::exchange(
                _operatorConsoleSession,
                std::move(session));
            break;

        case NetworkPeer::SCENARIO_SIMULATOR:
            previousSession = std::exchange(
                _scenarioSimulatorSession,
                std::move(session));
            break;
        }
    } // 여기서 락이 풀린다.

    // 함수가 끝나면 previousSession의 참조가 하나 줄어든다.
    // - 참조가 0이 되면 ~Session()이 실행되어 소켓을 닫는다.
    //   소켓을 닫는 시스템 호출이 락 밖에서 실행되므로 send()를 막지 않는다.
    // - 송신 스레드가 send() 중이라 복사본을 들고 있으면 아직 해제되지 않고,
    //   그 send()가 끝나 복사본이 사라질 때 소켓이 닫힌다.
}

/******************************************************************************************* */

namespace
{

Ipv4Endpoint parseEndpoint(const NetworkEndpoint& endpoint)
{
    const std::optional<uint32_t> address =
        native_socket::parseIpv4(endpoint.address);
    if (!address.has_value() || endpoint.port == 0)
    {
        throw std::invalid_argument(
            "Endpoint requires a valid IPv4 address and port");
    }

    return Ipv4Endpoint{*address, endpoint.port};
}

std::optional<Ipv4Endpoint> parsePeerEndpoint(const NetworkEndpoint& endpoint)
{
    if (endpoint.address.empty() && endpoint.port == 0)
    {
        return std::nullopt;
    }

    return parseEndpoint(endpoint);
}

bool matches(
    const std::optional<Ipv4Endpoint>& endpoint,
    const AcceptedSocket& accepted
)
{
    return endpoint.has_value() &&
        endpoint->address == accepted.address &&
        endpoint->port == accepted.port;
}

bool contains(
    const std::vector<SocketHandle>& sockets,
    SocketHandle socket
)
{
    return std::find(sockets.begin(), sockets.end(), socket) != sockets.end();
}


//올바르지 않은 주소에서 연결을 시도했을 때
void reportRejectedConnection(const AcceptedSocket& accepted)
{
    std::cerr
        << "[NetworkManager] Rejected unknown connection: remoteAddress="
        << native_socket::formatIpv4(accepted.address).value_or("unknown")
        << ", remotePort="
        << accepted.port
        << '\n';
}

} // namespace
} // namespace comm
