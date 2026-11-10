#include <comm/config/comm_endpoint.hpp>
#include <comm/error/comm_error.hpp>
#include <network/network_manager/network_manager.hpp>
#include <network/peer_connection/peer_connection.hpp>
#include <network/session/session.hpp>
#include <network/socket_selector/socket_selector.hpp>
#include <network/socket/socket.hpp>
#include <network/tcp_server/tcp_server.hpp>
#include <network/listen_socket/listen_socket.hpp>
#include <network/connected_socket/connected_socket.hpp>

#include <native_socket/native_socket_test_support.hpp>

#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <cstring>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <system_error>
#include <thread>
#include <type_traits>
#include <utility>
#include <vector>

namespace
{

constexpr std::chrono::seconds SOCKET_WAIT_TIMEOUT{2};
constexpr int BIND_ATTEMPTS = 10;

// 소켓 소유권을 직접 시험하기 위해 기반 클래스의 생성과 이동을 노출한다.
class TestSocket : public comm::Socket
{
public:
    TestSocket() = default;
    TestSocket(TestSocket&&) noexcept = default;
    TestSocket& operator=(TestSocket&&) noexcept = default;
};

static_assert(!std::is_copy_constructible_v<TestSocket>);
static_assert(std::is_nothrow_move_constructible_v<TestSocket>);
static_assert(!std::is_default_constructible_v<comm::ListenSocket>);
static_assert(!std::is_default_constructible_v<comm::ConnectedSocket>);
static_assert(!std::is_copy_constructible_v<comm::ListenSocket>);
static_assert(!std::is_copy_constructible_v<comm::ConnectedSocket>);
static_assert(std::is_nothrow_move_constructible_v<comm::ListenSocket>);
static_assert(std::is_nothrow_move_assignable_v<comm::ListenSocket>);
static_assert(std::is_nothrow_move_constructible_v<comm::ConnectedSocket>);
static_assert(std::is_nothrow_move_assignable_v<comm::ConnectedSocket>);
static_assert(!std::is_constructible_v<comm::Session, comm::ListenSocket, std::string>);
static_assert(!std::is_constructible_v<comm::ConnectedSocket, comm::SocketHandle, uint32_t, uint16_t>);

template <typename T, typename = void>
struct HasAccept : std::false_type
{
};
template <typename T>
struct HasAccept<T, std::void_t<decltype(std::declval<T&>().accept())>> : std::true_type
{
};
template <typename T, typename = void>
struct HasSend : std::false_type
{
};
template <typename T>
struct HasSend<T, std::void_t<decltype(std::declval<T&>().sendBytes(
    std::declval<const std::vector<uint8_t>&>()
))>> : std::true_type
{
};
static_assert(!HasAccept<comm::ConnectedSocket>::value);
static_assert(!HasSend<comm::ListenSocket>::value);
static_assert(!HasSend<TestSocket>::value);

using comm::native_socket::test::NativePeer;
using comm::native_socket::test::SignalInterruptor;

uint16_t availablePort()
{
    // 네이티브 API 검사는 일반 소켓 객체가 유지하는 API 수명 안에서 수행한다.
    TestSocket runtimeOwner;
    NativePeer reservation;
    if (reservation.bind() < 0)
    {
        throw std::runtime_error("Could not reserve a test port");
    }
    return reservation.localPort();
}

comm::ListenSocket openListener(uint16_t& port)
{
    for (int attempt = 0; attempt < BIND_ATTEMPTS; ++attempt)
    {
        port = availablePort();
        try
        {
            return comm::ListenSocket(comm::CommEndpoint("127.0.0.1", port), 2);
        }
        catch (const comm::ServerOpenError&)
        {
            if (attempt == BIND_ATTEMPTS - 1)
            {
                throw;
            }
        }
    }
    throw std::runtime_error("Could not open a test listener");
}

std::unique_ptr<comm::ListenSocket> openServer(uint16_t& port)
{
    return std::make_unique<comm::ListenSocket>(openListener(port));
}

comm::ConnectedSocket connectPeer(NativePeer& client)
{
    uint16_t port = 0;
    auto listener = openListener(port);
    if (client.connect(port) < 0)
    {
        throw std::runtime_error("Could not connect native test peer");
    }
    if (comm::SocketSelector({&listener}).waitReadable(SOCKET_WAIT_TIMEOUT).empty())
    {
        throw std::runtime_error("Native test connection did not become ready");
    }
    auto accepted = listener.accept();
    if (!accepted)
    {
        throw std::runtime_error("Could not accept native test peer");
    }
    return std::move(*accepted);
}

comm::CommConfig makeConfig(
    uint16_t serverPort,
    const NativePeer& operatorConsole,
    const NativePeer& scenarioSimulator
) {
    return comm::CommConfig(
        comm::CommEndpoint("127.0.0.1", serverPort),
        comm::CommEndpoint("127.0.0.1", operatorConsole.localPort()),
        comm::CommEndpoint("127.0.0.1", scenarioSimulator.localPort())
    );
}

// 대기 중인 연결 수락과 종료를 처리한다. 리셋된 연결의 세션은 종료된다.
void pollFor(comm::NetworkManager& manager, std::chrono::milliseconds duration)
{
    const auto deadline = std::chrono::steady_clock::now() + duration;
    while (std::chrono::steady_clock::now() < deadline)
    {
        try
        {
            manager.poll(std::chrono::milliseconds{10});
        }
        catch (const comm::ReceiveError&)
        {
        }
    }
}

TEST(NetworkManagerTest, OwnsConnectionsAndRoutesDestinationSpecificSends)
{
    TestSocket apiLifetime;
    NativePeer console;
    NativePeer simulator;
    ASSERT_EQ(console.bind(), 0);
    ASSERT_EQ(simulator.bind(), 0);
    const uint16_t port = availablePort();
    uint8_t received = 0;
    {
        comm::NetworkManager manager(comm::CommConfig(
            comm::CommEndpoint("127.0.0.1", port),
            comm::CommEndpoint("127.0.0.1", console.localPort()),
            comm::CommEndpoint("127.0.0.1", simulator.localPort())
        ));
        EXPECT_THROW(manager.poll(std::chrono::milliseconds{0}), comm::ReceiveError);
        EXPECT_NO_THROW(manager.sendToOperatorConsole({0x01}));
        EXPECT_NO_THROW(manager.sendToScenarioSimulator({0x02}));
        EXPECT_THROW(manager.sendToOperatorConsole({}), std::invalid_argument);
        EXPECT_THROW(
            manager.sendToScenarioSimulator({0x01}, std::chrono::milliseconds{-1}),
            std::invalid_argument
        );
        manager.start();
        EXPECT_NO_THROW(manager.start());
        ASSERT_EQ(console.connect(port), 0);
        EXPECT_FALSE(manager.poll(SOCKET_WAIT_TIMEOUT));
        ASSERT_EQ(simulator.connect(port), 0);
        EXPECT_FALSE(manager.poll(SOCKET_WAIT_TIMEOUT));

        manager.sendToOperatorConsole({0x31}, SOCKET_WAIT_TIMEOUT);
        manager.sendToScenarioSimulator({0x42}, SOCKET_WAIT_TIMEOUT);
        ASSERT_EQ(console.waitReadable(), 1);
        ASSERT_EQ(console.receiveByte(received), 1);
        EXPECT_EQ(received, 0x31);
        ASSERT_EQ(simulator.waitReadable(), 1);
        ASSERT_EQ(simulator.receiveByte(received), 1);
        EXPECT_EQ(received, 0x42);

        comm::protocol::ST_MsgHeader header{};
        header.message_Id = static_cast<uint16_t>(comm::protocol::MessageId::OPERATOR_CONTROL);
        header.block_size = sizeof(header);
        std::vector<uint8_t> bytes(sizeof(header));
        std::memcpy(bytes.data(), &header, sizeof(header));
        for (uint8_t byte : bytes)
        {
            ASSERT_EQ(console.sendByte(byte), 1);
        }
        std::optional<comm::ReceivedFrames> frames;
        const auto deadline = std::chrono::steady_clock::now() + SOCKET_WAIT_TIMEOUT;
        while (!frames && std::chrono::steady_clock::now() < deadline)
        {
            frames = manager.poll(std::chrono::milliseconds{10});
        }
        ASSERT_TRUE(frames);
        EXPECT_EQ(frames->peerName, "operator-console");
        ASSERT_EQ(frames->frames.size(), 1U);
        EXPECT_EQ(frames->frames[0].bytes, bytes);
    }
    // 관리자가 두 세션을 모두 소유하며, 소멸 시 해제한다.
    ASSERT_EQ(console.waitReadable(), 1);
    EXPECT_EQ(console.receiveByte(received), 0);
    ASSERT_EQ(simulator.waitReadable(), 1);
    EXPECT_EQ(simulator.receiveByte(received), 0);
}

TEST(NetworkManagerTest, PeerClosureDisconnectsOnlyThatPeer)
{
    TestSocket apiLifetime;
    NativePeer console;
    NativePeer simulator;
    ASSERT_EQ(console.bind(), 0);
    ASSERT_EQ(simulator.bind(), 0);
    const uint16_t port = availablePort();
    comm::NetworkManager manager(makeConfig(port, console, simulator));
    manager.start();
    ASSERT_EQ(console.connect(port), 0);
    EXPECT_FALSE(manager.poll(SOCKET_WAIT_TIMEOUT));
    ASSERT_EQ(simulator.connect(port), 0);
    EXPECT_FALSE(manager.poll(SOCKET_WAIT_TIMEOUT));

    console.close();
    EXPECT_FALSE(manager.poll(SOCKET_WAIT_TIMEOUT));
    manager.sendToScenarioSimulator({0x42}, SOCKET_WAIT_TIMEOUT);
    uint8_t received = 0;
    ASSERT_EQ(simulator.waitReadable(), 1);
    ASSERT_EQ(simulator.receiveByte(received), 1);
    EXPECT_EQ(received, 0x42);
}

TEST(NetworkManagerTest, ReconnectionFromSameEndpointReplacesSession)
{
    TestSocket apiLifetime;
    NativePeer previousConsole;
    NativePeer replacementConsole;
    NativePeer simulator;
    ASSERT_EQ(previousConsole.setReuseAddress(), 0);
    ASSERT_EQ(replacementConsole.setReuseAddress(), 0);
    ASSERT_EQ(previousConsole.bind(), 0);
    ASSERT_EQ(simulator.bind(), 0);
    const uint16_t port = availablePort();
    comm::NetworkManager manager(makeConfig(port, previousConsole, simulator));
    manager.start();
    ASSERT_EQ(previousConsole.connect(port), 0);
    EXPECT_FALSE(manager.poll(SOCKET_WAIT_TIMEOUT));

    // 리셋은 TIME_WAIT를 남기지 않으므로 같은 엔드포인트로 다시 연결할 수 있다.
    const uint16_t consolePort = previousConsole.localPort();
    ASSERT_EQ(previousConsole.setAbortiveClose(), 0);
    previousConsole.close();
    ASSERT_EQ(replacementConsole.bind(consolePort), 0);
    ASSERT_EQ(replacementConsole.connect(port), 0);
    pollFor(manager, std::chrono::milliseconds{200});

    manager.sendToOperatorConsole({0x6E}, SOCKET_WAIT_TIMEOUT);
    uint8_t received = 0;
    ASSERT_EQ(replacementConsole.waitReadable(), 1);
    ASSERT_EQ(replacementConsole.receiveByte(received), 1);
    EXPECT_EQ(received, 0x6E);
}

TEST(NetworkManagerTest, RejectsUnknownPeerWithoutReplacingKnownConnection)
{
    TestSocket apiLifetime;
    NativePeer console;
    NativePeer simulator;
    NativePeer unknown;
    ASSERT_EQ(console.bind(), 0);
    ASSERT_EQ(simulator.bind(), 0);
    const uint16_t port = availablePort();
    comm::NetworkManager manager(makeConfig(port, console, simulator));
    manager.start();
    ASSERT_EQ(console.connect(port), 0);
    EXPECT_FALSE(manager.poll(SOCKET_WAIT_TIMEOUT));
    ASSERT_EQ(unknown.connect(port), 0);
    EXPECT_FALSE(manager.poll(SOCKET_WAIT_TIMEOUT));

    uint8_t received = 0;
    ASSERT_EQ(unknown.waitReadable(), 1);
    EXPECT_EQ(unknown.receiveByte(received), 0);
    manager.sendToOperatorConsole({0x51}, SOCKET_WAIT_TIMEOUT);
    ASSERT_EQ(console.waitReadable(), 1);
    ASSERT_EQ(console.receiveByte(received), 1);
    EXPECT_EQ(received, 0x51);
}

TEST(PeerConnectionTest, DisconnectedSendPreservesValidationAndNoOpBehavior)
{
    comm::PeerConnection connection("test-peer", comm::CommEndpoint("127.0.0.1", 4100));
    EXPECT_NO_THROW(connection.send({0x01}));
    EXPECT_THROW(connection.send({}), std::invalid_argument);
    EXPECT_THROW(connection.send({0x01}, std::chrono::milliseconds{-1}), std::invalid_argument);
}

TEST(PeerConnectionTest, MatchesBothAddressAndPort)
{
    comm::PeerConnection connection(
        "operator-console",
        comm::CommEndpoint("127.0.0.1", 4100)
    );

    EXPECT_EQ(connection.getName(), "operator-console");
    EXPECT_FALSE(connection.getSession());
    EXPECT_TRUE(connection.matches(comm::CommEndpoint("127.0.0.1", 4100)));
    EXPECT_FALSE(connection.matches(comm::CommEndpoint("127.0.0.2", 4100)));
    EXPECT_FALSE(connection.matches(comm::CommEndpoint("127.0.0.1", 4101)));
}

TEST(TcpServerTest, ReportsNewConnectionsAsNewSessions)
{
    TestSocket apiLifetime;
    NativePeer firstClient;
    NativePeer secondClient;
    std::shared_ptr<comm::Session> first;
    std::shared_ptr<comm::Session> second;
    {
        const uint16_t port = availablePort();
        comm::TcpServer server(comm::CommEndpoint("127.0.0.1", port));
        EXPECT_EQ(server.poll({}, std::chrono::milliseconds{0}).kind, comm::ServerEventKind::NONE);
        ASSERT_EQ(firstClient.connect(port), 0);
        const comm::ServerEvent firstEvent = server.poll({}, SOCKET_WAIT_TIMEOUT);
        ASSERT_EQ(firstEvent.kind, comm::ServerEventKind::NEW_SESSION);
        first = firstEvent.session;
        ASSERT_TRUE(first);
        EXPECT_EQ(first->getRemoteEndpoint().getPort(), firstClient.localPort());
        EXPECT_EQ(first->getRemoteEndpoint().getAddress(), "127.0.0.1");
        EXPECT_EQ(first->getSocket().getRemoteEndpoint(), first->getRemoteEndpoint());
        ASSERT_EQ(secondClient.connect(port), 0);
        const comm::ServerEvent secondEvent = server.poll({first}, SOCKET_WAIT_TIMEOUT);
        ASSERT_EQ(secondEvent.kind, comm::ServerEventKind::NEW_SESSION);
        second = secondEvent.session;
        ASSERT_NE(first, second);
        EXPECT_EQ(second->getRemoteEndpoint().getPort(), secondClient.localPort());

        const comm::PeerConnection firstPeer(
            "first", comm::CommEndpoint("127.0.0.1", firstClient.localPort())
        );
        EXPECT_TRUE(firstPeer.matches(*first));
        EXPECT_FALSE(firstPeer.matches(*second));

        firstClient.close();
        const comm::ServerEvent readable = server.poll({first, second}, SOCKET_WAIT_TIMEOUT);
        EXPECT_EQ(readable.kind, comm::ServerEventKind::READABLE);
        EXPECT_EQ(readable.session, first);
        EXPECT_TRUE(first->receive().empty());
        EXPECT_FALSE(first->isOpen());
    }
    // 서버를 닫아도 이미 수락한 세션은 유효하다.
    second->send({0x6E}, SOCKET_WAIT_TIMEOUT);
    uint8_t received = 0;
    ASSERT_EQ(secondClient.waitReadable(), 1);
    ASSERT_EQ(secondClient.receiveByte(received), 1);
    EXPECT_EQ(received, 0x6E);
    second.reset();
    ASSERT_EQ(secondClient.waitReadable(), 1);
    EXPECT_EQ(secondClient.receiveByte(received), 0);
}

TEST(TcpServerTest, RotatesBetweenSocketsThatAreReadyAtTheSameTime)
{
    TestSocket apiLifetime;
    NativePeer firstClient;
    NativePeer secondClient;
    const uint16_t port = availablePort();
    comm::TcpServer server(comm::CommEndpoint("127.0.0.1", port));

    ASSERT_EQ(firstClient.connect(port), 0);
    const auto first = server.poll({}, SOCKET_WAIT_TIMEOUT).session;
    ASSERT_TRUE(first);
    ASSERT_EQ(secondClient.connect(port), 0);
    const auto second = server.poll({first}, SOCKET_WAIT_TIMEOUT).session;
    ASSERT_TRUE(second);

    ASSERT_EQ(firstClient.sendByte(0x31), 1);
    ASSERT_EQ(secondClient.sendByte(0x42), 1);
    // 루프백 전달은 send()보다 늦을 수 있으므로 둘 다 읽기 가능해질 때까지 기다린다.
    const comm::SocketSelector selector({&first->getSocket(), &second->getSocket()});
    const auto deadline = std::chrono::steady_clock::now() + SOCKET_WAIT_TIMEOUT;
    while (selector.waitReadable(std::chrono::milliseconds{10}).size() < 2 &&
        std::chrono::steady_clock::now() < deadline)
    {
    }

    const std::vector<std::shared_ptr<comm::Session>> sessions{first, second};
    EXPECT_EQ(server.poll(sessions, std::chrono::milliseconds{0}).session, first);
    EXPECT_EQ(server.poll(sessions, std::chrono::milliseconds{0}).session, second);
}

TEST(TcpServerTest, ReplacedSessionIsNeitherWatchedNorReportedAsNew)
{
    TestSocket apiLifetime;
    NativePeer previousClient;
    NativePeer replacementClient;
    const uint16_t port = availablePort();
    comm::TcpServer server(comm::CommEndpoint("127.0.0.1", port));
    ASSERT_EQ(previousClient.connect(port), 0);
    const comm::ServerEvent previous = server.poll({}, SOCKET_WAIT_TIMEOUT);
    ASSERT_EQ(previous.kind, comm::ServerEventKind::NEW_SESSION);
    ASSERT_EQ(replacementClient.connect(port), 0);
    const comm::ServerEvent replacement = server.poll({previous.session}, SOCKET_WAIT_TIMEOUT);
    ASSERT_EQ(replacement.kind, comm::ServerEventKind::NEW_SESSION);

    // 상대 객체가 이전 세션을 교체한 뒤에도 송신 측은 그 세션을 보유할 수 있다.
    previousClient.close();
    ASSERT_EQ(
        comm::SocketSelector({&previous.session->getSocket()}).waitReadable(SOCKET_WAIT_TIMEOUT).size(),
        1U
    );
    EXPECT_EQ(
        server.poll({replacement.session}, std::chrono::milliseconds{50}).kind,
        comm::ServerEventKind::NONE
    );

    ASSERT_EQ(replacementClient.sendByte(0x31), 1);
    const comm::ServerEvent readable = server.poll({replacement.session}, SOCKET_WAIT_TIMEOUT);
    EXPECT_EQ(readable.kind, comm::ServerEventKind::READABLE);
    EXPECT_EQ(readable.session, replacement.session);
}

TEST(SocketTest, OwnsApiAndSupportsMoveAndRepeatedClose)
{
    TestSocket socket;
    const comm::SocketHandle handle = socket.getHandle();
    ASSERT_NE(handle, comm::INVALID_SOCKET_HANDLE);
    NativePeer client;
    EXPECT_EQ(client.bind(), 0);
    TestSocket moved(std::move(socket));
    EXPECT_EQ(moved.getHandle(), handle);
    EXPECT_EQ(socket.getHandle(), comm::INVALID_SOCKET_HANDLE);
    socket.close();
    TestSocket target;
    target = std::move(moved);
    EXPECT_EQ(target.getHandle(), handle);
    EXPECT_EQ(moved.getHandle(), comm::INVALID_SOCKET_HANDLE);
    moved.close();
    target.close();
    EXPECT_EQ(target.getHandle(), comm::INVALID_SOCKET_HANDLE);
    EXPECT_NO_THROW(target.close());
}

TEST(ListenSocketTest, CloseAndMoveAssignmentReleaseListenerPorts)
{
    uint16_t firstPort = 0;
    auto first = openListener(firstPort);
    uint16_t secondPort = 0;
    auto second = openListener(secondPort);
    first = std::move(second);
    second.close();
    EXPECT_NO_THROW(comm::ListenSocket(comm::CommEndpoint("127.0.0.1", firstPort)));
    first.close();
    first.close();
    EXPECT_NO_THROW(comm::ListenSocket(comm::CommEndpoint("127.0.0.1", secondPort)));
}

TEST(ListenSocketTest, MoveConstructionPreservesListener)
{
    uint16_t port = 0;
    auto original = openListener(port);
    comm::ListenSocket moved(std::move(original));
    original.close();
    NativePeer client;
    ASSERT_EQ(client.connect(port), 0);
    ASSERT_FALSE(comm::SocketSelector({&moved}).waitReadable(SOCKET_WAIT_TIMEOUT).empty());
    EXPECT_TRUE(moved.accept());
}

TEST(ListenSocketTest, BindFailureReportsServerOpenError)
{
    // 주소 재사용 없이 바인딩한 네이티브 소켓은 두 운영체제 모두에서 두 번째 바인딩을 막는다.
    TestSocket apiLifetime;
    NativePeer reservation;
    ASSERT_EQ(reservation.bind(), 0);
    EXPECT_THROW(
        comm::ListenSocket(comm::CommEndpoint("127.0.0.1", reservation.localPort())),
        comm::ServerOpenError
    );
}

TEST(ListenSocketTest, DestructorReleasesListenerDuringExceptionUnwinding)
{
    uint16_t port = 0;
    EXPECT_THROW(
        {
            auto listener = openListener(port);
            throw std::runtime_error("Test exception");
        },
        std::runtime_error
    );
    EXPECT_NO_THROW(comm::ListenSocket(comm::CommEndpoint("127.0.0.1", port)));
}

TEST(ListenSocketTest, NonBlockingAcceptReturnsNoConnection)
{
    uint16_t port = 0;
    auto listener = openListener(port);
    EXPECT_FALSE(listener.accept());
}

TEST(SocketSelectorTest, RejectsNullAndReturnsNoSocketsForEmptyWait)
{
    EXPECT_THROW(comm::SocketSelector({nullptr}), std::invalid_argument);
    EXPECT_TRUE(comm::SocketSelector({}).waitReadable(SOCKET_WAIT_TIMEOUT).empty());
}

TEST(SocketSelectorTest, RetainsRegistrationsAcrossWaits)
{
    TestSocket apiLifetime;
    NativePeer client;
    auto socket = connectPeer(client);
    const comm::SocketSelector selector({&socket});
    auto ready = selector.waitWritable(std::chrono::microseconds{0});
    EXPECT_EQ(ready, (std::vector<const comm::Socket*>{&socket}));
    ready = selector.waitReadable(std::chrono::microseconds{0});
    EXPECT_TRUE(ready.empty());
    for (uint8_t byte : {0x41, 0x42})
    {
        ASSERT_EQ(client.sendByte(byte), 1);
        ready = selector.waitReadable(SOCKET_WAIT_TIMEOUT);
        EXPECT_EQ(ready, (std::vector<const comm::Socket*>{&socket}));
        const auto received = socket.receiveBytes(1);
        ASSERT_TRUE(received);
        EXPECT_EQ(*received, (std::vector<uint8_t>{byte}));
    }
}

TEST(SocketSelectorTest, ReturnsAllWritableConnections)
{
    TestSocket apiLifetime;
    NativePeer firstClient;
    NativePeer secondClient;
    auto first = connectPeer(firstClient);
    auto second = connectPeer(secondClient);
    const comm::SocketSelector selector({&first, &second});

    EXPECT_EQ(
        selector.waitWritable(SOCKET_WAIT_TIMEOUT),
        (std::vector<const comm::Socket*>{&first, &second})
    );
    EXPECT_EQ(
        selector.waitWritable(std::chrono::microseconds{0}),
        (std::vector<const comm::Socket*>{&first, &second})
    );
}

TEST(SocketSelectorTest, RejectsClosedSockets)
{
    TestSocket socket;
    const comm::SocketSelector selector({&socket});
    socket.close();

    EXPECT_THROW(selector.waitReadable(std::chrono::microseconds{0}), std::invalid_argument);
    EXPECT_THROW(selector.waitWritable(std::chrono::microseconds{0}), std::invalid_argument);
}

TEST(ConnectedSocketTest, LoopbackConnectionSupportsBidirectionalIo)
{
    uint16_t port = 0;
    auto listener = openListener(port);
    NativePeer client;
    ASSERT_EQ(client.connect(port), 0);

    ASSERT_FALSE(comm::SocketSelector({&listener}).waitReadable(SOCKET_WAIT_TIMEOUT).empty());
    std::optional<comm::ConnectedSocket> accepted = listener.accept();
    ASSERT_TRUE(accepted.has_value());
    const comm::CommEndpoint remoteEndpoint = accepted->getRemoteEndpoint();
    EXPECT_EQ(remoteEndpoint.getAddress(), "127.0.0.1");
    EXPECT_EQ(remoteEndpoint, comm::CommEndpoint("127.0.0.1", client.localPort()));
    EXPECT_EQ(remoteEndpoint.getPort(), client.localPort());

    ASSERT_EQ(client.sendByte(0x4A), 1);
    const auto ready = comm::SocketSelector({&listener, &*accepted}).waitReadable(SOCKET_WAIT_TIMEOUT);
    ASSERT_EQ(ready.size(), 1U);
    EXPECT_EQ(ready.front(), &*accepted);
    const auto receivedResult = accepted->receiveBytes(8);
    ASSERT_TRUE(receivedResult.has_value());
    EXPECT_EQ(*receivedResult, (std::vector<uint8_t>{0x4A}));

    ASSERT_NO_THROW(accepted->waitWritable(SOCKET_WAIT_TIMEOUT));
    EXPECT_NO_THROW(accepted->waitWritable(std::chrono::microseconds{0}));
    const uint8_t response = 0x7B;
    const auto sentResult = accepted->sendBytes({response});
    ASSERT_TRUE(sentResult.has_value());
    EXPECT_EQ(*sentResult, 1U);
    ASSERT_EQ(client.waitReadable(), 1);
    uint8_t received = 0;
    ASSERT_EQ(client.receiveByte(received), 1);
    EXPECT_EQ(received, response);

    EXPECT_TRUE(comm::SocketSelector({&*accepted}).waitReadable(std::chrono::microseconds{0}).empty());
    ASSERT_NO_THROW(accepted->shutdown());
    ASSERT_EQ(client.waitReadable(), 1);
    EXPECT_EQ(client.receiveByte(received), 0);
}

TEST(ConnectedSocketTest, ServerAcceptTransfersOwnershipToSession)
{
    uint16_t port = 0;
    const std::unique_ptr<comm::ListenSocket> server = openServer(port);
    EXPECT_FALSE(server->accept().has_value());

    NativePeer client;
    ASSERT_EQ(client.connect(port), 0);
    ASSERT_FALSE(comm::SocketSelector({server.get()}).waitReadable(SOCKET_WAIT_TIMEOUT).empty());
    {
        std::optional<comm::ConnectedSocket> accepted = server->accept();
        ASSERT_TRUE(accepted.has_value());
        comm::Session session(std::move(*accepted));
        accepted->close();

        session.send({0x6C}, SOCKET_WAIT_TIMEOUT);

        uint8_t received = 0;
        ASSERT_EQ(client.waitReadable(), 1);
        ASSERT_EQ(client.receiveByte(received), 1);
        EXPECT_EQ(received, 0x6C);
    }

    uint8_t received = 0;
    ASSERT_EQ(client.waitReadable(), 1);
    EXPECT_EQ(client.receiveByte(received), 0);
}

TEST(ConnectedSocketTest, UnclaimedAcceptedSocketClosesDuringExceptionUnwinding)
{
    uint16_t port = 0;
    const std::unique_ptr<comm::ListenSocket> server = openServer(port);
    NativePeer client;
    ASSERT_EQ(client.connect(port), 0);
    ASSERT_FALSE(comm::SocketSelector({server.get()}).waitReadable(SOCKET_WAIT_TIMEOUT).empty());

    EXPECT_THROW(
        {
            std::optional<comm::ConnectedSocket> accepted = server->accept();
            ASSERT_TRUE(accepted.has_value());
            throw std::runtime_error("Test exception before creating a session");
        },
        std::runtime_error
    );

    uint8_t received = 0;
    ASSERT_EQ(client.waitReadable(), 1);
    EXPECT_EQ(client.receiveByte(received), 0);
}

TEST(ConnectedSocketTest, SessionRemainsUsableAfterServerIsDestroyed)
{
    std::optional<comm::Session> session;
    std::unique_ptr<NativePeer> client;
    {
        uint16_t port = 0;
        const std::unique_ptr<comm::ListenSocket> server = openServer(port);
        client = std::make_unique<NativePeer>();
        ASSERT_EQ(client->connect(port), 0);
        ASSERT_FALSE(comm::SocketSelector({server.get()}).waitReadable(SOCKET_WAIT_TIMEOUT).empty());
        std::optional<comm::ConnectedSocket> accepted = server->accept();
        ASSERT_TRUE(accepted.has_value());
        session.emplace(std::move(*accepted));
    }

    session->send({0x5D}, SOCKET_WAIT_TIMEOUT);
    uint8_t received = 0;
    ASSERT_EQ(client->waitReadable(), 1);
    ASSERT_EQ(client->receiveByte(received), 1);
    EXPECT_EQ(received, 0x5D);
}

TEST(ConnectedSocketTest, NonBlockingReceiveDistinguishesWouldBlockFromClosure)
{
    TestSocket apiLifetime;
    NativePeer client;
    auto socket = connectPeer(client);

    const auto idle = socket.receiveBytes(1);
    EXPECT_FALSE(idle.has_value());

    client.close();
    ASSERT_FALSE(comm::SocketSelector({&socket}).waitReadable(SOCKET_WAIT_TIMEOUT).empty());
    const auto closed = socket.receiveBytes(1);
    ASSERT_TRUE(closed.has_value());
    EXPECT_TRUE(closed->empty());
}

TEST(ConnectedSocketTest, ReceiveLimitLeavesRemainingBytesForNextCall)
{
    TestSocket apiLifetime;
    NativePeer client;
    auto socket = connectPeer(client);

    ASSERT_EQ(client.sendByte(0x41), 1);
    ASSERT_EQ(client.sendByte(0x42), 1);
    ASSERT_FALSE(comm::SocketSelector({&socket}).waitReadable(SOCKET_WAIT_TIMEOUT).empty());
    const auto first = socket.receiveBytes(1);
    ASSERT_TRUE(first.has_value());
    EXPECT_EQ(*first, (std::vector<uint8_t>{0x41}));

    ASSERT_FALSE(comm::SocketSelector({&socket}).waitReadable(SOCKET_WAIT_TIMEOUT).empty());
    const auto second = socket.receiveBytes(1);
    ASSERT_TRUE(second.has_value());
    EXPECT_EQ(*second, (std::vector<uint8_t>{0x42}));
    EXPECT_EQ(*first, (std::vector<uint8_t>{0x41}));
}

TEST(ConnectedSocketTest, BufferAndTimeoutContractsAreExplicit)
{
    TestSocket apiLifetime;
    NativePeer client;
    auto socket = connectPeer(client);
    const auto empty = socket.sendBytes({});
    ASSERT_TRUE(empty.has_value());
    EXPECT_EQ(*empty, 0U);

    EXPECT_THROW(socket.sendBytes({}, 1), std::out_of_range);
    EXPECT_THROW(socket.receiveBytes(0), std::invalid_argument);
    EXPECT_THROW(socket.waitWritable(std::chrono::microseconds{-1}), std::invalid_argument);
    EXPECT_THROW(comm::SocketSelector({&socket}).waitReadable(std::chrono::microseconds{-1}), std::invalid_argument);
    const comm::SocketSelector emptySelector({});
    EXPECT_TRUE(emptySelector.waitWritable(std::chrono::microseconds{0}).empty());
    EXPECT_THROW(emptySelector.waitWritable(std::chrono::microseconds{-1}), std::invalid_argument);
}

TEST(ConnectedSocketTest, MoveAssignmentReleasesPreviousConnection)
{
    TestSocket apiLifetime;
    NativePeer firstClient;
    NativePeer secondClient;
    auto first = connectPeer(firstClient);
    auto second = connectPeer(secondClient);
    const comm::CommEndpoint secondEndpoint = second.getRemoteEndpoint();
    comm::ConnectedSocket moved(std::move(second));
    EXPECT_EQ(moved.getRemoteEndpoint(), secondEndpoint);
    second.close();
    first = std::move(moved);
    EXPECT_EQ(first.getRemoteEndpoint(), secondEndpoint);
    moved.close();
    uint8_t received = 0;
    ASSERT_EQ(firstClient.waitReadable(), 1);
    EXPECT_EQ(firstClient.receiveByte(received), 0);
    ASSERT_EQ(first.sendBytes({0x51}), std::optional<std::size_t>{1});
    ASSERT_EQ(secondClient.waitReadable(), 1);
    ASSERT_EQ(secondClient.receiveByte(received), 1);
    EXPECT_EQ(received, 0x51);
}

TEST(ConnectedSocketTest, BackpressureProducesPartialTransferAndWouldBlock)
{
    if (!comm::native_socket::test::supportsBoundedSendBuffer())
    {
        GTEST_SKIP() << "Bounded send-buffer stress is not supported";
    }
    TestSocket apiLifetime;
    NativePeer client;
    ASSERT_EQ(client.setReceiveBufferSize(4096), 0);
    auto socket = connectPeer(client);

    const std::vector<uint8_t> payload(8 * 1024 * 1024, 0x5A);
    bool partial = false;
    bool blocked = false;
    for (int attempt = 0; attempt < 32; ++attempt)
    {
        const auto result = socket.sendBytes(payload);
        if (!result)
        {
            blocked = true;
            break;
        }
        ASSERT_GT(*result, 0U);
        EXPECT_LE(*result, payload.size());
        partial = partial || *result < payload.size();
    }
    EXPECT_TRUE(partial);
    ASSERT_TRUE(blocked);

    const comm::SocketSelector selector({&socket});
    EXPECT_TRUE(selector.waitWritable(std::chrono::microseconds{0}).empty());
    try
    {
        socket.waitWritable(std::chrono::microseconds{0});
        FAIL() << "A blocked socket must time out when polled";
    }
    catch (const std::system_error& error)
    {
        EXPECT_EQ(error.code(), std::make_error_code(std::errc::timed_out));
        EXPECT_NE(std::string(error.what()).find("wait-writable"), std::string::npos);
    }
}

TEST(ConnectedSocketTest, SessionReceiveStaysOpenUntilPeerCloses)
{
    TestSocket apiLifetime;
    NativePeer client;
    auto socket = connectPeer(client);
    comm::Session session(std::move(socket));

    EXPECT_TRUE(session.receive().empty());
    EXPECT_TRUE(session.isOpen());
    client.close();
    const auto deadline = std::chrono::steady_clock::now() + SOCKET_WAIT_TIMEOUT;
    while (session.isOpen() && std::chrono::steady_clock::now() < deadline)
    {
        EXPECT_TRUE(session.receive().empty());
        std::this_thread::yield();
    }
    EXPECT_FALSE(session.isOpen());
}

TEST(ConnectedSocketTest, SessionSendCompletesEntireStream)
{
    TestSocket apiLifetime;
    NativePeer client;
    auto socket = connectPeer(client);
    comm::Session session(std::move(socket));
    const std::vector<uint8_t> payload(4 * 1024 * 1024, 0x4D);
    std::vector<uint8_t> received(payload.size());
    std::size_t receivedSize = 0;

    std::thread reader([&]
    {
        while (receivedSize < received.size())
        {
            if (client.waitReadable() != 1)
            {
                return;
            }
            const int count = client.receiveBytes(received.data() + receivedSize, received.size() - receivedSize);
            if (count <= 0)
            {
                return;
            }
            receivedSize += static_cast<std::size_t>(count);
        }
    }
    );
    EXPECT_NO_THROW(session.send(payload, SOCKET_WAIT_TIMEOUT));
    reader.join();

    ASSERT_EQ(receivedSize, payload.size());
    EXPECT_EQ(received, payload);
}

TEST(ConnectedSocketTest, SessionSendTimeoutReportsPartialProgress)
{
    if (!comm::native_socket::test::supportsBoundedSendBuffer())
    {
        GTEST_SKIP() << "Bounded send-buffer stress is not supported";
    }
    TestSocket apiLifetime;
    NativePeer client;
    ASSERT_EQ(client.setReceiveBufferSize(4096), 0);
    auto socket = connectPeer(client);
    comm::Session session(std::move(socket));
    const std::vector<uint8_t> payload(32 * 1024 * 1024, 0x5A);
    constexpr std::chrono::milliseconds TIMEOUT{40};

    const auto started = std::chrono::steady_clock::now();
    try
    {
        session.send(payload, TIMEOUT);
        FAIL() << "Sending to a stalled peer must time out";
    }
    catch (const comm::SendError& error)
    {
        const std::string message = error.what();
        EXPECT_NE(message.find("status=timeout"), std::string::npos);
        const std::size_t offset = message.find("bytesSent=");
        ASSERT_NE(offset, std::string::npos);
        const std::size_t bytesSent = std::stoull(message.substr(offset + 10));
        EXPECT_GT(bytesSent, 0U);
        EXPECT_LT(bytesSent, payload.size());
    }
    const auto elapsed = std::chrono::steady_clock::now() - started;
    EXPECT_GE(elapsed, TIMEOUT - std::chrono::milliseconds{1});
    EXPECT_LT(elapsed, SOCKET_WAIT_TIMEOUT);
}

TEST(ConnectedSocketTest, ReadWaitReportsInterruption)
{
    if (!SignalInterruptor::isSupported())
    {
        GTEST_SKIP() << "Signal interruption is not supported";
    }
    uint16_t port = 0;
    auto listener = openListener(port);
    const SignalInterruptor signals;
    EXPECT_TRUE(comm::SocketSelector({&listener}).waitReadable(SOCKET_WAIT_TIMEOUT).empty());
}

TEST(ConnectedSocketTest, InterruptedWriteWaitUsesOriginalDeadline)
{
    if (!SignalInterruptor::isSupported())
    {
        GTEST_SKIP() << "Signal interruption is not supported";
    }
    if (!comm::native_socket::test::supportsBoundedSendBuffer())
    {
        GTEST_SKIP() << "Bounded send-buffer stress is not supported";
    }
    TestSocket apiLifetime;
    NativePeer client;
    ASSERT_EQ(client.setReceiveBufferSize(4096), 0);
    auto socket = connectPeer(client);

    const std::vector<uint8_t> payload(8 * 1024 * 1024, 0x5A);
    bool blocked = false;
    for (int attempt = 0; attempt < 32; ++attempt)
    {
        if (!socket.sendBytes(payload))
        {
            blocked = true;
            break;
        }
    }
    ASSERT_TRUE(blocked);

    const comm::SocketSelector selector({&socket});
    const SignalInterruptor signals;
    EXPECT_TRUE(selector.waitWritable(SOCKET_WAIT_TIMEOUT).empty());

    constexpr std::chrono::milliseconds TIMEOUT{40};
    const auto started = std::chrono::steady_clock::now();
    try
    {
        socket.waitWritable(TIMEOUT);
        FAIL() << "Repeated interruptions must not extend the write wait deadline";
    }
    catch (const std::system_error& error)
    {
        EXPECT_EQ(error.code(), std::make_error_code(std::errc::timed_out));
    }
    const auto elapsed = std::chrono::steady_clock::now() - started;
    EXPECT_GE(elapsed, TIMEOUT - std::chrono::milliseconds{1});
    EXPECT_LT(elapsed, SOCKET_WAIT_TIMEOUT);
}

TEST(ConnectedSocketTest, NonBlockingReceiveReturnsNoDataUnderSignals)
{
    if (!SignalInterruptor::isSupported())
    {
        GTEST_SKIP() << "Signal interruption is not supported";
    }
    TestSocket apiLifetime;
    NativePeer client;
    auto socket = connectPeer(client);
    const SignalInterruptor signals;
    const auto result = socket.receiveBytes(1);
    EXPECT_FALSE(result.has_value());
}

TEST(ConnectedSocketTest, InterruptedSessionReceiveKeepsConnectionOpen)
{
    if (!SignalInterruptor::isSupported())
    {
        GTEST_SKIP() << "Signal interruption is not supported";
    }
    TestSocket apiLifetime;
    NativePeer client;
    auto socket = connectPeer(client);
    comm::Session session(std::move(socket));
    const SignalInterruptor signals;
    EXPECT_TRUE(session.receive().empty());
    EXPECT_TRUE(session.isOpen());
}

TEST(ConnectedSocketTest, InterruptedSessionSendUsesOriginalDeadline)
{
    if (!SignalInterruptor::isSupported())
    {
        GTEST_SKIP() << "Signal interruption is not supported";
    }
    if (!comm::native_socket::test::supportsBoundedSendBuffer())
    {
        GTEST_SKIP() << "Bounded send-buffer stress is not supported";
    }
    TestSocket apiLifetime;
    NativePeer client;
    ASSERT_EQ(client.setReceiveBufferSize(4096), 0);
    auto socket = connectPeer(client);
    comm::Session session(std::move(socket));
    const std::vector<uint8_t> payload(32 * 1024 * 1024, 0x5A);
    constexpr std::chrono::milliseconds TIMEOUT{40};
    const SignalInterruptor signals;
    const auto started = std::chrono::steady_clock::now();
    try
    {
        session.send(payload, TIMEOUT);
        FAIL() << "Repeated interruptions must not extend the deadline";
    }
    catch (const comm::SendError& error)
    {
        EXPECT_NE(std::string(error.what()).find("status=timeout"), std::string::npos);
    }
    const auto elapsed = std::chrono::steady_clock::now() - started;
    EXPECT_GE(elapsed, TIMEOUT - std::chrono::milliseconds{1});
    EXPECT_LT(elapsed, SOCKET_WAIT_TIMEOUT);
}

} // namespace
