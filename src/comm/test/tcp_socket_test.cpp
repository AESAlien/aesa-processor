#include <comm/config/comm_endpoint.hpp>
#include <comm/error/comm_error.hpp>
#include <network/network_manager/network_manager.hpp>
#include <network/peer_connection/peer_connection.hpp>
#include <network/session/session.hpp>
#include <network/socket_selector/socket_selector.hpp>
#include <network/tcp_socket/tcp_socket.hpp>
#include <network/tcp_server/tcp_server.hpp>

#ifdef _WIN32
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif

#ifndef NOMINMAX
#define NOMINMAX
#endif

#include <winsock2.h>
#include <ws2tcpip.h>
#else
#include <arpa/inet.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <unistd.h>
#endif

#ifdef __linux__
#include <pthread.h>
#include <signal.h>
#endif

#include <gtest/gtest.h>

#include <algorithm>
#include <atomic>
#include <chrono>
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

static_assert(!std::is_copy_constructible_v<comm::TcpSocket>);
static_assert(!std::is_copy_assignable_v<comm::TcpSocket>);
static_assert(std::is_nothrow_move_constructible_v<comm::TcpSocket>);
static_assert(std::is_nothrow_move_assignable_v<comm::TcpSocket>);
static_assert(!std::is_nothrow_default_constructible_v<comm::TcpSocket>);

sockaddr_in loopbackAddress(uint16_t port)
{
    sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    address.sin_port = htons(port);
    return address;
}

// An independent native peer exercises TcpSocket without exposing its handle.
class NativePeer
{
public:
    NativePeer()
    {
        if (_handle == INVALID_HANDLE)
        {
            throw std::runtime_error("Could not create native test peer");
        }
    }

    ~NativePeer()
    {
        close();
    }

    NativePeer(const NativePeer&) = delete;
    NativePeer& operator=(const NativePeer&) = delete;

    void close()
    {
        if (_handle != INVALID_HANDLE)
        {
#ifdef _WIN32
            closesocket(_handle);
#else
            ::close(_handle);
#endif
            _handle = INVALID_HANDLE;
        }
    }

    int bind()
    {
        const sockaddr_in address = loopbackAddress(0);
        return ::bind(_handle, reinterpret_cast<const sockaddr*>(&address), sizeof(address));
    }

    int connect(uint16_t port)
    {
        const sockaddr_in address = loopbackAddress(port);
        return ::connect(_handle, reinterpret_cast<const sockaddr*>(&address), sizeof(address));
    }

    int setReceiveBufferSize(int size)
    {
#ifdef _WIN32
        const char* optionValue = reinterpret_cast<const char*>(&size);
#else
        const int* optionValue = &size;
#endif
        return setsockopt(_handle, SOL_SOCKET, SO_RCVBUF, optionValue, sizeof(size));
    }

    uint16_t localPort() const
    {
        sockaddr_in address{};
#ifdef _WIN32
        int length = sizeof(address);
#else
        socklen_t length = sizeof(address);
#endif
        if (getsockname(_handle, reinterpret_cast<sockaddr*>(&address), &length) < 0)
        {
            throw std::runtime_error("Could not read native test peer address");
        }
        return ntohs(address.sin_port);
    }

    int waitReadable() const
    {
        fd_set sockets;
        FD_ZERO(&sockets);
        FD_SET(_handle, &sockets);
        timeval timeout{};
        timeout.tv_sec = static_cast<long>(SOCKET_WAIT_TIMEOUT.count());
#ifdef _WIN32
        return select(0, &sockets, nullptr, nullptr, &timeout);
#else
        return select(_handle + 1, &sockets, nullptr, nullptr, &timeout);
#endif
    }

    int sendByte(uint8_t value) const
    {
        return static_cast<int>(::send(_handle, reinterpret_cast<const char*>(&value), 1, 0));
    }

    int receiveByte(uint8_t& value) const
    {
        return static_cast<int>(recv(_handle, reinterpret_cast<char*>(&value), 1, 0));
    }

    int receiveBytes(uint8_t* buffer, std::size_t size) const
    {
        const int nativeSize = static_cast<int>(std::min(size, std::size_t{65536}));
        return static_cast<int>(recv(_handle, reinterpret_cast<char*>(buffer), nativeSize, 0));
    }

private:
#ifdef _WIN32
    using Handle = SOCKET;
    static constexpr Handle INVALID_HANDLE = INVALID_SOCKET;
#else
    using Handle = int;
    static constexpr Handle INVALID_HANDLE = -1;
#endif

    Handle _handle = ::socket(AF_INET, SOCK_STREAM, 0);
};

uint16_t availablePort()
{
    // Native probes borrow the API lifetime from a regular socket object.
    comm::TcpSocket runtimeOwner;
    NativePeer reservation;
    if (reservation.bind() < 0)
    {
        throw std::runtime_error("Could not reserve a test port");
    }
    return reservation.localPort();
}

uint16_t openListener(comm::TcpSocket& listener)
{
    // Retry if another process claims the ephemeral port after reservation.
    for (int attempt = 0; attempt < BIND_ATTEMPTS; ++attempt)
    {
        const uint16_t port = availablePort();
        comm::TcpSocket candidate(2);
        try
        {
            candidate.bind(comm::CommEndpoint("127.0.0.1", port));
        }
        catch (const std::system_error&)
        {
            if (attempt == BIND_ATTEMPTS - 1)
            {
                throw;
            }
            continue;
        }
        candidate.listen();
        candidate.setNonBlocking();
        listener = std::move(candidate);
        return port;
    }
    throw std::runtime_error("Could not bind a test listener");
}

std::unique_ptr<comm::TcpServer> openServer(uint16_t& port)
{
    for (int attempt = 0; attempt < BIND_ATTEMPTS; ++attempt)
    {
        port = availablePort();
        try
        {
            return std::make_unique<comm::TcpServer>(comm::CommEndpoint("127.0.0.1", port));
        }
        catch (const comm::ServerOpenError&)
        {
            if (attempt == BIND_ATTEMPTS - 1)
            {
                throw;
            }
        }
    }
    throw std::runtime_error("Could not open a test server");
}

comm::TcpSocket connectPeer(NativePeer& client, bool nonBlocking = true)
{
    comm::TcpSocket listener;
    const uint16_t port = openListener(listener);
    if (client.connect(port) < 0)
    {
        throw std::runtime_error("Could not connect native test peer");
    }
    std::vector<const comm::TcpSocket*> ready;
    if (comm::SocketSelector({&listener}).waitReadable(SOCKET_WAIT_TIMEOUT, ready) != comm::SocketWaitStatus::READY)
    {
        throw std::runtime_error("Native test connection did not become ready");
    }
    std::optional<comm::TcpSocket> accepted = listener.accept();
    if (!accepted)
    {
        throw std::runtime_error("Could not accept native test peer");
    }
    if (nonBlocking)
    {
        accepted->setNonBlocking();
    }
    return std::move(*accepted);
}

#ifdef __linux__
void handleTestSignal(int)
{
}

// Repeated signals avoid depending on which instruction begins the wait.
// Destruction stops delivery before restoring the process signal handler.
class SignalInterruptor
{
public:
    SignalInterruptor()
    {
        struct sigaction action{};
        action.sa_handler = handleTestSignal;
        sigemptyset(&action.sa_mask);
        if (sigaction(SIGUSR1, &action, &_previous) < 0)
        {
            throw std::runtime_error("Could not install test signal handler");
        }
        const pthread_t target = pthread_self();
        try
        {
            _sender = std::thread([this, target]
            {
                while (!_stop.load())
                {
                    pthread_kill(target, SIGUSR1);
                    std::this_thread::sleep_for(std::chrono::milliseconds{1});
                }
            });
        }
        catch (...)
        {
            sigaction(SIGUSR1, &_previous, nullptr);
            throw;
        }
    }

    ~SignalInterruptor()
    {
        _stop.store(true);
        _sender.join();
        sigaction(SIGUSR1, &_previous, nullptr);
    }

    SignalInterruptor(const SignalInterruptor&) = delete;
    SignalInterruptor& operator=(const SignalInterruptor&) = delete;

private:
    struct sigaction _previous{};
    std::atomic<bool> _stop{false};
    std::thread _sender;
};
#endif

TEST(NetworkManagerTest, RoutesRegisteredConnectionsByObjectNotDiagnosticName)
{
    comm::TcpSocket apiLifetime;
    NativePeer firstClient;
    NativePeer secondClient;
    ASSERT_EQ(firstClient.bind(), 0);
    ASSERT_EQ(secondClient.bind(), 0);
    comm::PeerConnection first(
        "same-name", comm::CommEndpoint("127.0.0.1", firstClient.localPort()));
    comm::PeerConnection second(
        "same-name", comm::CommEndpoint("127.0.0.1", secondClient.localPort()));
    const uint16_t port = availablePort();
    comm::NetworkManager manager(
        comm::CommEndpoint("127.0.0.1", port), {first, second});
    manager.start();

    ASSERT_EQ(firstClient.connect(port), 0);
    EXPECT_FALSE(manager.poll(SOCKET_WAIT_TIMEOUT));
    ASSERT_TRUE(first.getSession());
    EXPECT_FALSE(second.getSession());
    ASSERT_EQ(secondClient.connect(port), 0);
    EXPECT_FALSE(manager.poll(SOCKET_WAIT_TIMEOUT));
    ASSERT_TRUE(second.getSession());

    first.send({0x31}, SOCKET_WAIT_TIMEOUT);
    second.send({0x42}, SOCKET_WAIT_TIMEOUT);
    uint8_t received = 0;
    ASSERT_EQ(firstClient.waitReadable(), 1);
    ASSERT_EQ(firstClient.receiveByte(received), 1);
    EXPECT_EQ(received, 0x31);
    ASSERT_EQ(secondClient.waitReadable(), 1);
    ASSERT_EQ(secondClient.receiveByte(received), 1);
    EXPECT_EQ(received, 0x42);

    firstClient.close();
    EXPECT_FALSE(manager.poll(SOCKET_WAIT_TIMEOUT));
    EXPECT_FALSE(first.getSession());
    EXPECT_TRUE(second.getSession());
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
        comm::CommEndpoint("127.0.0.1", 4100));

    EXPECT_EQ(connection.getName(), "operator-console");
    EXPECT_FALSE(connection.getSession());
    EXPECT_TRUE(connection.matches(comm::CommEndpoint("127.0.0.1", 4100)));
    EXPECT_FALSE(connection.matches(comm::CommEndpoint("127.0.0.2", 4100)));
    EXPECT_FALSE(connection.matches(comm::CommEndpoint("127.0.0.1", 4101)));
}

TEST(PeerConnectionTest, ReconnectionKeepsSnapshotAliveAndIgnoresStaleDetach)
{
    comm::TcpSocket apiLifetime;
    NativePeer firstClient;
    NativePeer secondClient;
    comm::PeerConnection connection(
        "operator-console",
        comm::CommEndpoint("127.0.0.1", 4100));

    connection.attach(connectPeer(firstClient));
    auto previous = connection.getSession();
    ASSERT_TRUE(previous);
    const std::weak_ptr<comm::Session> previousLifetime = previous;

    connection.attach(connectPeer(secondClient));
    const auto current = connection.getSession();
    ASSERT_TRUE(current);
    ASSERT_NE(previous, current);
    connection.detachIfCurrent(previous);
    EXPECT_EQ(connection.getSession(), current);

    // A sender holding the old session can finish after reconnection.
    previous->send({0x5D}, SOCKET_WAIT_TIMEOUT);
    uint8_t received = 0;
    ASSERT_EQ(firstClient.waitReadable(), 1);
    ASSERT_EQ(firstClient.receiveByte(received), 1);
    EXPECT_EQ(received, 0x5D);
    previous.reset();
    EXPECT_TRUE(previousLifetime.expired());
    ASSERT_EQ(firstClient.waitReadable(), 1);
    EXPECT_EQ(firstClient.receiveByte(received), 0);

    current->send({0x6E}, SOCKET_WAIT_TIMEOUT);
    ASSERT_EQ(secondClient.waitReadable(), 1);
    ASSERT_EQ(secondClient.receiveByte(received), 1);
    EXPECT_EQ(received, 0x6E);
    connection.detachIfCurrent(current);
    EXPECT_FALSE(connection.getSession());
    EXPECT_NO_THROW(connection.detachIfCurrent(current));
}

TEST(TcpSocketTest, ConstructorCreatesSocketWithoutExplicitApiStartup)
{
    comm::TcpSocket socket;
    EXPECT_NO_THROW(socket.setNonBlocking());
    EXPECT_NO_THROW(socket.close());
    EXPECT_NO_THROW(socket.close());

    socket = comm::TcpSocket();
    EXPECT_NO_THROW(socket.setNonBlocking());
    EXPECT_NO_THROW(socket.close());
    EXPECT_NO_THROW(socket.close());
}

TEST(TcpSocketTest, CloseReleasesListenerPort)
{
    comm::TcpSocket listener;
    const uint16_t port = openListener(listener);
    listener.close();
    listener.close();

    comm::TcpSocket replacement;
    EXPECT_NO_THROW(replacement.bind(comm::CommEndpoint("127.0.0.1", port)));
}

TEST(TcpSocketTest, ReadWaitRejectsNullSocket)
{
    EXPECT_THROW(comm::SocketSelector({nullptr}), std::invalid_argument);
}

TEST(TcpSocketTest, SelectorRetainsRegistrationsAcrossWaits)
{
    comm::TcpSocket socket;
    NativePeer client;
    socket = connectPeer(client);
    const comm::SocketSelector selector({&socket});
    std::vector<const comm::TcpSocket*> ready{&socket};

    EXPECT_EQ(selector.waitWritable(std::chrono::microseconds{0}, ready), comm::SocketWaitStatus::READY);
    EXPECT_EQ(ready, (std::vector<const comm::TcpSocket*>{&socket}));
    EXPECT_EQ(selector.waitReadable(std::chrono::microseconds{0}, ready), comm::SocketWaitStatus::TIMED_OUT);
    EXPECT_TRUE(ready.empty());

    ASSERT_EQ(client.sendByte(0x41), 1);
    ASSERT_EQ(selector.waitReadable(SOCKET_WAIT_TIMEOUT, ready), comm::SocketWaitStatus::READY);
    EXPECT_EQ(ready, (std::vector<const comm::TcpSocket*>{&socket}));
    const auto received = socket.receiveBytes(1);
    ASSERT_TRUE(received.has_value());
    EXPECT_EQ(*received, (std::vector<uint8_t>{0x41}));

    EXPECT_EQ(selector.waitReadable(std::chrono::microseconds{0}, ready), comm::SocketWaitStatus::TIMED_OUT);
    EXPECT_TRUE(ready.empty());

    ASSERT_EQ(client.sendByte(0x42), 1);
    ASSERT_EQ(selector.waitReadable(SOCKET_WAIT_TIMEOUT, ready), comm::SocketWaitStatus::READY);
    EXPECT_EQ(ready, (std::vector<const comm::TcpSocket*>{&socket}));
}

TEST(TcpSocketTest, BindFailureThrowsSystemError)
{
    comm::TcpSocket listener;
    const uint16_t port = openListener(listener);
    EXPECT_THROW(listener.bind(comm::CommEndpoint("127.0.0.1", port)), std::system_error);
}

TEST(TcpSocketTest, MoveConstructionTransfersListenerOwnership)
{
    comm::TcpSocket moved;
    uint16_t port = 0;
    {
        comm::TcpSocket original;
        port = openListener(original);
        comm::TcpSocket intermediate(std::move(original));
        original.close();
        moved = std::move(intermediate);
        intermediate.close();
    }

    NativePeer client;
    ASSERT_EQ(client.connect(port), 0);
    std::vector<const comm::TcpSocket*> ready;
    ASSERT_EQ(comm::SocketSelector({&moved}).waitReadable(SOCKET_WAIT_TIMEOUT, ready), comm::SocketWaitStatus::READY);
    ASSERT_EQ(ready.size(), 1U);
    EXPECT_EQ(ready.front(), &moved);
}

TEST(TcpSocketTest, MoveAssignmentReleasesPreviousListener)
{
    comm::TcpSocket first;
    const uint16_t firstPort = openListener(first);
    comm::TcpSocket second;
    openListener(second);

    first = std::move(second);
    second.close();
    EXPECT_NO_THROW(first.setNonBlocking());

    comm::TcpSocket replacement;
    EXPECT_NO_THROW(replacement.bind(comm::CommEndpoint("127.0.0.1", firstPort)));
}

TEST(TcpSocketTest, ClosedSocketCanBeMovedWithoutThrowing)
{
    comm::TcpSocket closed;
    closed.close();

    comm::TcpSocket moved(std::move(closed));

    comm::TcpSocket target;
    EXPECT_NO_THROW(target = std::move(moved));
    target = comm::TcpSocket();
    EXPECT_NO_THROW(target.setNonBlocking());
}

TEST(TcpSocketTest, IndependentSocketKeepsApiAliveAfterAnotherIsDestroyed)
{
    std::optional<comm::TcpSocket> surviving;
    {
        comm::TcpSocket first;
        surviving.emplace();
    }

    EXPECT_NO_THROW(surviving->setNonBlocking());
    EXPECT_NO_THROW(surviving->bind(comm::CommEndpoint("127.0.0.1", availablePort())));
}

TEST(TcpSocketTest, DestructorReleasesListenerDuringExceptionUnwinding)
{
    uint16_t port = 0;
    EXPECT_THROW(
        {
            comm::TcpSocket listener;
            port = openListener(listener);
            throw std::runtime_error("Test exception");
        },
        std::runtime_error);

    comm::TcpSocket replacement;
    EXPECT_NO_THROW(replacement.bind(comm::CommEndpoint("127.0.0.1", port)));
}

TEST(TcpSocketTest, NonBlockingAcceptReturnsNoConnection)
{
    comm::TcpSocket listener;
    openListener(listener);
    EXPECT_FALSE(listener.accept().has_value());
}

TEST(TcpSocketTest, AcceptFailureThrowsSystemError)
{
    comm::TcpSocket socket;
    EXPECT_THROW(socket.accept(), std::system_error);
}

TEST(TcpSocketTest, RemoteEndpointRequiresConnectedSocket)
{
    comm::TcpSocket socket;
    EXPECT_THROW(socket.getRemoteEndpoint(), std::system_error);
}

TEST(TcpSocketTest, EmptyReadWaitClearsPreviousResults)
{
    comm::TcpSocket socket;
    std::vector<const comm::TcpSocket*> ready{&socket};
    EXPECT_EQ(comm::SocketSelector({}).waitReadable(SOCKET_WAIT_TIMEOUT, ready), comm::SocketWaitStatus::TIMED_OUT);
    EXPECT_TRUE(ready.empty());
}

TEST(TcpSocketTest, LoopbackConnectionSupportsReadinessAndBidirectionalIo)
{
    comm::TcpSocket listener;
    const uint16_t port = openListener(listener);
    NativePeer client;
    ASSERT_EQ(client.connect(port), 0);

    std::vector<const comm::TcpSocket*> ready;
    ASSERT_EQ(comm::SocketSelector({&listener}).waitReadable(SOCKET_WAIT_TIMEOUT, ready), comm::SocketWaitStatus::READY);
    std::optional<comm::TcpSocket> accepted = listener.accept();
    ASSERT_TRUE(accepted.has_value());
    const comm::CommEndpoint remoteEndpoint = accepted->getRemoteEndpoint();
    EXPECT_EQ(remoteEndpoint.getAddress(), "127.0.0.1");
    EXPECT_EQ(remoteEndpoint.getIntAddress(), htonl(INADDR_LOOPBACK));
    EXPECT_EQ(remoteEndpoint.getPort(), client.localPort());
    ASSERT_NO_THROW(accepted->setNonBlocking());

    ASSERT_EQ(client.sendByte(0x4A), 1);
    ASSERT_EQ(comm::SocketSelector({&listener, &*accepted}).waitReadable(SOCKET_WAIT_TIMEOUT, ready), comm::SocketWaitStatus::READY);
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

    EXPECT_EQ(comm::SocketSelector({&*accepted}).waitReadable(std::chrono::microseconds{0}, ready), comm::SocketWaitStatus::TIMED_OUT);
    EXPECT_TRUE(ready.empty());
    ASSERT_NO_THROW(accepted->shutdown());
    EXPECT_NO_THROW(accepted->setNonBlocking());
    ASSERT_EQ(client.waitReadable(), 1);
    EXPECT_EQ(client.receiveByte(received), 0);
}

TEST(TcpSocketTest, ServerAcceptTransfersOwnershipToSession)
{
    uint16_t port = 0;
    const std::unique_ptr<comm::TcpServer> server = openServer(port);
    EXPECT_FALSE(server->accept().has_value());

    NativePeer client;
    ASSERT_EQ(client.connect(port), 0);
    std::vector<const comm::TcpSocket*> ready;
    ASSERT_EQ(comm::SocketSelector({&server->getSocket()}).waitReadable(SOCKET_WAIT_TIMEOUT, ready), comm::SocketWaitStatus::READY);
    {
        std::optional<comm::TcpSocket> accepted = server->accept();
        ASSERT_TRUE(accepted.has_value());
        comm::Session session(std::move(*accepted), "operator-console");
        accepted->close();

        EXPECT_EQ(session.getSocket().getRemoteEndpoint().getPort(), client.localPort());
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

TEST(TcpSocketTest, UnclaimedAcceptedSocketClosesDuringExceptionUnwinding)
{
    uint16_t port = 0;
    const std::unique_ptr<comm::TcpServer> server = openServer(port);
    NativePeer client;
    ASSERT_EQ(client.connect(port), 0);
    std::vector<const comm::TcpSocket*> ready;
    ASSERT_EQ(comm::SocketSelector({&server->getSocket()}).waitReadable(SOCKET_WAIT_TIMEOUT, ready), comm::SocketWaitStatus::READY);

    EXPECT_THROW(
        {
            std::optional<comm::TcpSocket> accepted = server->accept();
            ASSERT_TRUE(accepted.has_value());
            throw std::runtime_error("Test exception before creating a session");
        },
        std::runtime_error);

    uint8_t received = 0;
    ASSERT_EQ(client.waitReadable(), 1);
    EXPECT_EQ(client.receiveByte(received), 0);
}

TEST(TcpSocketTest, SessionRemainsUsableAfterServerIsDestroyed)
{
    std::optional<comm::Session> session;
    std::unique_ptr<NativePeer> client;
    {
        uint16_t port = 0;
        const std::unique_ptr<comm::TcpServer> server = openServer(port);
        client = std::make_unique<NativePeer>();
        ASSERT_EQ(client->connect(port), 0);
        std::vector<const comm::TcpSocket*> ready;
        ASSERT_EQ(comm::SocketSelector({&server->getSocket()}).waitReadable(SOCKET_WAIT_TIMEOUT, ready), comm::SocketWaitStatus::READY);
        std::optional<comm::TcpSocket> accepted = server->accept();
        ASSERT_TRUE(accepted.has_value());
        session.emplace(std::move(*accepted), "operator-console");
    }

    EXPECT_EQ(session->getSocket().getRemoteEndpoint().getPort(), client->localPort());
    session->send({0x5D}, SOCKET_WAIT_TIMEOUT);
    uint8_t received = 0;
    ASSERT_EQ(client->waitReadable(), 1);
    ASSERT_EQ(client->receiveByte(received), 1);
    EXPECT_EQ(received, 0x5D);
}

TEST(TcpSocketTest, NonBlockingReceiveDistinguishesWouldBlockFromClosure)
{
    comm::TcpSocket socket;
    NativePeer client;
    socket = connectPeer(client);

    const auto idle = socket.receiveBytes(1);
    EXPECT_FALSE(idle.has_value());

    client.close();
    std::vector<const comm::TcpSocket*> ready;
    ASSERT_EQ(comm::SocketSelector({&socket}).waitReadable(SOCKET_WAIT_TIMEOUT, ready), comm::SocketWaitStatus::READY);
    const auto closed = socket.receiveBytes(1);
    ASSERT_TRUE(closed.has_value());
    EXPECT_TRUE(closed->empty());
}

TEST(TcpSocketTest, ReceiveLimitLeavesRemainingBytesForNextCall)
{
    comm::TcpSocket socket;
    NativePeer client;
    socket = connectPeer(client);

    ASSERT_EQ(client.sendByte(0x41), 1);
    ASSERT_EQ(client.sendByte(0x42), 1);
    std::vector<const comm::TcpSocket*> ready;
    ASSERT_EQ(comm::SocketSelector({&socket}).waitReadable(SOCKET_WAIT_TIMEOUT, ready), comm::SocketWaitStatus::READY);
    const auto first = socket.receiveBytes(1);
    ASSERT_TRUE(first.has_value());
    EXPECT_EQ(*first, (std::vector<uint8_t>{0x41}));

    ASSERT_EQ(comm::SocketSelector({&socket}).waitReadable(SOCKET_WAIT_TIMEOUT, ready), comm::SocketWaitStatus::READY);
    const auto second = socket.receiveBytes(1);
    ASSERT_TRUE(second.has_value());
    EXPECT_EQ(*second, (std::vector<uint8_t>{0x42}));
    EXPECT_EQ(*first, (std::vector<uint8_t>{0x41}));
}

TEST(TcpSocketTest, BufferAndTimeoutContractsAreExplicit)
{
    comm::TcpSocket socket;
    const auto empty = socket.sendBytes({});
    ASSERT_TRUE(empty.has_value());
    EXPECT_EQ(*empty, 0U);

    EXPECT_THROW(socket.sendBytes({}, 1), std::out_of_range);
    EXPECT_THROW(socket.receiveBytes(0), std::invalid_argument);
    EXPECT_THROW(socket.waitWritable(std::chrono::microseconds{-1}), std::invalid_argument);
    std::vector<const comm::TcpSocket*> ready;
    EXPECT_THROW(comm::SocketSelector({&socket}).waitReadable(std::chrono::microseconds{-1}, ready), std::invalid_argument);
    const comm::SocketSelector emptySelector({});
    ready.push_back(&socket);
    EXPECT_EQ(emptySelector.waitWritable(std::chrono::microseconds{0}, ready), comm::SocketWaitStatus::TIMED_OUT);
    EXPECT_TRUE(ready.empty());
    EXPECT_THROW(emptySelector.waitWritable(std::chrono::microseconds{-1}, ready), std::invalid_argument);
}

TEST(TcpSocketTest, UnexpectedIoFailureCarriesNativeErrorInException)
{
    comm::TcpSocket socket;
    EXPECT_THROW(socket.receiveBytes(1), std::system_error);
    const uint8_t byte = 0x42;
    try
    {
        socket.sendBytes({byte});
        FAIL() << "An unconnected send must fail";
    }
    catch (const std::system_error& error)
    {
        EXPECT_NE(error.code().value(), 0);
    }
}

#ifdef __linux__
// Backpressure thresholds depend on the provider. Winsock can buffer the whole
// payload, so these bounded-buffer stress assumptions are exercised on Linux.
TEST(TcpSocketTest, BackpressureProducesPartialTransferAndWouldBlock)
{
    comm::TcpSocket socket;
    NativePeer client;
    ASSERT_EQ(client.setReceiveBufferSize(4096), 0);
    socket = connectPeer(client);

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
    std::vector<const comm::TcpSocket*> ready{&socket};
    EXPECT_EQ(selector.waitWritable(std::chrono::microseconds{0}, ready), comm::SocketWaitStatus::TIMED_OUT);
    EXPECT_TRUE(ready.empty());
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
#endif

TEST(TcpSocketTest, SessionReceiveStaysOpenUntilPeerCloses)
{
    comm::TcpSocket socket;
    NativePeer client;
    socket = connectPeer(client);
    comm::Session session(std::move(socket), "operator-console");

    EXPECT_TRUE(session.receive().empty());
    EXPECT_TRUE(session.isOpen());
    client.close();
    std::vector<const comm::TcpSocket*> ready;
    ASSERT_EQ(comm::SocketSelector({&session.getSocket()}).waitReadable(SOCKET_WAIT_TIMEOUT, ready), comm::SocketWaitStatus::READY);
    EXPECT_TRUE(session.receive().empty());
    EXPECT_FALSE(session.isOpen());
}

TEST(TcpSocketTest, SessionSendCompletesEntireStream)
{
    comm::TcpSocket socket;
    NativePeer client;
    socket = connectPeer(client);
    comm::Session session(std::move(socket), "operator-console");
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
    });
    EXPECT_NO_THROW(session.send(payload, SOCKET_WAIT_TIMEOUT));
    reader.join();

    ASSERT_EQ(receivedSize, payload.size());
    EXPECT_EQ(received, payload);
}

#ifdef __linux__
TEST(TcpSocketTest, SessionSendTimeoutReportsPartialProgress)
{
    comm::TcpSocket socket;
    NativePeer client;
    ASSERT_EQ(client.setReceiveBufferSize(4096), 0);
    socket = connectPeer(client);
    comm::Session session(std::move(socket), "operator-console");
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

TEST(TcpSocketTest, ReadWaitReportsInterruption)
{
    comm::TcpSocket listener;
    openListener(listener);
    std::vector<const comm::TcpSocket*> ready;
    const SignalInterruptor signals;
    EXPECT_EQ(comm::SocketSelector({&listener}).waitReadable(SOCKET_WAIT_TIMEOUT, ready), comm::SocketWaitStatus::INTERRUPTED);
    EXPECT_TRUE(ready.empty());
}

TEST(TcpSocketTest, InterruptedWriteWaitUsesOriginalDeadline)
{
    comm::TcpSocket socket;
    NativePeer client;
    ASSERT_EQ(client.setReceiveBufferSize(4096), 0);
    socket = connectPeer(client);

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
    std::vector<const comm::TcpSocket*> ready{&socket};
    const SignalInterruptor signals;
    ASSERT_EQ(selector.waitWritable(SOCKET_WAIT_TIMEOUT, ready), comm::SocketWaitStatus::INTERRUPTED);
    EXPECT_TRUE(ready.empty());

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

TEST(TcpSocketTest, InterruptedReceiveReturnsNoData)
{
    comm::TcpSocket socket;
    NativePeer client;
    socket = connectPeer(client, false);
    const SignalInterruptor signals;
    const auto result = socket.receiveBytes(1);
    EXPECT_FALSE(result.has_value());
}

TEST(TcpSocketTest, InterruptedSessionReceiveKeepsConnectionOpen)
{
    comm::TcpSocket socket;
    NativePeer client;
    socket = connectPeer(client, false);
    comm::Session session(std::move(socket), "operator-console");
    const SignalInterruptor signals;
    EXPECT_TRUE(session.receive().empty());
    EXPECT_TRUE(session.isOpen());
}

TEST(TcpSocketTest, InterruptedSessionSendUsesOriginalDeadline)
{
    comm::TcpSocket socket;
    NativePeer client;
    ASSERT_EQ(client.setReceiveBufferSize(4096), 0);
    socket = connectPeer(client);
    comm::Session session(std::move(socket), "operator-console");
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
#endif

} // namespace
