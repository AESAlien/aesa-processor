#include <comm/config/comm_endpoint.hpp>
#include <comm/controller/comm_controller.hpp>
#include <comm/error/comm_error.hpp>
#include <network/network_manager.hpp>
#include <network/peer_connection.hpp>
#include <network/session.hpp>
#include <socket/socket_selector.hpp>
#include <socket/socket.hpp>
#include <network/tcp_server.hpp>
#include <socket/listen_socket.hpp>
#include <socket/connected_socket.hpp>

#include <protocol/ST_BeamTxMsg.hpp>
#include <protocol/ST_MonitorStatusMsg.hpp>
#include <protocol/ST_RadarAttMsg.hpp>

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
#include <variant>
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

// 모니터를 사용하지 않는 테스트에서 어떤 연결과도 일치하지 않는 모니터 엔드포인트
const comm::CommEndpoint UNUSED_MONITOR("127.0.0.2", 1);

comm::CommConfig makeConfig(
    uint16_t serverPort,
    const NativePeer& operatorConsole,
    const NativePeer& scenarioSimulator,
    const comm::CommEndpoint& monitor = UNUSED_MONITOR
) {
    return comm::CommConfig(
        comm::CommEndpoint("127.0.0.1", serverPort),
        comm::CommEndpoint("127.0.0.1", operatorConsole.localPort()),
        comm::CommEndpoint("127.0.0.1", scenarioSimulator.localPort()),
        monitor
    );
}

comm::CommConfig makeConfig(
    uint16_t serverPort,
    const NativePeer& operatorConsole,
    const NativePeer& scenarioSimulator,
    const NativePeer& monitor
) {
    return makeConfig(
        serverPort,
        operatorConsole,
        scenarioSimulator,
        comm::CommEndpoint("127.0.0.1", monitor.localPort())
    );
}

// 상대가 보낸 바이트를 지정한 길이만큼 모두 읽는다.
std::vector<uint8_t> receiveExactly(const NativePeer& peer, std::size_t size)
{
    std::vector<uint8_t> bytes(size);
    std::size_t offset = 0;
    while (offset < bytes.size())
    {
        if (peer.waitReadable() != 1)
        {
            break;
        }
        const int received = peer.receiveBytes(bytes.data() + offset, bytes.size() - offset);
        if (received <= 0)
        {
            break;
        }
        offset += static_cast<std::size_t>(received);
    }
    bytes.resize(offset);
    return bytes;
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
            comm::CommEndpoint("127.0.0.1", simulator.localPort()),
            UNUSED_MONITOR
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

// 모의기에서 받은 자세 정보를 전시기용 자세 메시지로 전시기에 보낸다.
TEST(CommControllerTest, SendsSimulatorAttitudeToConsole)
{
    TestSocket apiLifetime;
    NativePeer console;
    NativePeer simulator;
    ASSERT_EQ(console.bind(), 0);
    ASSERT_EQ(simulator.bind(), 0);
    const uint16_t port = availablePort();
    comm::CommController controller(makeConfig(port, console, simulator));
    controller.openServer();
    ASSERT_EQ(console.connect(port), 0);
    ASSERT_EQ(simulator.connect(port), 0);

    comm::protocol::ST_RadarAttMsg received{};
    received.msgHeader.message_Id =
        static_cast<uint16_t>(comm::protocol::MessageId::RADAR_ATTITUDE_FROM_SIMULATOR);
    received.msgHeader.block_size = sizeof(received);
    received.radar_lat_deg = 37.5;
    received.radar_lon_deg = 127.25;
    received.radar_alt_km = 1.25;
    received.roll_deg = -12.5;
    received.pitch_deg = 6.25;
    received.yaw_deg = 45.5;
    std::vector<uint8_t> bytes(sizeof(received));
    std::memcpy(bytes.data(), &received, sizeof(received));
    for (uint8_t byte : bytes)
    {
        ASSERT_EQ(simulator.sendByte(byte), 1);
    }

    // receiveData()는 두 연결을 수락한 뒤 자세 메시지를 반환한다.
    const std::optional<comm::IncomingDto> incoming = controller.receiveData();
    ASSERT_TRUE(incoming);
    const auto* attitude = std::get_if<beam::SetAttitudeCommand>(&*incoming);
    ASSERT_NE(attitude, nullptr);
    controller.sendRadarAttitude(*attitude);

    std::vector<uint8_t> sent(sizeof(comm::protocol::ST_RadarAttMsg));
    std::size_t offset = 0;
    while (offset < sent.size())
    {
        ASSERT_EQ(console.waitReadable(), 1);
        const int size = console.receiveBytes(sent.data() + offset, sent.size() - offset);
        ASSERT_GT(size, 0);
        offset += static_cast<std::size_t>(size);
    }
    comm::protocol::ST_RadarAttMsg forwarded{};
    std::memcpy(&forwarded, sent.data(), sizeof(forwarded));
    EXPECT_EQ(
        forwarded.msgHeader.message_Id,
        static_cast<uint16_t>(comm::protocol::MessageId::RADAR_ATTITUDE_TO_CONSOLE)
    );
    EXPECT_EQ(forwarded.msgHeader.block_size, sizeof(forwarded));
    EXPECT_DOUBLE_EQ(forwarded.radar_lat_deg, 37.5);
    EXPECT_DOUBLE_EQ(forwarded.radar_lon_deg, 127.25);
    EXPECT_DOUBLE_EQ(forwarded.radar_alt_km, 1.25);
    EXPECT_DOUBLE_EQ(forwarded.roll_deg, -12.5);
    EXPECT_DOUBLE_EQ(forwarded.pitch_deg, 6.25);
    EXPECT_DOUBLE_EQ(forwarded.yaw_deg, 45.5);
}

// 모니터 송신은 모니터 연결에만 전달하고, 모니터가 없으면 무시한다.
TEST(NetworkManagerTest, RoutesMonitorSendsOnlyToMonitor)
{
    // 준비: 세 상대의 엔드포인트로 관리자를 만들고 모니터 연결 전 송신을 시도한다.
    TestSocket apiLifetime;
    NativePeer console;
    NativePeer simulator;
    NativePeer monitor;
    ASSERT_EQ(console.bind(), 0);
    ASSERT_EQ(simulator.bind(), 0);
    ASSERT_EQ(monitor.bind(), 0);
    const uint16_t port = availablePort();
    comm::NetworkManager manager(makeConfig(port, console, simulator, monitor));
    manager.start();
    EXPECT_NO_THROW(manager.sendToMonitor({0x10}, SOCKET_WAIT_TIMEOUT));

    ASSERT_EQ(console.connect(port), 0);
    EXPECT_FALSE(manager.poll(SOCKET_WAIT_TIMEOUT));
    ASSERT_EQ(monitor.connect(port), 0);
    EXPECT_FALSE(manager.poll(SOCKET_WAIT_TIMEOUT));

    // 실행: 모니터와 운용 콘솔에 서로 다른 바이트를 보낸다.
    manager.sendToMonitor({0x61}, SOCKET_WAIT_TIMEOUT);
    manager.sendToOperatorConsole({0x31}, SOCKET_WAIT_TIMEOUT);

    // 검증: 각 상대는 자신에게 보낸 바이트만 받는다. 연결 전 송신은 전달되지 않는다.
    EXPECT_EQ(receiveExactly(monitor, 1), (std::vector<uint8_t>{0x61}));
    EXPECT_EQ(receiveExactly(console, 1), (std::vector<uint8_t>{0x31}));
}

// 상대의 연결 여부는 연결 수락과 종료 감지에 따라 바뀐다.
TEST(NetworkManagerTest, ReportsPeerConnectionState)
{
    // 준비: 아직 아무 상대도 연결하지 않은 관리자를 준비한다.
    TestSocket apiLifetime;
    NativePeer console;
    NativePeer simulator;
    ASSERT_EQ(console.bind(), 0);
    ASSERT_EQ(simulator.bind(), 0);
    const uint16_t port = availablePort();
    comm::NetworkManager manager(makeConfig(port, console, simulator));
    manager.start();
    EXPECT_FALSE(manager.isOperatorConsoleConnected());
    EXPECT_FALSE(manager.isScenarioSimulatorConnected());

    // 실행: 두 상대를 연결한 뒤 운용 콘솔만 연결을 끊는다.
    ASSERT_EQ(console.connect(port), 0);
    EXPECT_FALSE(manager.poll(SOCKET_WAIT_TIMEOUT));
    ASSERT_EQ(simulator.connect(port), 0);
    EXPECT_FALSE(manager.poll(SOCKET_WAIT_TIMEOUT));
    const bool consoleConnectedBeforeClose = manager.isOperatorConsoleConnected();
    console.close();
    pollFor(manager, SOCKET_WAIT_TIMEOUT);

    // 검증: 연결된 동안에는 true이고, 종료를 감지한 상대만 false가 된다.
    EXPECT_TRUE(consoleConnectedBeforeClose);
    EXPECT_FALSE(manager.isOperatorConsoleConnected());
    EXPECT_TRUE(manager.isScenarioSimulatorConnected());
}

// 모니터에는 다른 상대에 보내는 것과 같은 형식의 메시지를 전달한다.
TEST(CommControllerTest, SendsMessageCopiesToMonitor)
{
    // 준비: 세 상대가 연결하고, 모의기가 자세 메시지를 보낸다.
    TestSocket apiLifetime;
    NativePeer console;
    NativePeer simulator;
    NativePeer monitor;
    ASSERT_EQ(console.bind(), 0);
    ASSERT_EQ(simulator.bind(), 0);
    ASSERT_EQ(monitor.bind(), 0);
    const uint16_t port = availablePort();
    comm::CommController controller(makeConfig(port, console, simulator, monitor));
    controller.openServer();
    // 대기 중인 연결은 들어온 순서로 수락한다. 모의기를 마지막에 연결해야
    // 자세 메시지를 수신하기 전에 모니터 연결이 수락된다.
    ASSERT_EQ(monitor.connect(port), 0);
    ASSERT_EQ(console.connect(port), 0);
    ASSERT_EQ(simulator.connect(port), 0);

    comm::protocol::ST_RadarAttMsg attitudeMessage{};
    attitudeMessage.msgHeader.message_Id =
        static_cast<uint16_t>(comm::protocol::MessageId::RADAR_ATTITUDE_FROM_SIMULATOR);
    attitudeMessage.msgHeader.block_size = sizeof(attitudeMessage);
    attitudeMessage.yaw_deg = 45.5;
    std::vector<uint8_t> attitudeBytes(sizeof(attitudeMessage));
    std::memcpy(attitudeBytes.data(), &attitudeMessage, sizeof(attitudeMessage));
    for (uint8_t byte : attitudeBytes)
    {
        ASSERT_EQ(simulator.sendByte(byte), 1);
    }

    // receiveData()는 세 연결을 수락한 뒤 자세 메시지를 반환한다.
    const std::optional<comm::IncomingDto> incoming = controller.receiveData();
    ASSERT_TRUE(incoming);
    const auto* attitude = std::get_if<beam::SetAttitudeCommand>(&*incoming);
    ASSERT_NE(attitude, nullptr);

    const beam::TransmitBeamCommand beamCommand(
        beam::BeamType::TRACKING,
        std::chrono::milliseconds{1234},
        21,
        7,
        math::SphericalDirection(
            math::Angle::fromDegrees(10.5),
            math::Angle::fromDegrees(-4.5)
        ),
        beam::BeamWidth(math::Angle::fromDegrees(3.0), math::Angle::fromDegrees(3.0))
    );

    // 실행: 빔 명령과 자세를 모니터에 보낸다.
    controller.sendToMonitor(beamCommand);
    controller.sendToMonitor(*attitude);

    // 검증: 모니터는 빔 송신 메시지와 전시기용 자세 메시지를 차례로 받는다.
    const std::vector<uint8_t> beamBytes =
        receiveExactly(monitor, sizeof(comm::protocol::ST_BeamTxMsg));
    ASSERT_EQ(beamBytes.size(), sizeof(comm::protocol::ST_BeamTxMsg));
    comm::protocol::ST_BeamTxMsg forwardedBeam{};
    std::memcpy(&forwardedBeam, beamBytes.data(), sizeof(forwardedBeam));
    EXPECT_EQ(
        forwardedBeam.msgHeader.message_Id,
        static_cast<uint16_t>(comm::protocol::MessageId::BEAM_TRANSMIT)
    );
    EXPECT_EQ(forwardedBeam.beam_cmd.beamID, 21U);
    EXPECT_EQ(forwardedBeam.beam_cmd.timestamp, 1234U);

    const std::vector<uint8_t> forwardedAttitudeBytes =
        receiveExactly(monitor, sizeof(comm::protocol::ST_RadarAttMsg));
    ASSERT_EQ(forwardedAttitudeBytes.size(), sizeof(comm::protocol::ST_RadarAttMsg));
    comm::protocol::ST_RadarAttMsg forwardedAttitude{};
    std::memcpy(&forwardedAttitude, forwardedAttitudeBytes.data(), sizeof(forwardedAttitude));
    EXPECT_EQ(
        forwardedAttitude.msgHeader.message_Id,
        static_cast<uint16_t>(comm::protocol::MessageId::RADAR_ATTITUDE_TO_CONSOLE)
    );
    EXPECT_DOUBLE_EQ(forwardedAttitude.yaw_deg, 45.5);

    // 실행: 처리부 상태를 모니터에 보낸다.
    comm::MonitorStatusRequest status;
    status.processorTime = std::chrono::milliseconds{42310};
    controller.sendToMonitor(status);

    // 검증: 상태 메시지에 요청 값과 송신 시점의 상대 연결 상태가 담긴다.
    const std::vector<uint8_t> statusBytes =
        receiveExactly(monitor, sizeof(comm::protocol::ST_MonitorStatusMsg));
    ASSERT_EQ(statusBytes.size(), sizeof(comm::protocol::ST_MonitorStatusMsg));
    comm::protocol::ST_MonitorStatusMsg forwardedStatus{};
    std::memcpy(&forwardedStatus, statusBytes.data(), sizeof(forwardedStatus));
    EXPECT_EQ(
        forwardedStatus.msgHeader.message_Id,
        static_cast<uint16_t>(comm::protocol::MessageId::MONITOR_STATUS)
    );
    EXPECT_EQ(forwardedStatus.processorTime_ms, 42310U);
    EXPECT_EQ(forwardedStatus.operatorConsoleConnected, 1);
    EXPECT_EQ(forwardedStatus.scenarioSimulatorConnected, 1);
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
