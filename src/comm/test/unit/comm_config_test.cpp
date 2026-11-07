#include <comm/config/comm_config.hpp>

#include <gtest/gtest.h>

#include <array>
#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <utility>

namespace
{

static_assert(std::is_copy_assignable_v<comm::CommEndpoint>);
static_assert(std::is_copy_assignable_v<comm::CommConfig>);
static_assert(std::is_same_v<
    decltype(std::declval<const comm::CommEndpoint&>().getAddress()),
    const std::string&>
);
static_assert(std::is_same_v<
    decltype(std::declval<const comm::CommConfig&>().operatorConsole()),
    const comm::CommEndpoint&>
);

} // namespace

// 엔드포인트는 전달받은 주소와 포트를 보관한다.
TEST(CommEndpointTest, StoresValidEndpointBehindAccessors)
{
    // Given: 유효한 IPv4 주소와 포트를 준비한다.
    const std::string address = "127.0.0.1";
    const uint16_t port = 3200;

    // When: 주소와 포트로 통신 엔드포인트를 생성한다.
    const comm::CommEndpoint endpoint(address, port);

    // Then: 접근자로 전달한 주소와 포트를 조회할 수 있다.
    EXPECT_EQ(endpoint.getAddress(), "127.0.0.1");
    EXPECT_EQ(endpoint.getPort(), 3200);
}

// 소켓 API에 전달할 정수 주소는 네트워크 바이트 순서로 저장한다.
TEST(CommEndpointTest, StoresIntAddressForSocketApi)
{
    // Given: 각 옥텟을 구분할 수 있는 IPv4 주소를 준비한다.
    const comm::CommEndpoint endpoint("1.2.3.4", 3200);

    // When: 소켓 API용 정수 주소를 읽어 메모리의 바이트 순서를 확인한다.
    const std::uint32_t intAddress = endpoint.getIntAddress();

    std::array<std::uint8_t, 4> octets{};
    std::memcpy(octets.data(), &intAddress, octets.size());

    // Then: 호스트 바이트 순서에 관계없이 옥텟이 원래 주소 순서로 저장된다.
    EXPECT_EQ(octets, (std::array<std::uint8_t, 4>{1, 2, 3, 4}));
}

// IPv4 형식과 옥텟 범위 및 선행 0 규칙을 벗어난 주소를 거부한다.
TEST(CommEndpointTest, RejectsMalformedAddresses)
{
    // Given: 옥텟의 개수와 범위 또는 표기가 잘못된 주소들을 준비한다.
    const char* invalidAddresses[] =
    {
        "1.2.3",
        "1.2.3.4.5",
        "1.2.3.",
        ".1.2.3",
        "256.0.0.1",
        "01.2.3.4",
        "1.2.3.a"
    };

    // When: 각 주소로 통신 엔드포인트 생성을 시도한다.
    const auto construct = [](const char* address) { return comm::CommEndpoint(address, 3200); };

    // Then: 잘못된 주소는 모두 거부한다.
    for (const char* address : invalidAddresses)
    {
        EXPECT_THROW(
            construct(address),
            std::invalid_argument
        ) << address;
    }
}

// 주소가 비어 있거나 포트가 0이면 통신 엔드포인트를 생성할 수 없다.
TEST(CommEndpointTest, RejectsMissingAddressOrPort)
{
    // Given: 빈 주소와 포트 0 및 정상 주소와 포트를 준비한다.
    const std::string missingAddress;
    const std::string validAddress = "127.0.0.1";
    const uint16_t missingPort = 0;
    const uint16_t validPort = 3200;

    // When: 주소 또는 포트가 빠진 엔드포인트 생성을 시도한다.
    const auto construct = [](const std::string& address, uint16_t port) { return comm::CommEndpoint(address, port); };

    // Then: 주소와 포트 중 하나라도 빠지면 거부한다.
    EXPECT_THROW(construct(missingAddress, missingPort), std::invalid_argument);
    EXPECT_THROW(construct(missingAddress, validPort), std::invalid_argument);
    EXPECT_THROW(construct(validAddress, missingPort), std::invalid_argument);
}

// 잘못된 주소와 필수 값이 빠진 엔드포인트를 거부한다.
TEST(CommEndpointTest, RejectsInvalidAddressAndIncompleteEndpoint)
{
    // Given: 옥텟 범위 초과와 선행 0 및 주소나 포트가 빠진 입력을 준비한다.
    const std::string outOfRangeAddress = "300.1.1.1";
    const std::string leadingZeroAddress = "127.000.0.1";
    const std::string missingAddress;
    const std::string validAddress = "127.0.0.1";

    // When: 각 입력으로 엔드포인트 생성을 시도한다.
    const auto construct = [](const std::string& address, uint16_t port) { return comm::CommEndpoint(address, port); };

    // Then: 잘못된 주소와 불완전한 엔드포인트는 모두 거부한다.
    EXPECT_THROW(
        construct(outOfRangeAddress, 3200),
        std::invalid_argument
    );
    EXPECT_THROW(
        construct(validAddress, 0),
        std::invalid_argument
    );
    EXPECT_THROW(
        construct(missingAddress, 3200),
        std::invalid_argument
    );
    EXPECT_THROW(
        construct(leadingZeroAddress, 3200),
        std::invalid_argument
    );
}

// 서버와 서로 다른 두 피어의 엔드포인트로 통신 설정을 생성할 수 있다.
TEST(CommConfigTest, AcceptsServerAndBothPeers)
{
    // Given: 서버와 운용 콘솔 및 시나리오 시뮬레이터의 엔드포인트를 준비한다.
    const comm::CommEndpoint server("127.0.0.1", 3200);
    const comm::CommEndpoint operatorConsole("127.0.0.2", 3100);
    const comm::CommEndpoint scenarioSimulator("127.0.0.3", 3300);

    // When: 준비한 엔드포인트들로 통신 설정 생성을 시도한다.
    const auto construct = [&] { return comm::CommConfig(server, operatorConsole, scenarioSimulator); };

    // Then: 서로 다른 두 피어를 가진 설정은 정상 생성된다.
    EXPECT_NO_THROW(construct());
}

// 운용 콘솔과 시나리오 시뮬레이터의 엔드포인트가 같으면 설정을 거부한다.
TEST(CommConfigTest, RejectsInvalidEndpointRelationships)
{
    // Given: 두 피어가 공유하는 주소와 포트를 준비한다.
    const comm::CommEndpoint server("127.0.0.1", 3200);
    const comm::CommEndpoint sharedPeer("127.0.0.2", 3100);

    // When: 두 피어에 같은 엔드포인트를 지정해 설정 생성을 시도한다.
    const auto construct = [&] { return comm::CommConfig(server, sharedPeer, sharedPeer); };

    // Then: 두 피어를 구분할 수 없는 설정은 거부한다.
    EXPECT_THROW(construct(), std::invalid_argument);
}
