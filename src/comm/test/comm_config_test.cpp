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
    const std::string&>);
static_assert(std::is_same_v<
    decltype(std::declval<const comm::CommConfig&>().operatorConsole()),
    const comm::CommEndpoint&>);

TEST(CommEndpointTest, StoresValidEndpointBehindAccessors)
{
    const comm::CommEndpoint endpoint("127.0.0.1", 3200);
    EXPECT_EQ(endpoint.getAddress(), "127.0.0.1");
    EXPECT_EQ(endpoint.getPort(), 3200);
}

TEST(CommEndpointTest, StoresIntAddressForSocketApi)
{
    const comm::CommEndpoint endpoint("1.2.3.4", 3200);
    const std::uint32_t intAddress = endpoint.getIntAddress();

    std::array<std::uint8_t, 4> octets{};
    std::memcpy(octets.data(), &intAddress, octets.size());

    EXPECT_EQ(octets, (std::array<std::uint8_t, 4>{1, 2, 3, 4}));
}

TEST(CommEndpointTest, RejectsMalformedAddresses)
{
    for (const char* address : {
             "1.2.3",
             "1.2.3.4.5",
             "1.2.3.",
             ".1.2.3",
             "256.0.0.1",
             "01.2.3.4",
             "1.2.3.a"})
    {
        EXPECT_THROW(
            comm::CommEndpoint(address, 3200),
            std::invalid_argument) << address;
    }
}

TEST(CommEndpointTest, RejectsMissingAddressOrPort)
{
    EXPECT_THROW(comm::CommEndpoint("", 0), std::invalid_argument);
    EXPECT_THROW(comm::CommEndpoint("", 3200), std::invalid_argument);
    EXPECT_THROW(comm::CommEndpoint("127.0.0.1", 0), std::invalid_argument);
}

TEST(CommEndpointTest, RejectsInvalidAddressAndIncompleteEndpoint)
{
    EXPECT_THROW(
        comm::CommEndpoint("300.1.1.1", 3200),
        std::invalid_argument);
    EXPECT_THROW(
        comm::CommEndpoint("127.0.0.1", 0),
        std::invalid_argument);
    EXPECT_THROW(
        comm::CommEndpoint("", 3200),
        std::invalid_argument);
    EXPECT_THROW(
        comm::CommEndpoint("127.000.0.1", 3200),
        std::invalid_argument);
}

TEST(CommConfigTest, AcceptsServerAndBothPeers)
{
    EXPECT_NO_THROW(comm::CommConfig(
        comm::CommEndpoint("127.0.0.1", 3200),
        comm::CommEndpoint("127.0.0.2", 3100),
        comm::CommEndpoint("127.0.0.3", 3300)));
}

TEST(CommConfigTest, RejectsInvalidEndpointRelationships)
{
    EXPECT_THROW(
        comm::CommConfig(
            comm::CommEndpoint("127.0.0.1", 3200),
            comm::CommEndpoint("127.0.0.2", 3100),
            comm::CommEndpoint("127.0.0.2", 3100)),
        std::invalid_argument);
}

} // namespace
