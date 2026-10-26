#include <comm/config/comm_config.hpp>

#include <gtest/gtest.h>

#include <cstdint>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <utility>

namespace
{

static_assert(std::is_const_v<decltype(
    std::declval<comm::CommEndpoint>().address)>);
static_assert(std::is_const_v<decltype(
    std::declval<comm::CommEndpoint>().port)>);
static_assert(std::is_const_v<decltype(
    std::declval<comm::CommConfig>().server)>);
static_assert(std::is_const_v<decltype(
    std::declval<comm::CommConfig>().operatorConsole)>);
static_assert(std::is_const_v<decltype(
    std::declval<comm::CommConfig>().scenarioSimulator)>);
static_assert(std::is_same_v<
    decltype(std::declval<comm::CommConfig>().operatorConsole),
    const comm::CommEndpoint>);
static_assert(std::is_same_v<
    decltype(std::declval<comm::CommConfig>().scenarioSimulator),
    const comm::CommEndpoint>);

TEST(CommEndpointTest, StoresValidEndpointAsImmutableValue)
{
    const comm::CommEndpoint endpoint("127.0.0.1", 3200);
    EXPECT_EQ(endpoint.address, "127.0.0.1");
    EXPECT_EQ(endpoint.port, 3200);
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
