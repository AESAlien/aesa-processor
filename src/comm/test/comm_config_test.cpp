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

TEST(CommEndpointTest, StoresValidEndpointAsImmutableValue)
{
    const comm::CommEndpoint endpoint("127.0.0.1", 3200);

    EXPECT_EQ(endpoint.address, "127.0.0.1");
    EXPECT_EQ(endpoint.port, 3200);
    EXPECT_FALSE(endpoint.isDisabled());
}

TEST(CommEndpointTest, AllowsExplicitlyDisabledEndpoint)
{
    const comm::CommEndpoint endpoint("", 0);

    EXPECT_TRUE(endpoint.isDisabled());
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

TEST(CommConfigurationTest, AcceptsServerAndAtLeastOnePeer)
{
    EXPECT_NO_THROW(comm::CommConfiguration(
        comm::CommEndpoint("127.0.0.1", 3200),
        comm::CommEndpoint("127.0.0.2", 3100),
        comm::CommEndpoint("", 0)));
}

TEST(CommConfigurationTest, RejectsInvalidEndpointRelationships)
{
    EXPECT_THROW(
        comm::CommConfiguration(
            comm::CommEndpoint("", 0),
            comm::CommEndpoint("127.0.0.2", 3100),
            comm::CommEndpoint("", 0)),
        std::invalid_argument);
    EXPECT_THROW(
        comm::CommConfiguration(
            comm::CommEndpoint("127.0.0.1", 3200),
            comm::CommEndpoint("", 0),
            comm::CommEndpoint("", 0)),
        std::invalid_argument);
    EXPECT_THROW(
        comm::CommConfiguration(
            comm::CommEndpoint("127.0.0.1", 3200),
            comm::CommEndpoint("127.0.0.2", 3100),
            comm::CommEndpoint("127.0.0.2", 3100)),
        std::invalid_argument);
}

} // namespace
