#include <comm/error/comm_error.hpp>

#include <controller/comm_error_handler.hpp>
#include <network/network_error.hpp>

#include <gtest/gtest.h>

#include <string>

namespace
{

template<typename Error, typename Function>
std::string getErrorMessage(Function function)
{
    try
    {
        function();
    }
    catch (const Error& error)
    {
        return error.what();
    }
    catch (...)
    {
        ADD_FAILURE() << "An unexpected exception type was thrown";
        return {};
    }

    ADD_FAILURE() << "No exception was thrown";
    return {};
}

} // namespace

TEST(CommErrorHandlerTest, IncludesSystemErrorDetailsInExceptionMessage)
{
    const std::string message = getErrorMessage<comm::ServerOpenError>(
        []()
        {
            comm::throwSystemError<comm::ServerOpenError>(
                "Communication server could not be opened",
                "bind",
                10048);
        });

    EXPECT_EQ(
        message,
        "Communication server could not be opened: operation=bind, "
        "systemError=10048");
}

TEST(CommErrorHandlerTest, IncludesDecodingFailureDetailsInExceptionMessage)
{
    const std::string message = getErrorMessage<comm::ReceiveError>(
        []()
        {
            comm::controller_error::throwDecodingFailure(
                comm::NetworkPeer::SCENARIO_SIMULATOR,
                comm::DecodingError::UNSUPPORTED_VERSION);
        });

    EXPECT_EQ(
        message,
        "Communication data could not be received: stage=decoding, "
        "peer=scenario-simulator, error=unsupported-version");
}

TEST(CommErrorHandlerTest, IncludesEncodingFailureDetailsInExceptionMessage)
{
    const std::string message = getErrorMessage<comm::SendError>(
        []()
        {
            comm::controller_error::throwEncodingFailure(
                comm::NetworkPeer::SCENARIO_SIMULATOR,
                comm::EncodingError::TYPE_CONVERSION_ERROR);
        });

    EXPECT_EQ(
        message,
        "Communication data could not be sent: stage=encoding, "
        "peer=scenario-simulator, error=type-conversion-error");
}
