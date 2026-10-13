#include <comm/controller/comm_error.hpp>

#include <controller/comm_error_handler.hpp>

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

TEST(CommErrorHandlerTest, IncludesServerOpenFailureDetailsInExceptionMessage)
{
    const comm::OperationResult result
    {
        comm::OperationStatus::ERROR,
        comm::NetworkOperation::BIND,
        10048
    };

    const std::string message = getErrorMessage<comm::ServerOpenError>(
        [&result]()
        {
            comm::controller_error::throwServerOpenFailure(result);
        });

    EXPECT_EQ(
        message,
        "Communication server could not be opened: operation=bind, "
        "systemError=10048");
}

TEST(CommErrorHandlerTest, IncludesConnectionFailureDetailsInExceptionMessage)
{
    const comm::PollResult result
    {
        comm::PollStatus::ERROR,
        {},
        comm::NetworkOperation::ACCEPT,
        10061
    };

    const std::string message = getErrorMessage<comm::ConnectionError>(
        [&result]()
        {
            comm::controller_error::throwPollFailure(result);
        });

    EXPECT_EQ(
        message,
        "Communication connection could not be established: "
        "operation=accept, systemError=10061");
}

TEST(CommErrorHandlerTest, IncludesReceiveFailureDetailsInExceptionMessage)
{
    const std::string message = getErrorMessage<comm::ReceiveError>(
        []()
        {
            comm::controller_error::throwReceiveFailure(
                comm::NetworkOperation::RECEIVE,
                10054);
        });

    EXPECT_EQ(
        message,
        "Communication data could not be received: operation=receive, "
        "systemError=10054");
}

TEST(CommErrorHandlerTest, IncludesFramingFailureDetailsInExceptionMessage)
{
    const std::string message = getErrorMessage<comm::ReceiveError>(
        []()
        {
            comm::controller_error::throwFramingFailure(
                comm::NetworkPeer::OPERATOR_CONSOLE,
                comm::FramingError::INVALID_FRAME_SIZE);
        });

    EXPECT_EQ(
        message,
        "Communication data could not be received: stage=framing, "
        "peer=operator-console, error=invalid-frame-size");
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
