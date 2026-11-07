#include <network/socket_error/socket_error.hpp>

#include <native_socket/native_socket_test_support.hpp>

#include <gtest/gtest.h>

namespace
{

const auto ERROR_CODES = comm::native_socket::test::errorCodes();
const int INTERRUPTED_ERROR = ERROR_CODES.interrupted;
const int WOULD_BLOCK_ERROR = ERROR_CODES.wouldBlock;
const int CONNECTION_RESET_ERROR = ERROR_CODES.connectionReset;

class SocketErrorTest : public testing::Test
{
protected:
    void SetUp() override
    {
        _previousError = comm::native_socket::lastError();
    }

    void TearDown() override
    {
        setNativeError(_previousError);
    }

    static void setNativeError(int errorCode) noexcept
    {
        comm::native_socket::test::setLastError(errorCode);
    }

private:
    int _previousError = 0;
};

TEST_F(SocketErrorTest, CapturesNativeErrorOnce)
{
    setNativeError(WOULD_BLOCK_ERROR);
    const comm::SocketError captured = comm::SocketError::captureLast();
    setNativeError(INTERRUPTED_ERROR);
    const comm::SocketError current = comm::SocketError::captureLast();

    EXPECT_EQ(captured.code().value(), WOULD_BLOCK_ERROR);
    EXPECT_TRUE(captured.isWouldBlock());
    EXPECT_FALSE(captured.isInterrupted());
    EXPECT_EQ(current.code().value(), INTERRUPTED_ERROR);
    EXPECT_TRUE(current.isInterrupted());
    EXPECT_FALSE(current.isWouldBlock());
}

TEST_F(SocketErrorTest, ExplicitCodeIsIndependentOfNativeErrorState)
{
    setNativeError(WOULD_BLOCK_ERROR);
    const comm::SocketError error(INTERRUPTED_ERROR);
    setNativeError(CONNECTION_RESET_ERROR);

    EXPECT_EQ(error.code().value(), INTERRUPTED_ERROR);
    EXPECT_TRUE(error.isInterrupted());
    EXPECT_FALSE(error.isWouldBlock());
    EXPECT_TRUE(comm::SocketError(ERROR_CODES.tryAgain).isWouldBlock());
    EXPECT_TRUE(comm::SocketError(ERROR_CODES.wouldBlock).isWouldBlock());
}

TEST_F(SocketErrorTest, PreservesStandardCodeForUnclassifiedErrors)
{
    const comm::SocketError error(CONNECTION_RESET_ERROR);
    const std::system_error exception(error.code(), "receive");

    EXPECT_EQ(exception.code().value(), CONNECTION_RESET_ERROR);
    EXPECT_EQ(exception.code().category(), std::system_category());
    EXPECT_FALSE(error.isInterrupted());
    EXPECT_FALSE(error.isWouldBlock());
}

} // namespace
