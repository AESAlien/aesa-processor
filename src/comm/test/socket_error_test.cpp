#include <network/socket_error/socket_error.hpp>

#ifdef _WIN32
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif

#ifndef NOMINMAX
#define NOMINMAX
#endif

#include <winsock2.h>
#else
#include <cerrno>
#endif

#include <gtest/gtest.h>

namespace
{

#ifdef _WIN32
constexpr int INTERRUPTED_ERROR = WSAEINTR;
constexpr int WOULD_BLOCK_ERROR = WSAEWOULDBLOCK;
constexpr int CONNECTION_RESET_ERROR = WSAECONNRESET;
#else
constexpr int INTERRUPTED_ERROR = EINTR;
constexpr int WOULD_BLOCK_ERROR = EWOULDBLOCK;
constexpr int CONNECTION_RESET_ERROR = ECONNRESET;
#endif

class SocketErrorTest : public testing::Test
{
protected:
    void SetUp() override
    {
#ifdef _WIN32
        _previousError = WSAGetLastError();
#else
        _previousError = errno;
#endif
    }

    void TearDown() override
    {
        setNativeError(_previousError);
    }

    static void setNativeError(int errorCode) noexcept
    {
#ifdef _WIN32
        WSASetLastError(errorCode);
#else
        errno = errorCode;
#endif
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
#ifndef _WIN32
    EXPECT_TRUE(comm::SocketError(EAGAIN).isWouldBlock());
    EXPECT_TRUE(comm::SocketError(EWOULDBLOCK).isWouldBlock());
#endif
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
