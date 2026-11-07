#include <network/message_framer/message_framer.hpp>

#include <gtest/gtest.h>

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <vector>

namespace
{

std::vector<uint8_t> makeFrameBytes(
    comm::protocol::MessageId messageId,
    std::size_t frameSize
) {
    comm::protocol::ST_MsgHeader header{};
    header.message_Id = static_cast<uint16_t>(messageId);
    header.block_size = static_cast<uint32_t>(frameSize);

    std::vector<uint8_t> bytes(frameSize, 0xAB);
    std::memcpy(bytes.data(), &header, sizeof(header));
    return bytes;
}

std::vector<uint8_t> concatenate(
    const std::vector<uint8_t>& first,
    const std::vector<uint8_t>& second
) {
    std::vector<uint8_t> bytes(first);
    bytes.insert(bytes.end(), second.begin(), second.end());
    return bytes;
}

constexpr std::size_t HEADER_SIZE = sizeof(comm::protocol::ST_MsgHeader);

} // namespace

// 완성된 프레임을 입력하면 헤더와 원본 바이트를 함께 반환한다.
TEST(MessageFramerTest, ReturnsCompleteFrame)
{
    // Given: 완성된 운용 제어 프레임을 준비한다.
    comm::MessageFramer framer;
    const std::vector<uint8_t> bytes = makeFrameBytes(
        comm::protocol::MessageId::OPERATOR_CONTROL,
        HEADER_SIZE + 4
    );

    // When: 프레임 전체를 한 번에 입력한다.
    const auto frames = framer.append(bytes);

    // Then: 원본 바이트와 메시지 ID가 같은 프레임 하나를 반환한다.
    ASSERT_EQ(frames.size(), 1U);
    EXPECT_EQ(frames[0].bytes, bytes);
    EXPECT_EQ(
        frames[0].header.message_Id,
        static_cast<uint16_t>(comm::protocol::MessageId::OPERATOR_CONTROL)
    );
}

// 나누어 받은 프레임은 남은 데이터가 도착할 때까지 반환하지 않는다.
TEST(MessageFramerTest, WaitsUntilSplitFrameIsComplete)
{
    // Given: 헤더 중간에서 두 조각으로 나눈 프레임을 준비한다.
    comm::MessageFramer framer;
    const std::vector<uint8_t> bytes = makeFrameBytes(
        comm::protocol::MessageId::PLOT_INFORMATION,
        HEADER_SIZE + 10
    );

    const std::vector<uint8_t> firstPart(bytes.begin(), bytes.begin() + 7);
    const std::vector<uint8_t> secondPart(bytes.begin() + 7, bytes.end());

    // When: 앞 조각과 뒤 조각을 차례로 입력한다.
    const auto incomplete = framer.append(firstPart);
    const auto frames = framer.append(secondPart);

    // Then: 앞 조각만으로는 반환하지 않고 두 조각이 모이면 원본 프레임을 반환한다.
    EXPECT_TRUE(incomplete.empty());
    ASSERT_EQ(frames.size(), 1U);
    EXPECT_EQ(frames[0].bytes, bytes);
}

// 여러 프레임이 한 번에 도착하면 입력 순서대로 모두 반환한다.
TEST(MessageFramerTest, ReturnsEveryFrameInOneAppend)
{
    // Given: 종류와 길이가 서로 다른 프레임 두 개를 준비한다.
    comm::MessageFramer framer;
    const std::vector<uint8_t> first = makeFrameBytes(
        comm::protocol::MessageId::OPERATOR_CONTROL,
        HEADER_SIZE + 4
    );
    const std::vector<uint8_t> second = makeFrameBytes(
        comm::protocol::MessageId::RADAR_ATTITUDE_FROM_SIMULATOR,
        HEADER_SIZE + 8
    );

    const auto bytes = concatenate(first, second);

    // When: 이어 붙인 두 프레임을 한 번에 입력한다.
    const auto frames = framer.append(bytes);

    // Then: 두 프레임을 원래 순서와 바이트 그대로 반환한다.
    ASSERT_EQ(frames.size(), 2U);
    EXPECT_EQ(frames[0].bytes, first);
    EXPECT_EQ(frames[1].bytes, second);
}

// 정의되지 않은 메시지 ID를 가진 프레임을 거부한다.
TEST(MessageFramerTest, ThrowsOnUnknownMessageId)
{
    // Given: 메시지 ID를 정의되지 않은 값으로 바꾼 프레임을 준비한다.
    comm::MessageFramer framer;
    std::vector<uint8_t> bytes = makeFrameBytes(
        comm::protocol::MessageId::OPERATOR_CONTROL,
        HEADER_SIZE
    );
    bytes[0] = 0xFF;
    bytes[1] = 0xFF;

    // When & Then: 정의되지 않은 ID의 프레임을 입력하면 프레이밍 오류를 던진다.
    EXPECT_THROW(framer.append(bytes), comm::FramingError);
}

// 헤더보다 작은 크기를 선언한 프레임을 거부한다.
TEST(MessageFramerTest, ThrowsOnFrameSmallerThanHeader)
{
    // Given: 선언한 크기가 헤더보다 한 바이트 작은 프레임을 준비한다.
    comm::MessageFramer framer;
    const std::vector<uint8_t> bytes = makeFrameBytes(
        comm::protocol::MessageId::OPERATOR_CONTROL,
        HEADER_SIZE
    );

    std::vector<uint8_t> invalidBytes(bytes);
    comm::protocol::ST_MsgHeader header{};
    std::memcpy(&header, invalidBytes.data(), sizeof(header));
    header.block_size = HEADER_SIZE - 1;
    std::memcpy(invalidBytes.data(), &header, sizeof(header));

    // When & Then: 잘못된 크기의 프레임을 입력하면 프레이밍 오류를 던진다.
    EXPECT_THROW(framer.append(invalidBytes), comm::FramingError);
}

// 최대 허용 크기보다 큰 프레임을 거부한다.
TEST(MessageFramerTest, ThrowsOnFrameLargerThanMaximum)
{
    // Given: 최대 허용 크기를 한 바이트 초과한 프레임을 준비한다.
    comm::MessageFramer framer(HEADER_SIZE + 4);
    const std::vector<uint8_t> bytes = makeFrameBytes(
        comm::protocol::MessageId::OPERATOR_CONTROL,
        HEADER_SIZE + 5
    );

    // When & Then: 최대 크기를 초과한 프레임을 입력하면 프레이밍 오류를 던진다.
    EXPECT_THROW(framer.append(bytes), comm::FramingError);
}

// 초기화하면 이전에 수신한 미완성 데이터가 다음 프레임에 섞이지 않는다.
TEST(MessageFramerTest, DiscardsBufferedBytesOnReset)
{
    // Given: 프레임의 앞부분만 입력한 프레이머를 준비한다.
    comm::MessageFramer framer;
    const std::vector<uint8_t> bytes = makeFrameBytes(
        comm::protocol::MessageId::OPERATOR_CONTROL,
        HEADER_SIZE + 4
    );

    framer.append(std::vector<uint8_t>(bytes.begin(), bytes.begin() + 5));

    // When: 프레이머를 초기화한 뒤 완성된 프레임을 다시 입력한다.
    framer.reset();
    const auto frames = framer.append(bytes);

    // Then: 초기화 전에 남은 데이터 없이 원본 프레임 하나를 반환한다.
    ASSERT_EQ(frames.size(), 1U);
    EXPECT_EQ(frames[0].bytes, bytes);
}

// 프레임을 어느 위치에서 나누어 받아도 원본을 한 번만 반환한다.
TEST(MessageFramerTest, RestoresFrameAtEverySplitPosition)
{
    // Given: 헤더와 본문의 모든 분할 위치를 검사할 프레임을 준비한다.
    const std::vector<uint8_t> bytes = makeFrameBytes(
        comm::protocol::MessageId::PLOT_INFORMATION,
        HEADER_SIZE + 10
    );

    for (std::size_t split = 1; split < bytes.size(); ++split)
    {
        SCOPED_TRACE(split);
        comm::MessageFramer framer;
        const auto splitPosition = bytes.begin() + static_cast<std::ptrdiff_t>(split);
        const std::vector<uint8_t> firstPart(bytes.begin(), splitPosition);
        const std::vector<uint8_t> secondPart(splitPosition, bytes.end());

        // When: 두 조각을 차례로 입력한 뒤 새 데이터 없이 다시 조회한다.
        const auto incomplete = framer.append(firstPart);
        const auto frames = framer.append(secondPart);
        const auto repeated = framer.append({});

        // Then: 두 조각이 모였을 때만 원본 프레임 하나를 반환한다.
        EXPECT_TRUE(incomplete.empty());
        ASSERT_EQ(frames.size(), 1U);
        EXPECT_EQ(frames.front().bytes, bytes);
        EXPECT_TRUE(repeated.empty());
    }
}

// 완성된 프레임을 반환한 뒤에도 다음 프레임의 미완성 데이터를 보존한다.
TEST(MessageFramerTest, PreservesIncompleteFrameAfterReturningCompleteFrame)
{
    // Given: 첫 프레임 전체와 두 번째 프레임 일부가 함께 도착하는 입력을 준비한다.
    const std::vector<uint8_t> first = makeFrameBytes(
        comm::protocol::MessageId::OPERATOR_CONTROL,
        HEADER_SIZE + 4
    );
    const std::vector<uint8_t> second = makeFrameBytes(
        comm::protocol::MessageId::PLOT_INFORMATION,
        HEADER_SIZE + 40
    );

    // 남은 데이터가 짧거나 길어도 버퍼 정리 여부에 관계없이 보존되어야 한다.
    for (std::size_t split : {std::size_t{7}, second.size() - 1})
    {
        SCOPED_TRACE(split);
        comm::MessageFramer framer;
        const auto splitPosition = second.begin() + static_cast<std::ptrdiff_t>(split);
        const auto firstInput = concatenate(first, std::vector<uint8_t>(second.begin(), splitPosition));
        const std::vector<uint8_t> secondInput(splitPosition, second.end());

        // When: 첫 입력과 빈 입력 및 남은 데이터를 차례로 전달한다.
        const auto frames = framer.append(firstInput);
        const auto incomplete = framer.append({});
        const auto remaining = framer.append(secondInput);
        const auto repeated = framer.append({});

        // Then: 각 프레임은 완성된 시점에 한 번씩만 반환된다.
        ASSERT_EQ(frames.size(), 1U);
        EXPECT_EQ(frames.front().bytes, first);
        EXPECT_TRUE(incomplete.empty());
        ASSERT_EQ(remaining.size(), 1U);
        EXPECT_EQ(remaining.front().bytes, second);
        EXPECT_TRUE(repeated.empty());
    }
}

// 한 바이트씩 연속 수신해도 프레임의 순서와 내용을 보존한다.
TEST(MessageFramerTest, RestoresConsecutiveFramesOneByteAtATime)
{
    // Given: 종류와 길이가 다른 프레임 두 개를 이어 붙인다.
    comm::MessageFramer framer;
    const std::vector<uint8_t> first = makeFrameBytes(
        comm::protocol::MessageId::OPERATOR_CONTROL,
        HEADER_SIZE + 4
    );
    const std::vector<uint8_t> second = makeFrameBytes(
        comm::protocol::MessageId::RADAR_ATTITUDE_FROM_SIMULATOR,
        HEADER_SIZE + 8
    );
    std::vector<comm::MessageFrame> frames;
    const auto bytes = concatenate(first, second);

    // When: 모든 바이트를 하나씩 입력하고 완성된 프레임을 모은다.
    for (uint8_t byte : bytes)
    {
        const auto completed = framer.append({byte});
        frames.insert(frames.end(), completed.begin(), completed.end());
    }
    const auto repeated = framer.append({});

    // Then: 누락과 중복 없이 원본 프레임 두 개를 순서대로 반환한다.
    ASSERT_EQ(frames.size(), 2U);
    EXPECT_EQ(frames[0].bytes, first);
    EXPECT_EQ(frames[1].bytes, second);
    EXPECT_TRUE(repeated.empty());
}

// 최대 허용 크기와 정확히 같은 프레임은 수신할 수 있다.
TEST(MessageFramerTest, AcceptsFrameExactlyAtMaximumSize)
{
    // Given: 최대 허용 크기와 길이가 같은 프레임을 준비한다.
    constexpr std::size_t MAXIMUM_SIZE = HEADER_SIZE + 4;
    comm::MessageFramer framer(MAXIMUM_SIZE);
    const auto bytes = makeFrameBytes(comm::protocol::MessageId::OPERATOR_CONTROL, MAXIMUM_SIZE);

    // When: 최대 크기의 프레임을 입력한다.
    const auto frames = framer.append(bytes);

    // Then: 원본 바이트를 가진 프레임 하나를 반환한다.
    ASSERT_EQ(frames.size(), 1U);
    EXPECT_EQ(frames.front().bytes, bytes);
}

// 최대 프레임 크기는 헤더 크기보다 작게 설정할 수 없다.
TEST(MessageFramerTest, RejectsMaximumSizeSmallerThanHeader)
{
    // Given: 0과 헤더보다 한 바이트 작은 최대 크기를 준비한다.
    const std::size_t invalidSizes[] = {0, HEADER_SIZE - 1};

    // When: 각 크기로 프레이머 생성을 시도한다.
    const auto construct = [](std::size_t size) { return comm::MessageFramer(size); };

    // Then: 헤더보다 작은 최대 크기는 모두 거부한다.
    for (std::size_t size : invalidSizes)
    {
        EXPECT_THROW(construct(size), std::invalid_argument);
    }
}

// 최대 크기가 헤더 크기이면 헤더만 있는 프레임을 수신할 수 있다.
TEST(MessageFramerTest, AcceptsHeaderOnlyFrameAtMinimumMaximumSize)
{
    // Given: 최소 허용 최대 크기의 프레이머와 헤더만 있는 프레임을 준비한다.
    comm::MessageFramer framer(HEADER_SIZE);
    const auto bytes = makeFrameBytes(comm::protocol::MessageId::OPERATOR_CONTROL, HEADER_SIZE);

    // When: 헤더만 있는 프레임을 입력한다.
    const auto frames = framer.append(bytes);

    // Then: 헤더 바이트를 가진 프레임 하나를 반환한다.
    ASSERT_EQ(frames.size(), 1U);
    EXPECT_EQ(frames.front().bytes, bytes);
}
