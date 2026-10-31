#include <network/message_framer/message_framer.hpp>

#include <gtest/gtest.h>

#include <cstdint>
#include <cstring>
#include <vector>

namespace
{

std::vector<uint8_t> makeFrameBytes(
    comm::protocol::MessageId messageId,
    std::size_t frameSize
)
{
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
)
{
    std::vector<uint8_t> bytes(first);
    bytes.insert(bytes.end(), second.begin(), second.end());
    return bytes;
}

constexpr std::size_t HEADER_SIZE = sizeof(comm::protocol::ST_MsgHeader);

} // namespace

TEST(MessageFramerTest, ReturnsCompleteFrame)
{
    comm::MessageFramer framer;
    const std::vector<uint8_t> bytes = makeFrameBytes(
        comm::protocol::MessageId::OperatorControl,
        HEADER_SIZE + 4);

    const std::vector<comm::MessageFrame> frames = framer.append(bytes);

    ASSERT_EQ(frames.size(), 1U);
    EXPECT_EQ(frames[0].bytes, bytes);
    EXPECT_EQ(
        frames[0].header.message_Id,
        static_cast<uint16_t>(comm::protocol::MessageId::OperatorControl));
}

TEST(MessageFramerTest, WaitsUntilSplitFrameIsComplete)
{
    comm::MessageFramer framer;
    const std::vector<uint8_t> bytes = makeFrameBytes(
        comm::protocol::MessageId::PlotInformation,
        HEADER_SIZE + 10);

    const std::vector<uint8_t> firstPart(bytes.begin(), bytes.begin() + 7);
    const std::vector<uint8_t> secondPart(bytes.begin() + 7, bytes.end());

    EXPECT_TRUE(framer.append(firstPart).empty());

    const std::vector<comm::MessageFrame> frames = framer.append(secondPart);
    ASSERT_EQ(frames.size(), 1U);
    EXPECT_EQ(frames[0].bytes, bytes);
}

TEST(MessageFramerTest, ReturnsEveryFrameInOneAppend)
{
    comm::MessageFramer framer;
    const std::vector<uint8_t> first = makeFrameBytes(
        comm::protocol::MessageId::OperatorControl,
        HEADER_SIZE + 4);
    const std::vector<uint8_t> second = makeFrameBytes(
        comm::protocol::MessageId::RadarAttitudeFromSimulator,
        HEADER_SIZE + 8);

    const std::vector<comm::MessageFrame> frames =
        framer.append(concatenate(first, second));

    ASSERT_EQ(frames.size(), 2U);
    EXPECT_EQ(frames[0].bytes, first);
    EXPECT_EQ(frames[1].bytes, second);
}

TEST(MessageFramerTest, ThrowsOnUnknownMessageId)
{
    comm::MessageFramer framer;
    std::vector<uint8_t> bytes = makeFrameBytes(
        comm::protocol::MessageId::OperatorControl,
        HEADER_SIZE);
    bytes[0] = 0xFF;
    bytes[1] = 0xFF;

    EXPECT_THROW(framer.append(bytes), comm::FramingError);
}

TEST(MessageFramerTest, ThrowsOnFrameSmallerThanHeader)
{
    comm::MessageFramer framer;
    const std::vector<uint8_t> bytes = makeFrameBytes(
        comm::protocol::MessageId::OperatorControl,
        HEADER_SIZE);

    std::vector<uint8_t> invalidBytes(bytes);
    comm::protocol::ST_MsgHeader header{};
    std::memcpy(&header, invalidBytes.data(), sizeof(header));
    header.block_size = HEADER_SIZE - 1;
    std::memcpy(invalidBytes.data(), &header, sizeof(header));

    EXPECT_THROW(framer.append(invalidBytes), comm::FramingError);
}

TEST(MessageFramerTest, ThrowsOnFrameLargerThanMaximum)
{
    comm::MessageFramer framer(HEADER_SIZE + 4);
    const std::vector<uint8_t> bytes = makeFrameBytes(
        comm::protocol::MessageId::OperatorControl,
        HEADER_SIZE + 5);

    EXPECT_THROW(framer.append(bytes), comm::FramingError);
}

TEST(MessageFramerTest, DiscardsBufferedBytesOnReset)
{
    comm::MessageFramer framer;
    const std::vector<uint8_t> bytes = makeFrameBytes(
        comm::protocol::MessageId::OperatorControl,
        HEADER_SIZE + 4);

    framer.append(std::vector<uint8_t>(bytes.begin(), bytes.begin() + 5));
    framer.reset();

    const std::vector<comm::MessageFrame> frames = framer.append(bytes);
    ASSERT_EQ(frames.size(), 1U);
    EXPECT_EQ(frames[0].bytes, bytes);
}
