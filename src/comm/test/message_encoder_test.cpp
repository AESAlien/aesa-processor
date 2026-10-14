#include <network/message_encoder.hpp>

#include <protocol/ST_BeamTxMsg.hpp>
#include <protocol/ST_TrkMsg.hpp>

#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>
#include <type_traits>
#include <vector>

namespace
{

using namespace std::chrono_literals;

beam::BeamCommand makeBeamCommand(
    std::chrono::milliseconds transmitTime = 1234ms,
    double azimuth = 10.5
)
{
    return beam::BeamCommand(
        beam::BeamCommand::BeamType::TRACKING,
        transmitTime,
        21,
        7,
        math::Angle::fromDegrees(azimuth),
        math::Angle::fromDegrees(-4.5),
        math::Angle::fromDegrees(3.25),
        math::Angle::fromDegrees(2.75));
}

target::TrackEvent makeTrackEvent(
    std::chrono::milliseconds timestamp = 5678ms,
    double slantRangeKilometers = 12.5
)
{
    return target::TrackEvent(
        42,
        target::TrackEvent::TrackState::TRACKING,
        math::Distance::fromKilometers(slantRangeKilometers),
        math::Distance::fromKilometers(10.25),
        math::Angle::fromDegrees(15.5),
        math::Angle::fromDegrees(6.25),
        math::Velocity::fromMetersPerSecond(-120.5),
        math::Angle::fromDegrees(37.5),
        math::Angle::fromDegrees(127.25),
        math::Distance::fromKilometers(2.5),
        math::Velocity::fromMetersPerSecond(250.75),
        math::Angle::fromDegrees(45.5),
        math::Angle::fromDegrees(-2.25),
        timestamp);
}

template <typename Value>
Value readValue(
    const std::vector<uint8_t>& bytes,
    std::size_t offset = 0
)
{
    static_assert(
        std::is_trivially_copyable<Value>::value,
        "Protocol values must be trivially copyable");

    Value value{};
    std::memcpy(&value, bytes.data() + offset, sizeof(value));
    return value;
}

} // namespace

TEST(MessageEncoderTest, EncodesBeamCommand)
{
    const comm::EncodeResult result =
        comm::encodeBeamCommand(makeBeamCommand());

    ASSERT_EQ(result.error, comm::EncodingError::NONE);
    ASSERT_TRUE(result.bytes.has_value());
    ASSERT_EQ(result.bytes->size(), sizeof(comm::protocol::ST_BeamTxMsg));

    const comm::protocol::ST_BeamTxMsg message =
        readValue<comm::protocol::ST_BeamTxMsg>(*result.bytes);

    EXPECT_EQ(
        message.msgHeader.message_Id,
        static_cast<uint16_t>(comm::protocol::MessageId::BeamTransmit));
    EXPECT_EQ(message.msgHeader.block_size, sizeof(message));
    EXPECT_EQ(message.msgHeader.version, 0);
    EXPECT_EQ(message.msgHeader.timeSec, 0u);
    EXPECT_EQ(message.msgHeader.timeNsec, 0u);
    EXPECT_EQ(message.msgHeader.source_id, 0);
    EXPECT_EQ(message.msgHeader.dest_id, 0);

    EXPECT_EQ(
        message.beam_cmd.beamType,
        static_cast<uint8_t>(beam::BeamCommand::BeamType::TRACKING));
    EXPECT_EQ(message.beam_cmd.timestamp, 1234u);
    EXPECT_EQ(message.beam_cmd.beamID, 21u);
    EXPECT_EQ(message.beam_cmd.commandCount, 7u);
    EXPECT_FLOAT_EQ(message.beam_cmd.beam_az_deg, 10.5F);
    EXPECT_FLOAT_EQ(message.beam_cmd.beam_el_deg, -4.5F);
    EXPECT_FLOAT_EQ(message.beam_cmd.az_width_deg, 3.25F);
    EXPECT_FLOAT_EQ(message.beam_cmd.el_width_deg, 2.75F);

    for (uint8_t padding : message.beam_cmd.padding)
    {
        EXPECT_EQ(padding, 0);
    }
    for (uint8_t padding : message.beam_cmd.padding_data)
    {
        EXPECT_EQ(padding, 0);
    }
}

TEST(MessageEncoderTest, EncodesTrackEventsUsingActualMessageSize)
{
    const std::vector<target::TrackEvent> events
    {
        makeTrackEvent()
    };

    const comm::EncodeResult result = comm::encodeTrackEvents(events);

    ASSERT_EQ(result.error, comm::EncodingError::NONE);
    ASSERT_TRUE(result.bytes.has_value());

    const std::size_t expectedSize =
        sizeof(comm::protocol::ST_MsgHeader) +
        sizeof(comm::protocol::ST_TrkHeader) +
        sizeof(comm::protocol::ST_Trk);
    ASSERT_EQ(result.bytes->size(), expectedSize);

    const comm::protocol::ST_MsgHeader header =
        readValue<comm::protocol::ST_MsgHeader>(*result.bytes);
    EXPECT_EQ(
        header.message_Id,
        static_cast<uint16_t>(comm::protocol::MessageId::TrackInformation));
    EXPECT_EQ(header.block_size, expectedSize);
    EXPECT_EQ(header.version, 0);
    EXPECT_EQ(header.timeSec, 0u);
    EXPECT_EQ(header.timeNsec, 0u);
    EXPECT_EQ(header.source_id, 0);
    EXPECT_EQ(header.dest_id, 0);

    const std::size_t trackHeaderOffset =
        sizeof(comm::protocol::ST_MsgHeader);
    const comm::protocol::ST_TrkHeader trackHeader =
        readValue<comm::protocol::ST_TrkHeader>(
            *result.bytes,
            trackHeaderOffset);
    EXPECT_EQ(trackHeader.number, 1);

    const std::size_t trackOffset =
        trackHeaderOffset + sizeof(comm::protocol::ST_TrkHeader);
    const comm::protocol::ST_Trk track =
        readValue<comm::protocol::ST_Trk>(*result.bytes, trackOffset);

    EXPECT_EQ(track.ID, 42);
    EXPECT_EQ(
        track.type,
        static_cast<uint8_t>(
            target::TrackEvent::TrackState::TRACKING));
    EXPECT_EQ(track.memFlag, 0);
    EXPECT_FLOAT_EQ(track.slantR_km, 12.5F);
    EXPECT_FLOAT_EQ(track.groundR_km, 10.25F);
    EXPECT_FLOAT_EQ(track.azi_deg, 15.5F);
    EXPECT_FLOAT_EQ(track.elv_deg, 6.25F);
    EXPECT_FLOAT_EQ(track.doppler_mps, -120.5F);
    EXPECT_FLOAT_EQ(track.lat_deg, 37.5F);
    EXPECT_FLOAT_EQ(track.lon_deg, 127.25F);
    EXPECT_FLOAT_EQ(track.alt_km, 2.5F);
    EXPECT_FLOAT_EQ(track.velocity_mps, 250.75F);
    EXPECT_FLOAT_EQ(track.heading_deg, 45.5F);
    EXPECT_FLOAT_EQ(track.fpa_deg, -2.25F);
    EXPECT_EQ(track.timeStamp, 5678u);
    EXPECT_EQ(track.beamID, 0u);
    EXPECT_EQ(track.commandCount, 0u);

    for (uint8_t padding : track.padding)
    {
        EXPECT_EQ(padding, 0);
    }
}

TEST(MessageEncoderTest, RejectsMoreThanOneHundredTracks)
{
    const std::vector<target::TrackEvent> events(
        101,
        makeTrackEvent());

    const comm::EncodeResult result = comm::encodeTrackEvents(events);

    EXPECT_EQ(result.error, comm::EncodingError::TOO_MANY_TRACKS);
    EXPECT_FALSE(result.bytes.has_value());
}

TEST(MessageEncoderTest, AcceptsOneHundredTracks)
{
    const std::vector<target::TrackEvent> events(
        100,
        makeTrackEvent());

    const comm::EncodeResult result = comm::encodeTrackEvents(events);

    ASSERT_EQ(result.error, comm::EncodingError::NONE);
    ASSERT_TRUE(result.bytes.has_value());
    EXPECT_EQ(result.bytes->size(), sizeof(comm::protocol::ST_TrkMsg));
}

TEST(MessageEncoderTest, RejectsTimestampOutsideProtocolRange)
{
    const auto invalidTimestamp = std::chrono::milliseconds{
        static_cast<std::chrono::milliseconds::rep>(
            std::numeric_limits<uint32_t>::max()) + 1};

    const comm::EncodeResult beamResult =
        comm::encodeBeamCommand(makeBeamCommand(invalidTimestamp));
    const comm::EncodeResult trackResult = comm::encodeTrackEvents(
        {makeTrackEvent(-1ms)});

    EXPECT_EQ(
        beamResult.error,
        comm::EncodingError::TYPE_CONVERSION_ERROR);
    EXPECT_FALSE(beamResult.bytes.has_value());
    EXPECT_EQ(
        trackResult.error,
        comm::EncodingError::TYPE_CONVERSION_ERROR);
    EXPECT_FALSE(trackResult.bytes.has_value());
}

TEST(MessageEncoderTest, RejectsValueOutsideFloatRange)
{
    const double invalidAngle =
        static_cast<double>(std::numeric_limits<float>::max()) * 2.0;

    const comm::EncodeResult beamResult =
        comm::encodeBeamCommand(makeBeamCommand(1234ms, invalidAngle));
    const comm::EncodeResult trackResult = comm::encodeTrackEvents(
        {makeTrackEvent(5678ms, invalidAngle)});

    EXPECT_EQ(
        beamResult.error,
        comm::EncodingError::TYPE_CONVERSION_ERROR);
    EXPECT_FALSE(beamResult.bytes.has_value());
    EXPECT_EQ(
        trackResult.error,
        comm::EncodingError::TYPE_CONVERSION_ERROR);
    EXPECT_FALSE(trackResult.bytes.has_value());
}

TEST(MessageEncoderTest, PreservesNotANumberDuringFloatConversion)
{
    const double notANumber = std::numeric_limits<double>::quiet_NaN();
    const std::vector<target::TrackEvent> events
    {
        target::TrackEvent(
            42,
            target::TrackEvent::TrackState::TRACKING,
            math::Distance::fromKilometers(12.5),
            math::Distance::fromKilometers(10.25),
            math::Angle::fromDegrees(15.5),
            math::Angle::fromDegrees(6.25),
            math::Velocity::fromMetersPerSecond(-120.5),
            math::Angle::fromDegrees(notANumber),
            math::Angle::fromDegrees(notANumber),
            math::Distance::fromKilometers(2.5),
            math::Velocity::fromMetersPerSecond(250.75),
            math::Angle::fromDegrees(notANumber),
            math::Angle::fromDegrees(notANumber),
            5678ms)
    };

    const comm::EncodeResult result = comm::encodeTrackEvents(events);

    ASSERT_EQ(result.error, comm::EncodingError::NONE);
    ASSERT_TRUE(result.bytes.has_value());

    const std::size_t trackOffset =
        sizeof(comm::protocol::ST_MsgHeader) +
        sizeof(comm::protocol::ST_TrkHeader);
    const comm::protocol::ST_Trk track =
        readValue<comm::protocol::ST_Trk>(*result.bytes, trackOffset);

    EXPECT_TRUE(std::isnan(track.lat_deg));
    EXPECT_TRUE(std::isnan(track.lon_deg));
    EXPECT_TRUE(std::isnan(track.heading_deg));
    EXPECT_TRUE(std::isnan(track.fpa_deg));
}
