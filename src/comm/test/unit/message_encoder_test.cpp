#include <message/message_encoder.hpp>

#include <protocol/ST_BeamTxMsg.hpp>
#include <protocol/ST_RadarAttMsg.hpp>
#include <protocol/ST_TrkMsg.hpp>

#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>
#include <stdexcept>
#include <type_traits>
#include <vector>

namespace
{

using namespace math::literals;
using namespace std::chrono_literals;

beam::TransmitBeamCommand makeBeamCommand(
    std::chrono::milliseconds transmitTime = 1234ms,
    double azimuthBeamWidth = 3.25
) {
    return beam::TransmitBeamCommand(
        beam::BeamType::TRACKING,
        transmitTime,
        21,
        7,
        math::SphericalDirection(10.5_deg, -4.5_deg),
        beam::BeamWidth(math::Angle::fromDegrees(azimuthBeamWidth), 2.75_deg)
    );
}

target::TrackEvent makeTrackEvent(
    std::chrono::milliseconds timestamp = 5678ms,
    double slantRangeKilometers = 12.5
) {
    return target::TrackEvent::Builder()
        .id(42)
        .state(target::TrackEvent::TrackStatus::TRACKING)
        .slantRange(math::Distance::fromKilometers(slantRangeKilometers))
        .groundRange(math::Distance(10.25_km))
        .azimuth(15.5_deg)
        .elevation(6.25_deg)
        .dopplerVelocity(-120.5_mps)
        .latitude(37.5_deg)
        .longitude(127.25_deg)
        .altitude(2.5_km)
        .speed(math::Speed(250.75_mps))
        .heading(45.5_deg)
        .flightPathAngle(-2.25_deg)
        .timestamp(timestamp)
        .build();
}

pose::SetRadarPoseCommand makeAttitudeCommand(double longitudeDegrees = 127.25)
{
    return pose::SetRadarPoseCommand::Builder()
        .latitude(37.5_deg)
        .longitude(math::Angle::fromDegrees(longitudeDegrees))
        .altitude(1250.0_m)
        .roll(-12.5_deg)
        .pitch(6.25_deg)
        .yaw(45.5_deg)
        .build();
}

template <typename Value>
Value readValue(
    const std::vector<uint8_t>& bytes,
    std::size_t offset = 0
) {
    static_assert(
        std::is_trivially_copyable<Value>::value,
        "Protocol values must be trivially copyable"
    );

    Value value{};
    std::memcpy(&value, bytes.data() + offset, sizeof(value));
    return value;
}

// 방위각이 azimuthDegrees인 빔 명령을 인코딩해서 와이어의 방위각(beam_az_deg)을 반환한다.
// 방위각은 CircularAngle이 [0, 360)으로 정규화하며, 시뮬레이터 규격도 0-360이다.
float encodeAzimuthToWire(double azimuthDegrees)
{
    const beam::TransmitBeamCommand command(
        beam::BeamType::TRACKING,
        1234ms,
        21,
        7,
        math::SphericalDirection(math::Angle::fromDegrees(azimuthDegrees), -4.5_deg),
        beam::BeamWidth(3.25_deg, 2.75_deg)
    );

    const comm::EncodeResult result = comm::encodeBeamCommand(command);
    if (result.error != comm::EncodingError::NONE || !result.bytes.has_value())
    {
        throw std::runtime_error("failed to encode beam command");
    }

    return readValue<comm::protocol::ST_BeamTxMsg>(*result.bytes).beam_cmd.beam_az_deg;
}

} // namespace

// 빔 명령의 필드를 프로토콜 형식으로 변환하고 미사용 필드는 0으로 채운다.
TEST(MessageEncoderTest, EncodesBeamCommand)
{
    // 준비: 빔 종류와 ID 및 각 축의 값이 지정된 송신 명령을 준비한다.
    const auto command = makeBeamCommand();

    // 실행: 빔 송신 명령을 바이트 배열로 인코딩한다.
    const comm::EncodeResult result =
        comm::encodeBeamCommand(command);

    // 검증: 헤더와 빔 데이터가 입력과 일치하고 패딩은 모두 0이다.
    ASSERT_EQ(result.error, comm::EncodingError::NONE);
    ASSERT_TRUE(result.bytes.has_value());
    ASSERT_EQ(result.bytes->size(), sizeof(comm::protocol::ST_BeamTxMsg));

    const comm::protocol::ST_BeamTxMsg message =
        readValue<comm::protocol::ST_BeamTxMsg>(*result.bytes);

    EXPECT_EQ(
        message.msgHeader.message_Id,
        static_cast<uint16_t>(comm::protocol::MessageId::BEAM_TRANSMIT)
    );
    EXPECT_EQ(message.msgHeader.block_size, sizeof(message));
    EXPECT_EQ(message.msgHeader.version, 0);
    EXPECT_EQ(message.msgHeader.timeSec, 0u);
    EXPECT_EQ(message.msgHeader.timeNsec, 0u);
    EXPECT_EQ(message.msgHeader.source_id, 0);
    EXPECT_EQ(message.msgHeader.dest_id, 0);

    EXPECT_EQ(
        message.beam_cmd.beamType,
        static_cast<uint8_t>(beam::BeamType::TRACKING)
    );
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

// 레이더 자세를 전시기용 자세 메시지로 변환하고 고도는 킬로미터로 보낸다.
TEST(MessageEncoderTest, EncodesRadarAttitudeForConsole)
{
    // 준비: 축마다 다른 각도와 1250미터 고도를 가진 자세 명령을 준비한다.
    const auto command = makeAttitudeCommand();

    // 실행: 자세 명령을 바이트 배열로 인코딩한다.
    const comm::EncodeResult result = comm::encodeRadarAttitude(command);

    // 검증: 전시기용 메시지 ID와 크기를 갖고 각 축이 뒤바뀌지 않는다.
    ASSERT_EQ(result.error, comm::EncodingError::NONE);
    ASSERT_TRUE(result.bytes.has_value());
    ASSERT_EQ(result.bytes->size(), sizeof(comm::protocol::ST_RadarAttMsg));

    const comm::protocol::ST_RadarAttMsg message =
        readValue<comm::protocol::ST_RadarAttMsg>(*result.bytes);

    EXPECT_EQ(
        message.msgHeader.message_Id,
        static_cast<uint16_t>(comm::protocol::MessageId::RADAR_ATTITUDE_TO_CONSOLE)
    );
    EXPECT_EQ(message.msgHeader.block_size, sizeof(message));
    EXPECT_DOUBLE_EQ(message.radar_lat_deg, 37.5);
    EXPECT_DOUBLE_EQ(message.radar_lon_deg, 127.25);
    EXPECT_DOUBLE_EQ(message.radar_alt_km, 1.25);
    EXPECT_DOUBLE_EQ(message.roll_deg, -12.5);
    EXPECT_DOUBLE_EQ(message.pitch_deg, 6.25);
    EXPECT_DOUBLE_EQ(message.yaw_deg, 45.5);
}

// 음수 경도는 방위각과 같이 0도 이상 360도 미만의 값으로 전송한다.
TEST(MessageEncoderTest, EncodesNegativeLongitudeAsZeroToThreeSixtyOnWire)
{
    // 준비: 서경 122.5도에 해당하는 자세 명령을 준비한다.
    const auto command = makeAttitudeCommand(-122.5);

    // 실행: 자세 명령을 바이트 배열로 인코딩한다.
    const comm::EncodeResult result = comm::encodeRadarAttitude(command);

    // 검증: 경도는 같은 방향인 237.5도로 전송한다.
    ASSERT_TRUE(result.bytes.has_value());
    EXPECT_DOUBLE_EQ(
        readValue<comm::protocol::ST_RadarAttMsg>(*result.bytes).radar_lon_deg,
        237.5
    );
}

// 음수 방위각은 0도 이상 360도 미만의 값으로 전송한다.
TEST(MessageEncoderTest, EncodesNegativeAzimuthAsZeroToThreeSixtyOnWire)
{
    // 준비: 서로 다른 음수 방위각을 준비한다.
    const double negativeAzimuth = -10.0;
    const double halfTurnAzimuth = -180.0;

    // 실행: 각 방위각을 가진 빔 명령을 인코딩한다.
    const auto negativeResult = encodeAzimuthToWire(negativeAzimuth);
    const auto halfTurnResult = encodeAzimuthToWire(halfTurnAzimuth);

    // 검증: 한 바퀴 안의 양수 방위각으로 변환된다.
    EXPECT_FLOAT_EQ(negativeResult, 350.0F);
    EXPECT_FLOAT_EQ(halfTurnResult, 180.0F);
}

// 방위각 0도와 한 바퀴인 360도는 모두 0도로 전송한다.
TEST(MessageEncoderTest, EncodesFullTurnAzimuthAsZeroOnWire)
{
    // 준비: 방위각 0도와 360도를 준비한다.
    const double zeroAzimuth = 0.0;
    const double fullTurnAzimuth = 360.0;

    // 실행: 두 방위각을 가진 빔 명령을 각각 인코딩한다.
    const auto zeroResult = encodeAzimuthToWire(zeroAzimuth);
    const auto fullTurnResult = encodeAzimuthToWire(fullTurnAzimuth);

    // 검증: 한 바퀴 회전 여부와 관계없이 0도로 변환된다.
    EXPECT_FLOAT_EQ(zeroResult, 0.0F);
    EXPECT_FLOAT_EQ(fullTurnResult, 0.0F);
}

// 한 바퀴를 넘는 방위각은 회전 주기를 제거한 값으로 전송한다.
TEST(MessageEncoderTest, EncodesAzimuthBeyondFullTurnWithinRangeOnWire)
{
    // 준비: 한 바퀴와 두 바퀴를 각각 초과하는 방위각을 준비한다.
    const double beyondOneTurn = 370.0;
    const double beyondTwoTurns = 725.0;

    // 실행: 여러 바퀴 회전한 방위각을 가진 빔 명령을 인코딩한다.
    const auto oneTurnResult = encodeAzimuthToWire(beyondOneTurn);
    const auto twoTurnResult = encodeAzimuthToWire(beyondTwoTurns);

    // 검증: 완전한 회전 주기를 제외한 방위각으로 변환된다.
    EXPECT_FLOAT_EQ(oneTurnResult, 10.0F);
    EXPECT_FLOAT_EQ(twoTurnResult, 5.0F);
}

// 이미 정규화된 범위 안의 방위각은 입력값 그대로 전송한다.
TEST(MessageEncoderTest, KeepsAzimuthAlreadyInRangeOnWire)
{
    // 준비: 0도와 360도 사이에서 서로 다른 방위각을 준비한다.
    const double lowerAzimuth = 0.5;
    const double middleAzimuth = 180.0;
    const double upperAzimuth = 359.5;

    // 실행: 범위 안의 각 방위각을 인코딩한다.
    const auto lowerResult = encodeAzimuthToWire(lowerAzimuth);
    const auto middleResult = encodeAzimuthToWire(middleAzimuth);
    const auto upperResult = encodeAzimuthToWire(upperAzimuth);

    // 검증: 방위각의 값이 바뀌지 않는다.
    EXPECT_FLOAT_EQ(lowerResult, 0.5F);
    EXPECT_FLOAT_EQ(middleResult, 180.0F);
    EXPECT_FLOAT_EQ(upperResult, 359.5F);
}

// 추적 이벤트는 실제 항목 수에 맞는 길이와 프로토콜 필드로 인코딩한다.
TEST(MessageEncoderTest, EncodesTrackEventsUsingActualMessageSize)
{
    // 준비: 각 필드를 구분할 수 있는 추적 이벤트 하나를 준비한다.
    const std::vector<target::TrackEvent> events
    {
        makeTrackEvent()
    };

    const std::size_t expectedSize =
        sizeof(comm::protocol::ST_MsgHeader) +
        sizeof(comm::protocol::ST_TrkHeader) +
        sizeof(comm::protocol::ST_Trk);

    // 실행: 추적 이벤트 목록을 바이트 배열로 인코딩한다.
    const comm::EncodeResult result = comm::encodeTrackEvents(events);

    // 검증: 실제 항목 수에 맞는 길이와 입력 필드를 보존하고 미사용 필드는 0이다.
    ASSERT_EQ(result.error, comm::EncodingError::NONE);
    ASSERT_TRUE(result.bytes.has_value());
    ASSERT_EQ(result.bytes->size(), expectedSize);

    const comm::protocol::ST_MsgHeader header =
        readValue<comm::protocol::ST_MsgHeader>(*result.bytes);
    EXPECT_EQ(
        header.message_Id,
        static_cast<uint16_t>(comm::protocol::MessageId::TRACK_INFORMATION)
    );
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
            trackHeaderOffset
        );
    EXPECT_EQ(trackHeader.number, 1);

    const std::size_t trackOffset =
        trackHeaderOffset + sizeof(comm::protocol::ST_TrkHeader);
    const comm::protocol::ST_Trk track =
        readValue<comm::protocol::ST_Trk>(*result.bytes, trackOffset);

    EXPECT_EQ(track.ID, 42);
    EXPECT_EQ(
        track.type,
        static_cast<uint8_t>(
            target::TrackEvent::TrackStatus::TRACKING
        )
    );
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

// 기수각은 정규화하고 비행 경로각은 부호가 있는 양 끝 경계를 보존한다.
TEST(MessageEncoderTest, EncodesNormalizedHeadingAndSignedFlightPathBoundaries)
{
    // 준비: 음수 기수각과 비행 경로각의 양 끝 경계를 가진 추적 이벤트를 준비한다.
    for (const auto flightPathAngle : {-90_deg, 90_deg})
    {
        auto event = makeTrackEvent();
        event.heading = math::CircularAngle(-90_deg);
        event.flightPathAngle = math::TiltAngle(flightPathAngle);

        const std::vector<target::TrackEvent> events{event};

        // 실행: 경계 각도를 가진 추적 이벤트를 인코딩한다.
        const auto result = comm::encodeTrackEvents(events);

        // 검증: 기수각은 270도가 되고 비행 경로각은 부호와 크기를 보존한다.
        ASSERT_EQ(result.error, comm::EncodingError::NONE);
        ASSERT_TRUE(result.bytes.has_value());
        const auto track = readValue<comm::protocol::ST_Trk>(
            *result.bytes,
            sizeof(comm::protocol::ST_MsgHeader) + sizeof(comm::protocol::ST_TrkHeader)
        );
        EXPECT_FLOAT_EQ(track.heading_deg, 270.0F);
        EXPECT_FLOAT_EQ(track.fpa_deg, static_cast<float>(flightPathAngle.deg()));
    }
}

// 최대 허용 개수인 100개를 초과하는 추적 이벤트는 인코딩하지 않는다.
TEST(MessageEncoderTest, RejectsMoreThanOneHundredTracks)
{
    // 준비: 최대 허용 개수보다 하나 많은 추적 이벤트를 준비한다.
    const std::vector<target::TrackEvent> events(
        101,
        makeTrackEvent()
    );

    // 실행: 추적 이벤트 101개를 인코딩한다.
    const comm::EncodeResult result = comm::encodeTrackEvents(events);

    // 검증: 개수 초과 오류를 반환하고 바이트 배열은 반환하지 않는다.
    EXPECT_EQ(result.error, comm::EncodingError::TOO_MANY_TRACKS);
    EXPECT_FALSE(result.bytes.has_value());
}

// 최대 허용 개수인 100개의 추적 이벤트는 인코딩할 수 있다.
TEST(MessageEncoderTest, AcceptsOneHundredTracks)
{
    // 준비: 최대 허용 개수만큼의 추적 이벤트를 준비한다.
    const std::vector<target::TrackEvent> events(
        100,
        makeTrackEvent()
    );

    // 실행: 추적 이벤트 100개를 인코딩한다.
    const comm::EncodeResult result = comm::encodeTrackEvents(events);

    // 검증: 모든 항목을 담는 최대 크기의 바이트 배열을 반환한다.
    ASSERT_EQ(result.error, comm::EncodingError::NONE);
    ASSERT_TRUE(result.bytes.has_value());
    EXPECT_EQ(result.bytes->size(), sizeof(comm::protocol::ST_TrkMsg));
}

// 프로토콜의 시각 범위를 벗어난 빔 명령과 추적 이벤트를 거부한다.
TEST(MessageEncoderTest, RejectsTimestampOutsideProtocolRange)
{
    // 준비: uint32 상한을 초과한 빔 송신 시각과 음수 추적 시각을 준비한다.
    const auto invalidTimestamp = std::chrono::milliseconds{
        static_cast<std::chrono::milliseconds::rep>(
            std::numeric_limits<uint32_t>::max()
        ) + 1};
    const auto command = makeBeamCommand(invalidTimestamp);
    const std::vector<target::TrackEvent> events{makeTrackEvent(-1ms)};

    // 실행: 표현 범위를 벗어난 시각을 가진 명령과 이벤트를 인코딩한다.
    const comm::EncodeResult beamResult =
        comm::encodeBeamCommand(command);
    const comm::EncodeResult trackResult = comm::encodeTrackEvents(events);

    // 검증: 두 입력 모두 타입 변환 오류를 반환하고 바이트 배열은 반환하지 않는다.
    EXPECT_EQ(
        beamResult.error,
        comm::EncodingError::TYPE_CONVERSION_ERROR
    );
    EXPECT_FALSE(beamResult.bytes.has_value());
    EXPECT_EQ(
        trackResult.error,
        comm::EncodingError::TYPE_CONVERSION_ERROR
    );
    EXPECT_FALSE(trackResult.bytes.has_value());
}

// 프로토콜의 float 범위로 표현할 수 없는 값은 인코딩하지 않는다.
TEST(MessageEncoderTest, RejectsValueOutsideFloatRange)
{
    // 준비: float 상한을 넘는 빔 폭과 추적 거리를 가진 입력을 준비한다.
    const double invalidAngle =
        static_cast<double>(std::numeric_limits<float>::max()) * 2.0;
    const auto command = makeBeamCommand(1234ms, invalidAngle);
    const std::vector<target::TrackEvent> events{makeTrackEvent(5678ms, invalidAngle)};

    // 실행: 프로토콜의 실수 범위를 벗어난 입력을 인코딩한다.
    const comm::EncodeResult beamResult =
        comm::encodeBeamCommand(command);
    const comm::EncodeResult trackResult = comm::encodeTrackEvents(events);

    // 검증: 두 입력 모두 타입 변환 오류를 반환하고 바이트 배열은 반환하지 않는다.
    EXPECT_EQ(
        beamResult.error,
        comm::EncodingError::TYPE_CONVERSION_ERROR
    );
    EXPECT_FALSE(beamResult.bytes.has_value());
    EXPECT_EQ(
        trackResult.error,
        comm::EncodingError::TYPE_CONVERSION_ERROR
    );
    EXPECT_FALSE(trackResult.bytes.has_value());
}
