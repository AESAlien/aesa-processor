#include <network/message_decoder/message_decoder.hpp>

#include <protocol/ST_OperatorMsg.hpp>
#include <protocol/ST_PlotMsg.hpp>
#include <protocol/ST_RadarAttMsg.hpp>

#include <gtest/gtest.h>

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <iterator>
#include <limits>
#include <type_traits>
#include <variant>
#include <vector>

namespace
{

constexpr std::size_t PLOT_DATA_OFFSET = offsetof(comm::protocol::ST_PlotMsg, plot);
constexpr std::size_t MAXIMUM_PLOT_COUNT =
    std::extent<decltype(comm::protocol::ST_PlotMsg::plot)>::value;

template <typename Message>
comm::MessageFrame makeFrame(
    Message message,
    comm::protocol::MessageId messageId,
    std::size_t byteCount = sizeof(Message)
) {
    static_assert(std::is_trivially_copyable_v<Message>);
    message.msgHeader.message_Id = static_cast<uint16_t>(messageId);
    message.msgHeader.version = comm::protocol::MESSAGE_VERSION;
    message.msgHeader.block_size = static_cast<uint32_t>(byteCount);

    comm::MessageFrame frame{message.msgHeader, std::vector<uint8_t>(sizeof(Message))};
    std::memcpy(frame.bytes.data(), &message, sizeof(Message));
    frame.bytes.resize(byteCount);
    return frame;
}

comm::protocol::ST_RadarAttMsg makeAttitudeMessage()
{
    comm::protocol::ST_RadarAttMsg message{};
    message.radar_lat_deg = 37.5;
    message.radar_lon_deg = 127.25;
    message.radar_alt_km = 1.25;
    message.roll_deg = -12.5;
    message.pitch_deg = 6.25;
    message.yaw_deg = 45.5;
    return message;
}

comm::protocol::ST_PlotMsg makePlotMessage(uint16_t plotCount)
{
    comm::protocol::ST_PlotMsg message{};
    message.beam_cmd.beamType = static_cast<uint8_t>(beam::BeamType::TRACKING);
    message.beam_cmd.beamID = 42;
    message.beam_cmd.timestamp = 1234;
    message.plotHd.number = plotCount;
    for (std::size_t index = 0; index < MAXIMUM_PLOT_COUNT; ++index)
    {
        auto& plot = message.plot[index];
        plot.ID = static_cast<uint32_t>(index + 1);
        plot.slantR_km = 12.5F + static_cast<float>(index);
        plot.azi_deg = 15.5F;
        plot.elv_deg = -6.25F;
        plot.doppler_mps = -120.5F;
        plot.pwr_DB = 25.75F;
    }
    return message;
}

comm::MessageFrame makePlotFrame(const comm::protocol::ST_PlotMsg& message)
{
    return makeFrame(
        message,
        comm::protocol::MessageId::PLOT_INFORMATION,
        PLOT_DATA_OFFSET + message.plotHd.number * sizeof(comm::protocol::ST_Plot)
    );
}

} // namespace

// 운용 제어 메시지의 전원 상태를 켜기 또는 끄기 명령으로 변환한다.
TEST(MessageDecoderTest, DecodesOperatorOnAndOff)
{
    // 준비: 켜기와 끄기 상태를 가진 운용 제어 메시지를 준비한다.
    const comm::MessageDecoder decoder;
    for (uint8_t status : {uint8_t{0}, uint8_t{1}})
    {
        SCOPED_TRACE(static_cast<int>(status));
        comm::protocol::ST_OperatorMsg message{};
        message.oper_status = status;

        const auto frame = makeFrame(message, comm::protocol::MessageId::OPERATOR_CONTROL);

        // 실행: 운용 제어 메시지를 디코딩한다.
        const auto result = decoder.decode(frame);

        // 검증: 오류 없이 입력 상태에 맞는 전원 제어 명령을 반환한다.
        ASSERT_EQ(result.error, comm::DecodingError::NONE);
        ASSERT_TRUE(result.message.has_value());
        const auto* command = std::get_if<beam::SetOperationStateCommand>(&*result.message);
        ASSERT_NE(command, nullptr);
        EXPECT_EQ(command->powerState, status == 0 ? beam::OperationState::OFF : beam::OperationState::ON);
    }
}

// 정의되지 않은 운용 상태는 명령을 만들지 않고 필드 오류로 반환한다.
TEST(MessageDecoderTest, RejectsUnknownOperatorState)
{
    // 준비: 정상 상태인 0과 1 이외의 운용 상태를 준비한다.
    const comm::MessageDecoder decoder;
    for (uint8_t status : {uint8_t{2}, uint8_t{255}})
    {
        SCOPED_TRACE(static_cast<int>(status));
        comm::protocol::ST_OperatorMsg message{};
        message.oper_status = status;
        const auto frame = makeFrame(message, comm::protocol::MessageId::OPERATOR_CONTROL);

        // 실행: 정의되지 않은 상태를 가진 메시지를 디코딩한다.
        const auto result = decoder.decode(frame);

        // 검증: 필드 오류를 반환하고 명령은 반환하지 않는다.
        EXPECT_EQ(result.error, comm::DecodingError::INVALID_FIELD_VALUE);
        EXPECT_FALSE(result.message.has_value());
    }
}

// 헤더에 선언한 크기와 실제 바이트 수가 다르면 메시지를 거부한다.
TEST(MessageDecoderTest, RejectsMismatchBetweenDeclaredAndActualSize)
{
    // 준비: 실제 바이트 수보다 선언한 크기가 작거나 큰 프레임을 준비한다.
    const comm::MessageDecoder decoder;
    for (int difference : {-1, 1})
    {
        SCOPED_TRACE(difference);
        auto frame = makeFrame(comm::protocol::ST_OperatorMsg{}, comm::protocol::MessageId::OPERATOR_CONTROL);
        frame.header.block_size = static_cast<uint32_t>(static_cast<int>(frame.bytes.size()) + difference);

        // 실행: 크기가 일치하지 않는 프레임을 디코딩한다.
        const auto result = decoder.decode(frame);

        // 검증: 크기 오류를 반환하고 메시지는 반환하지 않는다.
        EXPECT_EQ(result.error, comm::DecodingError::INVALID_MESSAGE_SIZE);
        EXPECT_FALSE(result.message.has_value());
    }
}

// 고정 길이 메시지는 헤더와 실제 크기가 같아도 규격 길이와 다르면 거부한다.
TEST(MessageDecoderTest, RejectsIncorrectFixedMessageSizes)
{
    // 준비: 운용 제어와 자세 메시지의 규격보다 한 바이트 작거나 큰 프레임을 준비한다.
    const comm::MessageDecoder decoder;
    for (int difference : {-1, 1})
    {
        SCOPED_TRACE(difference);
        const auto operatorFrame = makeFrame(
            comm::protocol::ST_OperatorMsg{},
            comm::protocol::MessageId::OPERATOR_CONTROL,
            sizeof(comm::protocol::ST_OperatorMsg) + difference
        );
        const auto attitudeFrame = makeFrame(
            makeAttitudeMessage(),
            comm::protocol::MessageId::RADAR_ATTITUDE_FROM_SIMULATOR,
            sizeof(comm::protocol::ST_RadarAttMsg) + difference
        );

        // 실행: 두 종류의 고정 길이 메시지를 디코딩한다.
        const auto operatorResult = decoder.decode(operatorFrame);
        const auto attitudeResult = decoder.decode(attitudeFrame);

        // 검증: 두 프레임 모두 크기 오류를 반환하고 메시지는 반환하지 않는다.
        EXPECT_EQ(operatorResult.error, comm::DecodingError::INVALID_MESSAGE_SIZE);
        EXPECT_FALSE(operatorResult.message.has_value());
        EXPECT_EQ(attitudeResult.error, comm::DecodingError::INVALID_MESSAGE_SIZE);
        EXPECT_FALSE(attitudeResult.message.has_value());
    }
}

// 수신을 지원하지 않거나 정의되지 않은 메시지 ID를 거부한다.
TEST(MessageDecoderTest, RejectsUnsupportedAndUnknownMessageIds)
{
    // 준비: 송신 전용 ID와 정의되지 않은 ID를 가진 프레임을 준비한다.
    const comm::MessageDecoder decoder;
    for (auto messageId : {
             comm::protocol::MessageId::TRACK_INFORMATION,
             comm::protocol::MessageId::RADAR_ATTITUDE_TO_CONSOLE,
             comm::protocol::MessageId::TRACK_DELETION,
             comm::protocol::MessageId::BEAM_TRANSMIT,
             static_cast<comm::protocol::MessageId>(0xFFFF)})
    {
        SCOPED_TRACE(static_cast<uint16_t>(messageId));
        const auto frame = makeFrame(comm::protocol::ST_OperatorMsg{}, messageId);

        // 실행: 지원하지 않는 ID의 프레임을 디코딩한다.
        const auto result = decoder.decode(frame);

        // 검증: 지원하지 않는 ID 오류를 반환하고 메시지는 반환하지 않는다.
        EXPECT_EQ(result.error, comm::DecodingError::UNSUPPORTED_MESSAGE_ID);
        EXPECT_FALSE(result.message.has_value());
    }
}

// 자세 메시지의 각 축을 보존하고 킬로미터 고도를 미터로 변환한다.
TEST(MessageDecoderTest, DecodesRadarAttitudeAndConvertsAltitudeToMeters)
{
    // 준비: 축마다 다른 각도와 1.25킬로미터 고도를 가진 자세 메시지를 준비한다.
    const comm::MessageDecoder decoder;
    const auto frame = makeFrame(makeAttitudeMessage(), comm::protocol::MessageId::RADAR_ATTITUDE_FROM_SIMULATOR);

    // 실행: 자세 메시지를 디코딩한다.
    const auto result = decoder.decode(frame);

    // 검증: 각 축이 뒤바뀌지 않고 고도는 1250미터인 자세 명령을 반환한다.
    ASSERT_EQ(result.error, comm::DecodingError::NONE);
    ASSERT_TRUE(result.message.has_value());
    const auto* command = std::get_if<beam::SetAttitudeCommand>(&*result.message);
    ASSERT_NE(command, nullptr);
    EXPECT_DOUBLE_EQ(command->latitude.deg(), 37.5);
    EXPECT_DOUBLE_EQ(command->longitude.deg(), 127.25);
    EXPECT_DOUBLE_EQ(command->altitude.m(), 1250.0);
    EXPECT_DOUBLE_EQ(command->roll.deg(), -12.5);
    EXPECT_DOUBLE_EQ(command->pitch.deg(), 6.25);
    EXPECT_DOUBLE_EQ(command->yaw.deg(), 45.5);
}

// 자세 메시지의 각 필드에 NaN 또는 무한대가 있으면 거부한다.
TEST(MessageDecoderTest, RejectsNonFiniteRadarAttitudeFields)
{
    // 준비: 자세의 모든 실수 필드와 유효하지 않은 실수 값을 준비한다.
    const comm::MessageDecoder decoder;
    using Message = comm::protocol::ST_RadarAttMsg;
    double Message::* const fields[] =
    {
        &Message::radar_lat_deg,
        &Message::radar_lon_deg,
        &Message::radar_alt_km,
        &Message::roll_deg,
        &Message::pitch_deg,
        &Message::yaw_deg
    };
    for (std::size_t fieldIndex = 0; fieldIndex < std::size(fields); ++fieldIndex)
    {
        SCOPED_TRACE(fieldIndex);
        for (double value : {
                 std::numeric_limits<double>::quiet_NaN(),
                 std::numeric_limits<double>::infinity(),
                 -std::numeric_limits<double>::infinity()})
        {
            SCOPED_TRACE(value);
            auto message = makeAttitudeMessage();
            message.*fields[fieldIndex] = value;
            const auto frame = makeFrame(message, comm::protocol::MessageId::RADAR_ATTITUDE_FROM_SIMULATOR);

            // 실행: 한 필드만 유효하지 않은 자세 메시지를 디코딩한다.
            const auto result = decoder.decode(frame);

            // 검증: 어느 필드에서든 필드 오류를 반환하고 명령은 반환하지 않는다.
            EXPECT_EQ(result.error, comm::DecodingError::INVALID_FIELD_VALUE);
            EXPECT_FALSE(result.message.has_value());
        }
    }
}

// 플롯의 순서와 값을 보존하고 빔 정보와 단위를 올바르게 변환한다.
TEST(MessageDecoderTest, DecodesPlotsInOrderAndPreservesBeamMetadata)
{
    // 준비: 거리가 다른 플롯 두 개와 빔 ID 및 송신 시각을 가진 메시지를 준비한다.
    const comm::MessageDecoder decoder;
    const auto frame = makePlotFrame(makePlotMessage(2));

    // 실행: 플롯 정보 메시지를 디코딩한다.
    const auto result = decoder.decode(frame);

    // 검증: 플롯 순서와 빔 정보를 보존하고 거리는 미터로 변환한다.
    ASSERT_EQ(result.error, comm::DecodingError::NONE);
    ASSERT_TRUE(result.message.has_value());
    const auto* event = std::get_if<target::DetectionEvent>(&*result.message);
    ASSERT_NE(event, nullptr);
    EXPECT_EQ(event->beamType, beam::BeamType::TRACKING);
    EXPECT_EQ(event->requestId, 42U);
    EXPECT_EQ(event->transmitTime, std::chrono::milliseconds{1234});
    ASSERT_EQ(event->plots.size(), 2U);
    EXPECT_DOUBLE_EQ(event->plots[0].slantRange.m(), 12500.0);
    EXPECT_DOUBLE_EQ(event->plots[1].slantRange.m(), 13500.0);
    EXPECT_DOUBLE_EQ(event->plots[0].azimuth.deg(), 15.5);
    EXPECT_DOUBLE_EQ(event->plots[0].elevation.deg(), -6.25);
    EXPECT_DOUBLE_EQ(event->plots[0].dopplerVelocity.mps(), -120.5);
    EXPECT_FLOAT_EQ(event->plots[0].power_db, 25.75F);
}

// 탐색과 확인 및 추적 빔 종류를 모두 수신할 수 있다.
TEST(MessageDecoderTest, AcceptsEverySupportedBeamType)
{
    // 준비: 지원하는 빔 종류를 하나씩 가진 플롯 메시지를 준비한다.
    const comm::MessageDecoder decoder;
    for (auto beamType : {beam::BeamType::SEARCH, beam::BeamType::CONFIRMATION, beam::BeamType::TRACKING})
    {
        SCOPED_TRACE(static_cast<int>(beamType));
        auto message = makePlotMessage(1);
        message.beam_cmd.beamType = static_cast<uint8_t>(beamType);
        const auto frame = makePlotFrame(message);

        // 실행: 각 빔 종류의 플롯 메시지를 디코딩한다.
        const auto result = decoder.decode(frame);

        // 검증: 오류 없이 입력과 같은 빔 종류의 탐지 이벤트를 반환한다.
        ASSERT_EQ(result.error, comm::DecodingError::NONE);
        ASSERT_TRUE(result.message.has_value());
        const auto* event = std::get_if<target::DetectionEvent>(&*result.message);
        ASSERT_NE(event, nullptr);
        EXPECT_EQ(event->beamType, beamType);
    }
}

// 플롯 개수가 0개이거나 최대 허용 개수인 메시지를 수신할 수 있다.
TEST(MessageDecoderTest, AcceptsZeroAndMaximumPlotCounts)
{
    // 준비: 플롯 개수의 하한과 상한에 해당하는 메시지를 준비한다.
    const comm::MessageDecoder decoder;
    for (std::size_t count : {std::size_t{0}, MAXIMUM_PLOT_COUNT})
    {
        SCOPED_TRACE(count);
        const auto frame = makePlotFrame(makePlotMessage(static_cast<uint16_t>(count)));

        // 실행: 경계 개수의 플롯 메시지를 디코딩한다.
        const auto result = decoder.decode(frame);

        // 검증: 요청한 개수의 플롯을 반환하고 마지막 플롯까지 보존한다.
        ASSERT_EQ(result.error, comm::DecodingError::NONE);
        ASSERT_TRUE(result.message.has_value());
        const auto* event = std::get_if<target::DetectionEvent>(&*result.message);
        ASSERT_NE(event, nullptr);
        ASSERT_EQ(event->plots.size(), count);
        if (count != 0)
        {
            EXPECT_DOUBLE_EQ(event->plots.back().slantRange.m(), (12.5 + count - 1) * 1000.0);
        }
    }
}

// 최대 허용 개수를 초과한 플롯 메시지는 개수 오류로 거부한다.
TEST(MessageDecoderTest, RejectsPlotCountAboveMaximum)
{
    // 준비: 플롯 개수가 최대 허용 개수보다 하나 많은 메시지를 준비한다.
    const comm::MessageDecoder decoder;
    const auto message = makePlotMessage(static_cast<uint16_t>(MAXIMUM_PLOT_COUNT + 1));
    const auto frame = makeFrame(message, comm::protocol::MessageId::PLOT_INFORMATION, PLOT_DATA_OFFSET);

    // 실행: 플롯 개수가 범위를 벗어난 메시지를 디코딩한다.
    const auto result = decoder.decode(frame);

    // 검증: 개수 오류를 반환하고 메시지는 반환하지 않는다.
    EXPECT_EQ(result.error, comm::DecodingError::INVALID_ITEM_COUNT);
    EXPECT_FALSE(result.message.has_value());
}

// 플롯 데이터 앞의 헤더가 잘린 메시지를 읽기 전에 거부한다.
TEST(MessageDecoderTest, RejectsTruncatedPlotHeaders)
{
    // 준비: 빔 정보와 플롯 헤더가 끝나기 한 바이트 전에 잘린 메시지를 준비한다.
    const comm::MessageDecoder decoder;
    const auto frame = makeFrame(
        makePlotMessage(1), comm::protocol::MessageId::PLOT_INFORMATION, PLOT_DATA_OFFSET - 1
    );

    // 실행: 헤더가 잘린 플롯 메시지를 디코딩한다.
    const auto result = decoder.decode(frame);

    // 검증: 크기 오류를 반환하고 메시지는 반환하지 않는다.
    EXPECT_EQ(result.error, comm::DecodingError::INVALID_MESSAGE_SIZE);
    EXPECT_FALSE(result.message.has_value());
}

// 선언한 플롯 개수와 실제 데이터 길이가 다르면 메시지를 거부한다.
TEST(MessageDecoderTest, RejectsPlotPayloadSizeMismatch)
{
    // 준비: 플롯은 두 개로 선언하고 실제 데이터는 부족하거나 남는 메시지를 준비한다.
    const comm::MessageDecoder decoder;
    const auto message = makePlotMessage(2);
    for (std::size_t byteCount : {
             PLOT_DATA_OFFSET + sizeof(comm::protocol::ST_Plot),
             PLOT_DATA_OFFSET + 2 * sizeof(comm::protocol::ST_Plot) - 1,
             PLOT_DATA_OFFSET + 2 * sizeof(comm::protocol::ST_Plot) + 1})
    {
        SCOPED_TRACE(byteCount);
        const auto frame = makeFrame(message, comm::protocol::MessageId::PLOT_INFORMATION, byteCount);

        // 실행: 플롯 개수와 데이터 길이가 일치하지 않는 메시지를 디코딩한다.
        const auto result = decoder.decode(frame);

        // 검증: 크기 오류를 반환하고 메시지는 반환하지 않는다.
        EXPECT_EQ(result.error, comm::DecodingError::INVALID_MESSAGE_SIZE);
        EXPECT_FALSE(result.message.has_value());
    }
}

// 정의되지 않은 빔 종류는 탐지 이벤트를 만들지 않고 거부한다.
TEST(MessageDecoderTest, RejectsUnknownBeamType)
{
    // 준비: 정의된 빔 종류에 해당하지 않는 값을 가진 플롯 메시지를 준비한다.
    const comm::MessageDecoder decoder;
    for (uint8_t beamType : {uint8_t{0}, uint8_t{255}})
    {
        SCOPED_TRACE(static_cast<int>(beamType));
        auto message = makePlotMessage(1);
        message.beam_cmd.beamType = beamType;
        const auto frame = makePlotFrame(message);

        // 실행: 정의되지 않은 빔 종류의 메시지를 디코딩한다.
        const auto result = decoder.decode(frame);

        // 검증: 필드 오류를 반환하고 탐지 이벤트는 반환하지 않는다.
        EXPECT_EQ(result.error, comm::DecodingError::INVALID_FIELD_VALUE);
        EXPECT_FALSE(result.message.has_value());
    }
}

// 뒤쪽 플롯에 NaN 또는 무한대가 있으면 앞쪽 플롯도 부분 반환하지 않는다.
TEST(MessageDecoderTest, RejectsNonFinitePlotFieldsWithoutReturningPartialMessage)
{
    // 준비: 두 번째 플롯의 실수 필드 하나만 유효하지 않은 메시지를 준비한다.
    const comm::MessageDecoder decoder;
    using Plot = comm::protocol::ST_Plot;
    float Plot::* const fields[] =
    {
        &Plot::slantR_km,
        &Plot::azi_deg,
        &Plot::elv_deg,
        &Plot::doppler_mps,
        &Plot::pwr_DB
    };
    for (std::size_t fieldIndex = 0; fieldIndex < std::size(fields); ++fieldIndex)
    {
        SCOPED_TRACE(fieldIndex);
        for (float value : {
                 std::numeric_limits<float>::quiet_NaN(),
                 std::numeric_limits<float>::infinity(),
                 -std::numeric_limits<float>::infinity()})
        {
            SCOPED_TRACE(value);
            auto message = makePlotMessage(2);
            message.plot[1].*fields[fieldIndex] = value;
            const auto frame = makePlotFrame(message);

            // 실행: 정상 플롯 뒤에 잘못된 플롯이 있는 메시지를 디코딩한다.
            const auto result = decoder.decode(frame);

            // 검증: 필드 오류를 반환하고 정상 플롯만 따로 반환하지 않는다.
            EXPECT_EQ(result.error, comm::DecodingError::INVALID_FIELD_VALUE);
            EXPECT_FALSE(result.message.has_value());
        }
    }
}

// 음수 거리는 거부하고 경계인 거리 0은 허용한다.
TEST(MessageDecoderTest, RejectsNegativeRangeAndAcceptsZeroRange)
{
    // 준비: 거리가 음수인 플롯과 0인 플롯을 각각 준비한다.
    const comm::MessageDecoder decoder;
    auto negativeRange = makePlotMessage(1);
    negativeRange.plot[0].slantR_km = -0.5F;
    auto zeroRange = makePlotMessage(1);
    zeroRange.plot[0].slantR_km = 0.0F;
    const auto negativeFrame = makePlotFrame(negativeRange);
    const auto zeroFrame = makePlotFrame(zeroRange);

    // 실행: 음수 거리와 거리 0을 가진 메시지를 각각 디코딩한다.
    const auto negativeResult = decoder.decode(negativeFrame);
    const auto result = decoder.decode(zeroFrame);

    // 검증: 음수 거리는 오류로 반환하고 거리 0은 정상 플롯으로 반환한다.
    EXPECT_EQ(negativeResult.error, comm::DecodingError::INVALID_FIELD_VALUE);
    EXPECT_FALSE(negativeResult.message.has_value());
    ASSERT_EQ(result.error, comm::DecodingError::NONE);
    ASSERT_TRUE(result.message.has_value());
    const auto* event = std::get_if<target::DetectionEvent>(&*result.message);
    ASSERT_NE(event, nullptr);
    ASSERT_EQ(event->plots.size(), 1U);
    EXPECT_DOUBLE_EQ(event->plots.front().slantRange.m(), 0.0);
}
