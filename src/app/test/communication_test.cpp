#include "channel/console_channel.hpp"
#include "channel/simulator_channel.hpp"
#include "worker/comm_receive_worker.hpp"
#include "worker/console_send_worker.hpp"
#include "worker/simulator_send_worker.hpp"

#include <gtest/gtest.h>

#include <chrono>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <vector>

namespace
{
using namespace std::chrono_literals;

beam::SetAttitudeCommand makeAttitude()
{
    return beam::SetAttitudeCommand::Builder()
        .latitude(math::Angle::fromDegrees(37))
        .longitude(math::Angle::fromDegrees(127))
        .altitude(math::Length::fromMeters(100))
        .roll(math::Angle::fromDegrees(1))
        .pitch(math::Angle::fromDegrees(2))
        .yaw(math::Angle::fromDegrees(3))
        .build();
}

beam::TransmitBeamCommand makeBeam(uint32_t id)
{
    return beam::TransmitBeamCommand(
        beam::BeamType::SEARCH,
        10ms,
        id,
        2,
        math::SphericalDirection(math::Angle::fromDegrees(30), math::Angle::fromDegrees(15)),
        beam::BeamWidth(math::Angle::fromDegrees(3), math::Angle::fromDegrees(4)));
}

static_assert(!std::is_constructible_v<app::ConsoleEvent, beam::TransmitBeamCommand>);
static_assert(!std::is_invocable_v<
    decltype(&app::SimulatorChannel::write),
    app::SimulatorChannel&,
    beam::SetAttitudeCommand>);

TEST(AppCommunication, ReceiveRoutesMessagesAndCopiesAttitudeToBothDestinations)
{
    app::BeamChannel beamChannel;
    app::TargetChannel targetChannel;
    app::ConsoleChannel consoleChannel;
    std::vector<comm::IncomingDto> incoming;
    incoming.emplace_back(beam::SetOperationStateCommand(beam::OperationState::ON));
    incoming.emplace_back(makeAttitude());
    incoming.emplace_back(target::DetectionEvent(beam::BeamType::SEARCH, 42, 25ms, {}));
    std::size_t index = 0;
    app::CommReceiveWorker worker(
        beamChannel,
        targetChannel,
        consoleChannel,
        [&]() -> std::optional<comm::IncomingDto>
        {
            if (index == incoming.size())
            {
                return std::nullopt;
            }
            return incoming[index++];
        });
    worker.run();
    beamChannel.close();
    targetChannel.close();
    consoleChannel.close();

    const auto state = beamChannel.read();
    ASSERT_TRUE(state);
    EXPECT_EQ(std::get<beam::SetOperationStateCommand>(*state).powerState, beam::OperationState::ON);
    const auto beamAttitude = beamChannel.read();
    const auto consoleAttitude = consoleChannel.read();
    ASSERT_TRUE(beamAttitude);
    ASSERT_TRUE(consoleAttitude);
    EXPECT_EQ(std::get<beam::SetAttitudeCommand>(*beamAttitude).yaw, makeAttitude().yaw);
    EXPECT_EQ(std::get<beam::SetAttitudeCommand>(*consoleAttitude).latitude, makeAttitude().latitude);
    const auto detection = targetChannel.read();
    ASSERT_TRUE(detection);
    EXPECT_EQ(detection->requestId, 42u);
    EXPECT_EQ(detection->transmitTime, 25ms);
    EXPECT_FALSE(beamChannel.read());
    EXPECT_FALSE(targetChannel.read());
    EXPECT_FALSE(consoleChannel.read());
}

TEST(AppCommunication, SimulatorDrainsInOrderAndContinuesAfterSendFailure)
{
    app::SimulatorChannel channel;
    channel.write(makeBeam(10));
    channel.write(makeBeam(20));
    channel.close();
    std::vector<uint32_t> ids;
    app::SimulatorSendWorker worker(channel, [&](const beam::TransmitBeamCommand& command)
    {
        ids.push_back(command.beamId);
        EXPECT_EQ(command.transmitTime, 10ms);
        EXPECT_EQ(command.commandCount, 2u);
        if (command.beamId == 10)
        {
            throw std::runtime_error("Send failed");
        }
    });
    worker.run();
    EXPECT_EQ(ids, (std::vector<uint32_t>{10, 20}));
    EXPECT_FALSE(channel.read());
}

TEST(AppCommunication, ConsolePreservesMixedOrderAndSendsEmptyTrackListAfterFailure)
{
    app::ConsoleChannel channel;
    channel.write(makeAttitude());
    channel.write(std::vector<target::TrackEvent>{});
    channel.write(makeAttitude());
    channel.close();
    std::vector<std::string> calls;
    app::ConsoleSendWorker worker(
        channel,
        [&](const beam::SetAttitudeCommand& command)
        {
            calls.push_back("attitude");
            EXPECT_EQ(command.longitude, makeAttitude().longitude);
            if (calls.size() == 1)
            {
                throw std::runtime_error("Send failed");
            }
        },
        [&](const std::vector<target::TrackEvent>& tracks)
        {
            calls.push_back("tracks");
            EXPECT_TRUE(tracks.empty());
        });
    worker.run();
    EXPECT_EQ(calls, (std::vector<std::string>{"attitude", "tracks", "attitude"}));
}

TEST(AppCommunication, DestinationChannelsCloseIndependentlyAndRejectFurtherWrites)
{
    app::SimulatorChannel simulator;
    app::ConsoleChannel console;
    simulator.close();
    simulator.close();
    console.write(makeAttitude());
    EXPECT_FALSE(simulator.read());
    EXPECT_THROW(simulator.write(makeBeam(1)), app::ChannelClosedError);
    console.close();
    ASSERT_TRUE(console.read());
    EXPECT_FALSE(console.read());
    EXPECT_THROW(console.write(makeAttitude()), app::ChannelClosedError);
}

TEST(AppCommunication, ReceiveRetriesAfterFailure)
{
    app::BeamChannel beamChannel;
    app::TargetChannel targetChannel;
    app::ConsoleChannel consoleChannel;
    int calls = 0;
    app::CommReceiveWorker worker(
        beamChannel,
        targetChannel,
        consoleChannel,
        [&]() -> std::optional<comm::IncomingDto>
        {
            ++calls;
            if (calls == 1)
            {
                throw std::runtime_error("Receive failed");
            }
            if (calls == 2)
            {
                return beam::SetOperationStateCommand(beam::OperationState::OFF);
            }
            return std::nullopt;
        });
    worker.run();
    beamChannel.close();
    const auto state = beamChannel.read();
    ASSERT_TRUE(state);
    EXPECT_EQ(std::get<beam::SetOperationStateCommand>(*state).powerState, beam::OperationState::OFF);
    EXPECT_EQ(calls, 3);
    EXPECT_FALSE(beamChannel.read());
}
TEST(AppCommunication, ConsoleSendsTrackBatchesAndWrapsSingleTrack)
{
    using namespace math::literals;
    const target::TrackEvent track(
        7, target::TrackEvent::TrackStatus::TRACKING,
        math::Distance(1000_m), math::Distance(900_m),
        math::CircularAngle(20_deg), math::TiltAngle(10_deg), -5_mps,
        math::TiltAngle(37_deg), math::CircularAngle(127_deg), 100_m,
        math::Speed(50_mps), math::CircularAngle(90_deg), math::TiltAngle(0_deg), 123ms);
    app::ConsoleChannel channel;
    channel.write(track);
    channel.write(std::vector<target::TrackEvent>{track, track});
    channel.close();
    std::vector<std::size_t> sizes;
    app::ConsoleSendWorker worker(
        channel,
        [](const beam::SetAttitudeCommand&) { FAIL() << "Unexpected attitude"; },
        [&](const std::vector<target::TrackEvent>& tracks)
        {
            sizes.push_back(tracks.size());
            for (const auto& sent : tracks)
            {
                EXPECT_EQ(sent.id, track.id);
                EXPECT_EQ(sent.state, track.state);
                EXPECT_EQ(sent.slantRange, track.slantRange);
                EXPECT_EQ(sent.timestamp, track.timestamp);
            }
        });
    worker.run();
    EXPECT_EQ(sizes, (std::vector<std::size_t>{1, 2}));
}
} // namespace
