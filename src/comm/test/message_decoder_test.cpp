#include <network/message_decoder/message_decoder.hpp>
#include <network/network_manager/network_manager.hpp>

#include <chrono>
#include <csignal>
#include <cstdint>
#include <exception>
#include <iostream>
#include <optional>
#include <variant>
#include <vector>

namespace
{

volatile std::sig_atomic_t running = 1;

constexpr char SERVER_ADDRESS[] = "127.0.0.20";
constexpr uint16_t SERVER_PORT = 3200;
constexpr char SCENARIO_SIMULATOR_ADDRESS[] = "127.0.0.100";
constexpr uint16_t SCENARIO_SIMULATOR_PORT = 3300;

void stopRunning(int)
{
    running = 0;
}

const char* decodingErrorName(comm::DecodingError error)
{
    switch (error)
    {
    case comm::DecodingError::NONE:
        return "none";

    case comm::DecodingError::UNSUPPORTED_MESSAGE_ID:
        return "unsupported-message-id";

    case comm::DecodingError::UNSUPPORTED_VERSION:
        return "unsupported-version";

    case comm::DecodingError::INVALID_MESSAGE_SIZE:
        return "invalid-message-size";

    case comm::DecodingError::INVALID_ITEM_COUNT:
        return "invalid-item-count";

    case comm::DecodingError::INVALID_FIELD_VALUE:
        return "invalid-field-value";
    }

    return "unknown";
}

const char* operationStateName(beam::OperationState state)
{
    switch (state)
    {
    case beam::OperationState::ON:
        return "on";

    case beam::OperationState::OFF:
        return "off";
    }

    return "unknown";
}

const char* beamTypeName(beam::BeamType beamType)
{
    switch (beamType)
    {
    case beam::BeamType::SEARCH:
        return "search";

    case beam::BeamType::CONFIRMATION:
        return "confirmation";

    case beam::BeamType::TRACKING:
        return "tracking";
    }

    return "unknown";
}

void printDecodedMessage(const comm::IncomingDto& message)
{
    if (const auto* command =
            std::get_if<beam::SetOperationStateCommand>(&message))
    {
        std::cout << "[decoded] type=operation-state"
                  << ", state=" << operationStateName(command->powerState)
                  << '\n';
        return;
    }

    if (const auto* command =
            std::get_if<beam::SetAttitudeCommand>(&message))
    {
        std::cout << "[decoded] type=radar-attitude"
                  << ", latitude-deg=" << command->latitude.deg()
                  << ", longitude-deg=" << command->longitude.deg()
                  << ", altitude-km=" << command->altitude.km()
                  << ", roll-deg=" << command->roll.deg()
                  << ", pitch-deg=" << command->pitch.deg()
                  << ", yaw-deg=" << command->yaw.deg()
                  << '\n';
        return;
    }

    const auto& event = std::get<target::DetectionEvent>(message);
    std::cout << "[decoded] type=plot-information"
              << ", beam-type=" << beamTypeName(event.beamType)
              << ", request-id=" << event.requestId
              << ", transmit-time-ms=" << event.transmitTime.count()
              << ", plot-count=" << event.plots.size()
              << '\n';

    for (std::size_t index = 0; index < event.plots.size(); ++index)
    {
        const target::DetectionEvent::Plot& plot = event.plots[index];
        std::cout << "  [plot " << index << ']'
                  << " slant-range-km=" << plot.slantRange.km()
                  << ", azimuth-deg=" << plot.azimuth.deg()
                  << ", elevation-deg=" << plot.elevation.deg()
                  << ", doppler-mps=" << plot.dopplerVelocity.mps()
                  << ", power-db=" << plot.power_db
                  << '\n';
    }
}

void decodeFrames(
    const comm::MessageDecoder& messageDecoder,
    const std::vector<comm::MessageFrame>& frames
)
{
    for (const comm::MessageFrame& frame : frames)
    {
        const comm::DecodeResult decodeResult = messageDecoder.decode(frame);
        if (decodeResult.error != comm::DecodingError::NONE)
        {
            std::cerr << "[decoding-error] message-id=0x"
                      << std::hex << frame.header.message_Id << std::dec
                      << ", error=" << decodingErrorName(decodeResult.error)
                      << '\n';
            continue;
        }

        printDecodedMessage(*decodeResult.message);
    }
}

} // namespace

int main()
{
    const comm::NetworkEndpoint serverEndpoint
    {
        SERVER_ADDRESS,
        SERVER_PORT
    };
    const comm::NetworkEndpoint disabledEndpoint{};
    const comm::NetworkEndpoint scenarioSimulatorEndpoint
    {
        SCENARIO_SIMULATOR_ADDRESS,
        SCENARIO_SIMULATOR_PORT
    };

    comm::NetworkManager networkManager(
        serverEndpoint,
        disabledEndpoint,
        scenarioSimulatorEndpoint);
    const comm::MessageDecoder messageDecoder;

    try
    {
        networkManager.start();
    }
    catch (const std::exception& error)
    {
        std::cerr << error.what() << '\n';
        return 1;
    }

    std::signal(SIGINT, stopRunning);
    std::signal(SIGTERM, stopRunning);

    std::cout << "Listening on " << SERVER_ADDRESS << ':' << SERVER_PORT
              << " for " << SCENARIO_SIMULATOR_ADDRESS << ':'
              << SCENARIO_SIMULATOR_PORT << '\n'
              << "Press Ctrl+C to stop.\n";

    while (running != 0)
    {
        try
        {
            const std::optional<comm::ReceivedFrames> received =
                networkManager.receiveFrames(std::chrono::milliseconds{250});

            if (received.has_value())
            {
                std::cout << "[received] frames=" << received->frames.size()
                          << '\n';
                decodeFrames(messageDecoder, received->frames);
            }
        }
        catch (const std::exception& error)
        {
            std::cerr << "[error] " << error.what() << '\n';
        }
    }

    return 0;
}
