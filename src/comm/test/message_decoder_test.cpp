#include <network/message_decoder/message_decoder.hpp>
#include <network/message_framer/message_framer.hpp>
#include <network/network_manager/network_manager.hpp>

#include <chrono>
#include <csignal>
#include <cstdint>
#include <iostream>
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

const char* operationName(aesa_processor::comm::NetworkOperation operation)
{
    switch (operation)
    {
    case aesa_processor::comm::NetworkOperation::NONE:
        return "none";

    case aesa_processor::comm::NetworkOperation::CONFIGURATION:
        return "configuration";

    case aesa_processor::comm::NetworkOperation::SOCKET_API_STARTUP:
        return "socket-api-startup";

    case aesa_processor::comm::NetworkOperation::CREATE_SOCKET:
        return "create-socket";

    case aesa_processor::comm::NetworkOperation::SET_SOCKET_OPTION:
        return "set-socket-option";

    case aesa_processor::comm::NetworkOperation::BIND:
        return "bind";

    case aesa_processor::comm::NetworkOperation::LISTEN:
        return "listen";

    case aesa_processor::comm::NetworkOperation::SELECT:
        return "select";

    case aesa_processor::comm::NetworkOperation::ACCEPT:
        return "accept";

    case aesa_processor::comm::NetworkOperation::ADDRESS_CONVERSION:
        return "address-conversion";

    case aesa_processor::comm::NetworkOperation::RECEIVE:
        return "receive";

    case aesa_processor::comm::NetworkOperation::SEND:
        return "send";

    case aesa_processor::comm::NetworkOperation::CLOSE:
        return "close";

    case aesa_processor::comm::NetworkOperation::SOCKET_API_CLEANUP:
        return "socket-api-cleanup";
    }

    return "unknown";
}

const char* framingErrorName(aesa_processor::comm::FramingError error)
{
    switch (error)
    {
    case aesa_processor::comm::FramingError::NONE:
        return "none";

    case aesa_processor::comm::FramingError::UNKNOWN_MESSAGE_ID:
        return "unknown-message-id";

    case aesa_processor::comm::FramingError::INVALID_FRAME_SIZE:
        return "invalid-frame-size";
    }

    return "unknown";
}

const char* decodingErrorName(aesa_processor::comm::DecodingError error)
{
    switch (error)
    {
    case aesa_processor::comm::DecodingError::NONE:
        return "none";

    case aesa_processor::comm::DecodingError::UNSUPPORTED_MESSAGE_ID:
        return "unsupported-message-id";

    case aesa_processor::comm::DecodingError::UNSUPPORTED_VERSION:
        return "unsupported-version";

    case aesa_processor::comm::DecodingError::INVALID_MESSAGE_SIZE:
        return "invalid-message-size";

    case aesa_processor::comm::DecodingError::INVALID_ITEM_COUNT:
        return "invalid-item-count";

    case aesa_processor::comm::DecodingError::INVALID_FIELD_VALUE:
        return "invalid-field-value";
    }

    return "unknown";
}

const char* operationStateName(aesa_processor::beam::OperationState state)
{
    switch (state)
    {
    case aesa_processor::beam::OperationState::ON:
        return "on";

    case aesa_processor::beam::OperationState::OFF:
        return "off";
    }

    return "unknown";
}

const char* beamTypeName(aesa_processor::beam::BeamType beamType)
{
    switch (beamType)
    {
    case aesa_processor::beam::BeamType::SEARCH:
        return "search";

    case aesa_processor::beam::BeamType::CONFIRMATION:
        return "confirmation";

    case aesa_processor::beam::BeamType::TRACKING:
        return "tracking";
    }

    return "unknown";
}

void printDecodedMessage(const aesa_processor::comm::IncomingDto& message)
{
    if (const auto* command =
            std::get_if<aesa_processor::beam::SetOperationStateCommand>(&message))
    {
        std::cout << "[decoded] type=operation-state"
                  << ", state=" << operationStateName(command->powerState)
                  << '\n';
        return;
    }

    if (const auto* command =
            std::get_if<aesa_processor::beam::SetAttitudeCommand>(&message))
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

    const auto& event = std::get<aesa_processor::target::DetectionEvent>(message);
    std::cout << "[decoded] type=plot-information"
              << ", beam-type=" << beamTypeName(event.beamType)
              << ", request-id=" << event.requestId
              << ", transmit-time-ms=" << event.transmitTime.count()
              << ", plot-count=" << event.plots.size()
              << '\n';

    for (std::size_t index = 0; index < event.plots.size(); ++index)
    {
        const aesa_processor::target::DetectionEvent::Plot& plot = event.plots[index];
        std::cout << "  [plot " << index << ']'
                  << " slant-range-km=" << plot.slantRange.km()
                  << ", azimuth-deg=" << plot.azimuth.deg()
                  << ", elevation-deg=" << plot.elevation.deg()
                  << ", doppler-mps=" << plot.dopplerVelocity.mps()
                  << ", power-db=" << plot.power_db
                  << '\n';
    }
}

void decodeReceivedBytes(
    aesa_processor::comm::MessageFramer& messageFramer,
    const aesa_processor::comm::MessageDecoder& messageDecoder,
    const std::vector<uint8_t>& bytes
)
{
    const aesa_processor::comm::FrameResult frameResult = messageFramer.append(bytes);
    if (frameResult.error != aesa_processor::comm::FramingError::NONE)
    {
        std::cerr << "[framing-error] error="
                  << framingErrorName(frameResult.error)
                  << '\n';
        messageFramer.reset();
        return;
    }

    for (const aesa_processor::comm::MessageFrame& frame : frameResult.frames)
    {
        const aesa_processor::comm::DecodeResult decodeResult = messageDecoder.decode(frame);
        if (decodeResult.error != aesa_processor::comm::DecodingError::NONE)
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

void handleEvents(
    const std::vector<aesa_processor::comm::NetworkEvent>& events,
    aesa_processor::comm::MessageFramer& messageFramer,
    const aesa_processor::comm::MessageDecoder& messageDecoder
)
{
    for (const aesa_processor::comm::NetworkEvent& event : events)
    {
        if (const auto* connection =
                std::get_if<aesa_processor::comm::ConnectionEvent>(&event))
        {
            messageFramer.reset();
            if (connection->state == aesa_processor::comm::ConnectionState::DISCONNECTED)
            {
                std::cout << "[connection] disconnected"
                          << ", error=" << connection->errorCode
                          << '\n';
            }
            else
            {
                std::cout << "[connection] connected from "
                          << SCENARIO_SIMULATOR_ADDRESS << ':'
                          << SCENARIO_SIMULATOR_PORT << '\n';
            }
            continue;
        }

        if (const auto* received =
                std::get_if<aesa_processor::comm::DataReceivedEvent>(&event))
        {
            std::cout << "[received] bytes=" << received->bytes.size()
                      << '\n';
            decodeReceivedBytes(
                messageFramer,
                messageDecoder,
                received->bytes);
            continue;
        }

        const auto& rejected =
            std::get<aesa_processor::comm::ConnectionRejectedEvent>(event);
        std::cerr << "[connection-rejected] remote="
                  << rejected.remoteAddress << ':' << rejected.remotePort
                  << '\n';
    }
}

} // namespace

int main()
{
    const aesa_processor::comm::NetworkEndpoint serverEndpoint
    {
        SERVER_ADDRESS,
        SERVER_PORT
    };
    const aesa_processor::comm::NetworkEndpoint disabledEndpoint{};
    const aesa_processor::comm::NetworkEndpoint scenarioSimulatorEndpoint
    {
        SCENARIO_SIMULATOR_ADDRESS,
        SCENARIO_SIMULATOR_PORT
    };

    aesa_processor::comm::NetworkManager networkManager(
        serverEndpoint,
        disabledEndpoint,
        scenarioSimulatorEndpoint);
    aesa_processor::comm::MessageFramer messageFramer;
    const aesa_processor::comm::MessageDecoder messageDecoder;

    const aesa_processor::comm::OperationResult startResult = networkManager.start();
    if (startResult.status != aesa_processor::comm::OperationStatus::SUCCESS)
    {
        std::cerr << "Failed to start server: operation="
                  << operationName(startResult.operation)
                  << ", error=" << startResult.errorCode
                  << '\n';
        return 1;
    }

    std::signal(SIGINT, stopRunning);
    std::signal(SIGTERM, stopRunning);

    std::cout << "Listening on " << SERVER_ADDRESS << ':' << SERVER_PORT
              << " for " << SCENARIO_SIMULATOR_ADDRESS << ':'
              << SCENARIO_SIMULATOR_PORT << '\n'
              << "Press Ctrl+C to stop.\n";

    bool pollSucceeded = true;
    while (running != 0)
    {
        const aesa_processor::comm::PollResult pollResult = networkManager.poll(
            std::chrono::milliseconds{250});

        if (pollResult.status == aesa_processor::comm::PollStatus::TIMEOUT)
        {
            continue;
        }

        if (pollResult.status != aesa_processor::comm::PollStatus::EVENTS)
        {
            std::cerr << "Polling failed: operation="
                      << operationName(pollResult.failedOperation)
                      << ", error=" << pollResult.errorCode
                      << '\n';
            pollSucceeded = false;
            break;
        }

        handleEvents(
            pollResult.events,
            messageFramer,
            messageDecoder);
    }

    const aesa_processor::comm::OperationResult stopResult = networkManager.stop();
    if (stopResult.status != aesa_processor::comm::OperationStatus::SUCCESS)
    {
        std::cerr << "Failed to stop server: operation="
                  << operationName(stopResult.operation)
                  << ", error=" << stopResult.errorCode
                  << '\n';
        return 1;
    }

    return pollSucceeded ? 0 : 1;
}
