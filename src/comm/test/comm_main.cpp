#include <comm/controller/comm_controller.hpp>

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <iostream>
#include <thread>
#include <variant>

namespace
{

constexpr char SERVER_ADDRESS[] = "127.0.0.20";
constexpr uint16_t SERVER_PORT = 3200;
constexpr char OPERATOR_CONSOLE_ADDRESS[] = "127.0.0.10";
constexpr uint16_t OPERATOR_CONSOLE_PORT = 3100;
constexpr char SCENARIO_SIMULATOR_ADDRESS[] = "127.0.0.100";
constexpr uint16_t SCENARIO_SIMULATOR_PORT = 3300;
constexpr std::chrono::milliseconds RETRY_DELAY{100};

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

const char* beamTypeName(beam::BeamCommand::BeamType beamType)
{
    switch (beamType)
    {
    case beam::BeamCommand::BeamType::SEARCH:
        return "search";

    case beam::BeamCommand::BeamType::CONFIRMATION:
        return "confirmation";

    case beam::BeamCommand::BeamType::TRACKING:
        return "tracking";
    }

    return "unknown";
}

comm::CommConfiguration makeConfiguration()
{
    return
    {
        {SERVER_ADDRESS, SERVER_PORT},
        {OPERATOR_CONSOLE_ADDRESS, OPERATOR_CONSOLE_PORT},
        {SCENARIO_SIMULATOR_ADDRESS, SCENARIO_SIMULATOR_PORT}
    };
}

void printConfiguration()
{
    std::cout
        << "comm controller test server="
        << SERVER_ADDRESS
        << ':'
        << SERVER_PORT
        << '\n'
        << "operator-console="
        << OPERATOR_CONSOLE_ADDRESS
        << ':'
        << OPERATOR_CONSOLE_PORT
        << '\n'
        << "scenario-simulator="
        << SCENARIO_SIMULATOR_ADDRESS
        << ':'
        << SCENARIO_SIMULATOR_PORT
        << '\n'
        << "Press Ctrl+C to stop."
        << '\n';
}

void printIncomingData(const comm::IncomingDto& message)
{
    if (const auto* command =
            std::get_if<beam::SetOperationStateCommand>(&message))
    {
        std::cout
            << "[received] type=operation-state"
            << ", state="
            << operationStateName(command->powerState)
            << '\n';
        return;
    }

    if (const auto* command =
            std::get_if<beam::SetAttitudeCommand>(&message))
    {
        std::cout
            << "[received] type=radar-attitude"
            << ", latitude-deg="
            << command->latitude.deg()
            << ", longitude-deg="
            << command->longitude.deg()
            << ", altitude-km="
            << command->altitude.km()
            << ", roll-deg="
            << command->roll.deg()
            << ", pitch-deg="
            << command->pitch.deg()
            << ", yaw-deg="
            << command->yaw.deg()
            << '\n';
        return;
    }

    const auto& event = std::get<target::DetectionEvent>(message);
    std::cout
        << "[received] type=plot-information"
        << ", beam-type="
        << beamTypeName(event.beamType)
        << ", request-id="
        << event.requestId
        << ", transmit-time-ms="
        << event.transmitTime.count()
        << ", plot-count="
        << event.plots.size()
        << '\n';

    for (std::size_t index = 0; index < event.plots.size(); ++index)
    {
        const target::DetectionEvent::Plot& plot = event.plots[index];
        std::cout
            << "  [plot "
            << index
            << ']'
            << " slant-range-km="
            << plot.slantRange.km()
            << ", azimuth-deg="
            << plot.azimuth.deg()
            << ", elevation-deg="
            << plot.elevation.deg()
            << ", doppler-mps="
            << plot.dopplerVelocity.mps()
            << ", power-db="
            << plot.power_db
            << '\n';
    }
}

bool testOpenServer(comm::CommController& controller)
{
    try
    {
        controller.openServer();
        std::cout << "[success] openServer" << '\n';
        return true;
    }
    catch (const comm::ServerOpenError& error)
    {
        std::cerr
            << "[failure] openServer: "
            << error.what()
            << '\n';
        return false;
    }
}

int testReceiveData(comm::CommController& controller)
{
    while (true)
    {
        try
        {
            const std::optional<comm::IncomingDto> incoming =
                controller.receiveData();

            if (!incoming.has_value())
            {
                std::cout << "[finished] receiveData" << '\n';
                return 0;
            }

            printIncomingData(*incoming);
        }
        catch (const comm::ConnectionError& error)
        {
            std::cerr
                << "[connection-error] "
                << error.what()
                << '\n';
        }
        catch (const comm::ReceiveError& error)
        {
            std::cerr
                << "[receive-error] "
                << error.what()
                << '\n';
        }
        catch (const std::exception& error)
        {
            std::cerr
                << "[unexpected-error] "
                << error.what()
                << '\n';
            return 1;
        }

        std::this_thread::sleep_for(RETRY_DELAY);
    }
}

int runControllerTest()
{
    comm::CommController controller(makeConfiguration());
    if (!testOpenServer(controller))
    {
        return 1;
    }

    printConfiguration();
    return testReceiveData(controller);
}

} // namespace

int main()
{
    return runControllerTest();
}
