#pragma once

#include <app/channel/simulator_channel.hpp>

#include <functional>

namespace app
{
class SimulatorSendWorker
{
public:
    using SendBeamCommand = std::function<void(const beam::TransmitBeamCommand&)>;

    SimulatorSendWorker(
        SimulatorChannel& simulatorChannel,
        SendBeamCommand sendBeamCommand
    );

    void run();

private:
    SimulatorChannel& _simulatorChannel;
    SendBeamCommand _sendBeamCommand;
};
} // namespace app
