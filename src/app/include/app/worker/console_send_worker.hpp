#pragma once

#include <app/channel/console_channel.hpp>

#include <functional>

namespace app
{
class ConsoleSendWorker
{
public:
    using SendRadarAttitude = std::function<void(const beam::SetAttitudeCommand&)>;
    using SendTrackEvents = std::function<void(const std::vector<target::TrackEvent>&)>;

    ConsoleSendWorker(
        ConsoleChannel& consoleChannel,
        SendRadarAttitude sendRadarAttitude,
        SendTrackEvents sendTrackEvents
    );

    void run();

private:
    ConsoleChannel& _consoleChannel;
    SendRadarAttitude _sendRadarAttitude;
    SendTrackEvents _sendTrackEvents;
};
} // namespace app
