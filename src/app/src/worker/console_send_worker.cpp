#include <app/worker/console_send_worker.hpp>
#include <app/worker/worker_error.hpp>

#include <utility>
#include <variant>

namespace app
{
ConsoleSendWorker::ConsoleSendWorker(
    ConsoleChannel& consoleChannel,
    SendRadarAttitude sendRadarAttitude,
    SendTrackEvents sendTrackEvents
) : _consoleChannel(consoleChannel),
    _sendRadarAttitude(std::move(sendRadarAttitude)),
    _sendTrackEvents(std::move(sendTrackEvents))
{
}

void ConsoleSendWorker::run()
{
    while (auto event = _consoleChannel.read())
    {
        tryWorkerAction("CommSendConsole", [&]
        {
            if (const auto* attitude =
                    std::get_if<beam::SetAttitudeCommand>(&*event))
            {
                _sendRadarAttitude(*attitude);
                return;
            }

            const auto& tracks =
                std::get<std::vector<target::TrackEvent>>(*event);

            _sendTrackEvents(tracks);
        });
    }
}
} // namespace app
