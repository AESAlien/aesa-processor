#include <app/worker/beam_worker.hpp>
#include <app/worker/comm_worker.hpp>
#include <app/worker/target_worker.hpp>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <exception>
#include <iostream>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <thread>
#include <utility>
#include <variant>
#include <vector>

namespace demo
{
using Clock = std::chrono::steady_clock;

std::mutex mutex;
std::condition_variable ready;

std::deque<comm::IncomingDto> inbox;
std::vector<target::TrackEvent> tracks;

std::vector<int> operations;
std::vector<int> sentTrackIds;

bool closed = false;

int received = 0;
int targetErrors = 0;
int trackBatches = 0;
int detectionBatches = 0;

int searchCount = 0;
int confirmCount = 0;
int trackingCount = 0;

// 테스트용 논리 시각이며 종료 플래그가 아니다.
std::atomic<long long> tickTime{0};

const auto start = Clock::now();

void require(bool value, const char* message)
{
    if (!value)
    {
        throw std::runtime_error(message);
    }
}

// demo::mutex를 획득한 상태에서 호출한다.
template <typename... Values>
void log(const Values&... values)
{
    const auto elapsed =
        std::chrono::duration_cast<std::chrono::milliseconds>(
            Clock::now() - start).count();

    std::cout << '[' << elapsed << " ms] ";
    (std::cout << ... << values);
    std::cout << '\n';
}

beam::SetOperationStateCommand operation(
    beam::OperationState state)
{
    return beam::SetOperationStateCommand(state);
}

beam::SetAttitudeCommand attitude(double yaw)
{
    return beam::SetAttitudeCommand(
        math::Angle::fromDegrees(37),
        math::Angle::fromDegrees(127),
        math::Distance::fromMeters(100),
        math::Angle::fromDegrees(0),
        math::Angle::fromDegrees(0),
        math::Angle::fromDegrees(yaw));
}

target::DetectionEvent detection(
    beam::BeamCommand::BeamType type,
    std::uint32_t requestId,
    int count = 1)
{
    std::vector<target::DetectionEvent::Plot> plots;

    for (int i = 0; i < count; ++i)
    {
        plots.emplace_back(
            math::Distance::fromMeters(12000 + 1000 * i),
            math::Angle::fromDegrees(10),
            math::Angle::fromDegrees(5),
            math::Velocity::fromMetersPerSecond(20),
            30.0f);
    }

    return target::DetectionEvent(
        type,
        requestId,
        std::chrono::milliseconds(tickTime.load()),
        std::move(plots));
}

void push(comm::IncomingDto message)
{
    {
        std::lock_guard<std::mutex> lock(mutex);
        inbox.push_back(std::move(message));
    }

    ready.notify_all();
}

void finishInput()
{
    {
        std::lock_guard<std::mutex> lock(mutex);
        closed = true;
    }

    ready.notify_all();
}

void timer(app::BeamChannel& channel)
{
    auto next = Clock::now();

    // 테스트 타이머는 정해진 횟수만 실행한다.
    for (int tick = 0; tick < 150; ++tick)
    {
        next += std::chrono::milliseconds(10);
        std::this_thread::sleep_until(next);

        const auto time = tickTime.fetch_add(10) + 10;

        try
        {
            channel.write(
                app::NextBeamCommandEvent{
                    std::chrono::milliseconds(time)});
        }
        catch (const app::ChannelClosedError&)
        {
            // 오류 종료 경로에서 채널이 닫혔으면 타이머도 종료한다.
            return;
        }

        if (next < Clock::now())
        {
            next = Clock::now();
        }
    }
}
} // namespace demo

// 실제 TCP가 아닌 테스트용 CommController 정의
namespace comm
{
class CommController::Impl
{
public:
    explicit Impl(CommConfiguration value)
        : configuration(std::move(value))
    {
    }

    CommConfiguration configuration;
};

CommController::CommController(
    CommConfiguration configuration)
    : _impl(std::make_unique<Impl>(std::move(configuration)))
{
}

CommController::~CommController() = default;

void CommController::openServer()
{
    std::lock_guard<std::mutex> lock(demo::mutex);

    demo::log("TEST input opened; no TCP socket");
}

std::optional<IncomingDto> CommController::receiveData()
{
    std::unique_lock<std::mutex> lock(demo::mutex);

    // 입력이 없으면 기다리고, 입력 큐가 닫혀도 깨어난다.
    demo::ready.wait(lock, []
    {
        return demo::closed || !demo::inbox.empty();
    });

    if (demo::inbox.empty())
    {
        // 이 테스트에서는 nullopt를 수신 종료로 사용한다.
        return std::nullopt;
    }

    auto message = std::move(demo::inbox.front());
    demo::inbox.pop_front();

    ++demo::received;

    if (auto* state =
            std::get_if<beam::SetOperationStateCommand>(&message))
    {
        const int value =
            state->powerState == beam::OperationState::ON ? 1 : 0;

        demo::operations.push_back(value);
        demo::log("RX operation=", value);
    }
    else if (auto* command =
                 std::get_if<beam::SetAttitudeCommand>(&message))
    {
        demo::log(
            "RX attitude yaw=",
            command->yaw.deg());
    }

    demo::ready.notify_all();

    return message;
}

void CommController::sendBeamCommand(
    const beam::BeamCommand& command)
{
    std::lock_guard<std::mutex> lock(demo::mutex);

    switch (command.beamType)
    {
    case beam::BeamCommand::BeamType::SEARCH:
        ++demo::searchCount;
        break;

    case beam::BeamCommand::BeamType::CONFIRMATION:
        ++demo::confirmCount;
        break;

    case beam::BeamCommand::BeamType::TRACKING:
        ++demo::trackingCount;
        break;
    }

    demo::log(
        "TX simulator: type=",
        static_cast<int>(command.beamType),
        ", beamId=", command.beamId,
        ", count=", command.commandCount,
        ", transmitTime(ms)=",
        command.transmitTime.count());
}

void CommController::sendTrackEvents(
    const std::vector<target::TrackEvent>& events)
{
    std::lock_guard<std::mutex> lock(demo::mutex);

    demo::require(
        !events.empty(),
        "Empty track batch");

    for (const auto& track : events)
    {
        demo::sentTrackIds.push_back(track.id);

        demo::require(
            track.dopplerVelocity.mps() == 20 &&
            track.velocity.mps() == 200,
            "Track velocity conversion failed");

        demo::log(
            "TX console: id=", track.id,
            ", state=", static_cast<int>(track.state),
            ", range(m)=", track.slantRange.m(),
            ", az(deg)=", track.azimuth.deg(),
            ", el(deg)=", track.elevation.deg(),
            ", doppler(m/s)=", track.dopplerVelocity.mps(),
            ", velocity(m/s)=", track.velocity.mps(),
            ", time(ms)=", track.timestamp.count());
    }

    ++demo::trackBatches;
}
} // namespace comm

// 실제 추적 알고리즘이 아닌 DTO 전달 확인용 샘플
namespace target
{
std::optional<beam::BeamRequest>
TargetController::handleDetection(
    DetectionEvent detection)
{
    std::lock_guard<std::mutex> lock(demo::mutex);

    // 예외 발생 후 다음 이벤트가 처리되는지 확인한다.
    if (detection.requestId == 999)
    {
        ++demo::targetErrors;

        throw std::invalid_argument(
            "Injected detection error");
    }

    ++demo::detectionBatches;

    demo::log(
        "Target received type=",
        static_cast<int>(detection.beamType),
        ", plots=", detection.plots.size());

    demo::tracks.clear();

    const auto state = demo::detectionBatches == 1
        ? TrackEvent::TrackState::INIT
        : TrackEvent::TrackState::TRACKING;

    // 전달 확인을 위한 샘플이며 실제 표적 연관 로직은 아니다.
    for (std::size_t i = 0; i < detection.plots.size(); ++i)
    {
        const auto& plot = detection.plots[i];

        demo::tracks.emplace_back(
            static_cast<std::uint16_t>(101 + i),
            state,
            plot.slantRange,
            math::Distance::fromMeters(11000),
            plot.azimuth,
            plot.elevation,
            plot.dopplerVelocity,
            math::Angle::fromDegrees(37),
            math::Angle::fromDegrees(127),
            math::Distance::fromMeters(1000),
            math::Velocity::fromMetersPerSecond(200),
            math::Angle::fromDegrees(90),
            math::Angle::fromDegrees(0),
            detection.transmitTime);
    }

    return std::nullopt;
}

std::vector<TrackEvent>
TargetController::nextTrackEvents() const
{
    std::lock_guard<std::mutex> lock(demo::mutex);

    return demo::tracks;
}
} // namespace target

int main()
{
    using Type = beam::BeamCommand::BeamType;
    using State = beam::OperationState;

    app::BeamChannel beamChannel;
    app::CommChannel commChannel;
    app::TargetChannel targetChannel;

    // 실제 Beam 모듈 구현을 사용한다.
    beam::BeamController beamController;

    comm::CommController commController(
        comm::CommConfiguration{
            {"127.0.0.20", 3200},
            {"127.0.0.10", 3100},
            {"127.0.0.100", 3300}});

    target::TargetController targetController;

    app::BeamWorker beamWorker(
        beamChannel,
        commChannel,
        beamController);

    app::CommWorker commWorker(
        commChannel,
        beamChannel,
        targetChannel,
        commController);

    app::TargetWorker targetWorker(
        targetChannel,
        beamChannel,
        commChannel,
        targetController);

    std::mutex errorMutex;
    std::exception_ptr workerError;

    std::thread rx;
    std::thread simulatorTx;
    std::thread consoleTx;
    std::thread beamTask;
    std::thread targetTask;
    std::thread timerTask;

    auto startThread = [&](auto action)
    {
        return std::thread([&, action]
        {
            try
            {
                action();
            }
            catch (...)
            {
                {
                    std::lock_guard<std::mutex> lock(errorMutex);

                    if (!workerError)
                    {
                        workerError = std::current_exception();
                    }
                }

                // 워커 밖으로 나온 예외에 대한 테스트 종료 처리
                demo::finishInput();

                targetChannel.close();
                beamChannel.close();
                commChannel.close();
            }
        });
    };

    auto joinAll = [&]
    {
        for (auto* task : {
            &rx,
            &simulatorTx,
            &consoleTx,
            &beamTask,
            &targetTask,
            &timerTask})
        {
            if (task->joinable())
            {
                task->join();
            }
        }
    };

    try
    {
        std::vector<comm::IncomingDto> scenario{
            demo::operation(State::OFF),       //    0ms
            demo::operation(State::ON),        //  100ms
            demo::attitude(30),                //  200ms
            demo::detection(Type::SEARCH, 1),   //  300ms
            demo::detection(Type::CONFIRMATION, 2),
            demo::detection(Type::TRACKING, 3),
            demo::detection(Type::SEARCH, 999), //  600ms: 예외
            demo::detection(Type::SEARCH, 4, 2),
            demo::operation(State::OFF),       //  800ms
            demo::operation(State::ON),        //  900ms
            demo::attitude(-15),               // 1000ms
            demo::detection(Type::SEARCH, 5, 0),
            demo::detection(Type::SEARCH, 6),
            demo::operation(State::OFF),       // 1300ms
            demo::attitude(0)};

        commController.openServer();

        rx = startThread([&]
        {
            commWorker.runReceive();
        });

        simulatorTx = startThread([&]
        {
            commWorker.runSendSimulator();
        });

        consoleTx = startThread([&]
        {
            commWorker.runSendConsole();
        });

        beamTask = startThread([&]
        {
            beamWorker.run();
        });

        targetTask = startThread([&]
        {
            targetWorker.run();
        });

        timerTask = startThread([&]
        {
            demo::timer(beamChannel);
        });

        const auto begin = demo::Clock::now();

        for (std::size_t i = 0; i < scenario.size(); ++i)
        {
            std::this_thread::sleep_until(
                begin + std::chrono::milliseconds(100 * i));

            auto message = std::move(scenario[i]);

            if (auto* event =
                    std::get_if<target::DetectionEvent>(&message))
            {
                demo::push(
                    target::DetectionEvent(
                        event->beamType,
                        event->requestId,
                        std::chrono::milliseconds(
                            demo::tickTime.load()),
                        event->plots));
            }
            else
            {
                demo::push(std::move(message));
            }

            // 확인·추적빔 요청 전달은 직접 주입하여 검증한다.
            if (i == 4 || i == 5)
            {
                const beam::BeamRequest request(
                    i == 4
                        ? beam::BeamRequest::BeamType::CONFIRMATION
                        : beam::BeamRequest::BeamType::TRACKING,
                    std::chrono::milliseconds(
                        demo::tickTime.load() + 100),
                    static_cast<std::uint32_t>(i),
                    math::Angle::fromDegrees(10),
                    math::Angle::fromDegrees(5));

                // write는 void이며 실패하면 예외를 던진다.
                beamChannel.write(request);
            }
        }

        {
            std::unique_lock<std::mutex> lock(demo::mutex);

            demo::require(
                demo::ready.wait_for(
                    lock,
                    std::chrono::seconds(2),
                    [&]
                    {
                        return demo::received ==
                            static_cast<int>(scenario.size());
                    }),
                "Receive timed out");
        }

        // 남은 입력 처리 후 receiveData가 nullopt를 반환한다.
        demo::finishInput();

        timerTask.join();
        rx.join();

        targetChannel.close();
        targetTask.join();

        beamChannel.close();
        beamTask.join();

        commChannel.close();
        simulatorTx.join();
        consoleTx.join();

        if (workerError)
        {
            std::rethrow_exception(workerError);
        }

        demo::require(
            demo::operations ==
                std::vector<int>{0, 1, 0, 1, 0},
            "Operation routing failed");

        demo::require(
            demo::searchCount > 0 &&
            demo::confirmCount == 1 &&
            demo::trackingCount == 1,
            "Beam type delivery failed");

        demo::require(
            demo::targetErrors == 1,
            "Exception injection failed");

        demo::require(
            demo::trackBatches == 5 &&
            demo::sentTrackIds ==
                std::vector<int>{101, 101, 101, 101, 102, 101},
            "Console track routing or exception continuation failed");

        std::cout
            << "[PASS] Actual BeamController: "
               "SEARCH/CONFIRMATION/TRACKING\n";

        std::cout
            << "[PASS] Console tracks: 5 batches, 6 records\n";

        std::cout
            << "[PASS] Target exception handled; "
               "subsequent events processed\n";

        return 0;
    }
    catch (const std::exception& error)
    {
        std::cerr
            << "[FAIL] "
            << error.what()
            << '\n';
    }
    catch (...)
    {
        std::cerr << "[FAIL] Unknown error\n";
    }

    demo::finishInput();

    targetChannel.close();
    beamChannel.close();
    commChannel.close();

    joinAll();

    return 1;
}