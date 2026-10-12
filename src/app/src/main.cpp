#include <app/worker/beam_worker.hpp>
#include <app/worker/comm_worker.hpp>
#include <app/worker/target_worker.hpp>

#include <comm/protocol/ST_OperatorMsg.hpp>
#include <comm/protocol/ST_RadarAttMsg.hpp>
#include <comm/protocol/ST_PlotMsg.hpp>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <deque>
#include <exception>
#include <iostream>
#include <mutex>
#include <stdexcept>
#include <thread>
#include <utility>
#include <variant>
#include <vector>

namespace demo
{
namespace protocol = comm::protocol;

using Clock = std::chrono::steady_clock;

using Wire = std::variant<
    protocol::ST_OperatorMsg,
    protocol::ST_RadarAttMsg,
    protocol::ST_PlotMsg>;

using Incoming = std::variant<
    beam::SetAttitudeCommand,
    beam::SetOperationStateCommand,
    target::DetectionEvent>;

std::mutex mutex;
std::condition_variable ready;

// 실제 TCP 수신을 대신하는 테스트 입력 큐
std::deque<Wire> inbox;
std::deque<Incoming> pending;

// 테스트용 BeamController가 보관하는 요청
std::deque<beam::BeamRequest> beamRequests;

// 테스트용 TargetController의 현재 트랙 목록
std::vector<target::TrackEvent> currentTracks;

bool closed = false;

int received = 0;
int rejected = 0;
int generated = 0;
int sent = 0;
int requestCalls = 0;
int trackBatches = 0;

std::vector<int> operations;
std::vector<int> detections;
std::vector<int> sentRequestedTypes;
std::vector<int> sentTrackIds;
std::vector<int> sentTrackStates;
std::vector<double> attitudes;

const auto start = Clock::now();

void require(bool value, const char* message)
{
    if (!value)
    {
        throw std::runtime_error(message);
    }
}

long long elapsedMs()
{
    return std::chrono::duration_cast<std::chrono::milliseconds>(
        Clock::now() - start).count();
}

// demo::mutex를 획득한 상태에서 호출한다.
template <typename... T>
void log(const T&... values)
{
    std::cout << '[' << elapsedMs() << " ms] ";
    (std::cout << ... << values);
    std::cout << '\n';
}

protocol::ST_MsgHeader header(
    protocol::MessageId id,
    std::uint32_t size)
{
    protocol::ST_MsgHeader result{};

    result.message_Id = static_cast<std::uint16_t>(id);
    result.block_size = size;

    return result;
}

protocol::ST_OperatorMsg control(std::uint8_t value)
{
    protocol::ST_OperatorMsg message{};

    message.msgHeader = header(
        protocol::MessageId::OperatorControl,
        sizeof(message));

    message.oper_status = value;

    return message;
}

protocol::ST_RadarAttMsg attitude(double yaw)
{
    protocol::ST_RadarAttMsg message{};

    message.msgHeader = header(
        protocol::MessageId::RadarAttitudeFromSimulator,
        sizeof(message));

    message.radar_lat_deg = 37;
    message.radar_lon_deg = 127;
    message.radar_alt_km = 0.1;
    message.yaw_deg = yaw;

    return message;
}

protocol::ST_PlotMsg plot(
    std::uint8_t type,
    std::uint16_t count = 1)
{
    require(count <= 100, "Too many plots in fixture");

    protocol::ST_PlotMsg message{};

    message.msgHeader = header(
        protocol::MessageId::PlotInformation,
        84u + 24u * count);

    message.beam_cmd.beamType = type;
    message.plotHd.number = count;

    for (std::uint16_t i = 0; i < count; ++i)
    {
        message.plot[i] = {
            static_cast<std::uint32_t>(i + 1),
            12,  // slantR_km
            10,  // azi_deg
            5,   // elv_deg
            20,  // doppler_mps
            30   // pwr_DB
        };
    }

    return message;
}

void push(Wire message)
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

// 테스트용 구조체 -> DTO 변환.
// 실제 TCP 프레이밍과 바이트 순서 처리는 포함하지 않는다.
void decode(const Wire& packet)
{
    if (const auto* controlMessage =
            std::get_if<protocol::ST_OperatorMsg>(&packet))
    {
        log(
            "RX console 0xCA01: oper_status=",
            +controlMessage->oper_status);

        require(
            controlMessage->msgHeader.message_Id == 0xCA01 &&
            controlMessage->msgHeader.block_size == 24,
            "Invalid operator header");

        require(
            controlMessage->oper_status <= 1,
            "oper_status must be 0 or 1");

        pending.emplace_back(
            beam::SetOperationStateCommand{
                controlMessage->oper_status == 1
                    ? beam::OperationState::ON
                    : beam::OperationState::OFF
            });
    }
    else if (const auto* attitudeMessage =
                 std::get_if<protocol::ST_RadarAttMsg>(&packet))
    {
        log(
            "RX simulator 0xBA02: yaw=",
            attitudeMessage->yaw_deg);

        require(
            attitudeMessage->msgHeader.message_Id == 0xBA02 &&
            attitudeMessage->msgHeader.block_size == 68,
            "Invalid attitude header");

        pending.emplace_back(
            beam::SetAttitudeCommand{
                math::Angle::fromDegrees(
                    attitudeMessage->radar_lat_deg),
                math::Angle::fromDegrees(
                    attitudeMessage->radar_lon_deg),
                math::Distance::fromKilometers(
                    attitudeMessage->radar_alt_km),
                math::Angle::fromDegrees(
                    attitudeMessage->roll_deg),
                math::Angle::fromDegrees(
                    attitudeMessage->pitch_deg),
                math::Angle::fromDegrees(
                    attitudeMessage->yaw_deg)
            });
    }
    else
    {
        const auto& plotMessage =
            std::get<protocol::ST_PlotMsg>(packet);

        const auto count = plotMessage.plotHd.number;
        const auto type = plotMessage.beam_cmd.beamType;

        log(
            "RX simulator 0xBA01: beamType=",
            +type,
            ", plots=",
            count);

        require(
            plotMessage.msgHeader.message_Id == 0xBA01 &&
            count <= 100 &&
            plotMessage.msgHeader.block_size == 84u + 24u * count,
            "Invalid plot header/count/size");

        require(
            type >= 1 && type <= 3,
            "beamType must be 1, 2 or 3");

        for (std::uint16_t i = 0; i < count; ++i)
        {
            const auto& item = plotMessage.plot[i];

            pending.emplace_back(
                target::DetectionEvent{
                    static_cast<beam::BeamCommand::BeamType>(type),
                    math::Distance::fromKilometers(item.slantR_km),
                    math::Angle::fromDegrees(item.azi_deg),
                    math::Angle::fromDegrees(item.elv_deg),
                    item.doppler_mps,
                    item.pwr_DB
                });
        }
    }
}

void timer(
    app::BeamChannel& channel,
    const std::atomic_bool& stop)
{
    const auto period = std::chrono::milliseconds(10);
    auto next = Clock::now() + period;

    while (!stop.load())
    {
        std::this_thread::sleep_until(next);

        if (stop.load())
        {
            break;
        }

        require(
            channel.write(app::NextBeamCommandEvent{}),
            "Timer destination closed");

        next += period;

        if (next <= Clock::now())
        {
            next = Clock::now() + period;
        }
    }
}
} // namespace demo

// 테스트용 BeamController 구현
namespace beam
{
void BeamController::setOperationState(
    SetOperationStateCommand command)
{
    std::lock_guard<std::mutex> lock(demo::mutex);

    const int value =
        command.powerState == OperationState::ON ? 1 : 0;

    demo::operations.push_back(value);

    demo::log(
        "BeamController::setOperationState -> ",
        value == 1 ? "ON" : "STOP");

    // 실제 ON/OFF에 따른 기능 제어는 구현하지 않는다.
}

void BeamController::setAttitude(
    SetAttitudeCommand command)
{
    std::lock_guard<std::mutex> lock(demo::mutex);

    demo::require(
        command.latitude.deg() == 37 &&
        command.longitude.deg() == 127 &&
        command.altitude.m() == 100,
        "Attitude values/units changed");

    demo::attitudes.push_back(command.yaw.deg());

    demo::log(
        "BeamController::setAttitude yaw=",
        command.yaw.deg(),
        ", altitude(m)=",
        command.altitude.m());
}

void BeamController::requestBeam(
    BeamRequest request)
{
    std::lock_guard<std::mutex> lock(demo::mutex);

    ++demo::requestCalls;
    demo::beamRequests.push_back(request);

    demo::log(
        "BeamController::requestBeam type=",
        static_cast<int>(request.beamType));
}

BeamCommand BeamController::nextBeamCommand()
{
    std::lock_guard<std::mutex> lock(demo::mutex);

    BeamCommand command;
    command.beamType = BeamCommand::BeamType::SEARCH;

    if (!demo::beamRequests.empty())
    {
        const auto request = demo::beamRequests.front();
        demo::beamRequests.pop_front();

        switch (request.beamType)
        {
        case BeamRequest::BeamType::CONFIRMATION:
            command.beamType =
                BeamCommand::BeamType::CONFIRMATION;
            break;

        case BeamRequest::BeamType::TRACKING:
            command.beamType =
                BeamCommand::BeamType::TRACKING;
            break;

        default:
            throw std::runtime_error(
                "Invalid BeamRequest type");
        }

        command.azimuth_ant = request.azimuth_ant;
        command.elevation_ant = request.elevation_ant;
    }

    command.beamId = 1;

    command.commandCount =
        static_cast<std::uint32_t>(++demo::generated);

    command.timestamp =
        std::chrono::milliseconds(demo::elapsedMs());

    return command;
}
} // namespace beam

// 테스트용 CommController 구현
namespace comm
{
void CommController::openServer()
{
    std::lock_guard<std::mutex> lock(demo::mutex);

    demo::log("TEST input opened (no TCP socket)");
}

demo::Incoming CommController::receiveData()
{
    std::unique_lock<std::mutex> lock(demo::mutex);

    while (demo::pending.empty())
    {
        demo::ready.wait(lock, []
        {
            return demo::closed || !demo::inbox.empty();
        });

        if (demo::inbox.empty())
        {
            // 테스트 종료 시 수신 대기를 해제하기 위한 반환값.
            // runReceive가 stop을 확인하므로 실제 전달되지 않는다.
            return beam::SetOperationStateCommand{
                beam::OperationState::OFF
            };
        }

        auto packet = std::move(demo::inbox.front());
        demo::inbox.pop_front();

        ++demo::received;

        try
        {
            demo::decode(packet);
        }
        catch (const std::runtime_error& error)
        {
            ++demo::rejected;

            demo::log(
                "REJECT: ",
                error.what());
        }

        demo::ready.notify_all();
    }

    auto result = std::move(demo::pending.front());
    demo::pending.pop_front();

    return result;
}

void CommController::sendBeamCommand(
    const beam::BeamCommand& command)
{
    std::lock_guard<std::mutex> lock(demo::mutex);

    const auto type =
        static_cast<int>(command.beamType);

    demo::require(
        type >= 1 && type <= 3,
        "Invalid outgoing beam type");

    if (type != 1)
    {
        demo::sentRequestedTypes.push_back(type);

        demo::require(
            command.azimuth_ant.deg() == 10 &&
            command.elevation_ant.deg() == 5,
            "BeamRequest angles lost");
    }

    demo::require(
        command.commandCount ==
            static_cast<std::uint32_t>(demo::sent + 1),
        "Beam command lost/reordered");

    demo::protocol::ST_BeamTxMsg packet{};

    packet.msgHeader = demo::header(
        demo::protocol::MessageId::BeamTransmit,
        sizeof(packet));

    packet.beam_cmd.beamType =
        static_cast<std::uint8_t>(command.beamType);

    packet.beam_cmd.beamID = command.beamId;
    packet.beam_cmd.commandCount = command.commandCount;

    packet.beam_cmd.timestamp =
        static_cast<std::uint32_t>(
            command.timestamp.count());

    packet.beam_cmd.beam_az_deg =
        static_cast<float>(command.azimuth_ant.deg());

    packet.beam_cmd.beam_el_deg =
        static_cast<float>(command.elevation_ant.deg());

    packet.beam_cmd.az_width_deg =
        static_cast<float>(command.azimuthBeamWidth.deg());

    packet.beam_cmd.el_width_deg =
        static_cast<float>(command.elevationBeamWidth.deg());

    ++demo::sent;

    demo::log(
        "TX simulator mock 0xAB01, bytes=",
        packet.msgHeader.block_size,
        ", beamType=",
        +packet.beam_cmd.beamType,
        ", commandCount=",
        packet.beam_cmd.commandCount);
}

void CommController::sendTrackEvents(
    const std::vector<target::TrackEvent>& events)
{
    std::lock_guard<std::mutex> lock(demo::mutex);

    demo::require(
        events.size() == 2,
        "Wrong track batch size");

    const auto expectedState = demo::trackBatches == 0
        ? target::TrackEvent::TrackState::INIT
        : target::TrackEvent::TrackState::TRACKING;

    for (std::size_t i = 0; i < events.size(); ++i)
    {
        const auto& track = events[i];

        demo::require(
            track.id == 101 + i &&
            track.state == expectedState &&
            track.slantRange.m() == 12000 + 1000 * i &&
            track.groundRange.m() == 11000 &&
            track.azimuth.deg() == 10 &&
            track.elevation.deg() == 5 &&
            track.dopplerVelocity_mps == 20 &&
            track.latitude.deg() == 37 &&
            track.longitude.deg() == 127 &&
            track.altitude.m() == 1000 &&
            track.velocity_mps == 200 &&
            track.heading.deg() == 90 &&
            track.flightPathAngle.deg() == 0,
            "Track fields changed");

        demo::sentTrackIds.push_back(track.id);

        demo::sentTrackStates.push_back(
            static_cast<int>(track.state));

        demo::log(
            "TX console mock: id=", track.id,
            ", state=", static_cast<int>(track.state),
            ", slant(m)=", track.slantRange.m(),
            ", ground(m)=", track.groundRange.m(),
            ", az(deg)=", track.azimuth.deg(),
            ", el(deg)=", track.elevation.deg(),
            ", doppler(m/s)=", track.dopplerVelocity_mps,
            ", lat=", track.latitude.deg(),
            ", lon=", track.longitude.deg(),
            ", alt(m)=", track.altitude.m(),
            ", velocity(m/s)=", track.velocity_mps,
            ", heading(deg)=", track.heading.deg(),
            ", flightPath(deg)=", track.flightPathAngle.deg(),
            ", time(ms)=", track.timestamp.count());
    }

    ++demo::trackBatches;
}
} // namespace comm

// 테스트용 TargetController 구현
namespace target
{
std::optional<beam::BeamRequest>
TargetController::handleDetection(
    DetectionEvent detection)
{
    std::lock_guard<std::mutex> lock(demo::mutex);

    demo::require(
        detection.slantRange.m() == 12000 &&
        detection.azimuth.deg() == 10 &&
        detection.elevation.deg() == 5 &&
        detection.dopplerVelocity_mps == 20 &&
        detection.power_db == 30,
        "Detection values/units changed");

    demo::detections.push_back(
        static_cast<int>(detection.beamType));

    demo::log(
        "TargetController::handleDetection beamType=",
        static_cast<int>(detection.beamType),
        ", range(m)=",
        detection.slantRange.m());

    // 전달 검증용 고정 샘플.
    // 실제 표적 연관이나 추적 상태 판단 알고리즘은 아니다.
    const auto state = demo::detections.size() == 1
        ? TrackEvent::TrackState::INIT
        : TrackEvent::TrackState::TRACKING;

    const auto timestamp =
        std::chrono::milliseconds(demo::elapsedMs());

    demo::currentTracks.clear();

    for (int i = 0; i < 2; ++i)
    {
        demo::currentTracks.push_back(
            TrackEvent{
                static_cast<std::uint16_t>(101 + i),
                state,
                math::Distance::fromMeters(12000 + 1000 * i),
                math::Distance::fromMeters(11000),
                math::Angle::fromDegrees(10),
                math::Angle::fromDegrees(5),
                20,
                math::Angle::fromDegrees(37),
                math::Angle::fromDegrees(127),
                math::Distance::fromMeters(1000),
                200,
                math::Angle::fromDegrees(90),
                math::Angle::fromDegrees(0),
                timestamp
            });
    }

    // 빔 요청 판단은 구현하지 않는다.
    return std::nullopt;
}

std::vector<TrackEvent>
TargetController::nextTrackEvents() const
{
    std::lock_guard<std::mutex> lock(demo::mutex);

    return demo::currentTracks;
}
} // namespace target

int main()
{
    static_assert(
        sizeof(demo::protocol::ST_OperatorMsg) == 24);

    static_assert(
        sizeof(demo::protocol::ST_RadarAttMsg) == 68);

    static_assert(
        sizeof(demo::protocol::ST_BeamTxMsg) == 80);

    static_assert(
        offsetof(demo::protocol::ST_PlotMsg, plot) == 84);

    static_assert(
        sizeof(demo::protocol::ST_Plot) == 24);

    app::BeamChannel beamChannel;
    app::CommChannel commChannel;
    app::TargetChannel targetChannel;

    beam::BeamController beamController;
    comm::CommController commController;
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

    std::atomic_bool stop{false};

    std::thread rxTask;
    std::thread simulatorTxTask;
    std::thread consoleTxTask;
    std::thread beamTask;
    std::thread targetTask;
    std::thread timerTask;

    std::mutex errorMutex;
    std::exception_ptr workerError;

    auto startThread = [&](auto work)
    {
        return std::thread([&, work]
        {
            try
            {
                work();
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

                stop.store(true);
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
            &rxTask,
            &simulatorTxTask,
            &consoleTxTask,
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
        auto badAttitude = demo::attitude(99);
        badAttitude.msgHeader.block_size = 67;

        std::vector<demo::Wire> scenario{
            demo::control(0),     //    0ms: STOP 전달
            demo::attitude(30),   //  100ms: 자세
            demo::plot(1),        //  200ms: SEARCH 탐지
            demo::control(1),     //  300ms: ON 전달
            demo::plot(2),        //  400ms: CONFIRM 탐지
            demo::plot(3),        //  500ms: TRACK 탐지
            demo::control(2),     //  600ms: 잘못된 운용값
            demo::plot(1, 2),     //  700ms: plot 2개
            demo::control(0),     //  800ms: STOP 전달
            demo::attitude(-15),  //  900ms: 자세
            demo::plot(1, 0),     // 1000ms: plot 0개
            demo::control(1),     // 1100ms: ON 전달
            demo::plot(0),        // 1200ms: 잘못된 빔 종류
            demo::control(0),     // 1300ms: STOP 전달
            badAttitude          // 1400ms: 잘못된 길이
        };

        commController.openServer();

        rxTask = startThread([&]
        {
            commWorker.runReceive(stop);
        });

        simulatorTxTask = startThread([&]
        {
            commWorker.runSendSimulator();
        });

        consoleTxTask = startThread([&]
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
            demo::timer(beamChannel, stop);
        });

        const auto begin = demo::Clock::now();

        for (std::size_t i = 0; i < scenario.size(); ++i)
        {
            std::this_thread::sleep_until(
                begin + std::chrono::milliseconds(100 * i));

            demo::push(std::move(scenario[i]));

            // 확인/추적빔 송신 경로는 명시적 요청으로 확인한다.
            if (i == 4 || i == 5)
            {
                beam::BeamRequest request;

                request.beamType = i == 4
                    ? beam::BeamRequest::BeamType::CONFIRMATION
                    : beam::BeamRequest::BeamType::TRACKING;

                request.timestamp =
                    std::chrono::milliseconds(
                        demo::elapsedMs());

                request.azimuth_ant =
                    math::Angle::fromDegrees(10);

                request.elevation_ant =
                    math::Angle::fromDegrees(5);

                demo::require(
                    beamChannel.write(request),
                    "BeamRequest injection failed");
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
                "Input processing timed out");
        }

        // 생산 중단
        stop.store(true);
        demo::finishInput();

        timerTask.join();
        rxTask.join();

        // Target의 남은 탐지 처리와 트랙 전달 완료
        targetChannel.close();
        targetTask.join();

        // Beam의 남은 요청과 타이머 처리 완료
        beamChannel.close();
        beamTask.join();

        // 두 송신 큐의 남은 메시지 처리 완료
        commChannel.close();
        simulatorTxTask.join();
        consoleTxTask.join();

        if (workerError)
        {
            std::rethrow_exception(workerError);
        }

        demo::require(
            demo::operations ==
                std::vector<int>{0, 1, 0, 1, 0},
            "Wrong operation values/order");

        demo::require(
            demo::attitudes ==
                std::vector<double>{30, -15},
            "Attitude routing failed");

        demo::require(
            demo::detections ==
                std::vector<int>{1, 2, 3, 1, 1},
            "Plot type/count failed");

        demo::require(
            demo::rejected == 3,
            "Invalid input checks failed");

        demo::require(
            demo::requestCalls == 2 &&
            demo::trackBatches == 5,
            "Wrong request/track counts");

        demo::require(
            demo::sentRequestedTypes ==
                std::vector<int>{2, 3} &&
            demo::beamRequests.empty(),
            "CONFIRM/TRACK request delivery failed");

        demo::require(
            demo::sent > 0 &&
            demo::sent == demo::generated,
            "Beam output path failed");

        demo::require(
            demo::sentTrackIds == std::vector<int>{
                101, 102,
                101, 102,
                101, 102,
                101, 102,
                101, 102
            },
            "Track order/count failed");

        demo::require(
            demo::sentTrackStates == std::vector<int>{
                1, 1,
                2, 2,
                2, 2,
                2, 2,
                2, 2
            },
            "Track state sequence failed");

        std::cout
            << "[PASS] Console track delivery: "
               "5 batches, 10 records\n";

        std::cout
            << "[PASS] Message IDs, values, units, "
               "batch/empty plots, invalid input, routing\n";

        std::cout
            << "[PASS] Routing and SEARCH/CONFIRM/TRACK "
               "transmission; pipeline drained\n";

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

    stop.store(true);
    demo::finishInput();

    targetChannel.close();
    beamChannel.close();
    commChannel.close();

    joinAll();

    return 1;
}