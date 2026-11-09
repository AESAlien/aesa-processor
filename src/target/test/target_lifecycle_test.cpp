#include "domain/algorithm/nds_algorithm.hpp"
#include "domain/algorithm/constant_velocity_algorithm.hpp"
#include "domain/factory/detection_result_factory.hpp"
#include "domain/factory/track_factory.hpp"
#include "repository/track_repository.hpp"
#include "domain/selector/track_selector.hpp"

#include <target/controller/target_controller.hpp>

#include <gtest/gtest.h>

#include <limits>
#include <memory>
#include <type_traits>
#include <utility>

namespace target
{
namespace
{

using namespace math::literals;
using namespace std::chrono_literals;
using Time = std::chrono::milliseconds;
using BeamType = beam::BeamType;

static_assert(std::is_const_v<decltype(DetectionResultBase::requestId)>);
static_assert(std::is_const_v<decltype(DetectionResultBase::transmitTime)>);
static_assert(std::is_const_v<decltype(DetectionResultBase::observations)>);
static_assert(!std::is_copy_assignable_v<Observation>);
static_assert(!std::is_default_constructible_v<ConfirmedTrack>);
static_assert(!std::is_constructible_v<SearchDetectionResult, DetectionEvent>);
static_assert(!std::is_constructible_v<ConfirmationDetectionResult, DetectionEvent>);
static_assert(!std::is_constructible_v<TrackingDetectionResult, DetectionEvent>);
static_assert(!std::is_convertible_v<Observation, TentativeTrack>);
static_assert(!std::is_convertible_v<TentativeTrack, ConfirmedTrack>);

TargetConfig config()
{
    return TargetConfig(
        AssociationConfig(1000_mps, 30_mps, 11.345),
        MeasurementConfig(7_m),
        TrackingConfig(10_mps2, 0_mps2),
        TrackManagementConfig(100ms, 1000ms, 3, 100)
    );
}

Observation observation(Time time, math::Length range = 1000_m)
{
    return Observation(math::SphericalPosition(range, 0_deg, 0_deg), 0_mps, -10.0f, time);
}

DetectionEvent event(
    BeamType type,
    std::uint32_t id,
    Time time,
    bool detected = true,
    math::Length range = 1000_m
) {
    std::vector<DetectionEvent::Plot> plots;
    if (detected)
    {
        plots.emplace_back(
            math::Distance(range),
            math::CircularAngle(0_deg),
            math::TiltAngle(0_deg),
            0_mps,
            -10.0f
        );
    }
    return DetectionEvent(type, id, time, std::move(plots));
}

TEST(DetectionResultTest, FactoryChoosesSubtypeAndPreservesEmptyResults)
{
    const auto search = DetectionResultFactory::from(event(BeamType::SEARCH, 0, 0ms, false));
    ASSERT_TRUE(std::holds_alternative<SearchDetectionResult>(search));
    const auto& searchResult = std::get<SearchDetectionResult>(search);
    EXPECT_EQ(searchResult.requestId, 0u);
    EXPECT_EQ(searchResult.transmitTime, 0ms);
    EXPECT_TRUE(searchResult.observations.empty());
    const auto confirmation = DetectionResultFactory::from(event(BeamType::CONFIRMATION, 7, 100ms, false));
    ASSERT_TRUE(std::holds_alternative<ConfirmationDetectionResult>(confirmation));
    const auto& confirmationResult = std::get<ConfirmationDetectionResult>(confirmation);
    EXPECT_EQ(confirmationResult.requestId, 7u);
    EXPECT_EQ(confirmationResult.transmitTime, 100ms);
    EXPECT_TRUE(confirmationResult.observations.empty());
    const auto tracking = DetectionResultFactory::from(event(BeamType::TRACKING, 7, 1100ms, false));
    ASSERT_TRUE(std::holds_alternative<TrackingDetectionResult>(tracking));
    const auto& trackingResult = std::get<TrackingDetectionResult>(tracking);
    EXPECT_EQ(trackingResult.requestId, 7u);
    EXPECT_EQ(trackingResult.transmitTime, 1100ms);
    EXPECT_TRUE(trackingResult.observations.empty());
}

TEST(DetectionResultTest, RejectsInvalidMetadataEvenWithoutPlots)
{
    EXPECT_THROW(
        DetectionResultFactory::from(event(static_cast<BeamType>(-1), 1, 0ms, false)),
        std::invalid_argument
    );
    for (auto type : {BeamType::SEARCH, BeamType::CONFIRMATION, BeamType::TRACKING})
    {
        EXPECT_THROW(DetectionResultFactory::from(event(type, 1, -1ms, false)), std::invalid_argument);
    }
    for (auto type : {BeamType::CONFIRMATION, BeamType::TRACKING})
    {
        EXPECT_THROW(DetectionResultFactory::from(event(type, 0, 0ms, false)), std::invalid_argument);
    }
}

TEST(DetectionResultTest, OwnsValidatedSnapshotIndependentOfMutableEvent)
{
    for (auto type : {BeamType::SEARCH, BeamType::CONFIRMATION, BeamType::TRACKING})
    {
        SCOPED_TRACE(static_cast<int>(type));
        auto incoming = event(type, 3, 10ms);
        incoming.plots.emplace_back(
            math::Distance(2000_m),
            math::CircularAngle(90_deg),
            math::TiltAngle(10_deg),
            12_mps,
            -20.0f
        );
        const auto result = DetectionResultFactory::from(incoming);
        incoming.beamType = static_cast<BeamType>(-1);
        incoming.requestId = 99;
        incoming.transmitTime = 20ms;
        incoming.plots.front().power_db = std::numeric_limits<float>::quiet_NaN();
        incoming.plots.clear();

        std::visit(
            [](const auto& value)
            {
                EXPECT_EQ(value.requestId, 3u);
                EXPECT_EQ(value.transmitTime, 10ms);
                ASSERT_EQ(value.observations.size(), 2u);
                const auto& first = value.observations[0];
                EXPECT_EQ(first.lastObservationTime(), 10ms);
                EXPECT_EQ(first.power_db(), -10.0f);
                EXPECT_EQ(first.slantRange(), 1000_m);
                const auto& second = value.observations[1];
                EXPECT_EQ(second.lastObservationTime(), 10ms);
                EXPECT_EQ(second.slantRange(), 2000_m);
                EXPECT_EQ(second.azimuth(), math::CircularAngle(90_deg));
                EXPECT_EQ(second.elevation(), math::TiltAngle(10_deg));
                EXPECT_EQ(second.dopplerVelocity(), 12_mps);
                EXPECT_EQ(second.power_db(), -20.0f);
            },
            result
        );
    }
}

TEST(DetectionResultTest, RejectsInvalidPlotInEveryResultType)
{
    for (auto type : {BeamType::SEARCH, BeamType::CONFIRMATION, BeamType::TRACKING})
    {
        auto incoming = event(type, 1, 100ms);
        incoming.plots.emplace_back(
            math::Distance(0_m),
            math::CircularAngle(0_deg),
            math::TiltAngle(0_deg),
            0_mps,
            -10.0f
        );
        EXPECT_THROW(DetectionResultFactory::from(incoming), std::invalid_argument);
        incoming.plots.pop_back();
        incoming.plots.front().power_db = std::numeric_limits<float>::infinity();
        EXPECT_THROW(DetectionResultFactory::from(incoming), std::invalid_argument);
    }
}

TEST(TrackRepositoryTest, ReplacementRequiresAnExistingIdentity)
{
    TrackRepository repository;
    const TrackFactory factory(std::make_shared<ConstantVelocityAlgorithm>());
    const auto tentative = factory.createTentative(repository.nextId(), observation(0ms));
    auto track = tentative.confirm(observation(100ms, 1001_m));
    ASSERT_TRUE(track);

    EXPECT_FALSE(repository.replace(*track));
    EXPECT_TRUE(repository.targets().empty());
    EXPECT_EQ(repository.add(*track), tentative.id());
    const auto replacement = tentative.confirm(observation(200ms, 1002_m));
    ASSERT_TRUE(replacement);
    EXPECT_TRUE(repository.replace(*replacement));
    ASSERT_EQ(repository.targets().size(), 1u);
    ASSERT_NE(repository.findTrack(tentative.id()), nullptr);
    EXPECT_EQ(repository.findTrack(tentative.id())->state(), replacement->state());
}

TEST(TrackRepositoryTest, AddsPreparedTracksAndTentativeTracksInOrder)
{
    TrackRepository repository;
    const auto algorithm = std::make_shared<ConstantVelocityAlgorithm>();
    const auto first = repository.add(TentativeTrack(42, observation(0ms), algorithm));
    const TentativeTrack tentative(7, observation(10ms, 2000_m), algorithm);
    auto track = tentative.confirm(observation(110ms, 2001_m));
    ASSERT_TRUE(track);
    const auto state = track->state();

    const auto second = repository.add(std::move(*track));

    EXPECT_EQ(first, 42u);
    EXPECT_EQ(second, 7u);
    ASSERT_EQ(repository.targets().size(), 2u);
    auto current = repository.targets().begin();
    EXPECT_EQ(std::get<TentativeTrack>(*current++).id(), first);
    EXPECT_EQ(std::get<ConfirmedTrack>(*current).id(), second);
    EXPECT_EQ(repository.findTrack(first), nullptr);
    EXPECT_EQ(repository.findTentative(second), nullptr);
    const auto* stored = repository.findTrack(second);
    ASSERT_NE(stored, nullptr);
    EXPECT_EQ(stored->state(), state);
    EXPECT_EQ(stored->initialTime(), 10ms);
    EXPECT_EQ(stored->lastObservationTime(), 110ms);
    EXPECT_EQ(stored->slantRange(), 2001_m);
}

TEST(TrackRepositoryTest, RejectsDuplicateIdsAcrossBothTrackTypes)
{
    TrackRepository repository;
    TrackFactory factory(std::make_shared<ConstantVelocityAlgorithm>());
    const auto tentative = factory.createTentative(repository.nextId(), observation(0ms));
    const auto track = tentative.confirm(observation(100ms, 1001_m));
    ASSERT_TRUE(track);
    ASSERT_EQ(repository.add(tentative), tentative.id());
    const auto* original = repository.findTentative(tentative.id());

    EXPECT_THROW(repository.add(tentative), std::invalid_argument);
    EXPECT_THROW(repository.add(*track), std::invalid_argument);
    EXPECT_EQ(repository.targets().size(), 1u);
    EXPECT_EQ(repository.findTentative(tentative.id()), original);

    ASSERT_TRUE(repository.replace(*track));
    const auto* stored = repository.findTrack(tentative.id());
    EXPECT_THROW(repository.add(tentative), std::invalid_argument);
    EXPECT_THROW(repository.add(*track), std::invalid_argument);
    EXPECT_EQ(repository.targets().size(), 1u);
    EXPECT_EQ(repository.findTrack(tentative.id()), stored);
    EXPECT_EQ(stored->state(), track->state());
    EXPECT_EQ(repository.add(factory.createTentative(repository.nextId(), observation(200ms))), 2u);
}

TEST(TrackFactoryTest, TargetRetainsAlgorithmAfterFactoryIsDestroyed)
{
    std::weak_ptr<const ITrackingAlgorithm> algorithm;
    const auto tentative = [&algorithm]
    {
        const auto injectedAlgorithm = std::make_shared<ConstantVelocityAlgorithm>();
        algorithm = injectedAlgorithm;
        const TrackFactory factory(injectedAlgorithm);
        return factory.createTentative(1, observation(0ms));
    }();

    EXPECT_FALSE(algorithm.expired());
    EXPECT_EQ(tentative.id(), 1u);
    EXPECT_EQ(tentative.observation().slantRange(), 1000_m);
    const auto track = tentative.confirm(observation(100ms, 1001_m));
    ASSERT_TRUE(track.has_value());
    EXPECT_EQ(track->id(), tentative.id());
    EXPECT_TRUE(track->predict(200ms).has_value());
    EXPECT_EQ(tentative.observation().slantRange(), 1000_m);
    EXPECT_EQ(tentative.lastResultTime(), 0ms);
}

TEST(TrackFactoryTest, TrackRetainsAlgorithmAfterFactoryAndTentativeAreDestroyed)
{
    std::weak_ptr<const ITrackingAlgorithm> algorithm;
    {
        auto track = [&algorithm]
        {
            const auto injectedAlgorithm = std::make_shared<ConstantVelocityAlgorithm>();
            algorithm = injectedAlgorithm;
            const TrackFactory factory(injectedAlgorithm);
            const auto tentative = factory.createTentative(1, observation(10ms));
            return tentative.confirm(observation(110ms, 1001_m));
        }();

        ASSERT_TRUE(track);
        EXPECT_FALSE(algorithm.expired());
        EXPECT_EQ(track->id(), 1u);
        EXPECT_EQ(track->initialTime(), 10ms);
        EXPECT_EQ(track->lastObservationTime(), 110ms);
        EXPECT_EQ(track->lastResultTime(), 110ms);
        EXPECT_EQ(track->consecutiveMissCount(), 0u);
        EXPECT_EQ(track->slantRange(), 1001_m);
        const auto prediction = track->predict(210ms);
        ASSERT_TRUE(prediction);
        EXPECT_EQ(prediction->sphericalPosition().radius, 1002_m);
        EXPECT_TRUE(track->update(observation(1110ms, 1011_m)));
    }
    EXPECT_TRUE(algorithm.expired());
}

TEST(TargetLifecycleTest, ConfirmationPreservesIdentityAndRegistrationOrder)
{
    TrackRepository repository;
    const auto algorithm = std::make_shared<ConstantVelocityAlgorithm>();
    TrackFactory factory(algorithm);
    const auto first = repository.add(factory.createTentative(repository.nextId(), observation(0ms)));
    const auto second = repository.add(factory.createTentative(repository.nextId(), observation(0ms, 2000_m)));
    const auto third = repository.add(factory.createTentative(repository.nextId(), observation(0ms, 3000_m)));
    auto thirdTrack = repository.findTentative(third)->confirm(observation(100ms, 3001_m));
    ASSERT_TRUE(thirdTrack);
    ASSERT_TRUE(repository.replace(std::move(thirdTrack).value()));
    auto firstTrack = repository.findTentative(first)->confirm(observation(100ms, 1001_m));
    ASSERT_TRUE(firstTrack);
    ASSERT_TRUE(repository.replace(std::move(firstTrack).value()));

    auto current = repository.targets().begin();
    EXPECT_EQ(std::get<ConfirmedTrack>(*current++).id(), first);
    EXPECT_EQ(std::get<TentativeTrack>(*current++).id(), second);
    EXPECT_EQ(std::get<ConfirmedTrack>(*current).id(), third);
    EXPECT_EQ(repository.findTentative(first), nullptr);
    ASSERT_NE(repository.findTrack(first), nullptr);
    EXPECT_EQ(repository.findTrack(first)->initialTime(), 0ms);
    EXPECT_EQ(repository.findTrack(first)->lastObservationTime(), 100ms);
    EXPECT_TRUE(repository.remove(second));
    EXPECT_EQ(repository.findTentative(second), nullptr);
    EXPECT_EQ(repository.findTrack(third)->id(), third);
    const auto fourth = repository.add(factory.createTentative(repository.nextId(), observation(200ms, 4000_m)));
    EXPECT_GT(fourth, third);
}

TEST(TargetLifecycleTest, StaleAndFailedInitializationPreserveTentative)
{
    class NoInitializationAlgorithm final : public ITrackingAlgorithm
    {
    public:
        std::optional<TrackState> initialize(const Observation&, const Observation&) const override
        {
            return std::nullopt;
        }

        std::optional<TrackState> update(const ConfirmedTrack&, const Observation&) const override
        {
            return std::nullopt;
        }

        std::optional<TargetPrediction> predict(const ConfirmedTrack&, Time) const override
        {
            return std::nullopt;
        }
    };

    TrackRepository repository;
    const auto id = repository.add(
        TrackFactory(std::make_shared<NoInitializationAlgorithm>()).createTentative(repository.nextId(), observation(100ms))
    );
    const auto* tentative = repository.findTentative(id);
    for (auto time : {0ms, 100ms, 200ms})
    {
        EXPECT_FALSE(tentative->confirm(observation(time)));
        EXPECT_EQ(repository.findTentative(id), tentative);
        EXPECT_EQ(repository.findTrack(id), nullptr);
        EXPECT_EQ(tentative->lastResultTime(), 100ms);
        EXPECT_EQ(tentative->observation().slantRange(), 1000_m);
    }
}

TEST(TargetLifecycleTest, ConfirmationSelectorRanksPlotsAndRejectsStaleObservations)
{
    TrackRepository repository;
    const auto targetConfig = config();
    TrackSelector selector(targetConfig.association, std::make_unique<NdsAlgorithm>(targetConfig.measurement));
    const auto id = repository.add(
        TrackFactory(std::make_shared<ConstantVelocityAlgorithm>()).createTentative(repository.nextId(), observation(0ms))
    );
    const std::vector<Observation> observations({
        observation(100ms, 1030_m),
        observation(100ms, 1005_m),
        observation(100ms, 1005_m)
    });
    const auto selected = selector.selectConfirmation(*repository.findTentative(id), 100ms, observations);
    ASSERT_TRUE(selected);
    EXPECT_EQ(*selected, 1u);
    EXPECT_FALSE(selector.selectConfirmation(*repository.findTentative(id), 0ms, observations));
    EXPECT_FALSE(selector.selectConfirmation(*repository.findTentative(id), 101ms, observations));
    auto confirmedTrack = repository.findTentative(id)->confirm(observations[*selected]);
    ASSERT_TRUE(confirmedTrack);
    ASSERT_TRUE(repository.replace(std::move(confirmedTrack).value()));
    EXPECT_EQ(repository.findTentative(id), nullptr);
}

TEST(TargetLifecycleControllerTest, CopiedControllersKeepTheHandlerAndRepositoryAlive)
{
    TargetController controller(config());
    std::uint32_t id = 0;
    {
        TargetController source(config());
        TargetController copy = source;
        EXPECT_FALSE(source.handleDetection(event(BeamType::SEARCH, 99, 500ms, false)));
        EXPECT_FALSE(copy.handleDetection(event(BeamType::SEARCH, 99, 400ms)));
        const auto confirmation = source.handleDetection(event(BeamType::SEARCH, 7, 0ms));
        ASSERT_TRUE(confirmation);
        id = confirmation->requestId;
        EXPECT_TRUE(copy.nextTrackEvents().empty());

        const auto tracking = copy.handleDetection(
            event(
                BeamType::CONFIRMATION,
                id,
                confirmation->transmitTime
            )
        );
        ASSERT_TRUE(tracking);
        EXPECT_EQ(tracking->requestId, id);
        EXPECT_EQ(tracking->transmitTime, 1100ms);
        ASSERT_EQ(source.nextTrackEvents().size(), 1u);
        EXPECT_EQ(source.nextTrackEvents().front().timestamp, 100ms);
        controller = copy;
    }

    ASSERT_EQ(controller.nextTrackEvents().size(), 1u);
    ASSERT_TRUE(controller.handleDetection(event(BeamType::TRACKING, id, 1100ms)));
    const auto updatedEvents = controller.nextTrackEvents();
    ASSERT_EQ(updatedEvents.size(), 1u);
    EXPECT_EQ(updatedEvents.front().id, id);
    EXPECT_EQ(updatedEvents.front().timestamp, 1100ms);
    EXPECT_FALSE(controller.handleDetection(event(BeamType::SEARCH, 99, 400ms, true, 2000_m)));
    const auto next = controller.handleDetection(event(BeamType::SEARCH, 8, 1200ms, true, 2000_m));
    ASSERT_TRUE(next);
    EXPECT_EQ(next->requestId, id + 1);
}

TEST(TargetLifecycleControllerTest, IgnoresWrongPhaseAndStaleResultsWithoutConsumingIdentity)
{
    TargetController controller(config());
    const auto confirmation = controller.handleDetection(event(BeamType::SEARCH, 10, 0ms));
    ASSERT_TRUE(confirmation);
    const auto id = confirmation->requestId;
    EXPECT_FALSE(controller.handleDetection(event(BeamType::TRACKING, id, 100ms)));
    EXPECT_FALSE(controller.handleDetection(event(BeamType::CONFIRMATION, id, 0ms)));
    EXPECT_FALSE(controller.handleDetection(event(BeamType::CONFIRMATION, 999, 100ms)));
    EXPECT_TRUE(controller.nextTrackEvents().empty());

    const auto tracking = controller.handleDetection(event(BeamType::CONFIRMATION, id, 100ms));
    ASSERT_TRUE(tracking);
    EXPECT_EQ(tracking->requestId, id);
    EXPECT_FALSE(controller.handleDetection(event(BeamType::CONFIRMATION, id, 200ms, false)));
    EXPECT_FALSE(controller.handleDetection(event(BeamType::TRACKING, id, 100ms, false)));
    const auto retry = controller.handleDetection(event(BeamType::TRACKING, id, 1100ms, false));
    ASSERT_TRUE(retry);
    EXPECT_EQ(retry->requestId, id);
    const auto reports = controller.nextTrackEvents();
    ASSERT_EQ(reports.size(), 1u);
    EXPECT_EQ(reports.front().id, id);
    EXPECT_EQ(reports.front().timestamp, 100ms);
}

TEST(TargetLifecycleControllerTest, SearchOverflowDoesNotRegisterOrMarkResultProcessed)
{
    TargetController controller(config());
    const Time maximum(std::numeric_limits<Time::rep>::max());
    EXPECT_THROW(controller.handleDetection(event(BeamType::SEARCH, 1, maximum)), std::overflow_error);
    const auto confirmation = controller.handleDetection(event(BeamType::SEARCH, 1, 0ms));
    ASSERT_TRUE(confirmation);
    EXPECT_EQ(confirmation->requestId, 1u);
}

TEST(TargetLifecycleControllerTest, ConfirmationOverflowPreservesTentativeForEarlierValidResult)
{
    TargetController controller(config());
    const auto confirmation = controller.handleDetection(event(BeamType::SEARCH, 1, 0ms));
    ASSERT_TRUE(confirmation);
    const Time maximum(std::numeric_limits<Time::rep>::max());
    EXPECT_THROW(
        controller.handleDetection(event(BeamType::CONFIRMATION, confirmation->requestId, maximum)),
        std::overflow_error
    );
    EXPECT_TRUE(controller.nextTrackEvents().empty());
    EXPECT_TRUE(controller.handleDetection(event(BeamType::CONFIRMATION, confirmation->requestId, 100ms)));
}

TEST(TargetLifecycleControllerTest, InvalidTrackingBatchPreservesObservationAndMissHistory)
{
    TargetController controller(config());
    const auto confirmation = controller.handleDetection(event(BeamType::SEARCH, 1, 0ms));
    ASSERT_TRUE(confirmation);
    const auto id = confirmation->requestId;
    ASSERT_TRUE(controller.handleDetection(event(BeamType::CONFIRMATION, id, 100ms)));
    ASSERT_TRUE(controller.handleDetection(event(BeamType::TRACKING, id, 1100ms, false)));
    auto invalid = event(BeamType::TRACKING, id, 2100ms);
    invalid.plots.front().power_db = std::numeric_limits<float>::quiet_NaN();
    EXPECT_THROW(controller.handleDetection(invalid), std::invalid_argument);
    ASSERT_EQ(controller.nextTrackEvents().size(), 1u);
    EXPECT_EQ(controller.nextTrackEvents().front().timestamp, 100ms);
    ASSERT_TRUE(controller.handleDetection(event(BeamType::TRACKING, id, 2100ms, false)));
    EXPECT_FALSE(controller.handleDetection(event(BeamType::TRACKING, id, 3100ms, false)));
    EXPECT_TRUE(controller.nextTrackEvents().empty());
}

} // namespace
} // namespace target
