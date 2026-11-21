#include "track_state_assertions.hpp"
#include "domain/association/nds_algorithm.hpp"
#include "association_selection_helpers.hpp"
#include "domain/tracking/constant_velocity_algorithm.hpp"
#include "domain/tentative_track.hpp"
#include "domain/track_factory.hpp"
#include "repository/track_repository.hpp"

#include <target/config/target_config.hpp>
#include <target/controller/target_controller.hpp>

#include <gtest/gtest.h>

#include <chrono>
#include <cstddef>
#include <limits>
#include <memory>
#include <stdexcept>
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

static_assert(std::is_copy_assignable_v<TargetConfig>);
static_assert(
    std::is_same_v<decltype(std::declval<const TargetConfig&>().tracking.initialAccelerationStandardDeviation()), const math::LinearAcceleration&>
);
static_assert(
    std::is_same_v<decltype(std::declval<const TargetConfig&>().tracking.processAccelerationStandardDeviation()), const math::LinearAcceleration&>
);
static_assert(!std::is_default_constructible_v<TargetController>);
static_assert(!std::is_convertible_v<TargetConfig, TargetController>);
static_assert(!std::is_abstract_v<TargetController>);
static_assert(!std::is_polymorphic_v<TargetController>);
static_assert(std::is_constructible_v<TargetController, const TargetConfig&>);
static_assert(std::is_copy_constructible_v<TargetController>);
static_assert(std::is_copy_assignable_v<TargetController>);
static_assert(std::is_copy_assignable_v<TrackPrediction>);
static_assert(std::is_copy_assignable_v<TrackState>);
static_assert(
    std::is_same_v<decltype(std::declval<const ConfirmedTrack&>().state()), const TrackState&>
);

math::SphericalPosition position(math::Length range = 1000_m)
{
    return math::SphericalPosition(range, 0_deg, 0_deg);
}

Observation observation(
    Time time = 0ms,
    math::Length range = 1000_m,
    math::LinearVelocity doppler = 0_mps
) {
    return Observation(position(range), doppler, -10.0f, time);
}

TrackState state(Time time = 100ms, float power = -10.0f)
{
    return TrackState(
        position(),
        0_mps,
        power,
        time,
        math::CartesianPosition(0_m, 1000_m, 0_m),
        math::CartesianVelocity(0_mps, 0_mps, 0_mps)
    );
}

DetectionEvent result(BeamType type, std::uint32_t id, Time time, bool detected = true)
{
    std::vector<DetectionEvent::Plot> plots;
    if (detected)
    {
        plots.emplace_back(math::Distance(1000_m), math::CircularAngle(0_deg), math::TiltAngle(0_deg), 0_mps, -10.0f);
    }
    return DetectionEvent(type, id, time, std::move(plots));
}

class FaultAlgorithm final : public ITrackingAlgorithm
{
public:
    enum class Fault
    {
        NONE,
        INVALID_STATE,
        WRONG_TIME,
        NO_RESULT
    };

    std::optional<TrackingResult> initialize(const Observation& first, const Observation& second) const override
    {
        ++_initializationCount;
        return alter(_algorithm.initialize(first, second));
    }

    std::optional<TrackingResult> update(const ConfirmedTrack& current, const Observation& incoming) const override
    {
        return alter(_algorithm.update(current, incoming));
    }

    std::optional<TrackPrediction> predict(const ConfirmedTrack& current, Time time) const override
    {
        if (fault == Fault::NO_RESULT)
        {
            return std::nullopt;
        }
        if (fault == Fault::INVALID_STATE)
        {
            return TrackPrediction(time, position(0_m));
        }
        if (fault == Fault::WRONG_TIME)
        {
            return TrackPrediction(time + 1ms, position());
        }
        return _algorithm.predict(current, time);
    }

    Fault fault = Fault::NONE;

    std::size_t initializationCount() const
    {
        return _initializationCount;
    }

private:
    std::optional<TrackingResult> alter(std::optional<TrackingResult> calculated) const
    {
        if (fault == Fault::NO_RESULT)
        {
            return std::nullopt;
        }
        if (fault == Fault::INVALID_STATE)
        {
            return TrackingResult{state(100ms, std::numeric_limits<float>::quiet_NaN()), {}};
        }
        if (fault == Fault::WRONG_TIME && calculated)
        {
            const auto previous = calculated->state;
            calculated->state = TrackState(
                previous.sphericalPosition(),
                previous.dopplerVelocity(),
                previous.power_db(),
                previous.timestamp() + 1ms,
                previous.position(),
                previous.velocity()
            );
        }
        return calculated;
    }

    mutable std::size_t _initializationCount = 0;
    ConstantVelocityAlgorithm _algorithm;
};

TEST(DomainValidationTest, MathOriginIsAllowedButObservationOriginIsRejectedAtCreation)
{
    EXPECT_EQ(position(0_m).radius, 0_m);
    EXPECT_THROW(observation(0ms, 0_m), std::invalid_argument);
    EXPECT_THROW(observation(-1ms), std::invalid_argument);
    EXPECT_THROW(TentativeTrack(1, observation(), TrackLifecycleConfig(100ms, 1000ms, 3), nullptr), std::invalid_argument);
    EXPECT_THROW(
        TentativeTrack(0, observation(), TrackLifecycleConfig(100ms, 1000ms, 3), std::make_shared<ConstantVelocityAlgorithm>()),
        std::invalid_argument
    );
    EXPECT_NO_THROW(observation());
}

TEST(DomainValidationTest, ObservationRejectsNonfinitePowerButAcceptsNegativeDecibels)
{
    for (auto invalid : {std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::infinity(), -std::numeric_limits<float>::infinity()})
    {
        EXPECT_THROW(Observation(position(), 0_mps, invalid, 0ms), std::invalid_argument);
    }
    EXPECT_NO_THROW(Observation(position(), 0_mps, -120.0f, 0ms));
}

TEST(DomainValidationTest, PredictionAndTrackingStateValidateTheirOwnConstruction)
{
    EXPECT_THROW(TrackPrediction(-1ms, position()), std::invalid_argument);
    EXPECT_THROW(TrackPrediction(0ms, position(0_m)), std::invalid_argument);
    EXPECT_THROW(state(-1ms), std::invalid_argument);
    EXPECT_THROW(state(100ms, std::numeric_limits<float>::quiet_NaN()), std::invalid_argument);
    EXPECT_THROW(
        (TrackState(
            position(0_m),
            0_mps,
            0.0f,
            0ms,
            math::CartesianPosition(0_m, 0_m, 0_m),
            math::CartesianVelocity(0_mps, 0_mps, 0_mps)
        )),
        std::invalid_argument
    );
    const auto maximum = math::LinearVelocity::fromMetersPerSecond(std::numeric_limits<double>::max());
    EXPECT_THROW(
        (TrackState(
            position(),
            0_mps,
            0.0f,
            0ms,
            math::CartesianPosition(0_m, 1000_m, 0_m),
            math::CartesianVelocity(maximum, maximum, 0_mps)
        )),
        std::invalid_argument
    );
    EXPECT_NO_THROW(state());
    EXPECT_NO_THROW(TrackPrediction(0ms, position()));
}

TEST(ConfirmedTrackTest, RejectsInvalidIdentityAndTrackingAlgorithm)
{
    const auto algorithm = std::make_shared<ConstantVelocityAlgorithm>();
    EXPECT_THROW(ConfirmedTrack(0, 0ms, state(), TrackLifecycleConfig(100ms, 1000ms, 3), algorithm), std::invalid_argument);
    EXPECT_THROW(ConfirmedTrack(1, 0ms, state(), TrackLifecycleConfig(100ms, 1000ms, 3), nullptr), std::invalid_argument);
}

TEST(ConfirmedTrackTest, RequiresStateTimeAfterNonnegativeInitialTime)
{
    const auto algorithm = std::make_shared<ConstantVelocityAlgorithm>();
    EXPECT_THROW(ConfirmedTrack(1, -1ms, state(), TrackLifecycleConfig(100ms, 1000ms, 3), algorithm), std::invalid_argument);
    EXPECT_THROW(ConfirmedTrack(1, 100ms, state(), TrackLifecycleConfig(100ms, 1000ms, 3), algorithm), std::invalid_argument);
    EXPECT_THROW(ConfirmedTrack(1, 101ms, state(), TrackLifecycleConfig(100ms, 1000ms, 3), algorithm), std::invalid_argument);
}

TEST(ConfirmedTrackTest, OwnsValidatedStateWithoutFactoryAccess)
{
    const auto algorithm = std::make_shared<ConstantVelocityAlgorithm>();
    const auto track = ConfirmedTrack(7, 0ms, state(), TrackLifecycleConfig(100ms, 1000ms, 3), algorithm);
    EXPECT_EQ(track.id(), 7u);
    EXPECT_EQ(track.state().timestamp(), 100ms);
    EXPECT_FALSE(track.canProcessResult(100ms));
    EXPECT_TRUE(track.canProcessResult(101ms));
    EXPECT_FALSE(track.willExpireOnNextMiss());
    EXPECT_FALSE(track.hasReachedMissLimit());
    EXPECT_EQ(track.state().sphericalPosition().radius, 1000_m);
}

TEST(TentativeTrackTest, DelegatesInitializationOncePerAttemptWithoutChangingTheFirstObservation)
{
    const auto algorithm = std::make_shared<FaultAlgorithm>();
    auto factory = TrackFactory(algorithm, TrackLifecycleConfig(100ms, 1000ms, 3));
    const auto tentative = factory.createTentative(observation(100ms));
    EXPECT_FALSE(tentative.initialize(observation(0ms)));
    EXPECT_FALSE(tentative.initialize(observation(100ms)));
    EXPECT_EQ(algorithm->initializationCount(), 0u);

    const auto initialized = tentative.initialize(observation(200ms, 1010_m));
    ASSERT_TRUE(initialized);
    EXPECT_EQ(algorithm->initializationCount(), 1u);
    EXPECT_EQ(initialized->state.timestamp(), 200ms);
    EXPECT_EQ(initialized->state.sphericalPosition().radius, 1010_m);

    const auto track = tentative.confirm(observation(200ms, 1010_m));
    ASSERT_TRUE(track);
    EXPECT_EQ(algorithm->initializationCount(), 2u);
    EXPECT_EQ(track->id(), tentative.id());
    EXPECT_EQ(track->state().timestamp(), 200ms);

    const auto confirmed = tentative.confirm(observation(300ms, 1020_m));
    ASSERT_TRUE(confirmed);
    EXPECT_EQ(algorithm->initializationCount(), 3u);
    EXPECT_EQ(confirmed->state().timestamp(), 300ms);
    EXPECT_EQ(tentative.id(), 1u);
    EXPECT_FALSE(tentative.canProcessResult(100ms));
    EXPECT_TRUE(tentative.canProcessResult(101ms));
    EXPECT_EQ(tentative.observation().slantRange(), 1000_m);
}

TEST(TentativeTrackTest, InitializationFailuresPreserveTheSourceAndAllowRetry)
{
    const auto algorithm = std::make_shared<FaultAlgorithm>();
    const auto tentative = TrackFactory(algorithm, TrackLifecycleConfig(100ms, 1000ms, 3)).createTentative(observation(0ms));
    algorithm->fault = FaultAlgorithm::Fault::NO_RESULT;
    EXPECT_FALSE(tentative.initialize(observation(100ms)));
    EXPECT_EQ(algorithm->initializationCount(), 1u);

    std::size_t expectedCalls = 1;
    for (auto fault : {FaultAlgorithm::Fault::WRONG_TIME, FaultAlgorithm::Fault::INVALID_STATE})
    {
        algorithm->fault = fault;
        EXPECT_THROW(tentative.initialize(observation(100ms)), std::invalid_argument);
        EXPECT_EQ(algorithm->initializationCount(), ++expectedCalls);
        EXPECT_EQ(tentative.id(), 1u);
        EXPECT_FALSE(tentative.canProcessResult(0ms));
        EXPECT_TRUE(tentative.canProcessResult(1ms));
        EXPECT_EQ(tentative.observation().slantRange(), 1000_m);
    }
    algorithm->fault = FaultAlgorithm::Fault::NONE;
    EXPECT_TRUE(tentative.initialize(observation(100ms)));
    EXPECT_EQ(algorithm->initializationCount(), expectedCalls + 1);
    EXPECT_FALSE(tentative.canProcessResult(0ms));
    EXPECT_TRUE(tentative.canProcessResult(1ms));
}

TEST(DomainValidationTest, RejectedInitializationPreservesTentativeAndCanBeRetried)
{
    const auto algorithm = std::make_shared<FaultAlgorithm>();
    TrackRepository repository;
    const auto id = repository.add(
        TrackFactory(algorithm, TrackLifecycleConfig(100ms, 1000ms, 3)).createTentative(observation(0ms, 1000_m, 0_mps))
    );
    const auto& target = *repository.findTentativeTrack(id);
    for (auto fault : {FaultAlgorithm::Fault::INVALID_STATE, FaultAlgorithm::Fault::WRONG_TIME})
    {
        algorithm->fault = fault;
        EXPECT_THROW(target.confirm(observation(100ms)), std::invalid_argument);
        EXPECT_EQ(repository.findTentativeTrack(id), &target);
        EXPECT_EQ(repository.findConfirmedTrack(id), nullptr);
        EXPECT_FALSE(target.canProcessResult(0ms));
        EXPECT_TRUE(target.canProcessResult(1ms));
        EXPECT_EQ(target.observation().observationTime(), 0ms);
    }
    algorithm->fault = FaultAlgorithm::Fault::NONE;
    auto confirmedTrack = repository.findTentativeTrack(id)->confirm(observation(100ms));
    EXPECT_TRUE(confirmedTrack);
    EXPECT_TRUE(repository.replace(std::move(confirmedTrack).value()));
}

TEST(TrackFactoryTest, RejectedCreationPreservesTentativeForRetry)
{
    const auto algorithm = std::make_shared<FaultAlgorithm>();
    auto factory = TrackFactory(algorithm, TrackLifecycleConfig(100ms, 1000ms, 3));
    const auto tentative = factory.createTentative(observation(0ms));

    for (auto fault : {FaultAlgorithm::Fault::INVALID_STATE, FaultAlgorithm::Fault::WRONG_TIME})
    {
        algorithm->fault = fault;
        EXPECT_FALSE(tentative.confirm(observation(0ms)));
        EXPECT_THROW(tentative.confirm(observation(100ms)), std::invalid_argument);
        EXPECT_EQ(tentative.id(), 1u);
        EXPECT_FALSE(tentative.canProcessResult(0ms));
        EXPECT_TRUE(tentative.canProcessResult(1ms));
        EXPECT_EQ(tentative.observation().slantRange(), 1000_m);
    }

    algorithm->fault = FaultAlgorithm::Fault::NO_RESULT;
    EXPECT_FALSE(tentative.confirm(observation(100ms)));
    algorithm->fault = FaultAlgorithm::Fault::NONE;
    const auto track = tentative.confirm(observation(100ms));
    ASSERT_TRUE(track);
    EXPECT_EQ(track->id(), tentative.id());
    EXPECT_EQ(track->state().timestamp(), 100ms);
    EXPECT_FALSE(tentative.canProcessResult(0ms));
    EXPECT_TRUE(tentative.canProcessResult(1ms));
}

TEST(DomainValidationTest, RejectedUpdatePreservesStateAndMissHistory)
{
    const auto algorithm = std::make_shared<FaultAlgorithm>();
    TrackRepository repository;
    const auto id = repository.add(
        TrackFactory(algorithm, TrackLifecycleConfig(100ms, 1000ms, 3)).createTentative(observation(0ms, 1000_m, 0_mps))
    );
    auto confirmedTrack = repository.findTentativeTrack(id)->confirm(observation(100ms));
    ASSERT_TRUE(confirmedTrack);
    ASSERT_TRUE(repository.replace(std::move(confirmedTrack).value()));
    auto& target = *repository.findConfirmedTrack(id);
    ASSERT_TRUE(target.recordTrackingMiss(1100ms));
    ASSERT_TRUE(target.recordTrackingMiss(1200ms));
    const auto saved = target.state();
    for (auto fault : {FaultAlgorithm::Fault::INVALID_STATE, FaultAlgorithm::Fault::WRONG_TIME})
    {
        algorithm->fault = fault;
        EXPECT_THROW(target.update(observation(2100ms)), std::invalid_argument);
        test_support::expectTrackStateEquals(target.state(), saved);
        EXPECT_EQ(target.state().timestamp(), 100ms);
        EXPECT_FALSE(target.canProcessResult(1200ms));
        EXPECT_TRUE(target.canProcessResult(1201ms));
        EXPECT_TRUE(target.willExpireOnNextMiss());
        EXPECT_FALSE(target.hasReachedMissLimit());
        EXPECT_EQ(target.state().sphericalPosition().radius, 1000_m);
    }
    algorithm->fault = FaultAlgorithm::Fault::NONE;
    ASSERT_TRUE(target.update(observation(2100ms)));
    EXPECT_FALSE(target.willExpireOnNextMiss());
    EXPECT_FALSE(target.hasReachedMissLimit());
    EXPECT_EQ(target.state().timestamp(), 2100ms);
}

TEST(DomainValidationTest, PredictionContractErrorsThrowAndComputationFailuresRemainOptional)
{
    const auto algorithm = std::make_shared<FaultAlgorithm>();
    TrackRepository repository;
    const auto id = repository.add(
        TrackFactory(algorithm, TrackLifecycleConfig(100ms, 1000ms, 3)).createTentative(observation(0ms, 1000_m, 0_mps))
    );
    auto confirmedTrack = repository.findTentativeTrack(id)->confirm(observation(100ms));
    ASSERT_TRUE(confirmedTrack);
    ASSERT_TRUE(repository.replace(std::move(confirmedTrack).value()));
    auto& target = *repository.findConfirmedTrack(id);
    for (auto fault : {FaultAlgorithm::Fault::INVALID_STATE, FaultAlgorithm::Fault::WRONG_TIME})
    {
        algorithm->fault = fault;
        EXPECT_THROW(target.predict(1100ms), std::invalid_argument);
        EXPECT_FALSE(target.canProcessResult(100ms));
        EXPECT_TRUE(target.canProcessResult(101ms));
        EXPECT_EQ(target.state().timestamp(), 100ms);
    }
    algorithm->fault = FaultAlgorithm::Fault::NO_RESULT;
    EXPECT_FALSE(target.predict(1100ms));
    EXPECT_FALSE(target.update(observation(1100ms)));
    EXPECT_FALSE(target.canProcessResult(100ms));
    EXPECT_TRUE(target.canProcessResult(101ms));
}

TEST(TargetConfigTest, RejectsInvalidSettingsDuringConstruction)
{
    for (auto option = 0; option < 10; ++option)
    {
        const auto defaults = TargetConfig(
            AssociationConfig(1000_mps, 30_mps, 11.345),
            MeasurementConfig(7_m),
            TrackingConfig(10_mps2, 0_mps2),
            TrackLifecycleConfig(100ms, 1000ms, 3),
            TrackReportingConfig(100)
        );
        auto maxAssociationSpeed = defaults.association.maxAssociationSpeed().asLinearVelocity();
        auto maxInitialDopplerDifference = defaults.association.maxInitialDopplerDifference().asLinearVelocity();
        auto measurementStandardDeviation = defaults.measurement.standardDeviation();
        auto positionNdsThreshold = defaults.association.positionNdsThreshold();
        auto confirmationDelay = defaults.trackLifecycle.confirmationDelay();
        auto trackingInterval = defaults.trackLifecycle.trackingInterval();
        auto maxConsecutiveTrackingMisses = defaults.trackLifecycle.maxConsecutiveTrackingMisses();
        switch (option)
        {
        case 0:
            maxAssociationSpeed = -1_mps;
            break;
        case 1:
            maxInitialDopplerDifference = -1_mps;
            break;
        case 2:
            measurementStandardDeviation = 0_m;
            break;
        case 3:
            positionNdsThreshold = -1.0;
            break;
        case 4:
            positionNdsThreshold = std::numeric_limits<double>::quiet_NaN();
            break;
        case 5:
            confirmationDelay = 0ms;
            break;
        case 6:
            trackingInterval = -1ms;
            break;
        case 7:
            maxConsecutiveTrackingMisses = 0;
            break;
        case 8:
            measurementStandardDeviation = 1e308_m;
            break;
        case 9:
            measurementStandardDeviation = 1e-300_m;
            break;
        }
        SCOPED_TRACE(option);
        EXPECT_THROW(
            (TargetConfig(
                AssociationConfig(maxAssociationSpeed, maxInitialDopplerDifference, positionNdsThreshold),
                MeasurementConfig(measurementStandardDeviation),
                TrackingConfig(10_mps2, 0_mps2),
                TrackLifecycleConfig(confirmationDelay, trackingInterval, maxConsecutiveTrackingMisses),
                TrackReportingConfig(100)
            )),
            std::invalid_argument
        );
    }
}

TEST(TargetConfigTest, SelectorsUseInjectedSpeedDopplerNoiseAndGate)
{
    TrackRepository repository;
    const auto id = repository.add(
        TrackFactory(std::make_shared<ConstantVelocityAlgorithm>(), TrackLifecycleConfig(100ms, 1000ms, 3)).createTentative(observation())
    );
    const auto& tentative = *repository.findTentativeTrack(id);
    const auto association = AssociationConfig(1000_mps, 30_mps, 11.345);
    const auto speedRestricted = AssociationConfig(100_mps, 30_mps, 11.345);
    const auto measurement = MeasurementConfig(7_m);
    const auto defaults = TrackSelector(association, std::make_unique<NdsAlgorithm>(measurement));
    const auto speedSelector = TrackSelector(speedRestricted, std::make_unique<NdsAlgorithm>(measurement));

    EXPECT_FALSE(test_support::isDuplicate(observation(100ms, 1100_m), tentative, speedRestricted));
    EXPECT_TRUE(test_support::isDuplicate(observation(100ms, 1100_m), tentative, association));
    EXPECT_FALSE(speedSelector.selectConfirmationObservation(tentative, 100ms, {observation(100ms, 1100_m)}));
    EXPECT_TRUE(defaults.selectConfirmationObservation(tentative, 100ms, {observation(100ms, 1100_m)}));

    const auto dopplerRestricted = TrackSelector(AssociationConfig(1000_mps, 2_mps, 11.345), std::make_unique<NdsAlgorithm>(measurement));
    EXPECT_FALSE(dopplerRestricted.selectConfirmationObservation(tentative, 100ms, {observation(100ms, 1000_m, 3_mps)}));
    EXPECT_TRUE(defaults.selectConfirmationObservation(tentative, 100ms, {observation(100ms, 1000_m, 3_mps)}));

    EXPECT_FALSE(test_support::isDuplicate(observation(0ms, 1010_m), tentative, association, MeasurementConfig(1_m)));
    EXPECT_TRUE(test_support::isDuplicate(observation(0ms, 1010_m), tentative, association));
    const auto exactGate = AssociationConfig(1000_mps, 30_mps, 0.0);
    EXPECT_TRUE(test_support::isDuplicate(observation(), tentative, exactGate));
    EXPECT_FALSE(test_support::isDuplicate(observation(0ms, 1001_m), tentative, exactGate));
}

TEST(TargetConfigTest, ControllerOwnsTemporaryConfigAndUsesInjectedIntervalsAndMissLimit)
{
    auto controller = TargetController(
        TargetConfig(
            AssociationConfig(1000_mps, 30_mps, 11.345),
            MeasurementConfig(7_m),
            TrackingConfig(10_mps2, 0_mps2),
            TrackLifecycleConfig(250ms, 400ms, 2),
            TrackReportingConfig(100)
        )
    );
    controller.setEnuGeoTransform(pose::SetEnuGeoTransformCommand(pose::EnuGeoTransform(pose::GeoPosition(math::TiltAngle(0_deg), math::CircularAngle(0_deg), 100_m))));

    const auto confirmation = controller.handleDetection(result(BeamType::SEARCH, 1, 1000ms));
    ASSERT_TRUE(confirmation);
    EXPECT_EQ(confirmation->transmitTime, 1250ms);
    const auto tracking = controller.handleDetection(
        result(
            BeamType::CONFIRMATION,
            confirmation->requestId,
            confirmation->transmitTime
        )
    );
    ASSERT_TRUE(tracking);
    EXPECT_EQ(tracking->transmitTime, 1650ms);
    auto resultTime = tracking->transmitTime;
    for (std::uint32_t miss = 1; miss <= 2; ++miss)
    {
        const auto retry = controller.handleDetection(
            result(
                BeamType::TRACKING,
                confirmation->requestId,
                resultTime,
                false
            )
        );
        if (miss < 2)
        {
            ASSERT_TRUE(retry);
            EXPECT_EQ(retry->transmitTime, resultTime + 400ms);
            EXPECT_EQ(controller.getTrackEvents().size(), 1u);
        }
        else
        {
            EXPECT_FALSE(retry);
        }
        resultTime += 400ms;
    }
    EXPECT_TRUE(controller.getTrackEvents().empty());
}

TEST(TargetConfigTest, ControllerPassesInjectedSpeedLimitToConfirmationMatcher)
{
    for (bool restricted : {false, true})
    {
        SCOPED_TRACE(restricted);
        const auto config = TargetConfig(
            AssociationConfig(restricted ? 100_mps : 1000_mps, 30_mps, 11.345),
            MeasurementConfig(7_m),
            TrackingConfig(10_mps2, 0_mps2),
            TrackLifecycleConfig(100ms, 1000ms, 3),
            TrackReportingConfig(100)
        );
        auto controller = TargetController(config);
        controller.setEnuGeoTransform(pose::SetEnuGeoTransformCommand(pose::EnuGeoTransform(pose::GeoPosition(math::TiltAngle(0_deg), math::CircularAngle(0_deg), 100_m))));
        const auto confirmation = controller.handleDetection(result(BeamType::SEARCH, 1, 1000ms));
        ASSERT_TRUE(confirmation);

        // 100 ms에 100 m를 이동하므로 기본 속력 한도인 1000 m/s와 정확히 같다.
        const auto tracking = controller.handleDetection(
            DetectionEvent(
                BeamType::CONFIRMATION,
                confirmation->requestId,
                1100ms,
                {DetectionEvent::Plot(math::Distance(1100_m), math::CircularAngle(0_deg), math::TiltAngle(0_deg), 0_mps, -10.0f)}
            )
        );
        EXPECT_EQ(tracking.has_value(), !restricted);
        EXPECT_EQ(controller.getTrackEvents().size(), restricted ? 0u : 1u);
    }
}

TEST(DomainValidationTest, InvalidPlotBatchDoesNotPartiallyRegisterCandidates)
{
    auto controller = TargetController(
        TargetConfig(
            AssociationConfig(1000_mps, 30_mps, 11.345),
            MeasurementConfig(7_m),
            TrackingConfig(10_mps2, 0_mps2),
            TrackLifecycleConfig(100ms, 1000ms, 3),
            TrackReportingConfig(100)
        )
    );
    EXPECT_THROW(
        controller.handleDetection(
            DetectionEvent(
                BeamType::SEARCH,
                1,
                0ms,
                {DetectionEvent::Plot(math::Distance(1000_m), math::CircularAngle(0_deg), math::TiltAngle(0_deg), 0_mps, -10.0f),
                 DetectionEvent::Plot(math::Distance(0_m), math::CircularAngle(0_deg), math::TiltAngle(0_deg), 0_mps, -10.0f)}
            )
        ),
        std::invalid_argument
    );
    const auto request = controller.handleDetection(result(BeamType::SEARCH, 1, 0ms));
    ASSERT_TRUE(request);
    EXPECT_EQ(request->requestId, 1u);
}

} // namespace
} // namespace target
