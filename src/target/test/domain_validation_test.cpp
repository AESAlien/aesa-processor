#include "calculation/association/detection_matcher.hpp"
#include "calculation/tracking/constant_velocity_algorithm.hpp"
#include "domain/target.hpp"
#include "repository/target_repository.hpp"

#include <target/config/target_config.hpp>
#include <target/controller/target_controller.hpp>

#include <gtest/gtest.h>

#include <chrono>
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
static_assert(std::is_same_v<decltype(std::declval<const TargetConfig&>().initialAccelerationStandardDeviation()),
    const math::LinearAcceleration&>);
static_assert(std::is_same_v<decltype(std::declval<const TargetConfig&>().processAccelerationStandardDeviation()),
    const math::LinearAcceleration&>);
static_assert(!std::is_default_constructible_v<TargetController>);
static_assert(!std::is_convertible_v<TargetConfig, TargetController>);
static_assert(std::is_copy_assignable_v<TargetPrediction>);
static_assert(std::is_copy_assignable_v<TargetTrackingState>);
static_assert(std::is_same_v<decltype(std::declval<const Target&>().sphericalPosition()),
    const math::SphericalPosition&>);
static_assert(std::is_same_v<decltype(std::declval<const Target&>().trackingState()),
    const std::shared_ptr<const TargetTrackingState>&>);

math::SphericalPosition position(math::Length range = 1000_m)
{
    return math::SphericalPosition{math::Distance(range), math::SphericalDirection(math::CircularAngle(0_deg), math::TiltAngle(0_deg))};
}

Target observation(
    Time time = 0ms,
    math::Length range = 1000_m,
    math::LinearVelocity doppler = 0_mps,
    std::shared_ptr<const ITrackingAlgorithm> algorithm = std::make_shared<ConstantVelocityAlgorithm>())
{
    return Target(position(range), doppler, -10.0f, time, std::move(algorithm));
}

TargetTrackingState state(Time time = 100ms, float power = -10.0f)
{
    return TargetTrackingState(position(), 0_mps, power, time,
        math::CartesianPosition(0_m, 1000_m, 0_m),
        math::CartesianVelocity(0_mps, 0_mps, 0_mps));
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

    std::optional<TargetTrackingState> initialize(const Target& first, const Target& second) const override
    {
        return alter(_algorithm.initialize(first, second));
    }

    std::optional<TargetTrackingState> update(const Target& current, const Target& incoming) const override
    {
        return alter(_algorithm.update(current, incoming));
    }

    std::optional<TargetPrediction> predict(const Target& current, Time time) const override
    {
        if (fault == Fault::NO_RESULT)
        {
            return std::nullopt;
        }
        if (fault == Fault::INVALID_STATE)
        {
            return TargetPrediction(time, position(0_m));
        }
        if (fault == Fault::WRONG_TIME)
        {
            return TargetPrediction(time + 1ms, position());
        }
        return _algorithm.predict(current, time);
    }

    Fault fault = Fault::NONE;

private:
    std::optional<TargetTrackingState> alter(std::optional<TargetTrackingState> calculated) const
    {
        if (fault == Fault::NO_RESULT)
        {
            return std::nullopt;
        }
        if (fault == Fault::INVALID_STATE)
        {
            return state(100ms, std::numeric_limits<float>::quiet_NaN());
        }
        if (fault == Fault::WRONG_TIME && calculated)
        {
            const auto previous = *calculated;
            calculated.emplace(previous.sphericalPosition(), previous.dopplerVelocity(),
                previous.power_db(), previous.timestamp() + 1ms,
                previous.position(), previous.velocity());
        }
        return calculated;
    }

    ConstantVelocityAlgorithm _algorithm;
};

TEST(DomainValidationTest, MathOriginIsAllowedButTargetOriginIsRejectedAtCreation)
{
    EXPECT_EQ(position(0_m).radius, 0_m);
    EXPECT_THROW(observation(0ms, 0_m), std::invalid_argument);
    EXPECT_THROW(observation(-1ms), std::invalid_argument);
    EXPECT_THROW(observation(0ms, 1000_m, 0_mps, nullptr), std::invalid_argument);
    EXPECT_NO_THROW(observation());
}

TEST(DomainValidationTest, TargetRejectsNonfinitePowerButAcceptsNegativeDecibels)
{
    const auto algorithm = std::make_shared<ConstantVelocityAlgorithm>();
    for (float invalid : {std::numeric_limits<float>::quiet_NaN(),
             std::numeric_limits<float>::infinity(), -std::numeric_limits<float>::infinity()})
    {
        EXPECT_THROW(Target(position(), 0_mps, invalid, 0ms, algorithm), std::invalid_argument);
    }
    EXPECT_NO_THROW(Target(position(), 0_mps, -120.0f, 0ms, algorithm));
}

TEST(DomainValidationTest, PredictionAndTrackingStateValidateTheirOwnConstruction)
{
    EXPECT_THROW(TargetPrediction(-1ms, position()), std::invalid_argument);
    EXPECT_THROW(TargetPrediction(0ms, position(0_m)), std::invalid_argument);
    EXPECT_THROW(state(-1ms), std::invalid_argument);
    EXPECT_THROW(state(100ms, std::numeric_limits<float>::quiet_NaN()), std::invalid_argument);
    EXPECT_THROW((TargetTrackingState(position(0_m), 0_mps, 0.0f, 0ms,
        math::CartesianPosition(0_m, 0_m, 0_m),
        math::CartesianVelocity(0_mps, 0_mps, 0_mps))), std::invalid_argument);
    const auto maximum = math::LinearVelocity::fromMetersPerSecond(std::numeric_limits<double>::max());
    EXPECT_THROW((TargetTrackingState(position(), 0_mps, 0.0f, 0ms,
        math::CartesianPosition(0_m, 1000_m, 0_m),
        math::CartesianVelocity(maximum, maximum, 0_mps))), std::invalid_argument);
    EXPECT_NO_THROW(state());
    EXPECT_NO_THROW(TargetPrediction(0ms, position()));
}

TEST(DomainValidationTest, RejectedInitializationPreservesTentativeAndCanBeRetried)
{
    const auto algorithm = std::make_shared<FaultAlgorithm>();
    TargetRepository repository;
    const auto id = repository.add(observation(0ms, 1000_m, 0_mps, algorithm));
    auto& target = *repository.find(id);
    for (auto fault : {FaultAlgorithm::Fault::INVALID_STATE, FaultAlgorithm::Fault::WRONG_TIME})
    {
        algorithm->fault = fault;
        EXPECT_THROW(target.confirm(observation(100ms)), std::invalid_argument);
        EXPECT_EQ(target.state(), TargetState::TENTATIVE);
        EXPECT_FALSE(target.trackingState());
        EXPECT_EQ(target.lastResultTime(), 0ms);
        EXPECT_EQ(target.initialTime(), 0ms);
    }
    algorithm->fault = FaultAlgorithm::Fault::NONE;
    EXPECT_TRUE(target.confirm(observation(100ms)));
}

TEST(DomainValidationTest, RejectedUpdatePreservesStateAndMissHistory)
{
    const auto algorithm = std::make_shared<FaultAlgorithm>();
    TargetRepository repository;
    const auto id = repository.add(observation(0ms, 1000_m, 0_mps, algorithm));
    auto& target = *repository.find(id);
    ASSERT_TRUE(target.confirm(observation(100ms)));
    ASSERT_TRUE(target.recordTrackingMiss(1100ms));
    const auto* saved = target.trackingState().get();
    for (auto fault : {FaultAlgorithm::Fault::INVALID_STATE, FaultAlgorithm::Fault::WRONG_TIME})
    {
        algorithm->fault = fault;
        EXPECT_THROW(target.update(observation(2100ms)), std::invalid_argument);
        EXPECT_EQ(target.trackingState().get(), saved);
        EXPECT_EQ(target.lastObservationTime(), 100ms);
        EXPECT_EQ(target.lastResultTime(), 1100ms);
        EXPECT_EQ(target.consecutiveMissCount(), 1u);
        EXPECT_EQ(target.sphericalPosition().radius, 1000_m);
    }
    algorithm->fault = FaultAlgorithm::Fault::NONE;
    ASSERT_TRUE(target.update(observation(2100ms)));
    EXPECT_EQ(target.consecutiveMissCount(), 0u);
    EXPECT_EQ(target.lastObservationTime(), 2100ms);
}

TEST(DomainValidationTest, PredictionContractErrorsThrowAndComputationFailuresRemainOptional)
{
    const auto algorithm = std::make_shared<FaultAlgorithm>();
    TargetRepository repository;
    const auto id = repository.add(observation(0ms, 1000_m, 0_mps, algorithm));
    auto& target = *repository.find(id);
    ASSERT_TRUE(target.confirm(observation(100ms)));
    for (auto fault : {FaultAlgorithm::Fault::INVALID_STATE, FaultAlgorithm::Fault::WRONG_TIME})
    {
        algorithm->fault = fault;
        EXPECT_THROW(target.predict(1100ms), std::invalid_argument);
        EXPECT_EQ(target.lastResultTime(), 100ms);
        EXPECT_EQ(target.lastObservationTime(), 100ms);
    }
    algorithm->fault = FaultAlgorithm::Fault::NO_RESULT;
    EXPECT_FALSE(target.predict(1100ms));
    EXPECT_FALSE(target.update(observation(1100ms)));
    EXPECT_EQ(target.lastResultTime(), 100ms);
}

TEST(TargetConfigTest, RejectsInvalidSettingsDuringConstruction)
{
    for (int option = 0; option < 10; ++option)
    {
        const TargetConfig defaults(
            1000_mps, 30_mps, 7_m, 11.345, 100ms, 1000ms, 3, 10_mps2, 100, 0_mps2);
        auto maxAssociationSpeed = defaults.maxAssociationSpeed();
        auto maxInitialDopplerDifference = defaults.maxInitialDopplerDifference();
        auto measurementStandardDeviation = defaults.measurementStandardDeviation();
        auto positionNdsThreshold = defaults.positionNdsThreshold();
        auto confirmationDelay = defaults.confirmationDelay();
        auto trackingInterval = defaults.trackingInterval();
        auto maxConsecutiveTrackingMisses = defaults.maxConsecutiveTrackingMisses();
        switch (option)
        {
        case 0: maxAssociationSpeed = -1_mps; break;
        case 1: maxInitialDopplerDifference = -1_mps; break;
        case 2: measurementStandardDeviation = 0_m; break;
        case 3: positionNdsThreshold = -1.0; break;
        case 4: positionNdsThreshold = std::numeric_limits<double>::quiet_NaN(); break;
        case 5: confirmationDelay = 0ms; break;
        case 6: trackingInterval = -1ms; break;
        case 7: maxConsecutiveTrackingMisses = 0; break;
        case 8: measurementStandardDeviation = 1e308_m; break;
        case 9: measurementStandardDeviation = 1e-300_m; break;
        }
        SCOPED_TRACE(option);
        EXPECT_THROW((TargetConfig(maxAssociationSpeed, maxInitialDopplerDifference,
            measurementStandardDeviation, positionNdsThreshold,
            confirmationDelay, trackingInterval, maxConsecutiveTrackingMisses,
            10_mps2, 100, 0_mps2)),
            std::invalid_argument);
    }
}

TEST(TargetConfigTest, MatcherUsesInjectedSpeedDopplerNoiseAndGate)
{
    TargetRepository repository;
    const auto id = repository.add(observation());
    const auto& tentative = *repository.find(id);
    const TargetConfig speedConfig(100_mps, 30_mps, 7_m, 11.345, 100ms, 1000ms, 3, 10_mps2, 100, 0_mps2);
    EXPECT_FALSE(DetectionMatcher(speedConfig).matchesPosition(observation(100ms, 1100_m), tentative));
    EXPECT_TRUE(DetectionMatcher(TargetConfig(
        1000_mps, 30_mps, 7_m, 11.345, 100ms, 1000ms, 3, 10_mps2, 100, 0_mps2))
        .matchesPosition(observation(100ms, 1100_m), tentative));
    EXPECT_FALSE(DetectionMatcher(speedConfig).matchesConfirmation(observation(100ms, 1100_m), tentative));
    EXPECT_TRUE(DetectionMatcher(TargetConfig(
        1000_mps, 30_mps, 7_m, 11.345, 100ms, 1000ms, 3, 10_mps2, 100, 0_mps2))
        .matchesConfirmation(observation(100ms, 1100_m), tentative));

    const TargetConfig dopplerConfig(1000_mps, 2_mps, 7_m, 11.345, 100ms, 1000ms, 3, 10_mps2, 100, 0_mps2);
    EXPECT_FALSE(DetectionMatcher(dopplerConfig).matchesConfirmation(observation(100ms, 1000_m, 3_mps), tentative));
    EXPECT_TRUE(DetectionMatcher(TargetConfig(
        1000_mps, 30_mps, 7_m, 11.345, 100ms, 1000ms, 3, 10_mps2, 100, 0_mps2))
        .matchesConfirmation(observation(100ms, 1000_m, 3_mps), tentative));

    const TargetConfig noiseConfig(1000_mps, 30_mps, 1_m, 11.345, 100ms, 1000ms, 3, 10_mps2, 100, 0_mps2);
    EXPECT_FALSE(DetectionMatcher(noiseConfig).matchesPosition(observation(0ms, 1010_m), tentative));
    EXPECT_TRUE(DetectionMatcher(TargetConfig(
        1000_mps, 30_mps, 7_m, 11.345, 100ms, 1000ms, 3, 10_mps2, 100, 0_mps2))
        .matchesPosition(observation(0ms, 1010_m), tentative));
    const TargetConfig gateConfig(1000_mps, 30_mps, 7_m, 0.0, 100ms, 1000ms, 3, 10_mps2, 100, 0_mps2);
    EXPECT_TRUE(DetectionMatcher(gateConfig).matchesPosition(observation(), tentative));
    EXPECT_FALSE(DetectionMatcher(gateConfig).matchesPosition(observation(0ms, 1001_m), tentative));
}

TEST(TargetConfigTest, ControllerOwnsTemporaryConfigAndUsesInjectedIntervalsAndMissLimit)
{
    TargetController controller{TargetConfig(
        1000_mps, 30_mps, 7_m, 11.345, 250ms, 400ms, 2, 10_mps2, 100, 0_mps2)};

    const auto confirmation = controller.handleDetection(result(BeamType::SEARCH, 1, 1000ms));
    ASSERT_TRUE(confirmation);
    EXPECT_EQ(confirmation->transmitTime, 1250ms);
    const auto tracking = controller.handleDetection(result(BeamType::CONFIRMATION,
        confirmation->requestId, confirmation->transmitTime));
    ASSERT_TRUE(tracking);
    EXPECT_EQ(tracking->transmitTime, 1650ms);
    auto resultTime = tracking->transmitTime;
    for (std::uint32_t miss = 1; miss <= 2; ++miss)
    {
        const auto retry = controller.handleDetection(result(BeamType::TRACKING,
            confirmation->requestId, resultTime, false));
        if (miss < 2)
        {
            ASSERT_TRUE(retry);
            EXPECT_EQ(retry->transmitTime, resultTime + 400ms);
            EXPECT_EQ(controller.nextTrackEvents().size(), 1u);
        }
        else
        {
            EXPECT_FALSE(retry);
        }
        resultTime += 400ms;
    }
    EXPECT_TRUE(controller.nextTrackEvents().empty());
}

TEST(TargetConfigTest, ControllerPassesInjectedSpeedLimitToConfirmationMatcher)
{
    for (bool restricted : {false, true})
    {
        SCOPED_TRACE(restricted);
        const TargetConfig config(restricted ? 100_mps : 1000_mps,
            30_mps, 7_m, 11.345, 100ms, 1000ms, 3, 10_mps2, 100, 0_mps2);
        TargetController controller(config);
        const auto confirmation = controller.handleDetection(result(BeamType::SEARCH, 1, 1000ms));
        ASSERT_TRUE(confirmation);

        // A 100 m displacement in 100 ms passes only the default 1000 m/s limit.
        const auto tracking = controller.handleDetection(DetectionEvent(BeamType::CONFIRMATION,
            confirmation->requestId, 1100ms, {{math::Distance(1100_m), math::CircularAngle(0_deg), math::TiltAngle(0_deg), 0_mps, -10.0f}}));
        EXPECT_EQ(tracking.has_value(), !restricted);
        EXPECT_EQ(controller.nextTrackEvents().size(), restricted ? 0u : 1u);
    }
}

TEST(DomainValidationTest, InvalidPlotBatchDoesNotPartiallyRegisterCandidates)
{
    TargetController controller(TargetConfig(1000_mps, 30_mps, 7_m, 11.345, 100ms, 1000ms, 3, 10_mps2, 100, 0_mps2));
    EXPECT_THROW(controller.handleDetection(DetectionEvent(BeamType::SEARCH, 1, 0ms,
        {{math::Distance(1000_m), math::CircularAngle(0_deg), math::TiltAngle(0_deg), 0_mps, -10.0f},
            {math::Distance(0_m), math::CircularAngle(0_deg), math::TiltAngle(0_deg), 0_mps, -10.0f}})), std::invalid_argument);
    const auto request = controller.handleDetection(result(BeamType::SEARCH, 1, 0ms));
    ASSERT_TRUE(request);
    EXPECT_EQ(request->requestId, 1u);
}

} // namespace
} // namespace target
