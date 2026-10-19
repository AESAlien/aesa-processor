#include "calculation/association/detection_matcher.hpp"
#include "calculation/tracking/constant_velocity_algorithm.hpp"
#include "domain/target.hpp"
#include "repository/target_repository.hpp"

#include "config/target_config.hpp"
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
using BeamType = beam::BeamCommand::BeamType;

static_assert(!std::is_copy_assignable_v<TargetPrediction>);
static_assert(!std::is_copy_assignable_v<TargetTrackingState>);
static_assert(std::is_const_v<decltype(TargetPrediction::timestamp)>);
static_assert(std::is_const_v<decltype(TargetTrackingState::sphericalPosition)>);
static_assert(std::is_const_v<decltype(TargetTrackingState::power_db)>);
static_assert(std::is_same_v<decltype(std::declval<const Target&>().sphericalPosition()),
    const math::SphericalPosition&>);
static_assert(std::is_same_v<decltype(std::declval<const Target&>().trackingState()),
    const std::optional<TargetTrackingState>&>);

math::SphericalPosition position(math::Length range = 1000_m)
{
    return math::SphericalPosition(range, math::SphericalDirection(0_deg, 0_deg));
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
        plots.emplace_back(1000_m, 0_deg, 0_deg, 0_mps, -10.0f);
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
            calculated.emplace(previous.sphericalPosition, previous.dopplerVelocity,
                previous.power_db, previous.timestamp + 1ms,
                previous.position, previous.velocity);
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
    const auto* saved = &target.trackingState().value();
    for (auto fault : {FaultAlgorithm::Fault::INVALID_STATE, FaultAlgorithm::Fault::WRONG_TIME})
    {
        algorithm->fault = fault;
        EXPECT_THROW(target.update(observation(2100ms)), std::invalid_argument);
        EXPECT_EQ(&target.trackingState().value(), saved);
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

TEST(TargetConfigTest, RejectsInvalidSettingsBeforeConstructingConsumers)
{
    for (int option = 0; option < 10; ++option)
    {
        TargetConfig config;
        switch (option)
        {
        case 0: config.maxAssociationSpeed = -1_mps; break;
        case 1: config.maxInitialDopplerDifference = -1_mps; break;
        case 2: config.measurementStandardDeviation = 0_m; break;
        case 3: config.positionNdsThreshold = -1.0; break;
        case 4: config.positionNdsThreshold = std::numeric_limits<double>::quiet_NaN(); break;
        case 5: config.confirmationDelay = 0ms; break;
        case 6: config.trackingInterval = -1ms; break;
        case 7: config.maxConsecutiveTrackingMisses = 0; break;
        case 8: config.measurementStandardDeviation = math::Length::fromMeters(1e308); break;
        case 9: config.measurementStandardDeviation = math::Length::fromMeters(1e-300); break;
        }
        SCOPED_TRACE(option);
        EXPECT_THROW(config.validate(), std::invalid_argument);
        EXPECT_THROW(static_cast<void>(DetectionMatcher(config)), std::invalid_argument);
    }
}

TEST(TargetConfigTest, MatcherUsesInjectedSpeedDopplerNoiseAndGate)
{
    TargetRepository repository;
    const auto id = repository.add(observation());
    const auto& tentative = *repository.find(id);
    TargetConfig config;
    config.maxAssociationSpeed = 100_mps;
    EXPECT_FALSE(DetectionMatcher(config).matchesPosition(observation(100ms, 1100_m), tentative));
    EXPECT_TRUE(DetectionMatcher().matchesPosition(observation(100ms, 1100_m), tentative));
    EXPECT_FALSE(DetectionMatcher(config).matchesConfirmation(observation(100ms, 1100_m), tentative));
    EXPECT_TRUE(DetectionMatcher().matchesConfirmation(observation(100ms, 1100_m), tentative));

    config = TargetConfig{};
    config.maxInitialDopplerDifference = 2_mps;
    EXPECT_FALSE(DetectionMatcher(config).matchesConfirmation(observation(100ms, 1000_m, 3_mps), tentative));
    EXPECT_TRUE(DetectionMatcher().matchesConfirmation(observation(100ms, 1000_m, 3_mps), tentative));

    config = TargetConfig{};
    config.measurementStandardDeviation = 1_m;
    EXPECT_FALSE(DetectionMatcher(config).matchesPosition(observation(0ms, 1010_m), tentative));
    EXPECT_TRUE(DetectionMatcher().matchesPosition(observation(0ms, 1010_m), tentative));
    config = TargetConfig{};
    config.positionNdsThreshold = 0.0;
    EXPECT_TRUE(DetectionMatcher(config).matchesPosition(observation(), tentative));
    EXPECT_FALSE(DetectionMatcher(config).matchesPosition(observation(0ms, 1001_m), tentative));
}

TEST(TargetConfigTest, ControllerUsesInternalIntervalsAndMissLimit)
{
    const TargetConfig config;
    TargetController controller;

    const auto confirmation = controller.handleDetection(result(BeamType::SEARCH, 1, 1000ms));
    ASSERT_TRUE(confirmation);
    EXPECT_EQ(confirmation->transmitTime, 1000ms + config.confirmationDelay);
    const auto tracking = controller.handleDetection(result(BeamType::CONFIRMATION,
        confirmation->requestId, confirmation->transmitTime));
    ASSERT_TRUE(tracking);
    EXPECT_EQ(tracking->transmitTime, confirmation->transmitTime + config.trackingInterval);
    auto resultTime = tracking->transmitTime;
    for (std::uint32_t miss = 1; miss <= config.maxConsecutiveTrackingMisses; ++miss)
    {
        const auto retry = controller.handleDetection(result(BeamType::TRACKING,
            confirmation->requestId, resultTime, false));
        if (miss < config.maxConsecutiveTrackingMisses)
        {
            ASSERT_TRUE(retry);
            EXPECT_EQ(retry->transmitTime, resultTime + config.trackingInterval);
            EXPECT_EQ(controller.nextTrackEvents().size(), 1u);
        }
        else
        {
            EXPECT_FALSE(retry);
        }
        resultTime += config.trackingInterval;
    }
    EXPECT_TRUE(controller.nextTrackEvents().empty());
}

TEST(DomainValidationTest, InvalidPlotBatchDoesNotPartiallyRegisterCandidates)
{
    TargetController controller;
    EXPECT_THROW(controller.handleDetection(DetectionEvent(BeamType::SEARCH, 1, 0ms,
        {{1000_m, 0_deg, 0_deg, 0_mps, -10.0f},
            {0_m, 0_deg, 0_deg, 0_mps, -10.0f}})), std::invalid_argument);
    const auto request = controller.handleDetection(result(BeamType::SEARCH, 1, 0ms));
    ASSERT_TRUE(request);
    EXPECT_EQ(request->requestId, 1u);
}

} // namespace
} // namespace target
