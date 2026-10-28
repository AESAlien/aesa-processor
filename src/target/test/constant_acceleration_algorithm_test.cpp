#include "calculation/association/detection_matcher.hpp"
#include "calculation/tracking/constant_acceleration_algorithm.hpp"
#include "domain/target.hpp"
#include "repository/target_repository.hpp"

#include <math/geometry/cartesian_position.hpp>
#include <math/geometry/spherical_position.hpp>
#include <math/tolerance.hpp>

#include <gtest/gtest.h>

#include <chrono>
#include <limits>
#include <memory>
#include <stdexcept>
#include <utility>

namespace target
{
namespace
{

using namespace math::literals;
using namespace std::chrono_literals;
using Time = std::chrono::milliseconds;

math::CartesianPosition position(double x, double y, double z)
{
    return math::CartesianPosition(math::Length::fromMeters(x),
        math::Length::fromMeters(y), math::Length::fromMeters(z));
}

Target observation(
    const math::CartesianPosition& point,
    Time time,
    std::shared_ptr<const ITrackingAlgorithm> algorithm,
    math::LinearVelocity doppler = 0_mps,
    float power = -10.0f)
{
    return Target(point.toSpherical(), doppler, power, time, std::move(algorithm));
}

TargetConfig processNoiseConfig(math::LinearAcceleration sigma)
{
    return TargetConfig(1000_mps, 30_mps, 7_m, 11.345,
        100ms, 1000ms, 3, 10_mps2, 100, sigma);
}

class SeedStateAlgorithm final : public ITrackingAlgorithm
{
public:
    explicit SeedStateAlgorithm(TargetTrackingState state)
        : _state(std::move(state))
    {
    }

    std::optional<TargetTrackingState> initialize(const Target&, const Target&) const override
    {
        return _state;
    }

    std::optional<TargetTrackingState> update(const Target&, const Target&) const override
    {
        return std::nullopt;
    }

    std::optional<TargetPrediction> predict(const Target&, Time) const override
    {
        return std::nullopt;
    }

private:
    const TargetTrackingState _state;
};

Target seededTarget(
    math::CartesianPosition point,
    math::CartesianVelocity velocity,
    math::CartesianAcceleration acceleration,
    std::optional<math::SquareMatrix> covariance,
    Time time = 100ms)
{
    const auto algorithm = std::make_shared<SeedStateAlgorithm>(TargetTrackingState(
        point.toSpherical(), 0_mps, -10.0f, time, point, velocity,
        std::move(acceleration), std::move(covariance)));
    TargetRepository repository;
    const auto id = repository.add(observation(point, 0ms, algorithm));
    auto& target = *repository.find(id);
    EXPECT_TRUE(target.confirm(observation(point, time, algorithm)));
    return target;
}

class ConstantAccelerationAlgorithmTest : public ::testing::Test
{
protected:
    Target makeTracking(
        math::CartesianPosition first = position(0.0, 1000.0, 0.0),
        math::CartesianPosition second = position(0.0, 1000.0, 0.0),
        Time firstTime = 0ms,
        Time secondTime = 100ms,
        std::shared_ptr<const ITrackingAlgorithm> algorithm = nullptr)
    {
        if (!algorithm)
        {
            algorithm = _algorithm;
        }
        TargetRepository repository;
        const auto id = repository.add(observation(first, firstTime, algorithm));
        auto& target = *repository.find(id);
        EXPECT_TRUE(target.confirm(observation(second, secondTime, algorithm)));
        return target;
    }

    const TargetConfig _config{
        1000_mps, 30_mps, 7_m, 11.345, 100ms, 1000ms, 3, 10_mps2, 100, 0_mps2};
    const std::shared_ptr<const ConstantAccelerationAlgorithm> _algorithm =
        std::make_shared<ConstantAccelerationAlgorithm>(_config);
};

TEST_F(ConstantAccelerationAlgorithmTest, InitializesExactTwoPointCovarianceAndZeroAcceleration)
{
    const auto target = makeTracking();
    const auto& state = *target.trackingState();
    ASSERT_TRUE(state.errorCovariance());
    const auto& covariance = *state.errorCovariance();
    EXPECT_TRUE(covariance.isSymmetric(math::NONE_TOLERANCE));
    for (std::size_t axis = 0; axis < 3; ++axis)
    {
        const double accelerationComponents[] = {
            state.acceleration().x.mps2(),
            state.acceleration().y.mps2(),
            state.acceleration().z.mps2()
        };
        EXPECT_DOUBLE_EQ(accelerationComponents[axis], 0.0);
        EXPECT_NEAR(covariance(axis, axis), 49.0, 1e-10);
        EXPECT_NEAR(covariance(axis, axis + 3), 490.0, 1e-10);
        EXPECT_NEAR(covariance(axis + 3, axis + 3), 9800.25, 1e-9);
        EXPECT_NEAR(covariance(axis + 3, axis + 6), 5.0, 1e-10);
        EXPECT_NEAR(covariance(axis + 6, axis + 6), 100.0, 1e-10);
        EXPECT_DOUBLE_EQ(covariance(axis, axis + 6), 0.0);
        for (std::size_t other = 0; other < 9; ++other)
        {
            if (other % 3 != axis)
            {
                EXPECT_DOUBLE_EQ(covariance(axis, other), 0.0);
                EXPECT_DOUBLE_EQ(covariance(axis + 3, other), 0.0);
                EXPECT_DOUBLE_EQ(covariance(axis + 6, other), 0.0);
            }
        }
    }
    EXPECT_NO_THROW(math::cholesky(covariance));
}

TEST_F(ConstantAccelerationAlgorithmTest, InitialVelocityUsesActualThreeDimensionalTimeDifference)
{
    const auto target = makeTracking(position(1000.0, 2000.0, 3000.0),
        position(1003.0, 1994.0, 3009.0), 100ms, 400ms);
    const auto& state = *target.trackingState();
    EXPECT_NEAR(state.velocity().x.mps(), 10.0, 1e-8);
    EXPECT_NEAR(state.velocity().y.mps(), -20.0, 1e-8);
    EXPECT_NEAR(state.velocity().z.mps(), 30.0, 1e-8);
    EXPECT_EQ(state.timestamp(), 400ms);
    ASSERT_TRUE(state.errorCovariance());
    EXPECT_NEAR((*state.errorCovariance())(0, 3), 49.0 / 0.3, 1e-9);
    EXPECT_NEAR((*state.errorCovariance())(3, 3), 98.0 / 0.09 + 2.25, 1e-8);
    EXPECT_NEAR((*state.errorCovariance())(3, 6), 15.0, 1e-10);
}

TEST_F(ConstantAccelerationAlgorithmTest, PredictsCaPositionAndPositionCovarianceWithoutAdvancingState)
{
    const auto target = makeTracking(position(1000.0, 1000.0, 1000.0),
        position(1001.0, 1002.0, 1003.0));
    const auto saved = target.trackingState();
    const auto first = target.predict(1100ms);
    const auto repeated = target.predict(1100ms);
    ASSERT_TRUE(first);
    ASSERT_TRUE(repeated);
    const auto point = first->sphericalPosition().toCartesian();
    EXPECT_NEAR(point.x.m(), 1011.0, 1e-8);
    EXPECT_NEAR(point.y.m(), 1022.0, 1e-8);
    EXPECT_NEAR(point.z.m(), 1033.0, 1e-8);
    ASSERT_TRUE(first->positionCovariance());
    EXPECT_NEAR((*first->positionCovariance())(0, 0), 10859.25, 1e-7);
    EXPECT_TRUE(first->positionCovariance()->equals(*repeated->positionCovariance(), math::NONE_TOLERANCE));
    EXPECT_EQ(target.trackingState(), saved);
    EXPECT_EQ(target.lastObservationTime(), 100ms);
    EXPECT_EQ(target.lastResultTime(), 100ms);
}

TEST_F(ConstantAccelerationAlgorithmTest, UsesStoredAccelerationAndUnequalPredictionInterval)
{
    const auto target = seededTarget(position(1000.0, 2000.0, 3000.0),
        math::CartesianVelocity(10_mps, -20_mps, 30_mps),
        math::CartesianAcceleration(1_mps2, -2_mps2, 3_mps2),
        math::SquareMatrix::identity(9));
    const auto predicted = _algorithm->predict(target, 2600ms);
    ASSERT_TRUE(predicted);
    const auto point = predicted->sphericalPosition().toCartesian();
    EXPECT_NEAR(point.x.m(), 1028.125, 1e-8);
    EXPECT_NEAR(point.y.m(), 1943.75, 1e-8);
    EXPECT_NEAR(point.z.m(), 3084.375, 1e-8);
    const auto corrected = _algorithm->update(target,
        observation(point, 2600ms, _algorithm));
    ASSERT_TRUE(corrected);
    EXPECT_NEAR(corrected->velocity().x.mps(), 12.5, 1e-8);
    EXPECT_NEAR(corrected->velocity().y.mps(), -25.0, 1e-8);
    EXPECT_NEAR(corrected->velocity().z.mps(), 37.5, 1e-8);
    EXPECT_NEAR(corrected->acceleration().x.mps2(), 1.0, 1e-8);
    EXPECT_NEAR(corrected->acceleration().y.mps2(), -2.0, 1e-8);
    EXPECT_NEAR(corrected->acceleration().z.mps2(), 3.0, 1e-8);
}

TEST_F(ConstantAccelerationAlgorithmTest, MeasurementUpdateMatchesIndependentScalarClosedForm)
{
    const auto target = makeTracking(position(0.0, 1000.0, 0.0),
        position(0.0, 1000.0, 0.0), 0ms, 1000ms);
    const auto updated = _algorithm->update(target,
        observation(position(0.0, 1010.0, 0.0), 2000ms, _algorithm, -12_mps, 8.0f));
    ASSERT_TRUE(updated);
    // At dt_init = dt_predict = 1: Pminus column_y = [345, 297, 100]^T.
    constexpr double INNOVATION_VARIANCE = 394.0;
    EXPECT_NEAR(updated->position().y.m(), 1000.0 + 3450.0 / INNOVATION_VARIANCE, 1e-9);
    EXPECT_NEAR(updated->velocity().y.mps(), 2970.0 / INNOVATION_VARIANCE, 1e-9);
    EXPECT_NEAR(updated->acceleration().y.mps2(), 1000.0 / INNOVATION_VARIANCE, 1e-9);
    EXPECT_NEAR(updated->position().x.m(), 0.0, 1e-9);
    EXPECT_NEAR(updated->position().z.m(), 0.0, 1e-9);
    EXPECT_EQ(updated->dopplerVelocity(), -12_mps);
    EXPECT_FLOAT_EQ(updated->power_db(), 8.0f);
    EXPECT_EQ(updated->timestamp(), 2000ms);
    ASSERT_TRUE(updated->errorCovariance());
    const auto& covariance = *updated->errorCovariance();
    EXPECT_NEAR(covariance(1, 1), 345.0 * 49.0 / INNOVATION_VARIANCE, 1e-9);
    EXPECT_NEAR(covariance(1, 4), 297.0 * 49.0 / INNOVATION_VARIANCE, 1e-9);
    EXPECT_NEAR(covariance(1, 7), 100.0 * 49.0 / INNOVATION_VARIANCE, 1e-9);
    EXPECT_NEAR(covariance(4, 4), 323.0 - 297.0 * 297.0 / INNOVATION_VARIANCE, 1e-9);
    EXPECT_TRUE(covariance.isSymmetric(math::NONE_TOLERANCE));
    EXPECT_NO_THROW(math::cholesky(covariance));
    const auto canonical = updated->position().toSpherical().toCartesian();
    EXPECT_NEAR(canonical.y.m(), updated->sphericalPosition().toCartesian().y.m(), 1e-9);
}

TEST_F(ConstantAccelerationAlgorithmTest, PreservesCrossAxisCovarianceInTheGain)
{
    auto covariance = math::SquareMatrix::identity(9);
    covariance(0, 1) = 0.5;
    covariance(1, 0) = 0.5;
    const auto target = seededTarget(position(1000.0, 2000.0, 3000.0),
        math::CartesianVelocity(0_mps, 0_mps, 0_mps),
        math::CartesianAcceleration(0_mps2, 0_mps2, 0_mps2), covariance);
    const auto updated = _algorithm->update(target,
        observation(position(1000.0, 2010.0, 3000.0), 1100ms, _algorithm));
    ASSERT_TRUE(updated);
    // Predicted position variances are 1 + 1 + 1/4, with xy covariance 1/2.
    const double expectedCrossGain = 49.0 * 0.5 / (51.25 * 51.25 - 0.25);
    EXPECT_NEAR(updated->position().x.m(), 1000.0 + expectedCrossGain * 10.0, 1e-9);
    EXPECT_NEAR(updated->position().z.m(), 3000.0, 1e-9);
    ASSERT_TRUE(updated->errorCovariance());
    EXPECT_GT((*updated->errorCovariance())(0, 1), 0.0);
    EXPECT_NO_THROW(math::cholesky(*updated->errorCovariance()));
}

TEST_F(ConstantAccelerationAlgorithmTest, NonzeroProcessNoisePredictsExactFractionalIntervalOnEveryAxis)
{
    const auto algorithm = std::make_shared<ConstantAccelerationAlgorithm>(processNoiseConfig(10_mps2));
    const auto target = seededTarget(position(1000.0, 2000.0, 3000.0),
        math::CartesianVelocity(10_mps, -20_mps, 30_mps),
        math::CartesianAcceleration(1_mps2, -2_mps2, 3_mps2),
        math::SquareMatrix::identity(9));
    const auto saved = target.trackingState();
    const auto predicted = algorithm->predict(target, 2600ms);
    ASSERT_TRUE(predicted);
    const auto point = predicted->sphericalPosition().toCartesian();
    EXPECT_NEAR(point.x.m(), 1028.125, 1e-8);
    EXPECT_NEAR(point.y.m(), 1943.75, 1e-8);
    EXPECT_NEAR(point.z.m(), 3084.375, 1e-8);
    ASSERT_TRUE(predicted->positionCovariance());
    // dt = 2.5 s: F I F^T gives 17.015625, and Qpp = 100 * dt^4 / 4.
    for (std::size_t row = 0; row < 3; ++row)
    {
        for (std::size_t column = 0; column < 3; ++column)
        {
            EXPECT_DOUBLE_EQ((*predicted->positionCovariance())(row, column),
                row == column ? 993.578125 : 0.0);
        }
    }
    EXPECT_EQ(target.trackingState(), saved);
}

TEST_F(ConstantAccelerationAlgorithmTest, NonzeroProcessNoiseUpdateMatchesIndependentScalarClosedForm)
{
    const auto algorithm = std::make_shared<ConstantAccelerationAlgorithm>(processNoiseConfig(10_mps2));
    const auto target = seededTarget(position(1000.0, 2000.0, 3000.0),
        math::CartesianVelocity(10_mps, -20_mps, 30_mps),
        math::CartesianAcceleration(1_mps2, -2_mps2, 3_mps2),
        math::SquareMatrix::identity(9));
    const auto updated = algorithm->update(target,
        observation(position(1020.125, 1959.75, 3060.375), 1600ms, algorithm));
    ASSERT_TRUE(updated);

    // Independently expanded scalar [p,v,a] covariance at dt = 1.5 s, sigma^2 = 100.
    constexpr double PREDICTED_COVARIANCE[3][3] = {
        {131.078125, 171.9375, 113.625},
        {171.9375, 228.25, 151.5},
        {113.625, 151.5, 101.0}
    };
    constexpr double INNOVATION_VARIANCE = 180.078125;
    constexpr double RESIDUALS[] = {4.0, -8.0, 12.0};
    constexpr double PREDICTED_POSITIONS[] = {1016.125, 1967.75, 3048.375};
    constexpr double PREDICTED_VELOCITIES[] = {11.5, -23.0, 34.5};
    constexpr double PREDICTED_ACCELERATIONS[] = {1.0, -2.0, 3.0};
    const double updatedPositions[] = {
        updated->position().x.m(), updated->position().y.m(), updated->position().z.m()
    };
    const double updatedVelocities[] = {
        updated->velocity().x.mps(), updated->velocity().y.mps(), updated->velocity().z.mps()
    };
    const double updatedAccelerations[] = {
        updated->acceleration().x.mps2(), updated->acceleration().y.mps2(), updated->acceleration().z.mps2()
    };
    for (std::size_t axis = 0; axis < 3; ++axis)
    {
        EXPECT_NEAR(updatedPositions[axis], PREDICTED_POSITIONS[axis]
            + PREDICTED_COVARIANCE[0][0] * RESIDUALS[axis] / INNOVATION_VARIANCE, 1e-9);
        EXPECT_NEAR(updatedVelocities[axis], PREDICTED_VELOCITIES[axis]
            + PREDICTED_COVARIANCE[1][0] * RESIDUALS[axis] / INNOVATION_VARIANCE, 1e-9);
        EXPECT_NEAR(updatedAccelerations[axis], PREDICTED_ACCELERATIONS[axis]
            + PREDICTED_COVARIANCE[2][0] * RESIDUALS[axis] / INNOVATION_VARIANCE, 1e-9);
    }
    ASSERT_TRUE(updated->errorCovariance());
    const auto& covariance = *updated->errorCovariance();
    for (std::size_t row = 0; row < 9; ++row)
    {
        for (std::size_t column = 0; column < 9; ++column)
        {
            const auto componentRow = row / 3;
            const auto componentColumn = column / 3;
            const double expected = row % 3 == column % 3
                ? PREDICTED_COVARIANCE[componentRow][componentColumn]
                    - PREDICTED_COVARIANCE[componentRow][0]
                        * PREDICTED_COVARIANCE[0][componentColumn] / INNOVATION_VARIANCE
                : 0.0;
            EXPECT_NEAR(covariance(row, column), expected, 1e-9);
        }
    }
    EXPECT_TRUE(covariance.isSymmetric(math::NONE_TOLERANCE));
    EXPECT_NO_THROW(math::cholesky(covariance));
}

TEST_F(ConstantAccelerationAlgorithmTest, DoublingProcessSigmaQuadruplesAddedPredictionCovariance)
{
    const auto target = seededTarget(position(1000.0, 2000.0, 3000.0),
        math::CartesianVelocity(0_mps, 0_mps, 0_mps),
        math::CartesianAcceleration(0_mps2, 0_mps2, 0_mps2),
        math::SquareMatrix::identity(9));
    const ConstantAccelerationAlgorithm sigmaFive(processNoiseConfig(5_mps2));
    const ConstantAccelerationAlgorithm sigmaTen(processNoiseConfig(10_mps2));
    const auto baseline = _algorithm->predict(target, 2600ms);
    const auto five = sigmaFive.predict(target, 2600ms);
    const auto ten = sigmaTen.predict(target, 2600ms);
    ASSERT_TRUE(baseline);
    ASSERT_TRUE(five);
    ASSERT_TRUE(ten);
    ASSERT_TRUE(baseline->positionCovariance());
    ASSERT_TRUE(five->positionCovariance());
    ASSERT_TRUE(ten->positionCovariance());
    for (std::size_t axis = 0; axis < 3; ++axis)
    {
        const double addedFive = (*five->positionCovariance())(axis, axis)
            - (*baseline->positionCovariance())(axis, axis);
        const double addedTen = (*ten->positionCovariance())(axis, axis)
            - (*baseline->positionCovariance())(axis, axis);
        EXPECT_DOUBLE_EQ(addedFive, 244.140625);
        EXPECT_DOUBLE_EQ(addedTen, 4.0 * addedFive);
    }
}

TEST_F(ConstantAccelerationAlgorithmTest, ZeroElapsedPredictionAddsNoProcessNoise)
{
    const auto algorithm = std::make_shared<ConstantAccelerationAlgorithm>(processNoiseConfig(10_mps2));
    const auto target = makeTracking(position(0.0, 1000.0, 0.0),
        position(0.0, 1001.0, 0.0), 0ms, 100ms, algorithm);
    const auto saved = target.trackingState();
    const auto predicted = target.predict(100ms);
    const auto repeated = target.predict(100ms);
    ASSERT_TRUE(predicted);
    ASSERT_TRUE(repeated);
    ASSERT_TRUE(predicted->positionCovariance());
    ASSERT_TRUE(repeated->positionCovariance());
    for (std::size_t row = 0; row < 3; ++row)
    {
        for (std::size_t column = 0; column < 3; ++column)
        {
            EXPECT_DOUBLE_EQ((*predicted->positionCovariance())(row, column),
                (*saved->errorCovariance())(row, column));
        }
    }
    EXPECT_TRUE(predicted->positionCovariance()->equals(*repeated->positionCovariance(), math::NONE_TOLERANCE));
    EXPECT_EQ(target.trackingState(), saved);
    EXPECT_EQ(target.lastObservationTime(), 100ms);
    EXPECT_EQ(target.lastResultTime(), 100ms);
}

TEST_F(ConstantAccelerationAlgorithmTest, MissesAndRepeatedPredictionsUseFullElapsedProcessNoise)
{
    const auto algorithm = std::make_shared<ConstantAccelerationAlgorithm>(processNoiseConfig(10_mps2));
    auto target = makeTracking(position(0.0, 1000.0, 0.0),
        position(0.0, 1001.0, 0.0), 0ms, 100ms, algorithm);
    const auto control = target;
    const auto saved = target.trackingState();
    ASSERT_TRUE(target.predict(1100ms));
    ASSERT_TRUE(target.recordTrackingMiss(1100ms));
    ASSERT_TRUE(target.predict(2100ms));
    ASSERT_TRUE(target.recordTrackingMiss(2100ms));
    const auto predicted = target.predict(3100ms);
    const auto repeated = target.predict(3100ms);
    ASSERT_TRUE(predicted);
    ASSERT_TRUE(repeated);
    ASSERT_TRUE(predicted->positionCovariance());
    ASSERT_TRUE(repeated->positionCovariance());
    EXPECT_NEAR(predicted->sphericalPosition().toCartesian().y.m(), 1031.0, 1e-9);
    // F(3s) P F(3s)^T + Q(3s); Qpp is 100 * 3^4 / 4 = 2025.
    const double expectedVariance = 49.0 + 6.0 * 490.0 + 9.0 * 9800.25
        + 27.0 * 5.0 + 81.0 * 25.0 + 2025.0;
    for (std::size_t axis = 0; axis < 3; ++axis)
    {
        EXPECT_NEAR((*predicted->positionCovariance())(axis, axis), expectedVariance, 1e-7);
    }
    EXPECT_TRUE(predicted->positionCovariance()->equals(*repeated->positionCovariance(), math::NONE_TOLERANCE));
    EXPECT_EQ(target.trackingState(), saved);
    EXPECT_EQ(target.lastObservationTime(), 100ms);
    EXPECT_EQ(target.lastResultTime(), 2100ms);
    EXPECT_EQ(target.consecutiveMissCount(), 2u);

    const auto incoming = observation(position(0.0, 1041.0, 0.0), 3100ms, algorithm);
    const auto updated = algorithm->update(target, incoming);
    const auto withoutMisses = algorithm->update(control, incoming);
    ASSERT_TRUE(updated);
    ASSERT_TRUE(withoutMisses);
    ASSERT_TRUE(updated->errorCovariance());
    ASSERT_TRUE(withoutMisses->errorCovariance());
    EXPECT_NEAR(updated->position().y.m(), withoutMisses->position().y.m(), 1e-9);
    EXPECT_NEAR(updated->velocity().y.mps(), withoutMisses->velocity().y.mps(), 1e-9);
    EXPECT_NEAR(updated->acceleration().y.mps2(), withoutMisses->acceleration().y.mps2(), 1e-9);
    EXPECT_TRUE(updated->errorCovariance()->equals(*withoutMisses->errorCovariance(), math::NONE_TOLERANCE));
}

TEST_F(ConstantAccelerationAlgorithmTest, ProcessNoiseReachesRealTrackingAndSearchNdsGates)
{
    const auto algorithm = std::make_shared<ConstantAccelerationAlgorithm>(processNoiseConfig(10_mps2));
    const auto zeroNoise = makeTracking(position(0.0, 1000.0, 0.0),
        position(0.0, 1000.0, 0.0), 0ms, 1000ms);
    const auto withNoise = makeTracking(position(0.0, 1000.0, 0.0),
        position(0.0, 1000.0, 0.0), 0ms, 1000ms, algorithm);
    // Use one matcher with unchanged measurement noise and threshold for both real filters.
    const DetectionMatcher matcher(_config);
    const auto incoming = observation(position(0.0, 1068.0, 0.0), 2000ms, algorithm);
    EXPECT_FALSE(matcher.matchesTracking(incoming, zeroNoise));
    EXPECT_FALSE(matcher.matchesPosition(incoming, zeroNoise));
    const auto trackingScore = matcher.matchesTracking(incoming, withNoise);
    const auto searchScore = matcher.matchesPosition(incoming, withNoise);
    ASSERT_TRUE(trackingScore);
    ASSERT_TRUE(searchScore);
    // S = Ppp + R = (345 + 25) + 49 = 419; the zero-noise S is 394.
    EXPECT_NEAR(*trackingScore, 68.0 * 68.0 / 419.0, 1e-10);
    EXPECT_NEAR(*searchScore, *trackingScore, 1e-10);
    const auto outside = observation(position(0.0, 1069.0, 0.0), 2000ms, algorithm);
    EXPECT_FALSE(matcher.matchesTracking(outside, withNoise));
    EXPECT_FALSE(matcher.matchesPosition(outside, withNoise));
    EXPECT_EQ(withNoise.lastObservationTime(), 1000ms);
    EXPECT_EQ(withNoise.lastResultTime(), 1000ms);
}

TEST_F(ConstantAccelerationAlgorithmTest, LearnsAccelerationWithZeroProcessNoise)
{
    Target target = makeTracking(position(0.0, 1000.0, 0.0),
        position(0.0, 1011.0, 0.0), 0ms, 1000ms);
    const auto initialAccelerationVariance = (*target.trackingState()->errorCovariance())(7, 7);
    for (int second = 2; second <= 30; ++second)
    {
        const double elapsed = static_cast<double>(second);
        const double y = 1000.0 + 10.0 * elapsed + elapsed * elapsed;
        ASSERT_TRUE(target.update(observation(position(0.0, y, 0.0),
            Time(second * 1000), _algorithm)));
    }
    EXPECT_NEAR(target.trackingState()->acceleration().y.mps2(), 2.0, 0.001);
    EXPECT_NEAR(target.trackingState()->velocity().y.mps(), 70.0, 0.01);
    EXPECT_NEAR(target.trackingState()->position().y.m(), 2200.0, 0.1);
    EXPECT_LT((*target.trackingState()->errorCovariance())(7, 7), initialAccelerationVariance);
}

TEST_F(ConstantAccelerationAlgorithmTest, MissesDoNotShortenElapsedTimeFromTheLastObservation)
{
    Target target = makeTracking(position(0.0, 1000.0, 0.0), position(0.0, 1001.0, 0.0));
    const auto saved = target.trackingState();
    ASSERT_TRUE(target.recordTrackingMiss(1100ms));
    ASSERT_TRUE(target.recordTrackingMiss(2100ms));
    const auto predicted = target.predict(3100ms);
    ASSERT_TRUE(predicted);
    EXPECT_NEAR(predicted->sphericalPosition().toCartesian().y.m(), 1031.0, 1e-9);
    ASSERT_TRUE(predicted->positionCovariance());
    // F(3s) P(100ms) F(3s)^T; misses do not replace P with repeated predictions.
    EXPECT_NEAR((*predicted->positionCovariance())(1, 1),
        49.0 + 6.0 * 490.0 + 9.0 * 9800.25 + 27.0 * 5.0 + 81.0 * 25.0, 1e-7);
    EXPECT_EQ(target.trackingState(), saved);
    EXPECT_EQ(target.lastObservationTime(), 100ms);
    EXPECT_EQ(target.lastResultTime(), 2100ms);
    EXPECT_EQ(target.consecutiveMissCount(), 2u);
}

TEST_F(ConstantAccelerationAlgorithmTest, RejectsUnavailableCovarianceAndNonIncreasingTimes)
{
    const auto target = makeTracking();
    const auto incoming = observation(position(0.0, 1000.0, 0.0), 100ms, _algorithm);
    EXPECT_FALSE(_algorithm->initialize(incoming, incoming));
    EXPECT_FALSE(_algorithm->update(target, incoming));
    EXPECT_FALSE(_algorithm->predict(target, 99ms));
    EXPECT_TRUE(_algorithm->predict(target, 100ms));
    const auto withoutCovariance = seededTarget(position(0.0, 1000.0, 0.0),
        math::CartesianVelocity(0_mps, 0_mps, 0_mps),
        math::CartesianAcceleration(0_mps2, 0_mps2, 0_mps2), std::nullopt);
    EXPECT_FALSE(_algorithm->predict(withoutCovariance, 1100ms));
    EXPECT_FALSE(_algorithm->update(withoutCovariance,
        observation(position(0.0, 1000.0, 0.0), 1100ms, _algorithm)));
}

TEST_F(ConstantAccelerationAlgorithmTest, RejectsOriginAndNonfinitePredictionsWithoutChangingState)
{
    const auto originTarget = seededTarget(position(0.0, 1000.0, 0.0),
        math::CartesianVelocity(0_mps, -1000_mps, 0_mps),
        math::CartesianAcceleration(0_mps2, 0_mps2, 0_mps2),
        math::SquareMatrix::identity(9));
    EXPECT_FALSE(_algorithm->predict(originTarget, 1100ms));
    const double maximum = std::numeric_limits<double>::max();
    const auto overflowTarget = seededTarget(position(0.0, 1000.0, 0.0),
        math::CartesianVelocity(0_mps, 0_mps, 0_mps),
        math::CartesianAcceleration(
            math::LinearAcceleration::fromMetersPerSecondSquared(maximum), 0_mps2, 0_mps2),
        math::SquareMatrix::identity(9));
    const auto saved = overflowTarget.trackingState();
    EXPECT_FALSE(_algorithm->predict(overflowTarget, 10100ms));
    EXPECT_EQ(overflowTarget.trackingState(), saved);
}

TEST_F(ConstantAccelerationAlgorithmTest, PropagatesInvalidInnovationCovarianceAsMathError)
{
    auto covariance = math::SquareMatrix::identity(9);
    covariance(0, 1) = 100.0;
    covariance(1, 0) = 100.0;
    const auto target = seededTarget(position(1000.0, 2000.0, 3000.0),
        math::CartesianVelocity(0_mps, 0_mps, 0_mps),
        math::CartesianAcceleration(0_mps2, 0_mps2, 0_mps2), covariance);
    const auto saved = target.trackingState();
    EXPECT_THROW(_algorithm->update(target,
        observation(position(1000.0, 2000.0, 3000.0), 1100ms, _algorithm)), std::domain_error);
    EXPECT_EQ(target.trackingState(), saved);
}

TEST(ConstantAccelerationConfigTest, ValidatesInitialAccelerationVarianceAndSingleMeasurementNoise)
{
    for (double sigma : {0.0, -1.0, std::numeric_limits<double>::quiet_NaN(),
             std::numeric_limits<double>::infinity(), std::numeric_limits<double>::max(), 1e-300})
    {
        EXPECT_THROW(TargetConfig(1000_mps, 30_mps, 7_m, 11.345, 100ms, 1000ms, 3,
            math::LinearAcceleration::fromMetersPerSecondSquared(sigma), 100, 0_mps2),
            std::invalid_argument);
    }
    EXPECT_THROW(TargetConfig(1000_mps, 30_mps, 1.2e-162_m,
        11.345, 100ms, 1000ms, 3, 10_mps2, 100, 0_mps2), std::invalid_argument);
    const TargetConfig compatible(
        1000_mps, 30_mps, 7_m, 11.345, 100ms, 1000ms, 3, 10_mps2, 100, 0_mps2);
    EXPECT_EQ(compatible.initialAccelerationStandardDeviation(), 10_mps2);
}

TEST(ConstantAccelerationConfigTest, AcceptsZeroAndValidatesProcessAccelerationVariance)
{
    const auto zero = processNoiseConfig(0_mps2);
    EXPECT_EQ(zero.processAccelerationStandardDeviation(), 0_mps2);
    const auto nonzero = processNoiseConfig(10_mps2);
    EXPECT_EQ(nonzero.processAccelerationStandardDeviation(), 10_mps2);
    for (double sigma : {-1.0, std::numeric_limits<double>::quiet_NaN(),
             std::numeric_limits<double>::infinity(), -std::numeric_limits<double>::infinity(),
             std::numeric_limits<double>::max(), 1e-300})
    {
        SCOPED_TRACE(sigma);
        EXPECT_THROW(processNoiseConfig(math::LinearAcceleration::fromMetersPerSecondSquared(sigma)),
            std::invalid_argument);
    }
}

TEST(ConstantAccelerationConfigTest, UsesInjectedInitialAccelerationUncertainty)
{
    const TargetConfig config(
        1000_mps, 30_mps, 7_m, 11.345, 100ms, 1000ms, 3, 2_mps2, 100, 0_mps2);
    const auto algorithm = std::make_shared<ConstantAccelerationAlgorithm>(config);
    const auto state = algorithm->initialize(observation(position(0.0, 1000.0, 0.0), 0ms, algorithm),
        observation(position(0.0, 1000.0, 0.0), 100ms, algorithm));
    ASSERT_TRUE(state);
    ASSERT_TRUE(state->errorCovariance());
    EXPECT_DOUBLE_EQ((*state->errorCovariance())(6, 6), 4.0);
    EXPECT_NEAR((*state->errorCovariance())(3, 6), 0.2, 1e-12);
    EXPECT_NEAR((*state->errorCovariance())(3, 3), 9800.01, 1e-8);
}

} // namespace
} // namespace target
