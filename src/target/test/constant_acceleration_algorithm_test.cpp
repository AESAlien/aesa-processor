#include "association_selection_helpers.hpp"
#include "domain/tracking/constant_acceleration_algorithm.hpp"
#include "domain/tentative_track.hpp"
#include "domain/track_factory.hpp"
#include "repository/track_repository.hpp"

#include <target/config/target_config.hpp>

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
namespace test_support
{

class KalmanStateAccess : private ITrackingAlgorithm
{
public:
    static std::any makeState(math::SquareMatrix covariance)
    {
        std::any state;
        ConstantAccelerationAlgorithm::storeCovariance(state, std::move(covariance));
        return state;
    }

    static const math::SquareMatrix* covariance(const ConfirmedTrack& track) noexcept
    {
        return covariance(algorithmState(track));
    }

    static const math::SquareMatrix* covariance(const std::any& state) noexcept
    {
        return ConstantAccelerationAlgorithm::stateCovariance(state);
    }
};

} // namespace test_support

namespace
{

using namespace math::literals;
using namespace std::chrono_literals;
using test_support::KalmanStateAccess;
using Time = std::chrono::milliseconds;

math::CartesianPosition position(double x, double y, double z)
{
    return math::CartesianPosition(
        math::Length::fromMeters(x),
        math::Length::fromMeters(y),
        math::Length::fromMeters(z)
    );
}

Observation observation(
    const math::CartesianPosition& point,
    Time time,
    math::LinearVelocity doppler = 0_mps,
    float power = -10.0f
) {
    return Observation(point.toSpherical(), doppler, power, time);
}

TrackingConfig processNoiseConfig(math::LinearAcceleration sigma)
{
    return TrackingConfig(10_mps2, sigma);
}

class SeedStateAlgorithm final : public ITrackingAlgorithm
{
public:
    SeedStateAlgorithm(TrackState state, std::any data) : _state(std::move(state)), _data(std::move(data))
    {
    }

    std::optional<TrackingResult> initialize(const Observation&, const Observation&) const override
    {
        return TrackingResult{_state, _data};
    }

    std::optional<TrackingResult> update(const ConfirmedTrack&, const Observation&) const override
    {
        return std::nullopt;
    }

    std::optional<TrackPrediction> predict(const ConfirmedTrack&, Time) const override
    {
        return std::nullopt;
    }

private:
    const TrackState _state;
    const std::any _data;
};

ConfirmedTrack seededTarget(
    math::CartesianPosition point,
    math::CartesianVelocity velocity,
    math::CartesianAcceleration acceleration,
    std::optional<math::SquareMatrix> covariance,
    Time time = 100ms
) {
    auto state = TrackState(
        point.toSpherical(),
        0_mps,
        -10.0f,
        time,
        point,
        velocity,
        std::move(acceleration)
    );
    const auto algorithm = std::make_shared<SeedStateAlgorithm>(
        std::move(state), covariance ? KalmanStateAccess::makeState(std::move(*covariance)) : std::any{}
    );
    TrackRepository repository;
    const auto id = repository.add(TrackFactory(algorithm).createTentative(repository.nextId(), observation(point, 0ms)));
    auto confirmedTrack = repository.findTentativeTrack(id)->confirm(observation(point, time));
    EXPECT_TRUE(confirmedTrack);
    EXPECT_TRUE(repository.replace(std::move(confirmedTrack).value()));
    auto& target = *repository.findConfirmedTrack(id);
    return target;
}

class ConstantAccelerationAlgorithmTest : public ::testing::Test
{
protected:
    ConfirmedTrack makeTracking(
        math::CartesianPosition first = position(0.0, 1000.0, 0.0),
        math::CartesianPosition second = position(0.0, 1000.0, 0.0),
        Time firstTime = 0ms,
        Time secondTime = 100ms,
        std::shared_ptr<const ITrackingAlgorithm> algorithm = nullptr
    ) {
        if (!algorithm)
        {
            algorithm = _algorithm;
        }
        TrackRepository repository;
        const auto id = repository.add(
            TrackFactory(algorithm).createTentative(
                repository.nextId(),
                observation(first, firstTime)
            )
        );
        auto confirmedTrack = repository.findTentativeTrack(id)->confirm(observation(second, secondTime));
        EXPECT_TRUE(confirmedTrack);
        EXPECT_TRUE(repository.replace(std::move(confirmedTrack).value()));
        auto& target = *repository.findConfirmedTrack(id);
        return target;
    }

    const TargetConfig _config = TargetConfig(
        AssociationConfig(1000_mps, 30_mps, 11.345),
        MeasurementConfig(7_m),
        TrackingConfig(10_mps2, 0_mps2),
        TrackManagementConfig(100ms, 1000ms, 3, 100)
    );
    const std::shared_ptr<const ConstantAccelerationAlgorithm> _algorithm =
        std::make_shared<ConstantAccelerationAlgorithm>(_config.tracking, _config.measurement);
};

TEST_F(ConstantAccelerationAlgorithmTest, InitializesExactTwoPointCovarianceAndZeroAcceleration)
{
    const auto target = makeTracking();
    const auto& state = *target.state();
    ASSERT_TRUE(KalmanStateAccess::covariance(target));
    const auto& covariance = *KalmanStateAccess::covariance(target);
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
    const auto target = makeTracking(
        position(1000.0, 2000.0, 3000.0),
        position(1003.0, 1994.0, 3009.0),
        100ms,
        400ms
    );
    const auto& state = *target.state();
    EXPECT_NEAR(state.velocity().x.mps(), 10.0, 1e-8);
    EXPECT_NEAR(state.velocity().y.mps(), -20.0, 1e-8);
    EXPECT_NEAR(state.velocity().z.mps(), 30.0, 1e-8);
    EXPECT_EQ(state.timestamp(), 400ms);
    ASSERT_TRUE(KalmanStateAccess::covariance(target));
    EXPECT_NEAR((*KalmanStateAccess::covariance(target))(0, 3), 49.0 / 0.3, 1e-9);
    EXPECT_NEAR((*KalmanStateAccess::covariance(target))(3, 3), 98.0 / 0.09 + 2.25, 1e-8);
    EXPECT_NEAR((*KalmanStateAccess::covariance(target))(3, 6), 15.0, 1e-10);
}

TEST_F(ConstantAccelerationAlgorithmTest, PredictsCaPositionAndPositionCovarianceWithoutAdvancingState)
{
    const auto target = makeTracking(
        position(1000.0, 1000.0, 1000.0),
        position(1001.0, 1002.0, 1003.0)
    );
    const auto saved = target.state();
    std::optional<math::SquareMatrix> firstCovariance;
    const auto first = IPositionUncertainty::predict(target, 1100ms, firstCovariance);
    std::optional<math::SquareMatrix> repeatedCovariance;
    const auto repeated = IPositionUncertainty::predict(target, 1100ms, repeatedCovariance);
    ASSERT_TRUE(first);
    ASSERT_TRUE(repeated);
    const auto point = first->sphericalPosition().toCartesian();
    EXPECT_NEAR(point.x.m(), 1011.0, 1e-8);
    EXPECT_NEAR(point.y.m(), 1022.0, 1e-8);
    EXPECT_NEAR(point.z.m(), 1033.0, 1e-8);
    ASSERT_TRUE(firstCovariance);
    EXPECT_NEAR((*firstCovariance)(0, 0), 10859.25, 1e-7);
    EXPECT_TRUE(firstCovariance->equals(
        *repeatedCovariance,
        math::NONE_TOLERANCE
    ));
    EXPECT_EQ(target.state(), saved);
    EXPECT_EQ(target.lastObservationTime(), 100ms);
    EXPECT_EQ(target.lastResultTime(), 100ms);
}

TEST_F(ConstantAccelerationAlgorithmTest, UsesStoredAccelerationAndUnequalPredictionInterval)
{
    const auto target = seededTarget(
        position(1000.0, 2000.0, 3000.0),
        math::CartesianVelocity(10_mps, -20_mps, 30_mps),
        math::CartesianAcceleration(1_mps2, -2_mps2, 3_mps2),
        math::SquareMatrix::identity(9)
    );
    std::optional<math::SquareMatrix> predictedCovariance;
    const auto predicted = _algorithm->predictWithPositionCovariance(target, 2600ms, predictedCovariance);
    ASSERT_TRUE(predicted);
    const auto point = predicted->sphericalPosition().toCartesian();
    EXPECT_NEAR(point.x.m(), 1028.125, 1e-8);
    EXPECT_NEAR(point.y.m(), 1943.75, 1e-8);
    EXPECT_NEAR(point.z.m(), 3084.375, 1e-8);
    const auto correctedResult = _algorithm->update(
        target,
        observation(point, 2600ms)
    );
    ASSERT_TRUE(correctedResult);
    const auto& [corrected, correctedData] = *correctedResult;
    EXPECT_NEAR(corrected.velocity().x.mps(), 12.5, 1e-8);
    EXPECT_NEAR(corrected.velocity().y.mps(), -25.0, 1e-8);
    EXPECT_NEAR(corrected.velocity().z.mps(), 37.5, 1e-8);
    EXPECT_NEAR(corrected.acceleration().x.mps2(), 1.0, 1e-8);
    EXPECT_NEAR(corrected.acceleration().y.mps2(), -2.0, 1e-8);
    EXPECT_NEAR(corrected.acceleration().z.mps2(), 3.0, 1e-8);
}

TEST_F(ConstantAccelerationAlgorithmTest, MeasurementUpdateMatchesIndependentScalarClosedForm)
{
    const auto target = makeTracking(
        position(0.0, 1000.0, 0.0),
        position(0.0, 1000.0, 0.0),
        0ms,
        1000ms
    );
    const auto updatedResult = _algorithm->update(
        target,
        observation(position(0.0, 1010.0, 0.0), 2000ms, -12_mps, 8.0f)
    );
    ASSERT_TRUE(updatedResult);
    const auto& [updated, updatedData] = *updatedResult;
    // 초기화와 예측 간격이 각각 1초일 때 y축 [p, v, a] 공분산의 위치 열은 [345, 297, 100]^T다.
    constexpr auto INNOVATION_VARIANCE = 394.0;
    EXPECT_NEAR(updated.position().y.m(), 1000.0 + 3450.0 / INNOVATION_VARIANCE, 1e-9);
    EXPECT_NEAR(updated.velocity().y.mps(), 2970.0 / INNOVATION_VARIANCE, 1e-9);
    EXPECT_NEAR(updated.acceleration().y.mps2(), 1000.0 / INNOVATION_VARIANCE, 1e-9);
    EXPECT_NEAR(updated.position().x.m(), 0.0, 1e-9);
    EXPECT_NEAR(updated.position().z.m(), 0.0, 1e-9);
    EXPECT_EQ(updated.dopplerVelocity(), -12_mps);
    EXPECT_FLOAT_EQ(updated.power_db(), 8.0f);
    EXPECT_EQ(updated.timestamp(), 2000ms);
    ASSERT_TRUE(KalmanStateAccess::covariance(updatedData));
    const auto& covariance = *KalmanStateAccess::covariance(updatedData);
    EXPECT_NEAR(covariance(1, 1), 345.0 * 49.0 / INNOVATION_VARIANCE, 1e-9);
    EXPECT_NEAR(covariance(1, 4), 297.0 * 49.0 / INNOVATION_VARIANCE, 1e-9);
    EXPECT_NEAR(covariance(1, 7), 100.0 * 49.0 / INNOVATION_VARIANCE, 1e-9);
    EXPECT_NEAR(covariance(4, 4), 323.0 - 297.0 * 297.0 / INNOVATION_VARIANCE, 1e-9);
    EXPECT_TRUE(covariance.isSymmetric(math::NONE_TOLERANCE));
    EXPECT_NO_THROW(math::cholesky(covariance));
    const auto canonical = updated.position().toSpherical().toCartesian();
    EXPECT_NEAR(canonical.y.m(), updated.sphericalPosition().toCartesian().y.m(), 1e-9);
}

TEST_F(ConstantAccelerationAlgorithmTest, PreservesCrossAxisCovarianceInTheGain)
{
    auto covariance = math::SquareMatrix::identity(9);
    covariance(0, 1) = 0.5;
    covariance(1, 0) = 0.5;
    const auto target = seededTarget(
        position(1000.0, 2000.0, 3000.0),
        math::CartesianVelocity(0_mps, 0_mps, 0_mps),
        math::CartesianAcceleration(0_mps2, 0_mps2, 0_mps2),
        covariance
    );
    const auto updatedResult = _algorithm->update(
        target,
        observation(position(1000.0, 2010.0, 3000.0), 1100ms)
    );
    ASSERT_TRUE(updatedResult);
    const auto& [updated, updatedData] = *updatedResult;
    // 예측 위치의 축별 분산은 1 + 1 + 1/4이고, x와 y의 공분산은 1/2이다.
    const auto expectedCrossGain = 49.0 * 0.5 / (51.25 * 51.25 - 0.25);
    EXPECT_NEAR(updated.position().x.m(), 1000.0 + expectedCrossGain * 10.0, 1e-9);
    EXPECT_NEAR(updated.position().z.m(), 3000.0, 1e-9);
    ASSERT_TRUE(KalmanStateAccess::covariance(updatedData));
    EXPECT_GT((*KalmanStateAccess::covariance(updatedData))(0, 1), 0.0);
    EXPECT_NO_THROW(math::cholesky(*KalmanStateAccess::covariance(updatedData)));
}

TEST_F(ConstantAccelerationAlgorithmTest, NonzeroProcessNoisePredictsExactFractionalIntervalOnEveryAxis)
{
    const auto algorithm = std::make_shared<ConstantAccelerationAlgorithm>(processNoiseConfig(10_mps2), MeasurementConfig(7_m));
    const auto target = seededTarget(
        position(1000.0, 2000.0, 3000.0),
        math::CartesianVelocity(10_mps, -20_mps, 30_mps),
        math::CartesianAcceleration(1_mps2, -2_mps2, 3_mps2),
        math::SquareMatrix::identity(9)
    );
    const auto saved = target.state();
    std::optional<math::SquareMatrix> predictedCovariance;
    const auto predicted = algorithm->predictWithPositionCovariance(target, 2600ms, predictedCovariance);
    ASSERT_TRUE(predicted);
    const auto point = predicted->sphericalPosition().toCartesian();
    EXPECT_NEAR(point.x.m(), 1028.125, 1e-8);
    EXPECT_NEAR(point.y.m(), 1943.75, 1e-8);
    EXPECT_NEAR(point.z.m(), 3084.375, 1e-8);
    ASSERT_TRUE(predictedCovariance);
    // dt = 2.5 s일 때 F I F^T의 위치 분산은 17.015625, 잡음 항 Qpp는 100 * dt^4 / 4다.
    for (std::size_t row = 0; row < 3; ++row)
    {
        for (std::size_t column = 0; column < 3; ++column)
        {
            EXPECT_DOUBLE_EQ(
                (*predictedCovariance)(row, column),
                row == column ? 993.578125 : 0.0
            );
        }
    }
    EXPECT_EQ(target.state(), saved);
}

TEST_F(ConstantAccelerationAlgorithmTest, NonzeroProcessNoiseUpdateMatchesIndependentScalarClosedForm)
{
    const auto algorithm = std::make_shared<ConstantAccelerationAlgorithm>(processNoiseConfig(10_mps2), MeasurementConfig(7_m));
    const auto target = seededTarget(
        position(1000.0, 2000.0, 3000.0),
        math::CartesianVelocity(10_mps, -20_mps, 30_mps),
        math::CartesianAcceleration(1_mps2, -2_mps2, 3_mps2),
        math::SquareMatrix::identity(9)
    );
    const auto updatedResult = algorithm->update(
        target,
        observation(position(1020.125, 1959.75, 3060.375), 1600ms)
    );
    ASSERT_TRUE(updatedResult);
    const auto& [updated, updatedData] = *updatedResult;

    // dt = 1.5 s, sigma^2 = 100을 대입해 한 축의 [p, v, a] 공분산을 별도로 계산한 값이다.
    constexpr double PREDICTED_COVARIANCE[3][3] = {
        {131.078125, 171.9375, 113.625},
        {171.9375, 228.25, 151.5},
        {113.625, 151.5, 101.0}
    };
    constexpr auto INNOVATION_VARIANCE = 180.078125;
    constexpr double RESIDUALS[] = {4.0, -8.0, 12.0};
    constexpr double PREDICTED_POSITIONS[] = {1016.125, 1967.75, 3048.375};
    constexpr double PREDICTED_VELOCITIES[] = {11.5, -23.0, 34.5};
    constexpr double PREDICTED_ACCELERATIONS[] = {1.0, -2.0, 3.0};
    const double updatedPositions[] = {
        updated.position().x.m(),
        updated.position().y.m(),
        updated.position().z.m()
    };
    const double updatedVelocities[] = {
        updated.velocity().x.mps(),
        updated.velocity().y.mps(),
        updated.velocity().z.mps()
    };
    const double updatedAccelerations[] = {
        updated.acceleration().x.mps2(),
        updated.acceleration().y.mps2(),
        updated.acceleration().z.mps2()
    };
    for (std::size_t axis = 0; axis < 3; ++axis)
    {
        EXPECT_NEAR(
            updatedPositions[axis],
            PREDICTED_POSITIONS[axis]
                + PREDICTED_COVARIANCE[0][0] * RESIDUALS[axis] / INNOVATION_VARIANCE,
            1e-9
        );
        EXPECT_NEAR(
            updatedVelocities[axis],
            PREDICTED_VELOCITIES[axis]
                + PREDICTED_COVARIANCE[1][0] * RESIDUALS[axis] / INNOVATION_VARIANCE,
            1e-9
        );
        EXPECT_NEAR(
            updatedAccelerations[axis],
            PREDICTED_ACCELERATIONS[axis]
                + PREDICTED_COVARIANCE[2][0] * RESIDUALS[axis] / INNOVATION_VARIANCE,
            1e-9
        );
    }
    ASSERT_TRUE(KalmanStateAccess::covariance(updatedData));
    const auto& covariance = *KalmanStateAccess::covariance(updatedData);
    for (std::size_t row = 0; row < 9; ++row)
    {
        for (std::size_t column = 0; column < 9; ++column)
        {
            const auto componentRow = row / 3;
            const auto componentColumn = column / 3;
            const auto expected = row % 3 == column % 3
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
    const auto target = seededTarget(
        position(1000.0, 2000.0, 3000.0),
        math::CartesianVelocity(0_mps, 0_mps, 0_mps),
        math::CartesianAcceleration(0_mps2, 0_mps2, 0_mps2),
        math::SquareMatrix::identity(9)
    );
    const auto sigmaFive = ConstantAccelerationAlgorithm(processNoiseConfig(5_mps2), MeasurementConfig(7_m));
    const auto sigmaTen = ConstantAccelerationAlgorithm(processNoiseConfig(10_mps2), MeasurementConfig(7_m));
    std::optional<math::SquareMatrix> baselineCovariance;
    const auto baseline = _algorithm->predictWithPositionCovariance(target, 2600ms, baselineCovariance);
    std::optional<math::SquareMatrix> fiveCovariance;
    const auto five = sigmaFive.predictWithPositionCovariance(target, 2600ms, fiveCovariance);
    std::optional<math::SquareMatrix> tenCovariance;
    const auto ten = sigmaTen.predictWithPositionCovariance(target, 2600ms, tenCovariance);
    ASSERT_TRUE(baseline);
    ASSERT_TRUE(five);
    ASSERT_TRUE(ten);
    ASSERT_TRUE(baselineCovariance);
    ASSERT_TRUE(fiveCovariance);
    ASSERT_TRUE(tenCovariance);
    for (std::size_t axis = 0; axis < 3; ++axis)
    {
        const auto addedFive = (*fiveCovariance)(axis, axis)
            - (*baselineCovariance)(axis, axis);
        const auto addedTen = (*tenCovariance)(axis, axis)
            - (*baselineCovariance)(axis, axis);
        EXPECT_DOUBLE_EQ(addedFive, 244.140625);
        EXPECT_DOUBLE_EQ(addedTen, 4.0 * addedFive);
    }
}

TEST_F(ConstantAccelerationAlgorithmTest, ZeroElapsedPredictionAddsNoProcessNoise)
{
    const auto algorithm = std::make_shared<ConstantAccelerationAlgorithm>(processNoiseConfig(10_mps2), MeasurementConfig(7_m));
    const auto target = makeTracking(
        position(0.0, 1000.0, 0.0),
        position(0.0, 1001.0, 0.0),
        0ms,
        100ms,
        algorithm
    );
    const auto saved = target.state();
    std::optional<math::SquareMatrix> predictedCovariance;
    const auto predicted = IPositionUncertainty::predict(target, 100ms, predictedCovariance);
    std::optional<math::SquareMatrix> repeatedCovariance;
    const auto repeated = IPositionUncertainty::predict(target, 100ms, repeatedCovariance);
    ASSERT_TRUE(predicted);
    ASSERT_TRUE(repeated);
    ASSERT_TRUE(predictedCovariance);
    ASSERT_TRUE(repeatedCovariance);
    for (std::size_t row = 0; row < 3; ++row)
    {
        for (std::size_t column = 0; column < 3; ++column)
        {
            EXPECT_DOUBLE_EQ(
                (*predictedCovariance)(row, column),
                (*KalmanStateAccess::covariance(target))(row, column)
            );
        }
    }
    EXPECT_TRUE(predictedCovariance->equals(
        *repeatedCovariance,
        math::NONE_TOLERANCE
    ));
    EXPECT_EQ(target.state(), saved);
    EXPECT_EQ(target.lastObservationTime(), 100ms);
    EXPECT_EQ(target.lastResultTime(), 100ms);
}

TEST_F(ConstantAccelerationAlgorithmTest, MissesAndRepeatedPredictionsUseFullElapsedProcessNoise)
{
    const auto algorithm = std::make_shared<ConstantAccelerationAlgorithm>(processNoiseConfig(10_mps2), MeasurementConfig(7_m));
    auto target = makeTracking(
        position(0.0, 1000.0, 0.0),
        position(0.0, 1001.0, 0.0),
        0ms,
        100ms,
        algorithm
    );
    const auto control = target;
    const auto saved = target.state();
    ASSERT_TRUE(target.predict(1100ms));
    ASSERT_TRUE(target.recordTrackingMiss(1100ms));
    ASSERT_TRUE(target.predict(2100ms));
    ASSERT_TRUE(target.recordTrackingMiss(2100ms));
    std::optional<math::SquareMatrix> predictedCovariance;
    const auto predicted = IPositionUncertainty::predict(target, 3100ms, predictedCovariance);
    std::optional<math::SquareMatrix> repeatedCovariance;
    const auto repeated = IPositionUncertainty::predict(target, 3100ms, repeatedCovariance);
    ASSERT_TRUE(predicted);
    ASSERT_TRUE(repeated);
    ASSERT_TRUE(predictedCovariance);
    ASSERT_TRUE(repeatedCovariance);
    EXPECT_NEAR(predicted->sphericalPosition().toCartesian().y.m(), 1031.0, 1e-9);
    // 3초 뒤 공분산 F(3s) P F(3s)^T + Q(3s)에서 위치 잡음 항은 100 * 3^4 / 4 = 2025다.
    const auto expectedVariance = 49.0 + 6.0 * 490.0 + 9.0 * 9800.25
        + 27.0 * 5.0 + 81.0 * 25.0 + 2025.0;
    for (std::size_t axis = 0; axis < 3; ++axis)
    {
        EXPECT_NEAR((*predictedCovariance)(axis, axis), expectedVariance, 1e-7);
    }
    EXPECT_TRUE(predictedCovariance->equals(
        *repeatedCovariance,
        math::NONE_TOLERANCE
    ));
    EXPECT_EQ(target.state(), saved);
    EXPECT_EQ(target.lastObservationTime(), 100ms);
    EXPECT_EQ(target.lastResultTime(), 2100ms);
    EXPECT_EQ(target.consecutiveMissCount(), 2u);

    const auto incoming = observation(position(0.0, 1041.0, 0.0), 3100ms);
    const auto updatedResult = algorithm->update(target, incoming);
    const auto withoutMissesResult = algorithm->update(control, incoming);
    ASSERT_TRUE(updatedResult);
    const auto& [updated, updatedData] = *updatedResult;
    ASSERT_TRUE(withoutMissesResult);
    const auto& [withoutMisses, withoutMissesData] = *withoutMissesResult;
    ASSERT_TRUE(KalmanStateAccess::covariance(updatedData));
    ASSERT_TRUE(KalmanStateAccess::covariance(withoutMissesData));
    EXPECT_NEAR(updated.position().y.m(), withoutMisses.position().y.m(), 1e-9);
    EXPECT_NEAR(updated.velocity().y.mps(), withoutMisses.velocity().y.mps(), 1e-9);
    EXPECT_NEAR(updated.acceleration().y.mps2(), withoutMisses.acceleration().y.mps2(), 1e-9);
    EXPECT_TRUE(KalmanStateAccess::covariance(updatedData)->equals(
        *KalmanStateAccess::covariance(withoutMissesData),
        math::NONE_TOLERANCE
    ));
}

TEST_F(ConstantAccelerationAlgorithmTest, ProcessNoiseReachesRealTrackingAndSearchNdsGates)
{
    const auto algorithm = std::make_shared<ConstantAccelerationAlgorithm>(processNoiseConfig(10_mps2), MeasurementConfig(7_m));
    const auto zeroNoise = makeTracking(
        position(0.0, 1000.0, 0.0),
        position(0.0, 1000.0, 0.0),
        0ms,
        1000ms
    );
    const auto withNoise = makeTracking(
        position(0.0, 1000.0, 0.0),
        position(0.0, 1000.0, 0.0),
        0ms,
        1000ms,
        algorithm
    );
    const auto incoming = observation(position(0.0, 1068.0, 0.0), 2000ms);
    EXPECT_FALSE(test_support::selectTracking(incoming, zeroNoise, _config.association, _config.measurement));
    EXPECT_FALSE(test_support::isDuplicate(incoming, zeroNoise, _config.association, _config.measurement));
    EXPECT_TRUE(test_support::selectTracking(incoming, withNoise, _config.association, _config.measurement));
    EXPECT_TRUE(test_support::isDuplicate(incoming, withNoise, _config.association, _config.measurement));
    // 가속도 잡음이 위치 분산에 25를 더해 잔차 분산이 394에서 419가 된다.
    const auto expected = 68.0 * 68.0 / 419.0;
    const auto larger = AssociationConfig(1000_mps, 30_mps, expected + 1e-10);
    const auto smaller = AssociationConfig(1000_mps, 30_mps, expected - 1e-10);
    EXPECT_TRUE(test_support::selectTracking(incoming, withNoise, larger, _config.measurement));
    EXPECT_FALSE(test_support::selectTracking(incoming, withNoise, smaller, _config.measurement));
    EXPECT_TRUE(test_support::isDuplicate(incoming, withNoise, larger, _config.measurement));
    EXPECT_FALSE(test_support::isDuplicate(incoming, withNoise, smaller, _config.measurement));
    const auto outside = observation(position(0.0, 1069.0, 0.0), 2000ms);
    EXPECT_FALSE(test_support::selectTracking(outside, withNoise, _config.association, _config.measurement));
    EXPECT_FALSE(test_support::isDuplicate(outside, withNoise, _config.association, _config.measurement));
    EXPECT_EQ(withNoise.lastObservationTime(), 1000ms);
    EXPECT_EQ(withNoise.lastResultTime(), 1000ms);
}

TEST_F(ConstantAccelerationAlgorithmTest, LearnsAccelerationWithZeroProcessNoise)
{
    auto target = makeTracking(
        position(0.0, 1000.0, 0.0),
        position(0.0, 1011.0, 0.0),
        0ms,
        1000ms
    );
    const auto initialAccelerationVariance = (*KalmanStateAccess::covariance(target))(7, 7);
    for (auto second = 2; second <= 30; ++second)
    {
        const auto elapsed = static_cast<double>(second);
        const auto y = 1000.0 + 10.0 * elapsed + elapsed * elapsed;
        ASSERT_TRUE(target.update(observation(position(0.0, y, 0.0), Time(second * 1000))));
    }
    EXPECT_NEAR(target.state()->acceleration().y.mps2(), 2.0, 0.001);
    EXPECT_NEAR(target.state()->velocity().y.mps(), 70.0, 0.01);
    EXPECT_NEAR(target.state()->position().y.m(), 2200.0, 0.1);
    EXPECT_LT((*KalmanStateAccess::covariance(target))(7, 7), initialAccelerationVariance);
}

TEST_F(ConstantAccelerationAlgorithmTest, MissesDoNotShortenElapsedTimeFromTheLastObservation)
{
    auto target = makeTracking(position(0.0, 1000.0, 0.0), position(0.0, 1001.0, 0.0));
    const auto saved = target.state();
    ASSERT_TRUE(target.recordTrackingMiss(1100ms));
    ASSERT_TRUE(target.recordTrackingMiss(2100ms));
    std::optional<math::SquareMatrix> predictedCovariance;
    const auto predicted = IPositionUncertainty::predict(target, 3100ms, predictedCovariance);
    ASSERT_TRUE(predicted);
    EXPECT_NEAR(predicted->sphericalPosition().toCartesian().y.m(), 1031.0, 1e-9);
    ASSERT_TRUE(predictedCovariance);
    // 연속 미탐 뒤에도 100 ms에 저장한 공분산에서 3초를 한 번에 예측한다.
    EXPECT_NEAR(
        (*predictedCovariance)(1, 1),
        49.0 + 6.0 * 490.0 + 9.0 * 9800.25 + 27.0 * 5.0 + 81.0 * 25.0,
        1e-7
    );
    EXPECT_EQ(target.state(), saved);
    EXPECT_EQ(target.lastObservationTime(), 100ms);
    EXPECT_EQ(target.lastResultTime(), 2100ms);
    EXPECT_EQ(target.consecutiveMissCount(), 2u);
}

TEST_F(ConstantAccelerationAlgorithmTest, RejectsUnavailableCovarianceAndNonIncreasingTimes)
{
    const auto target = makeTracking();
    const auto incoming = observation(position(0.0, 1000.0, 0.0), 100ms);
    EXPECT_FALSE(_algorithm->initialize(incoming, incoming));
    EXPECT_FALSE(_algorithm->update(target, incoming));
    EXPECT_FALSE(_algorithm->predict(target, 99ms));
    EXPECT_TRUE(_algorithm->predict(target, 100ms));
    const auto withoutCovariance = seededTarget(
        position(0.0, 1000.0, 0.0),
        math::CartesianVelocity(0_mps, 0_mps, 0_mps),
        math::CartesianAcceleration(0_mps2, 0_mps2, 0_mps2),
        std::nullopt
    );
    EXPECT_FALSE(_algorithm->predict(withoutCovariance, 1100ms));
    EXPECT_FALSE(
        _algorithm->update(
            withoutCovariance,
            observation(position(0.0, 1000.0, 0.0), 1100ms)
        )
    );
}

TEST_F(ConstantAccelerationAlgorithmTest, RejectsOriginAndNonfinitePredictionsWithoutChangingState)
{
    const auto originTarget = seededTarget(
        position(0.0, 1000.0, 0.0),
        math::CartesianVelocity(0_mps, -1000_mps, 0_mps),
        math::CartesianAcceleration(0_mps2, 0_mps2, 0_mps2),
        math::SquareMatrix::identity(9)
    );
    EXPECT_FALSE(_algorithm->predict(originTarget, 1100ms));
    const auto maximum = std::numeric_limits<double>::max();
    const auto overflowTarget = seededTarget(
        position(0.0, 1000.0, 0.0),
        math::CartesianVelocity(0_mps, 0_mps, 0_mps),
        math::CartesianAcceleration(
            math::LinearAcceleration::fromMetersPerSecondSquared(maximum),
            0_mps2,
            0_mps2
        ),
        math::SquareMatrix::identity(9)
    );
    const auto saved = overflowTarget.state();
    EXPECT_FALSE(_algorithm->predict(overflowTarget, 10100ms));
    EXPECT_EQ(overflowTarget.state(), saved);
}

TEST_F(ConstantAccelerationAlgorithmTest, RejectsIndefiniteStateCovarianceAtConstruction)
{
    auto covariance = math::SquareMatrix::identity(9);
    covariance(0, 1) = 100.0;
    covariance(1, 0) = 100.0;
    EXPECT_THROW(
        seededTarget(
            position(1000.0, 2000.0, 3000.0),
            math::CartesianVelocity(0_mps, 0_mps, 0_mps),
            math::CartesianAcceleration(0_mps2, 0_mps2, 0_mps2),
            covariance
        ),
        std::invalid_argument
    );
}

TEST(KalmanStateTest, CopiesCovarianceIndependentlyOfTheSource)
{
    auto covariance = math::SquareMatrix::identity(TrackState::STATE_DIMENSION) * 25.0;
    covariance(0, 3) = 2.0;
    covariance(3, 0) = 2.0;
    auto state = KalmanStateAccess::makeState(covariance);
    const auto copy = state;
    state.reset();
    covariance(0, 0) = 99.0;

    const auto* stored = KalmanStateAccess::covariance(copy);
    ASSERT_NE(stored, nullptr);
    EXPECT_EQ(stored->size(), 9u);
    EXPECT_DOUBLE_EQ((*stored)(0, 0), 25.0);
    EXPECT_DOUBLE_EQ((*stored)(0, 3), 2.0);
    EXPECT_DOUBLE_EQ((*stored)(3, 0), 2.0);
}

TEST(KalmanStateTest, RejectsCovarianceWithWrongDimension)
{
    for (std::size_t size : {0u, 3u, 8u, 10u})
    {
        SCOPED_TRACE(size);
        EXPECT_THROW(KalmanStateAccess::makeState(math::SquareMatrix(size)), std::invalid_argument);
    }
}

TEST(KalmanStateTest, RejectsNonfiniteOrAsymmetricCovariance)
{
    for (auto invalid : {std::numeric_limits<double>::quiet_NaN(),
             std::numeric_limits<double>::infinity(),
             -std::numeric_limits<double>::infinity()})
    {
        auto covariance = math::SquareMatrix::identity(TrackState::STATE_DIMENSION);
        covariance(0, 0) = invalid;
        EXPECT_THROW(KalmanStateAccess::makeState(covariance), std::invalid_argument);
    }
    auto asymmetric = math::SquareMatrix::identity(TrackState::STATE_DIMENSION);
    asymmetric(0, 1) = std::numeric_limits<double>::epsilon();
    EXPECT_THROW(KalmanStateAccess::makeState(asymmetric), std::invalid_argument);
}

TEST(KalmanStateTest, RejectsNegativeOrSingularCovariance)
{
    auto negative = math::SquareMatrix::identity(TrackState::STATE_DIMENSION);
    negative(0, 0) = -1.0;
    EXPECT_THROW(KalmanStateAccess::makeState(negative), std::invalid_argument);
    EXPECT_THROW(KalmanStateAccess::makeState(math::SquareMatrix(TrackState::STATE_DIMENSION)),
        std::invalid_argument);
}

TEST(ConstantAccelerationConfigTest, ValidatesInitialAccelerationVarianceAndSingleMeasurementNoise)
{
    for (auto sigma : {0.0, -1.0, std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::infinity(), std::numeric_limits<double>::max(), 1e-300})
    {
        EXPECT_THROW(
            TargetConfig(
                AssociationConfig(1000_mps, 30_mps, 11.345),
                MeasurementConfig(7_m),
                TrackingConfig(math::LinearAcceleration::fromMetersPerSecondSquared(sigma), 0_mps2),
                TrackManagementConfig(100ms, 1000ms, 3, 100)
            ),
            std::invalid_argument
        );
    }
    EXPECT_THROW(
        TargetConfig(
            AssociationConfig(1000_mps, 30_mps, 11.345),
            MeasurementConfig(1.2e-162_m),
            TrackingConfig(10_mps2, 0_mps2),
            TrackManagementConfig(100ms, 1000ms, 3, 100)
        ),
        std::invalid_argument
    );
    const auto compatible = TargetConfig(
        AssociationConfig(1000_mps, 30_mps, 11.345),
        MeasurementConfig(7_m),
        TrackingConfig(10_mps2, 0_mps2),
        TrackManagementConfig(100ms, 1000ms, 3, 100)
    );
    EXPECT_EQ(compatible.tracking.initialAccelerationStandardDeviation(), 10_mps2);
}

TEST(ConstantAccelerationConfigTest, AcceptsZeroAndValidatesProcessAccelerationVariance)
{
    const auto zero = processNoiseConfig(0_mps2);
    EXPECT_EQ(zero.processAccelerationStandardDeviation(), 0_mps2);
    const auto nonzero = processNoiseConfig(10_mps2);
    EXPECT_EQ(nonzero.processAccelerationStandardDeviation(), 10_mps2);
    for (auto sigma : {-1.0, std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::infinity(), -std::numeric_limits<double>::infinity(), std::numeric_limits<double>::max(), 1e-300})
    {
        SCOPED_TRACE(sigma);
        EXPECT_THROW(
            processNoiseConfig(math::LinearAcceleration::fromMetersPerSecondSquared(sigma)),
            std::invalid_argument
        );
    }
}

TEST(ConstantAccelerationConfigTest, UsesInjectedInitialAccelerationUncertainty)
{
    const auto config = TargetConfig(
        AssociationConfig(1000_mps, 30_mps, 11.345),
        MeasurementConfig(7_m),
        TrackingConfig(2_mps2, 0_mps2),
        TrackManagementConfig(100ms, 1000ms, 3, 100)
    );
    const auto algorithm = std::make_shared<ConstantAccelerationAlgorithm>(config.tracking, config.measurement);
    const auto tentative = TrackFactory(algorithm).createTentative(
        1,
        observation(position(0.0, 1000.0, 0.0), 0ms)
    );
    const auto state = tentative.initialize(
        observation(position(0.0, 1000.0, 0.0), 100ms)
    );
    ASSERT_TRUE(state);
    EXPECT_EQ(state->state.timestamp(), 100ms);
    EXPECT_EQ(state->state.sphericalPosition().radius, 1000_m);
    ASSERT_TRUE(KalmanStateAccess::covariance(state->algorithmState));
    EXPECT_DOUBLE_EQ((*KalmanStateAccess::covariance(state->algorithmState))(6, 6), 4.0);
    EXPECT_NEAR((*KalmanStateAccess::covariance(state->algorithmState))(3, 6), 0.2, 1e-12);
    EXPECT_NEAR((*KalmanStateAccess::covariance(state->algorithmState))(3, 3), 9800.01, 1e-8);
}

} // namespace
} // namespace target
