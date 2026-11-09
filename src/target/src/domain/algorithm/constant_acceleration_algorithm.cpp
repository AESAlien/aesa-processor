#include "constant_acceleration_algorithm.hpp"

#include "domain/confirmed_track.hpp"

#include <math/geometry/cartesian_displacement.hpp>
#include <math/geometry/cartesian_position.hpp>
#include <math/geometry/cartesian_velocity.hpp>
#include <math/geometry/spherical_position.hpp>
#include <math/linear_algebra/matrix.hpp>
#include <math/linear_algebra/vector.hpp>

#include <cmath>
#include <cstddef>
#include <stdexcept>
#include <utility>

namespace target
{
namespace
{

using namespace math::literals;

// 상태 벡터는 안테나 직교 좌표계에서 위치, 속도, 가속도 순서로 각 XYZ 성분을 담는다.
constexpr std::size_t POSITION_DIMENSION = 3;
constexpr std::size_t VELOCITY_OFFSET = 3;
constexpr std::size_t ACCELERATION_OFFSET = 6;
constexpr auto STATE_DIMENSION = TrackState::STATE_DIMENSION;

math::Vector stateVector(const TrackState& state)
{
    return math::Vector({
        state.position().x.m(),
        state.position().y.m(),
        state.position().z.m(),
        state.velocity().x.mps(),
        state.velocity().y.mps(),
        state.velocity().z.mps(),
        state.acceleration().x.mps2(),
        state.acceleration().y.mps2(),
        state.acceleration().z.mps2()
    });
}

math::Vector measurementVector(const Observation& observation)
{
    const auto position = observation.sphericalPosition().toCartesian();
    return math::Vector({position.x.m(), position.y.m(), position.z.m()});
}

math::Matrix measurementMatrix()
{
    // 관측 행렬 H는 9개 상태 성분에서 관측과 비교할 XYZ 위치만 꺼낸다.
    auto measurement = math::Matrix(POSITION_DIMENSION, STATE_DIMENSION);
    for (std::size_t axis = 0; axis < POSITION_DIMENSION; ++axis)
    {
        measurement(axis, axis) = 1.0;
    }
    return measurement;
}

math::Matrix kalmanGain(
    const math::Matrix& crossCovariance,
    const math::SquareMatrix& innovationCovariance
) {
    // 이득 K는 K S = P^- H^T를 만족한다. S = L L^T로 분해한 뒤
    // K의 각 행에 대해 삼각 연립방정식 두 개를 차례로 풀어 역행렬 계산을 피한다.
    // S가 양의 정부호가 아니면 분해 예외를 호출자에게 전달한다.
    const auto lower = math::cholesky(innovationCovariance);
    const auto upper = math::transpose(lower);
    auto gain = math::Matrix(STATE_DIMENSION, POSITION_DIMENSION);
    for (std::size_t row = 0; row < STATE_DIMENSION; ++row)
    {
        const auto rightHandSide = math::Vector({
            crossCovariance(row, 0),
            crossCovariance(row, 1),
            crossCovariance(row, 2)
        });
        const auto solved = upper.solveUpperTriangular(lower.solveLowerTriangular(rightHandSide));
        for (std::size_t column = 0; column < POSITION_DIMENSION; ++column)
        {
            gain(row, column) = solved[column];
        }
    }
    return gain;
}

std::optional<math::CartesianVelocity> initialVelocity(
    const math::CartesianPosition& first,
    const math::CartesianPosition& second,
    double seconds
) {
    try
    {
        const auto velocity = (second - first) / std::chrono::duration<double>(seconds);
        static_cast<void>(velocity.speed());
        return velocity;
    }
    catch (const std::invalid_argument&)
    {
        // 두 위치가 유한해도 변위나 속력이 표현 범위를 벗어날 수 있다.
        return std::nullopt;
    }
}

math::SquareMatrix initialStateCovariance(
    double seconds,
    math::Length measurementStandardDeviation,
    math::LinearAcceleration accelerationStandardDeviation
) {
    const auto sigma = measurementStandardDeviation.m();
    const auto measurementVariance = sigma * sigma;
    const auto accelerationSigma = accelerationStandardDeviation.mps2();
    const auto accelerationVariance = accelerationSigma * accelerationSigma;
    // 두 위치의 측정 오차가 독립이면 차분 속도의 분산은 2 sigma^2 / dt^2다.
    // 두 번째 관측이 초기 위치와 속도에 모두 쓰이므로 위치와 속도 사이에 공분산이 생긴다.
    // 두 관측 사이의 평균 속도를 두 번째 시각의 속도로 옮길 때 가속도 불확실성도 반영한다.
    const auto positionVelocityCovariance = measurementVariance / seconds;
    const auto velocityVariance = 2.0 * measurementVariance / seconds / seconds
        + 0.25 * seconds * seconds * accelerationVariance;
    const auto velocityAccelerationCovariance = 0.5 * seconds * accelerationVariance;

    auto covariance = math::SquareMatrix(STATE_DIMENSION);
    for (std::size_t axis = 0; axis < POSITION_DIMENSION; ++axis)
    {
        covariance(axis, axis) = measurementVariance;
        covariance(axis, axis + VELOCITY_OFFSET) = positionVelocityCovariance;
        covariance(axis + VELOCITY_OFFSET, axis) = positionVelocityCovariance;
        covariance(axis + VELOCITY_OFFSET, axis + VELOCITY_OFFSET) = velocityVariance;
        covariance(axis + VELOCITY_OFFSET, axis + ACCELERATION_OFFSET) = velocityAccelerationCovariance;
        covariance(axis + ACCELERATION_OFFSET, axis + VELOCITY_OFFSET) = velocityAccelerationCovariance;
        covariance(axis + ACCELERATION_OFFSET, axis + ACCELERATION_OFFSET) = accelerationVariance;
    }
    return covariance;
}

math::SquareMatrix positionCovariance(const math::SquareMatrix& covariance)
{
    auto position = math::SquareMatrix(POSITION_DIMENSION);
    for (std::size_t row = 0; row < POSITION_DIMENSION; ++row)
    {
        for (std::size_t column = 0; column < POSITION_DIMENSION; ++column)
        {
            position(row, column) = covariance(row, column);
        }
    }
    return position;
}

std::optional<math::SquareMatrix> symmetrizeCovariance(math::SquareMatrix covariance)
{
    if (!covariance.isFinite())
    {
        return std::nullopt;
    }
    for (std::size_t row = 0; row < covariance.size(); ++row)
    {
        if (covariance(row, row) < 0.0)
        {
            return std::nullopt;
        }
        for (std::size_t column = 0; column < row; ++column)
        {
            // 반올림 오차로 어긋난 대칭을 복원한다.
            const auto entry = 0.5 * covariance(row, column)
                + 0.5 * covariance(column, row);
            covariance(row, column) = entry;
            covariance(column, row) = entry;
        }
    }
    return covariance;
}

struct PredictedState
{
    math::Vector values;
    math::SquareMatrix covariance;
};

math::SquareMatrix processNoiseCovariance(
    double seconds,
    math::LinearAcceleration standardDeviation
) {
    if (seconds == 0.0 || standardDeviation == 0_mps2)
    {
        return math::SquareMatrix(STATE_DIMENSION);
    }

    // 한 예측 구간에서 축별로 독립인 가속도 잡음이 위치, 속도, 가속도에 함께 전파된다.
    // 전파 행렬 G의 세 블록은 (dt^2/2) I, dt I, I이며 공정 잡음 공분산은 Q = G (sigma^2 I) G^T다.
    auto disturbanceMapping = math::Matrix(STATE_DIMENSION, POSITION_DIMENSION);
    for (std::size_t axis = 0; axis < POSITION_DIMENSION; ++axis)
    {
        disturbanceMapping(axis, axis) = 0.5 * seconds * seconds;
        disturbanceMapping(axis + VELOCITY_OFFSET, axis) = seconds;
        disturbanceMapping(axis + ACCELERATION_OFFSET, axis) = 1.0;
    }
    const auto sigma = standardDeviation.mps2();
    const auto disturbanceCovariance = math::SquareMatrix::identity(POSITION_DIMENSION)
        * (sigma * sigma);
    return math::SquareMatrix(
        disturbanceMapping * disturbanceCovariance.asMatrix()
            * math::transpose(disturbanceMapping)
    );
}

std::optional<PredictedState> predictState(
    const TrackState& state,
    const math::SquareMatrix* errorCovariance,
    std::chrono::milliseconds time,
    math::LinearAcceleration processAccelerationStandardDeviation
) {
    if (!errorCovariance || time < state.timestamp())
    {
        return std::nullopt;
    }

    // 마지막으로 갱신한 상태 시각부터 예측한다. 중간에 관측을 놓친 시간도 이 간격에 포함된다.
    const auto seconds = std::chrono::duration<double>(time - state.timestamp()).count();
    const auto halfSquaredSeconds = 0.5 * seconds * seconds;
    if (!std::isfinite(halfSquaredSeconds))
    {
        return std::nullopt;
    }
    // 상태 전이 F는 p' = p + v dt + a dt^2/2, v' = v + a dt, a' = a를 적용한다.
    auto transition = math::SquareMatrix::identity(STATE_DIMENSION);
    for (std::size_t axis = 0; axis < POSITION_DIMENSION; ++axis)
    {
        transition(axis, axis + VELOCITY_OFFSET) = seconds;
        transition(axis, axis + ACCELERATION_OFFSET) = halfSquaredSeconds;
        transition(axis + VELOCITY_OFFSET, axis + ACCELERATION_OFFSET) = seconds;
    }
    auto values = transition * stateVector(state);
    // 기존 오차를 F로 전파한 뒤 이 구간의 공정 잡음 Q를 더해 P^- = F P F^T + Q를 구한다.
    auto covariance = symmetrizeCovariance(
        transition * *errorCovariance * math::transpose(transition)
        + processNoiseCovariance(seconds, processAccelerationStandardDeviation)
    );
    if (!values.isFinite() || !covariance)
    {
        return std::nullopt;
    }
    return PredictedState{std::move(values), std::move(*covariance)};
}

std::optional<math::CartesianPosition> calculatedPosition(const math::Vector& values)
{
    try
    {
        const auto position = math::CartesianPosition(
            math::Length::fromMeters(values[0]),
            math::Length::fromMeters(values[1]),
            math::Length::fromMeters(values[2])
        );
        if (position.distance() == 0_m)
        {
            return std::nullopt;
        }
        return position;
    }
    catch (const std::invalid_argument&)
    {
        return std::nullopt;
    }
}

std::optional<math::CartesianVelocity> calculatedVelocity(const math::Vector& values)
{
    try
    {
        const auto velocity = math::CartesianVelocity(
            math::LinearVelocity::fromMetersPerSecond(values[3]),
            math::LinearVelocity::fromMetersPerSecond(values[4]),
            math::LinearVelocity::fromMetersPerSecond(values[5])
        );
        static_cast<void>(velocity.speed());
        return velocity;
    }
    catch (const std::invalid_argument&)
    {
        return std::nullopt;
    }
}

math::CartesianAcceleration calculatedAcceleration(const math::Vector& values)
{
    return math::CartesianAcceleration(
        math::LinearAcceleration::fromMetersPerSecondSquared(values[ACCELERATION_OFFSET]),
        math::LinearAcceleration::fromMetersPerSecondSquared(values[ACCELERATION_OFFSET + 1]),
        math::LinearAcceleration::fromMetersPerSecondSquared(values[ACCELERATION_OFFSET + 2])
    );
}

} // namespace

ConstantAccelerationAlgorithm::ConstantAccelerationAlgorithm(
    TrackingConfig config,
    MeasurementConfig measurementConfig
) : _config(std::move(config)),
    _measurementConfig(std::move(measurementConfig))
{
}

void ConstantAccelerationAlgorithm::storeCovariance(std::any& state, math::SquareMatrix covariance)
{
    if (covariance.size() != STATE_DIMENSION)
    {
        throw std::invalid_argument("Tracking covariance requires a 9x9 matrix");
    }

    try
    {
        static_cast<void>(math::cholesky(covariance));
    }
    catch (const std::domain_error&)
    {
        throw std::invalid_argument("Tracking covariance must be positive definite");
    }
    // 다음 예측에서 쓸 검증된 공분산을 트랙별 전략 상태에 불변 값으로 저장한다.
    state = std::make_shared<const math::SquareMatrix>(std::move(covariance));
}

const math::SquareMatrix* ConstantAccelerationAlgorithm::stateCovariance(const std::any& state) noexcept
{
    const auto* covariance = std::any_cast<std::shared_ptr<const math::SquareMatrix>>(&state);
    return covariance ? covariance->get() : nullptr;
}

std::optional<TrackState> ConstantAccelerationAlgorithm::initialize(
    const Observation& first,
    const Observation& second,
    std::any& nextAlgorithmState
) const {
    const auto firstTime = first.observationTime();
    const auto secondTime = second.observationTime();
    if (secondTime <= firstTime)
    {
        return std::nullopt;
    }

    const auto seconds = std::chrono::duration<double>(secondTime - firstTime).count();
    const auto firstPosition = first.sphericalPosition().toCartesian();
    const auto position = second.sphericalPosition().toCartesian();
    // 두 번째 관측 위치에서 시작하고, 두 관측의 변위와 경과 시간으로 초기 속도를 구한다.
    const auto velocity = initialVelocity(firstPosition, position, seconds);
    if (!velocity)
    {
        return std::nullopt;
    }
    auto initialCovariance = initialStateCovariance(
        seconds,
        _measurementConfig.standardDeviation(),
        _config.initialAccelerationStandardDeviation()
    );
    if (!initialCovariance.isFinite())
    {
        return std::nullopt;
    }
    auto state = TrackState(
        position.toSpherical(),
        second.dopplerVelocity(),
        second.power_db(),
        secondTime,
        position,
        *velocity
    );
    storeCovariance(nextAlgorithmState, std::move(initialCovariance));
    return state;
}

std::optional<TrackState> ConstantAccelerationAlgorithm::update(
    const ConfirmedTrack& current,
    const Observation& observation,
    std::any& nextAlgorithmState
) const {
    const auto& state = *current.state();
    const auto time = observation.observationTime();
    if (time <= state.timestamp() || time <= current.lastResultTime())
    {
        return std::nullopt;
    }
    // 새 관측과 같은 시각에서 비교할 수 있도록 상태와 공분산을 먼저 예측한다.
    const auto predicted = predictState(
        state,
        stateCovariance(algorithmState(current)),
        time,
        _config.processAccelerationStandardDeviation()
    );
    if (!predicted)
    {
        return std::nullopt;
    }

    const auto measurement = measurementVector(observation);
    const auto measurementMapping = measurementMatrix();
    const auto sigma = _measurementConfig.standardDeviation().m();
    const auto measurementNoise = math::SquareMatrix::identity(POSITION_DIMENSION) * (sigma * sigma);
    const auto crossCovariance = predicted->covariance.asMatrix() * math::transpose(measurementMapping);
    // 잔차 공분산 S = H P^- H^T + R에서 첫 항은 예측 공분산의 3×3 위치 블록이다.
    auto innovationCovariance = symmetrizeCovariance(
        positionCovariance(predicted->covariance) + measurementNoise
    );
    if (!innovationCovariance)
    {
        return std::nullopt;
    }

    const auto gain = kalmanGain(crossCovariance, *innovationCovariance);
    // 실제 관측 z와 예측 관측 H x^-의 차이에 K를 곱해 위치, 속도, 가속도를 함께 보정한다.
    const auto residual = measurement - measurementMapping * predicted->values;
    const auto values = predicted->values + gain * residual;
    // 조지프 형태는 전파된 상태 오차와 관측 잡음의 기여를 따로 계산해 반올림 오차에 더 안정적이다.
    const auto remaining = math::SquareMatrix::identity(STATE_DIMENSION).asMatrix()
        - gain * measurementMapping;
    auto covariance = symmetrizeCovariance(
        math::SquareMatrix(
            remaining * predicted->covariance.asMatrix() * math::transpose(remaining)
            + gain * measurementNoise.asMatrix() * math::transpose(gain)
        )
    );
    if (!values.isFinite() || !covariance)
    {
        return std::nullopt;
    }
    const auto position = calculatedPosition(values);
    const auto velocity = calculatedVelocity(values);
    if (!position || !velocity)
    {
        return std::nullopt;
    }
    auto nextState = TrackState(
        position->toSpherical(),
        observation.dopplerVelocity(),
        observation.power_db(),
        time,
        *position,
        *velocity,
        calculatedAcceleration(values)
    );
    storeCovariance(nextAlgorithmState, std::move(*covariance));
    return nextState;
}

std::optional<TrackPrediction> ConstantAccelerationAlgorithm::predict(
    const ConfirmedTrack& current,
    std::chrono::milliseconds time
) const {
    return predictPosition(current, time, nullptr);
}

std::optional<TrackPrediction> ConstantAccelerationAlgorithm::predictWithPositionCovariance(
    const ConfirmedTrack& current,
    std::chrono::milliseconds time,
    std::optional<math::SquareMatrix>& covariance
) const {
    covariance.reset();
    return predictPosition(current, time, &covariance);
}

std::optional<math::SquareMatrix> ConstantAccelerationAlgorithm::observedPositionCovariance(
    const ConfirmedTrack& current
) const {
    const auto* covariance = stateCovariance(algorithmState(current));
    if (!covariance)
    {
        return std::nullopt;
    }
    return positionCovariance(*covariance);
}

std::optional<TrackPrediction> ConstantAccelerationAlgorithm::predictPosition(
    const ConfirmedTrack& current,
    std::chrono::milliseconds time,
    std::optional<math::SquareMatrix>* covariance
) const {
    const auto predicted = predictState(
        *current.state(),
        stateCovariance(algorithmState(current)),
        time,
        _config.processAccelerationStandardDeviation()
    );
    if (!predicted)
    {
        return std::nullopt;
    }
    const auto position = calculatedPosition(predicted->values);
    if (!position)
    {
        return std::nullopt;
    }
    auto prediction = TrackPrediction(time, position->toSpherical());
    if (covariance)
    {
        *covariance = positionCovariance(predicted->covariance);
    }
    return prediction;
}

} // namespace target
