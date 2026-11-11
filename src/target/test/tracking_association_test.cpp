#include "domain/association/nds_algorithm.hpp"
#include "association_selection_helpers.hpp"
#include "domain/tracking/constant_velocity_algorithm.hpp"
#include "domain/track_factory.hpp"
#include "repository/track_repository.hpp"
#include "domain/association/track_selector.hpp"

#include <target/config/target_config.hpp>
#include <target/controller/target_controller.hpp>

#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <initializer_list>
#include <limits>
#include <memory>
#include <optional>
#include <stdexcept>
#include <utility>
#include <vector>

namespace target
{
namespace
{

using namespace math::literals;
using namespace std::chrono_literals;
using test_support::CovarianceAlgorithm;
using test_support::selectTracking;
using test_support::isDuplicate;
using Time = std::chrono::milliseconds;
using BeamType = beam::BeamType;

static_assert(
    std::is_same_v<
        decltype(std::declval<const ConfirmedTrack&>().slantRange()),
        math::Distance>
);

math::SphericalPosition position(math::Length range)
{
    return math::SphericalPosition(range, 0_deg, 0_deg);
}

struct PreparationCounts
{
    int measurements = 0;
    int predictions = 0;
    int states = 0;
    int evaluations = 0;
    bool failPrediction = false;
};

class InjectedCalculation final : public IAssociationCalculation
{
public:
    explicit InjectedCalculation(std::shared_ptr<PreparationCounts> counts) : _counts(std::move(counts))
    {
    }

    std::optional<double> calculate(
        const math::SphericalPosition& observation,
        const math::SphericalPosition&,
        std::optional<math::Distance>
    ) const override {
        ++_counts->evaluations;
        return observation.radius.m() == 1020.0 ? 1.0 : 100.0;
    }

    std::optional<double> calculate(const math::Vector&) const override
    {
        return std::nullopt;
    }

private:
    const std::shared_ptr<PreparationCounts> _counts;
};

class InjectedAssociationAlgorithm final : public IAssociationAlgorithm
{
public:
    explicit InjectedAssociationAlgorithm(std::shared_ptr<PreparationCounts> counts) : _counts(std::move(counts))
    {
    }

    AssociationCalculation prepareMeasurements() const override
    {
        ++_counts->measurements;
        return std::make_unique<InjectedCalculation>(_counts);
    }

    std::optional<AssociationPrediction> predictAndPrepare(const ConfirmedTrack& track, Time time) const override
    {
        auto prediction = track.predict(time);
        if (!prediction)
        {
            return std::nullopt;
        }
        ++_counts->predictions;
        if (_counts->failPrediction)
        {
            return std::nullopt;
        }
        return AssociationPrediction{std::move(*prediction), std::make_unique<InjectedCalculation>(_counts)};
    }

    AssociationCalculation prepareFromLastObservation(const ConfirmedTrack&) const override
    {
        ++_counts->states;
        return std::make_unique<InjectedCalculation>(_counts);
    }

private:
    const std::shared_ptr<PreparationCounts> _counts;
};

DetectionEvent result(
    BeamType type,
    std::uint32_t id,
    Time time,
    std::initializer_list<math::Length> ranges
) {
    std::vector<DetectionEvent::Plot> plots;
    for (const auto range : ranges)
    {
        plots.emplace_back(math::Distance(range), math::CircularAngle(0_deg), math::TiltAngle(0_deg), 0_mps, -10.0f);
    }
    return DetectionEvent(type, id, time, std::move(plots));
}

class NoPredictionAlgorithm final : public ITrackingAlgorithm
{
public:
    std::optional<TrackingResult> initialize(const Observation& first, const Observation& second) const override
    {
        return _algorithm.initialize(first, second);
    }

    std::optional<TrackingResult> update(const ConfirmedTrack& current, const Observation& incoming) const override
    {
        return _algorithm.update(current, incoming);
    }

    std::optional<TrackPrediction> predict(const ConfirmedTrack&, Time) const override
    {
        return std::nullopt;
    }

private:
    ConstantVelocityAlgorithm _algorithm;
};

class CountingPredictionAlgorithm final : public ITrackingAlgorithm
{
public:
    explicit CountingPredictionAlgorithm(bool predictionAvailable = true) : _predictionAvailable(predictionAvailable)
    {
    }

    std::optional<TrackingResult> initialize(
        const Observation& first,
        const Observation& second
    ) const override {
        return _algorithm.initialize(first, second);
    }

    std::optional<TrackingResult> update(
        const ConfirmedTrack& current,
        const Observation& incoming
    ) const override {
        return _algorithm.update(current, incoming);
    }

    std::optional<TrackPrediction> predict(const ConfirmedTrack& current, Time time) const override
    {
        ++_predictionCount;
        return _predictionAvailable ? _algorithm.predict(current, time) : std::nullopt;
    }

    std::size_t predictionCount() const
    {
        return _predictionCount;
    }

private:
    const bool _predictionAvailable;
    mutable std::size_t _predictionCount = 0;
    ConstantVelocityAlgorithm _algorithm;
};

class TrackingAssociationTest : public ::testing::Test
{
protected:
    Observation observation(math::Length range, Time time) const
    {
        return Observation(position(range), 0_mps, -10.0f, time);
    }

    std::uint32_t addTracking(
        math::Length firstRange = 1000_m,
        math::Length secondRange = 1000_m,
        std::shared_ptr<const ITrackingAlgorithm> algorithm = nullptr
    ) {
        if (!algorithm)
        {
            algorithm = _algorithm;
        }
        const auto id = _repository.add(
            TrackFactory(std::move(algorithm)).createTentative(_repository.nextId(), observation(firstRange, 0ms))
        );
        auto confirmedTrack = _repository.findTentativeTrack(id)->confirm(observation(secondRange, 100ms));
        EXPECT_TRUE(confirmedTrack);
        EXPECT_TRUE(_repository.replace(std::move(confirmedTrack).value()));
        return id;
    }

    const std::shared_ptr<const ITrackingAlgorithm> _algorithm = std::make_shared<ConstantVelocityAlgorithm>();
    TrackRepository _repository;
    TrackSelector _selector = TrackSelector(
        AssociationConfig(1000_mps, 30_mps, 11.345), std::make_unique<NdsAlgorithm>(MeasurementConfig(7_m))
    );
};

TEST_F(TrackingAssociationTest, ReturnsSquaredCartesianDistanceScore)
{
    const auto id = addTracking();
    const auto& track = *_repository.findConfirmedTrack(id);
    const auto incoming = observation(1010_m, 1100ms);
    // 10 m 잔차의 제곱을 두 관측의 차에 대한 분산 2 * 7^2으로 나눈다.
    const auto expected = 100.0 / 98.0;
    EXPECT_TRUE(selectTracking(incoming, track, AssociationConfig(1000_mps, 30_mps, expected + 1e-12)));
    EXPECT_FALSE(selectTracking(incoming, track, AssociationConfig(1000_mps, 30_mps, expected - 1e-12)));
}

TEST(TentativeSelectorTest, UsesOnlyTheSuppliedTracksWithoutARepository)
{
    const auto selector = TentativeSelector(AssociationConfig(1000_mps, 30_mps, 11.345), std::make_unique<NdsAlgorithm>(MeasurementConfig(7_m)));
    const auto factory = TrackFactory(std::make_shared<ConstantVelocityAlgorithm>());
    const auto observation = [](math::Length range, Time time)
    {
        return Observation(position(range), 0_mps, -10.0f, time);
    };
    const auto tentative = factory.createTentative(1, observation(1000_m, 0ms));
    const auto confirmed = factory.createTentative(2, observation(2000_m, 0ms)).confirm(observation(2000_m, 100ms));
    ASSERT_TRUE(confirmed);
    auto tracks = std::list<Track>{tentative, *confirmed};
    const auto candidates = std::vector<Observation>{
        observation(1000_m, 100ms), observation(2000_m, 100ms),
        observation(3000_m, 100ms), observation(4000_m, 100ms)
    };
    const auto selected = selector.selectNewCandidate(candidates, tracks);
    ASSERT_TRUE(selected);
    EXPECT_EQ(selected->slantRange(), 3000_m);
    EXPECT_EQ(tracks.size(), 2u);
    EXPECT_FALSE(selector.selectNewCandidate({candidates[0], candidates[1]}, tracks));
    EXPECT_FALSE(selector.selectNewCandidate({}, tracks));

    tracks.clear();
    const auto first = selector.selectNewCandidate(candidates, tracks);
    ASSERT_TRUE(first);
    EXPECT_EQ(first->slantRange(), 1000_m);
}

TEST(SelectorConstructionTest, RejectsMissingOwnedAlgorithm)
{
    const auto config = AssociationConfig(1000_mps, 30_mps, 11.345);
    EXPECT_THROW(TrackSelector(config, nullptr), std::invalid_argument);
    EXPECT_THROW(TentativeSelector(config, nullptr), std::invalid_argument);
}

TEST_F(TrackingAssociationTest, SelectorsUseInjectedScoresForEveryComparisonMode)
{
    const auto id = addTracking();
    const auto counts = std::make_shared<PreparationCounts>();
    const auto config = AssociationConfig(1000_mps, 30_mps, 11.345);
    const auto trackSelector = TrackSelector(config, std::make_unique<InjectedAssociationAlgorithm>(counts));
    const auto tentativeSelector = TentativeSelector(config, std::make_unique<InjectedAssociationAlgorithm>(counts));
    EXPECT_EQ(counts->measurements, 0);

    // 주입한 알고리즘은 더 먼 관측에 더 낮은 점수를 준다.
    const auto selected = trackSelector.selectTrackingObservation(
        *_repository.findConfirmedTrack(id), 1100ms, {observation(1000_m, 1100ms), observation(1020_m, 1100ms)}
    );
    ASSERT_TRUE(selected);
    EXPECT_EQ(selected->slantRange(), 1020_m);
    EXPECT_EQ(counts->predictions, 1);
    EXPECT_EQ(counts->evaluations, 2);

    const auto fresh = tentativeSelector.selectNewCandidate({observation(1020_m, 1100ms), observation(1000_m, 1100ms)}, _repository.tracks());
    ASSERT_TRUE(fresh);
    EXPECT_EQ(fresh->slantRange(), 1000_m);
    EXPECT_FALSE(tentativeSelector.selectNewCandidate({observation(1020_m, 50ms)}, _repository.tracks()));
    EXPECT_EQ(counts->states, 1);

    const auto tentative = TrackFactory(_algorithm).createTentative(_repository.nextId(), observation(1000_m, 0ms));
    const auto confirmation = trackSelector.selectConfirmationObservation(
        tentative, 100ms, {observation(1000_m, 100ms), observation(1020_m, 100ms)}
    );
    ASSERT_TRUE(confirmation);
    EXPECT_EQ(confirmation->slantRange(), 1020_m);
    ASSERT_TRUE(_repository.remove(id));
    _repository.add(tentative);
    EXPECT_FALSE(tentativeSelector.selectNewCandidate({observation(1020_m, 0ms)}, _repository.tracks()));
}

TEST_F(TrackingAssociationTest, FailedInjectedPreparationIsAttemptedOnlyOncePerBatch)
{
    const auto id = addTracking();
    const auto counts = std::make_shared<PreparationCounts>();
    counts->failPrediction = true;
    const auto selector = TrackSelector(AssociationConfig(1000_mps, 30_mps, 11.345), std::make_unique<InjectedAssociationAlgorithm>(counts));
    EXPECT_FALSE(selector.selectTrackingObservation(*_repository.findConfirmedTrack(id), 1100ms, {}));
    EXPECT_EQ(counts->predictions, 0);
    const auto candidates = std::vector<Observation>({observation(1000_m, 1100ms), observation(1020_m, 1100ms)});
    EXPECT_FALSE(selector.selectTrackingObservation(*_repository.findConfirmedTrack(id), 1100ms, candidates));
    EXPECT_EQ(counts->predictions, 1);
    EXPECT_EQ(counts->evaluations, 0);
    EXPECT_FALSE(selector.selectTrackingObservation(*_repository.findConfirmedTrack(id), 1100ms, candidates));
    EXPECT_EQ(counts->predictions, 2);
}

TEST_F(TrackingAssociationTest, EqualSlantRangesStillUseAzimuthAndElevationDisplacement)
{
    const auto id = addTracking();
    const auto& tracking = *_repository.findConfirmedTrack(id);
    for (bool elevationOnly : {false, true})
    {
        SCOPED_TRACE(elevationOnly);
        const auto near = Observation(
            math::SphericalPosition(
                1000_m,
                elevationOnly ? 0_deg : 1_deg,
                elevationOnly ? 1_deg : 0_deg
            ),
            0_mps,
            -10.0f,
            1100ms
        );
        const auto far = Observation(
            math::SphericalPosition(
                1000_m,
                elevationOnly ? 0_deg : 3_deg,
                elevationOnly ? 3_deg : 0_deg
            ),
            0_mps,
            -10.0f,
            1100ms
        );
        // 거리 1000 m에서 각도 차가 1°면 변위는 약 17.45 m, 3°면 약 52.35 m다.
        EXPECT_TRUE(selectTracking(near, tracking));
        EXPECT_FALSE(selectTracking(far, tracking));
    }
}

TEST_F(TrackingAssociationTest, UsesReceiveTimePredictionWithoutTentativeMotionAllowance)
{
    const auto id = addTracking(1000_m, 1010_m);
    const auto& tracking = *_repository.findConfirmedTrack(id);
    // 두 관측으로 계산한 속력은 100 m/s다. 100 ms의 1010 m에서 1100 ms에는 1110 m로 예측된다.
    const auto selected = selectTracking(observation(1120_m, 1100ms), tracking);
    ASSERT_TRUE(selected);
    EXPECT_EQ(selected->slantRange(), 1120_m);
    EXPECT_FALSE(selectTracking(observation(1000_m, 1100ms), tracking));
    EXPECT_FALSE(selectTracking(observation(1150_m, 1100ms), tracking));
    EXPECT_EQ(tracking.slantRange(), 1010_m);
    EXPECT_EQ(tracking.lastObservationTime(), 100ms);
    EXPECT_EQ(tracking.lastResultTime(), 100ms);
    EXPECT_EQ(tracking.consecutiveMissCount(), 0u);
}

TEST_F(TrackingAssociationTest, SharesConfiguredNdsGateAndAcceptsTheBoundary)
{
    const auto id = addTracking();
    const auto& tracking = *_repository.findConfirmedTrack(id);
    EXPECT_TRUE(selectTracking(observation(1032_m, 1100ms), tracking));
    EXPECT_FALSE(selectTracking(observation(1034_m, 1100ms), tracking));

    const auto exact = observation(1000_m, 1100ms);
    const auto zeroGate = AssociationConfig(1000_mps, 30_mps, 0.0);
    EXPECT_TRUE(selectTracking(exact, tracking, zeroGate));
    EXPECT_TRUE(isDuplicate(exact, tracking, zeroGate));
    const auto incoming = observation(1010_m, 1100ms);
    const auto expected = 100.0 / 98.0;
    const auto larger = AssociationConfig(1000_mps, 30_mps, expected + 1e-12);
    const auto smaller = AssociationConfig(1000_mps, 30_mps, expected - 1e-12);
    EXPECT_TRUE(selectTracking(incoming, tracking, larger));
    EXPECT_TRUE(isDuplicate(incoming, tracking, larger));
    EXPECT_FALSE(selectTracking(incoming, tracking, smaller));
    EXPECT_FALSE(isDuplicate(incoming, tracking, smaller));
}

TEST_F(TrackingAssociationTest, UsesInjectedMeasurementNoiseForScoreAndGate)
{
    const auto id = addTracking();
    const auto& tracking = *_repository.findConfirmedTrack(id);
    const auto near = observation(1004_m, 1100ms);
    EXPECT_TRUE(selectTracking(near, tracking, AssociationConfig(1000_mps, 30_mps, 2.0 + 1e-12), MeasurementConfig(2_m)));
    EXPECT_FALSE(selectTracking(near, tracking, AssociationConfig(1000_mps, 30_mps, 2.0 - 1e-12), MeasurementConfig(2_m)));
    EXPECT_FALSE(selectTracking(observation(1010_m, 1100ms), tracking, AssociationConfig(1000_mps, 30_mps, 11.345), MeasurementConfig(2_m)));
    EXPECT_TRUE(selectTracking(observation(1010_m, 1100ms), tracking));
}

TEST_F(TrackingAssociationTest, UsesPredictedAnisotropicCovariancePlusOneMeasurementNoise)
{
    const auto algorithm = std::make_shared<CovarianceAlgorithm>(
        math::SquareMatrix({{151.0, 0.0, 0.0}, {0.0, 351.0, 0.0}, {0.0, 0.0, 751.0}})
    );
    const auto id = addTracking(1000_m, 1000_m, algorithm);
    const auto& tracking = *_repository.findConfirmedTrack(id);
    const auto incoming = Observation(math::CartesianPosition(10_m, 1020_m, 30_m).toSpherical(), 0_mps, -10.0f, 1100ms);
    const auto score = selectTracking(incoming, tracking);
    const auto searchScore = isDuplicate(incoming, tracking);
    ASSERT_TRUE(score);
    ASSERT_TRUE(searchScore);
    // 잔차 공분산은 S = diag(151, 351, 751) + 49 I, 잔차는 r = [10, 20, 30]이다.
    const auto expected = 100.0 / 200.0 + 400.0 / 400.0 + 900.0 / 800.0;
    const auto larger = AssociationConfig(1000_mps, 30_mps, expected + 1e-12);
    const auto smaller = AssociationConfig(1000_mps, 30_mps, expected - 1e-12);
    EXPECT_TRUE(selectTracking(incoming, tracking, larger));
    EXPECT_FALSE(selectTracking(incoming, tracking, smaller));
    EXPECT_TRUE(isDuplicate(incoming, tracking, larger));
    EXPECT_FALSE(isDuplicate(incoming, tracking, smaller));
}

TEST_F(TrackingAssociationTest, RetainsPredictedCrossAxisCorrelations)
{
    const auto algorithm = std::make_shared<CovarianceAlgorithm>(
        math::SquareMatrix({{51.0, 50.0, 0.0}, {50.0, 151.0, 0.0}, {0.0, 0.0, 251.0}})
    );
    const auto id = addTracking(1000_m, 1000_m, algorithm);
    const auto& tracking = *_repository.findConfirmedTrack(id);
    const auto incoming = Observation(math::CartesianPosition(10_m, 1010_m, 0_m).toSpherical(), 0_mps, -10.0f, 1100ms);
    const auto score = selectTracking(incoming, tracking);
    ASSERT_TRUE(score);
    // 잔차 공분산에서 x축과 y축에 해당하는 블록 [[100, 50], [50, 200]]의 행렬식은 17500이다.
    EXPECT_TRUE(selectTracking(incoming, tracking, AssociationConfig(1000_mps, 30_mps, 8.0 / 7.0 + 1e-12)));
    EXPECT_FALSE(selectTracking(incoming, tracking, AssociationConfig(1000_mps, 30_mps, 8.0 / 7.0 - 1e-12)));
}

TEST_F(TrackingAssociationTest, PredictionUncertaintyChangesTheGateAtTheRequestedTime)
{
    const auto algorithm = std::make_shared<CovarianceAlgorithm>(
        math::SquareMatrix::identity(3),
        100.0
    );
    const auto id = addTracking(1000_m, 1000_m, algorithm);
    const auto& tracking = *_repository.findConfirmedTrack(id);
    EXPECT_FALSE(selectTracking(observation(1050_m, 1100ms), tracking));
    const auto later = selectTracking(observation(1050_m, 2100ms), tracking);
    ASSERT_TRUE(later);
    const auto incoming = observation(1050_m, 2100ms);
    EXPECT_TRUE(selectTracking(incoming, tracking, AssociationConfig(1000_mps, 30_mps, 10.0 + 1e-12)));
    EXPECT_FALSE(selectTracking(incoming, tracking, AssociationConfig(1000_mps, 30_mps, 10.0 - 1e-12)));
    EXPECT_TRUE(isDuplicate(observation(1050_m, 2100ms), tracking));
    EXPECT_EQ(tracking.lastObservationTime(), 100ms);
}

TEST_F(TrackingAssociationTest, PastSearchUsesStoredPositionCovarianceAndMotionAllowance)
{
    const auto algorithm = std::make_shared<CovarianceAlgorithm>(
        math::SquareMatrix({{51.0, 0.0, 0.0}, {0.0, 351.0, 0.0}, {0.0, 0.0, 51.0}})
    );
    const auto id = addTracking(1000_m, 1000_m, algorithm);
    const auto& tracking = *_repository.findConfirmedTrack(id);
    // 50 ms 동안 이동할 수 있는 50 m를 빼면 y축 잔차는 20 m다.
    const auto score = isDuplicate(observation(1070_m, 50ms), tracking);
    ASSERT_TRUE(score);
    const auto incoming = observation(1070_m, 50ms);
    EXPECT_TRUE(isDuplicate(incoming, tracking, AssociationConfig(1000_mps, 30_mps, 1.0 + 1e-12)));
    EXPECT_FALSE(isDuplicate(incoming, tracking, AssociationConfig(1000_mps, 30_mps, 1.0 - 1e-12)));
    EXPECT_EQ(tracking.lastObservationTime(), 100ms);
}

TEST_F(TrackingAssociationTest, ValidatesCovarianceProvidedForPrediction)
{
    const auto prepare = [this](const math::SquareMatrix& covariance)
    {
        const auto id = addTracking(1000_m, 1000_m, std::make_shared<CovarianceAlgorithm>(covariance));
        auto algorithm = NdsAlgorithm{MeasurementConfig(7_m)};
        return algorithm.predictAndPrepare(*_repository.findConfirmedTrack(id), 1100ms);
    };
    EXPECT_TRUE(prepare(math::SquareMatrix::identity(3)));
    for (std::size_t dimension : {2u, 4u, 9u})
    {
        EXPECT_THROW(
            prepare(math::SquareMatrix::identity(dimension)),
            std::invalid_argument
        );
    }
    auto covariance = math::SquareMatrix::identity(3);
    covariance(0, 1) = 0.5;
    EXPECT_THROW(prepare(covariance), std::invalid_argument);
    covariance(1, 0) = 0.5;
    EXPECT_TRUE(prepare(covariance));
    EXPECT_THROW(prepare(math::SquareMatrix(3)), std::invalid_argument);
    covariance(0, 0) = -1.0;
    EXPECT_THROW(prepare(covariance), std::invalid_argument);
    for (auto invalid : {std::numeric_limits<double>::quiet_NaN(),
             std::numeric_limits<double>::infinity()})
    {
        covariance(0, 0) = invalid;
        EXPECT_THROW(prepare(covariance), std::invalid_argument);
    }
}

TEST_F(TrackingAssociationTest, RejectsStaleOrDuplicateResults)
{
    const auto tentativeId = _repository.add(
        TrackFactory(_algorithm).createTentative(_repository.nextId(), observation(1000_m, 0ms))
    );
    EXPECT_EQ(_repository.findConfirmedTrack(tentativeId), nullptr);

    const auto id = addTracking();
    auto& tracking = *_repository.findConfirmedTrack(id);
    EXPECT_FALSE(selectTracking(observation(1000_m, 99ms), tracking));
    EXPECT_FALSE(selectTracking(observation(1000_m, 100ms), tracking));
    ASSERT_TRUE(tracking.recordTrackingMiss(1100ms));
    EXPECT_FALSE(selectTracking(observation(1000_m, 1000ms), tracking));
    EXPECT_FALSE(selectTracking(observation(1000_m, 1100ms), tracking));
    EXPECT_FALSE(_selector.selectTrackingObservation(*_repository.findConfirmedTrack(id), 1100ms, {observation(1000_m, 1100ms)}));
    EXPECT_EQ(tracking.consecutiveMissCount(), 1u);
}

TEST_F(TrackingAssociationTest, ReturnsNoMatchWhenPredictionIsUnavailable)
{
    const auto id = addTracking(1000_m, 1000_m, std::make_shared<NoPredictionAlgorithm>());
    const auto incoming = observation(1000_m, 1100ms);
    EXPECT_FALSE(selectTracking(incoming, *_repository.findConfirmedTrack(id)));
    EXPECT_FALSE(_selector.selectTrackingObservation(*_repository.findConfirmedTrack(id), 1100ms, {incoming}));
    EXPECT_EQ(_repository.findConfirmedTrack(id)->lastResultTime(), 100ms);
}

TEST_F(TrackingAssociationTest, SelectsMinimumScoreIncludingZeroAndKeepsFirstTie)
{
    const auto id = addTracking();
    const auto nearest = _selector.selectTrackingObservation(
        *_repository.findConfirmedTrack(id),
        1100ms,
        {observation(1020_m, 1100ms), observation(1005_m, 1100ms), observation(1010_m, 1100ms)}
    );
    ASSERT_TRUE(nearest);
    EXPECT_EQ(nearest->slantRange(), 1005_m);
    const auto exact = _selector.selectTrackingObservation(
        *_repository.findConfirmedTrack(id),
        1100ms,
        {
            observation(1010_m, 1100ms),
            observation(1000_m, 1100ms),
            Observation(position(1000_m), 0_mps, -20.0f, 1100ms)
        }
    );
    ASSERT_TRUE(exact);
    EXPECT_EQ(exact->slantRange(), 1000_m);
    EXPECT_FLOAT_EQ(exact->power_db(), -10.0f);
    const auto tied = _selector.selectTrackingObservation(
        *_repository.findConfirmedTrack(id),
        1100ms,
        {observation(1010_m, 1100ms), observation(990_m, 1100ms)}
    );
    ASSERT_TRUE(tied);
    EXPECT_EQ(tied->slantRange(), 1010_m);
    EXPECT_EQ(_repository.findConfirmedTrack(id)->lastObservationTime(), 100ms);
}

TEST_F(TrackingAssociationTest, ComparesOnlyTheSuppliedTrackAndMatchingResultTime)
{
    const auto firstId = addTracking();
    const auto secondId = addTracking(2000_m, 2000_m);
    const auto observations = std::vector<Observation>({observation(2000_m, 1100ms), observation(1005_m, 1100ms)});
    const auto first = _selector.selectTrackingObservation(*_repository.findConfirmedTrack(firstId), 1100ms, observations);
    const auto second = _selector.selectTrackingObservation(*_repository.findConfirmedTrack(secondId), 1100ms, observations);
    ASSERT_TRUE(first);
    ASSERT_TRUE(second);
    EXPECT_EQ(first->slantRange(), 1005_m);
    EXPECT_EQ(second->slantRange(), 2000_m);
    EXPECT_FALSE(_selector.selectTrackingObservation(*_repository.findConfirmedTrack(firstId), 1200ms, observations));
    EXPECT_EQ(_repository.tracks().size(), 2u);
}

TEST_F(TrackingAssociationTest, ReusesOnePredictionPerBatchAndRefreshesAfterAnUpdate)
{
    const auto algorithm = std::make_shared<CountingPredictionAlgorithm>();
    const auto id = addTracking(1000_m, 1000_m, algorithm);
    auto& track = *_repository.findConfirmedTrack(id);
    const auto previousState = track.state();

    const auto selected = _selector.selectTrackingObservation(
        track,
        1100ms,
        {observation(1000_m, 1200ms), observation(1020_m, 1100ms), observation(1005_m, 1100ms), observation(995_m, 1100ms)}
    );
    ASSERT_TRUE(selected);
    EXPECT_EQ(selected->slantRange(), 1005_m);
    EXPECT_EQ(algorithm->predictionCount(), 1u);
    EXPECT_EQ(track.state(), previousState);
    EXPECT_EQ(track.lastResultTime(), 100ms);

    ASSERT_TRUE(track.update(observation(1010_m, 1100ms)));
    const auto next = _selector.selectTrackingObservation(
        *_repository.findConfirmedTrack(id),
        2100ms,
        {observation(1000_m, 2100ms), observation(1020_m, 2100ms)}
    );
    ASSERT_TRUE(next);
    EXPECT_EQ(next->slantRange(), 1020_m);
    EXPECT_EQ(algorithm->predictionCount(), 2u);
    EXPECT_EQ(track.lastObservationTime(), 1100ms);
}

TEST_F(TrackingAssociationTest, SkipsPredictionWithoutEligibleObservations)
{
    const auto algorithm = std::make_shared<CountingPredictionAlgorithm>();
    const auto id = addTracking(1000_m, 1000_m, algorithm);
    const auto& track = *_repository.findConfirmedTrack(id);

    EXPECT_FALSE(_selector.selectTrackingObservation(track, 1100ms, {}));
    EXPECT_FALSE(
        _selector.selectTrackingObservation(
            track,
            1100ms,
            {observation(1000_m, 1000ms), observation(1000_m, 1200ms)}
        )
    );
    EXPECT_FALSE(_selector.selectTrackingObservation(track, 100ms, {observation(1000_m, 100ms)}));
    EXPECT_EQ(algorithm->predictionCount(), 0u);
}

TEST_F(TrackingAssociationTest, DoesNotRetryFailedPredictionUntilTheNextBatch)
{
    const auto algorithm = std::make_shared<CountingPredictionAlgorithm>(false);
    const auto id = addTracking(1000_m, 1000_m, algorithm);
    const auto& track = *_repository.findConfirmedTrack(id);
    const auto previousState = track.state();
    const auto observations = std::vector<Observation>({
        observation(1000_m, 1100ms),
        observation(1001_m, 1100ms),
        observation(1002_m, 1100ms)
    });

    EXPECT_FALSE(_selector.selectTrackingObservation(track, 1100ms, observations));
    EXPECT_EQ(algorithm->predictionCount(), 1u);
    EXPECT_FALSE(_selector.selectTrackingObservation(track, 1100ms, observations));
    EXPECT_EQ(algorithm->predictionCount(), 2u);
    EXPECT_EQ(track.state(), previousState);
    EXPECT_EQ(track.lastResultTime(), 100ms);
    EXPECT_EQ(track.consecutiveMissCount(), 0u);
}

TEST_F(TrackingAssociationTest, RejectsIndefiniteCovarianceProvidedForObservedState)
{
    const auto indefinite = math::SquareMatrix({{1.0, 100.0, 0.0}, {100.0, 1.0, 0.0}, {0.0, 0.0, 1.0}});
    const auto id = addTracking(1000_m, 1000_m, std::make_shared<CovarianceAlgorithm>(indefinite));
    auto algorithm = NdsAlgorithm{MeasurementConfig(7_m)};
    EXPECT_THROW(algorithm.prepareFromLastObservation(*_repository.findConfirmedTrack(id)), std::invalid_argument);
}

TEST(TrackingAssociationControllerTest, OutOfGatePlotsKeepRetryingUntilThreeMissesDeleteTheTarget)
{
    auto controller = TargetController(
        TargetConfig(
            AssociationConfig(1000_mps, 30_mps, 11.345),
            MeasurementConfig(7_m),
            TrackingConfig(10_mps2, 0_mps2),
            TrackManagementConfig(100ms, 1000ms, 3, 100)
        )
    );
    const auto confirmation = controller.handleDetection(result(BeamType::SEARCH, 1, 0ms, {1000_m}));
    ASSERT_TRUE(confirmation);
    const auto tracking = controller.handleDetection(
        result(
            BeamType::CONFIRMATION,
            confirmation->requestId,
            100ms,
            {1000_m}
        )
    );
    ASSERT_TRUE(tracking);

    for (std::uint32_t miss = 1; miss <= 3; ++miss)
    {
        const auto time = 100ms + std::chrono::milliseconds(1000 * miss);
        const auto retry = controller.handleDetection(
            result(
                BeamType::TRACKING,
                confirmation->requestId,
                time,
                {5000_m}
            )
        );
        if (miss < 3)
        {
            ASSERT_TRUE(retry);
            EXPECT_EQ(retry->requestId, confirmation->requestId);
            EXPECT_EQ(retry->transmitTime, time + 1000ms);
            const auto events = controller.nextTrackEvents();
            ASSERT_EQ(events.size(), 1u);
            EXPECT_EQ(events[0].timestamp, 100ms);
        }
        else
        {
            EXPECT_FALSE(retry);
            EXPECT_TRUE(controller.nextTrackEvents().empty());
        }
    }
    const auto next = controller.handleDetection(result(BeamType::SEARCH, 1, 3200ms, {1200_m}));
    ASSERT_TRUE(next);
    EXPECT_EQ(next->requestId, 2u);
}

TEST(TrackingAssociationControllerTest, NearestSuccessResetsMissesAndPreservesOtherIdsWithoutRegisteringExtraPlots)
{
    auto controller = TargetController(
        TargetConfig(
            AssociationConfig(1000_mps, 30_mps, 11.345),
            MeasurementConfig(7_m),
            TrackingConfig(10_mps2, 0_mps2),
            TrackManagementConfig(100ms, 1000ms, 3, 100)
        )
    );
    const auto firstConfirmation = controller.handleDetection(result(BeamType::SEARCH, 1, 0ms, {1000_m}));
    ASSERT_TRUE(firstConfirmation);
    ASSERT_TRUE(
        controller.handleDetection(
            result(
                BeamType::CONFIRMATION,
                firstConfirmation->requestId,
                100ms,
                {1000_m}
            )
        )
    );
    const auto secondConfirmation = controller.handleDetection(result(BeamType::SEARCH, 2, 200ms, {2000_m}));
    ASSERT_TRUE(secondConfirmation);
    ASSERT_TRUE(
        controller.handleDetection(
            result(
                BeamType::CONFIRMATION,
                secondConfirmation->requestId,
                300ms,
                {2000_m}
            )
        )
    );

    ASSERT_TRUE(
        controller.handleDetection(
            result(
                BeamType::TRACKING,
                firstConfirmation->requestId,
                1100ms,
                {5000_m}
            )
        )
    );
    const auto recovered = controller.handleDetection(
        result(
            BeamType::TRACKING,
            firstConfirmation->requestId,
            2100ms,
            {1010_m, 1000_m, 2000_m, 3000_m}
        )
    );
    ASSERT_TRUE(recovered);
    EXPECT_EQ(recovered->requestId, firstConfirmation->requestId);
    const auto events = controller.nextTrackEvents();
    ASSERT_EQ(events.size(), 2u);
    EXPECT_EQ(events[0].id, firstConfirmation->requestId);
    EXPECT_EQ(events[0].slantRange, 1000_m);
    EXPECT_EQ(events[0].timestamp, 2100ms);
    EXPECT_EQ(events[1].id, secondConfirmation->requestId);
    EXPECT_EQ(events[1].slantRange, 2000_m);
    EXPECT_EQ(events[1].timestamp, 300ms);

    // 두 번째 트랙도 예정된 추적 결과를 받아 응답 없는 트랙으로 삭제되지 않게 한다.
    ASSERT_TRUE(
        controller.handleDetection(
            result(
                BeamType::TRACKING,
                secondConfirmation->requestId,
                1300ms,
                {2000_m}
            )
        )
    );

    ASSERT_TRUE(
        controller.handleDetection(
            result(
                BeamType::TRACKING,
                firstConfirmation->requestId,
                3100ms,
                {}
            )
        )
    );
    ASSERT_TRUE(
        controller.handleDetection(
            result(
                BeamType::TRACKING,
                firstConfirmation->requestId,
                4100ms,
                {}
            )
        )
    );
    EXPECT_FALSE(
        controller.handleDetection(
            result(
                BeamType::TRACKING,
                firstConfirmation->requestId,
                5100ms,
                {}
            )
        )
    );
    const auto remaining = controller.nextTrackEvents();
    ASSERT_EQ(remaining.size(), 1u);
    EXPECT_EQ(remaining[0].id, secondConfirmation->requestId);
    EXPECT_EQ(remaining[0].timestamp, 1300ms);

    // 남은 트랙은 3.9초간 관측되지 않아 예측 불확실성과 연관 허용 범위가 커진다.
    const auto next = controller.handleDetection(result(BeamType::SEARCH, 1, 5200ms, {10000_m}));
    ASSERT_TRUE(next);
    EXPECT_EQ(next->requestId, 3u);
}

} // namespace
} // namespace target
