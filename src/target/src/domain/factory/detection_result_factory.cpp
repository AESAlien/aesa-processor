#include "detection_result_factory.hpp"

#include "observation_factory.hpp"

#include <stdexcept>

namespace target
{

DetectionResult DetectionResultFactory::from(const DetectionEvent& detection)
{
    switch (detection.beamType)
    {
    case beam::BeamType::SEARCH:
        return SearchDetectionResult(
            detection.requestId,
            detection.transmitTime,
            ObservationFactory::from(detection)
        );

    case beam::BeamType::CONFIRMATION:
        return ConfirmationDetectionResult(
            detection.requestId,
            detection.transmitTime,
            ObservationFactory::from(detection)
        );

    case beam::BeamType::TRACKING:
        return TrackingDetectionResult(
            detection.requestId,
            detection.transmitTime,
            ObservationFactory::from(detection)
        );

    default:
        throw std::invalid_argument("Unsupported detection beam type");
    }
}

} // namespace target
