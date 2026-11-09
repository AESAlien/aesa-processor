#include "detection_result.hpp"

#include <stdexcept>
#include <utility>

namespace target
{

DetectionResultBase::DetectionResultBase(
    std::uint32_t requestId,
    std::chrono::milliseconds transmitTime,
    std::vector<Observation> observations
) : requestId(requestId),
    transmitTime(transmitTime),
    observations(std::move(observations))
{
    if (transmitTime.count() < 0)
    {
        throw std::invalid_argument("Invalid beam detection metadata");
    }
}

SearchDetectionResult::SearchDetectionResult(
    std::uint32_t requestId,
    std::chrono::milliseconds transmitTime,
    std::vector<Observation> observations
) : DetectionResultBase(requestId, transmitTime, std::move(observations))
{
}

ConfirmationDetectionResult::ConfirmationDetectionResult(
    std::uint32_t requestId,
    std::chrono::milliseconds transmitTime,
    std::vector<Observation> observations
) : DetectionResultBase(requestId, transmitTime, std::move(observations))
{
    if (requestId == 0)
    {
        throw std::invalid_argument("Invalid beam detection metadata");
    }
}

TrackingDetectionResult::TrackingDetectionResult(
    std::uint32_t requestId,
    std::chrono::milliseconds transmitTime,
    std::vector<Observation> observations
) : DetectionResultBase(requestId, transmitTime, std::move(observations))
{
    if (requestId == 0)
    {
        throw std::invalid_argument("Invalid beam detection metadata");
    }
}

} // namespace target
