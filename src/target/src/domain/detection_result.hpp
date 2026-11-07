#pragma once

#include "observation.hpp"

#include <chrono>
#include <cstdint>
#include <variant>
#include <vector>

namespace target
{

class DetectionResultFactory;

struct DetectionResultBase
{
    const std::uint32_t requestId;
    const std::chrono::milliseconds transmitTime;
    const std::vector<Observation> observations;

    virtual ~DetectionResultBase() = default;

protected:
    DetectionResultBase(
        std::uint32_t requestId,
        std::chrono::milliseconds transmitTime,
        std::vector<Observation> observations
    );
};

struct SearchDetectionResult final : DetectionResultBase
{
private:
    friend class DetectionResultFactory;
    SearchDetectionResult(
        std::uint32_t requestId,
        std::chrono::milliseconds transmitTime,
        std::vector<Observation> observations
    );
};

struct ConfirmationDetectionResult final : DetectionResultBase
{
private:
    friend class DetectionResultFactory;
    ConfirmationDetectionResult(
        std::uint32_t requestId,
        std::chrono::milliseconds transmitTime,
        std::vector<Observation> observations
    );
};

struct TrackingDetectionResult final : DetectionResultBase
{
private:
    friend class DetectionResultFactory;
    TrackingDetectionResult(
        std::uint32_t requestId,
        std::chrono::milliseconds transmitTime,
        std::vector<Observation> observations
    );
};

using DetectionResult = std::variant<
    SearchDetectionResult,
    ConfirmationDetectionResult,
    TrackingDetectionResult>;

} // namespace target
