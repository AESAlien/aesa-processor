#pragma once

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <optional>

namespace target
{

class TrackManagementConfig
{
public:
    // 콘솔 메시지 한 건에 담을 수 있는 최대 트랙 수다.
    static constexpr std::size_t MAXIMUM_REPORTED_TRACK_COUNT = 100;

    class Builder
    {
    public:
        Builder& confirmationDelay(std::chrono::milliseconds value)
        {
            _confirmationDelay = value;
            return *this;
        }

        Builder& trackingInterval(std::chrono::milliseconds value)
        {
            _trackingInterval = value;
            return *this;
        }

        Builder& maxConsecutiveTrackingMisses(std::uint32_t value)
        {
            _maxConsecutiveTrackingMisses = value;
            return *this;
        }

        Builder& maxReportedTrackCount(std::size_t value)
        {
            _maxReportedTrackCount = value;
            return *this;
        }

        TrackManagementConfig build() const;

    private:
        std::optional<std::chrono::milliseconds> _confirmationDelay;
        std::optional<std::chrono::milliseconds> _trackingInterval;
        std::optional<std::uint32_t> _maxConsecutiveTrackingMisses;
        std::optional<std::size_t> _maxReportedTrackCount;
    };

    TrackManagementConfig(
        std::chrono::milliseconds confirmationDelay,
        std::chrono::milliseconds trackingInterval,
        std::uint32_t maxConsecutiveTrackingMisses,
        std::size_t maxReportedTrackCount
    );

    std::chrono::milliseconds confirmationDelay() const noexcept;
    std::chrono::milliseconds trackingInterval() const noexcept;
    std::uint32_t maxConsecutiveTrackingMisses() const noexcept;
    std::size_t maxReportedTrackCount() const noexcept;

private:
    std::chrono::milliseconds _confirmationDelay;
    std::chrono::milliseconds _trackingInterval;
    std::uint32_t _maxConsecutiveTrackingMisses;
    std::size_t _maxReportedTrackCount;
    void validate() const;
};

} // namespace target
