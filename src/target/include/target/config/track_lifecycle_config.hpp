#pragma once

#include <chrono>
#include <cstdint>
#include <optional>

namespace target
{

class TrackLifecycleConfig
{
public:
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

        TrackLifecycleConfig build() const;

    private:
        std::optional<std::chrono::milliseconds> _confirmationDelay;
        std::optional<std::chrono::milliseconds> _trackingInterval;
        std::optional<std::uint32_t> _maxConsecutiveTrackingMisses;
    };

    TrackLifecycleConfig(
        std::chrono::milliseconds confirmationDelay,
        std::chrono::milliseconds trackingInterval,
        std::uint32_t maxConsecutiveTrackingMisses
    );

    std::chrono::milliseconds confirmationDelay() const noexcept;
    std::chrono::milliseconds trackingInterval() const noexcept;
    std::uint32_t maxConsecutiveTrackingMisses() const noexcept;
    std::chrono::milliseconds responseTimeout() const noexcept;

private:
    std::chrono::milliseconds _confirmationDelay;
    std::chrono::milliseconds _trackingInterval;
    std::uint32_t _maxConsecutiveTrackingMisses;
};

} // namespace target
