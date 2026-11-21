#pragma once

#include <cstddef>
#include <optional>

namespace target
{

class TrackReportingConfig
{
public:
    // 콘솔 메시지 한 건에 담을 수 있는 최대 트랙 수다.
    static constexpr std::size_t MAXIMUM_REPORTED_TRACK_COUNT = 100;

    class Builder
    {
    public:
        Builder& maxReportedTrackCount(std::size_t value)
        {
            _maxReportedTrackCount = value;
            return *this;
        }

        TrackReportingConfig build() const;

    private:
        std::optional<std::size_t> _maxReportedTrackCount;
    };

    explicit TrackReportingConfig(std::size_t maxReportedTrackCount);
    std::size_t maxReportedTrackCount() const noexcept;

private:
    std::size_t _maxReportedTrackCount;
};

} // namespace target
