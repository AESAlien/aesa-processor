#include "scheduler/request_beam_policy.hpp"

#include <stdexcept>

namespace beam
{
namespace
{

class ConfirmationBeamPolicy final : public IRequestBeamPolicy
{
public:
    BeamType transmitBeamType() const override
    {
        return BeamType::CONFIRMATION;
    }

    math::Angle beamWidth(const RequestBeamConfig& config) const override
    {
        return config.confirmationBeamWidth();
    }

    int priority() const override
    {
        return 1;
    }
};

class TrackingBeamPolicy final : public IRequestBeamPolicy
{
public:
    BeamType transmitBeamType() const override
    {
        return BeamType::TRACKING;
    }

    math::Angle beamWidth(const RequestBeamConfig& config) const override
    {
        return config.trackingBeamWidth();
    }

    int priority() const override
    {
        return 0;
    }
};

const ConfirmationBeamPolicy CONFIRMATION_POLICY{};
const TrackingBeamPolicy TRACKING_POLICY{};

} // namespace

// 요청 빔 종류에 따른 분기는 이 함수 한 곳에만 둔다.
const IRequestBeamPolicy& requestBeamPolicyOf(RequestBeamType beamType)
{
    switch (beamType)
    {
    case RequestBeamType::CONFIRMATION:
        return CONFIRMATION_POLICY;

    case RequestBeamType::TRACKING:
        return TRACKING_POLICY;
    }

    throw std::invalid_argument("Unknown beam request type");
}

} // namespace beam
